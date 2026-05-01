// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cow context logic.
 */

#include <linux/cleanup.h>
#include <linux/maple_tree.h>
#include <linux/lockdep.h>
#include <linux/mm.h>
#include <linux/mmap_lock.h>
#include <linux/rculist.h>
#include <linux/rmap.h>
#include "internal.h"

/* Simple dynamic array. */
struct dynarray {
	int nr, cap;
	void *contents[] __counted_by(cap);
};

/*
 * Represents different types of remap entry:
 *
 *  Simple (1 entry)             - val << 2 | 1, maybe (1<<1)
 * Complex (N entries)           - struct multi_remaps *
 * Complex (N entries), on stack - struct multi_remaps * | (1<<1)
 */
typedef union {
	struct dynarray *multi;
	unsigned long raw;
	long offset;
	void *entry;
} remaps_entry_t;

/* Dynamic array API. */

struct dynarray *dynarray_dup(struct dynarray *arr, gfp_t gfp);
struct dynarray *__dynarray_append(struct dynarray *arr, void *val, gfp_t gfp);
struct dynarray *__dynarray_append_stack(struct dynarray *arr,
					 void *val, gfp_t gfp);
void dynarray_shrink_rcu(struct dynarray *arr);

#define EMPTY_DYNARRAY ((struct dynarray) {})

#define __DECLARE_DYNARRAY(_name, _nr, _cap, _val)			\
	union {								\
		struct {						\
			int nr, cap;					\
			void *contents[_cap];				\
		};							\
		struct dynarray arr;					\
	} _name = {							\
		.nr = _nr,						\
		.cap = _cap,						\
		.contents = { _val }					\
	}

#define DECLARE_DYNARRAY_SINGLE(_name, _val)			 \
	__DECLARE_DYNARRAY(_name ## _arr, 1, 1, (void *)(_val)); \
	struct dynarray *_name = &((_name ## _arr).arr)

#define DECLARE_DYNARRAY(_name, _cap)				\
	__DECLARE_DYNARRAY(_name ## _arr, /*nr=*/0, _cap,);	\
	struct dynarray *_name = &((_name ## _arr).arr)

#define DYNARRAY_DEFAULT_CAP 16


#define __dynarray_get_t(type_or_val, _idx, _arr)		\
	((typeof(type_or_val))READ_ONCE(_arr->contents[_idx]))

#define __dynarray_for_each_reverse(_idx, _arr, _val)			  \
	for (_idx = dynarray_nr(_arr) - 1;				  \
	     _idx >= 0 && (_val = __dynarray_get_t(_val, _idx, _arr), 1); \
	     _idx--)

#define __dynarray_set(_idx, _arr, _val)			\
	do {							\
		WRITE_ONCE(_arr->contents[_idx], (void *)_val);	\
	} while (0)

/* Remaps API. */

#define remaps_for_each_entry(_mas_remaps, _remaps, _pgoff_last)	\
	lockdep_assert_in_rcu_read_lock();				\
	mas_for_each(_mas_remaps, (_remaps).entry, _pgoff_last)

#define remaps_for_each_entry_offset(_idx, _remaps, _curr_offset)	\
	for (_idx = 0;							\
	     _idx < nr_remaps(_remaps) &&				\
		     ((_curr_offset = get_remap(_remaps, _idx)), 1);	\
	     _idx++)

#define remaps_for_each(_idx, _mas_remaps, _remaps, _curr_offset, _pgoff_last)	\
	remaps_for_each_entry(_mas_remaps, _remaps, _pgoff_last)		\
		remaps_for_each_entry_offset(_idx, _remaps, _curr_offset)

#define REMAP_IS_SIMPLE   (1UL)
#define REMAP_IS_ON_STACK (1UL << 1)
#define REMAP_ENTRY_MASK (REMAP_IS_SIMPLE | REMAP_IS_ON_STACK)
#define REMAP_SIMPLE_SHIFT 2

#define EMPTY_REMAPS_ENTRY ((remaps_entry_t)NULL)

#define DECLARE_STACK_DYNARRAY_SIMPLE_REMAP(_name, _remap)		\
	DECLARE_DYNARRAY_SINGLE(_name, get_simple_remap(_remap))

remaps_entry_t mk_stack_multi_remaps(struct dynarray *stack_multi);
remaps_entry_t mk_simple_remap(long offset);
bool same_remaps(remaps_entry_t a, remaps_entry_t b);
int nr_remaps(remaps_entry_t remaps);
long get_remap(remaps_entry_t remaps, int index);
void set_multi_remap(remaps_entry_t remaps, int index, long offset);
void free_remaps(remaps_entry_t remaps);

void store_new_remaps_entry(struct ma_state *mas_remaps, pgoff_t pgoff,
			    pgoff_t pgoff_last, remaps_entry_t remaps);
void store_new_simple_remap(struct ma_state *mas_remaps, pgoff_t pgoff,
			    pgoff_t pgoff_last, long new_offset);

static int __dynarray_nr(struct dynarray *arr)
{
	return READ_ONCE(arr->nr);
}

static int dynarray_nr(struct dynarray *arr)
{
	return smp_load_acquire(&arr->nr);
}

static void dynarray_set_nr(struct dynarray *arr, int new)
{
	smp_store_release(&arr->nr, new);
}

static int dynarray_cap(struct dynarray *arr)
{
	/* Will only be updated once RCU-assigned. */
	return arr->cap;
}

static struct dynarray *__dynarray_dup(struct dynarray *arr, int cap, gfp_t gfp)
{
	const int nr = __dynarray_nr(arr);
	struct dynarray *ret;

	ret = kmalloc_flex(*arr, contents, cap, gfp);
	if (!ret)
		return NULL;

	/* Just allocated, safe to assign. */
	ret->nr = nr;
	ret->cap = cap;

	if (arr)
		memcpy(ret->contents, arr->contents, sizeof(void *) * nr);
	return ret;
}

struct dynarray *dynarray_dup(struct dynarray *arr, gfp_t gfp)
{
	return __dynarray_dup(arr, dynarray_cap(arr), gfp);
}

/*
 * TODO: Insertion sort into a sorted array, O(n^2) aggregate but cache
 *       friendly. We can probably do better though :)
 */
static void __dynarray_append_value(struct dynarray *arr, void *new_val)
{
	int i, insert_idx;
	unsigned long val;
	unsigned long new_val_raw = (unsigned long)new_val;

	VM_WARN_ON_ONCE(arr->nr == arr->cap);

	/* Check if dupe, find insert point... */
	__dynarray_for_each_reverse(i, arr, val) {
		if (val == new_val_raw)
			return;
		if (val < new_val_raw)
			break;
	}
	insert_idx = i + 1;

	/* ...Move everything up... */
	__dynarray_for_each_reverse(i, arr, val) {
		if (i <= insert_idx)
			break;
		__dynarray_set(i + 1, arr, val);
	}
	/* ...and insert. */
	__dynarray_set(insert_idx, arr, new_val);
	/* We assume we are the exclusive writer, so no risk of a tear. */
	dynarray_set_nr(arr, __dynarray_nr(arr) + 1);
}

static struct dynarray *__dynarray_expand(struct dynarray *arr,
					  bool from_stack, gfp_t gfp)
{
	const int new_cap = 2 * dynarray_cap(arr);
	struct dynarray *ret;
	size_t new_size;

	VM_WARN_ON_ONCE(__dynarray_nr(arr) != dynarray_cap(arr));

	if (from_stack)
		return __dynarray_dup(arr, new_cap, gfp);

	new_size = struct_size(arr, contents, new_cap);
	ret = krealloc(arr, new_size, gfp);
	if (!ret)
		return ret;

	/*
	 * Other fields copied over, only need to update capacity. Safe as
	 * nothing references this yet.
	 */
	ret->cap = new_cap;
	return ret;
}

static bool __dynarray_needs_expand(struct dynarray *arr)
{
	VM_WARN_ON_ONCE(arr->nr > arr->cap);

	return dynarray_cap(arr) == __dynarray_nr(arr);
}

static struct dynarray *__dynarray_maybe_expand(struct dynarray *arr,
						bool from_stack, gfp_t gfp)
{
	if (__dynarray_needs_expand(arr))
		return __dynarray_expand(arr, from_stack, gfp);
	return arr;
}

struct dynarray *__dynarray_append(struct dynarray *arr, void *val, gfp_t gfp)
{
	struct dynarray *new_arr;

	new_arr = __dynarray_maybe_expand(arr, /*from_stack=*/false, gfp);
	__dynarray_append_value(new_arr, val);
	return new_arr;
}

struct dynarray *__dynarray_append_stack(struct dynarray *arr,
					 void *val, gfp_t gfp)
{
	struct dynarray *new_arr;

	new_arr = __dynarray_maybe_expand(arr, /*from_stack=*/true, gfp);
	__dynarray_append_value(new_arr, val);
	return new_arr;
}

static void dynarray_free_rcu(struct dynarray *arr)
{
	/* We're being accessed by RCU readers. */
	kfree_rcu_mightsleep(arr);
}

void dynarray_shrink_rcu(struct dynarray *arr)
{
	const int nr = __dynarray_nr(arr);

	/* We are the exclusive writer so no chance of a tear. */
	if (nr == 1) {
		dynarray_set_nr(arr, 0);
		dynarray_free_rcu(arr);
	}
}

remaps_entry_t mk_stack_multi_remaps(struct dynarray *stack_multi)
{
	remaps_entry_t ret = {
		.multi = stack_multi,
	};

	ret.raw |= REMAP_IS_ON_STACK;
	return ret;
}

static bool is_simple_remap_entry(remaps_entry_t remaps)
{
	return remaps.raw & REMAP_IS_SIMPLE;
}

static bool is_remap_entry_on_stack(remaps_entry_t remaps)
{
	return remaps.raw & REMAP_IS_ON_STACK;
}

static long get_simple_remap(remaps_entry_t remaps)
{
	/* Signedness retained. */
	return remaps.offset >> REMAP_SIMPLE_SHIFT;
}

static struct dynarray *get_multi_remaps(remaps_entry_t remaps)
{
	remaps_entry_t copy = remaps;

	copy.raw &= ~REMAP_ENTRY_MASK;
	return copy.multi;
}

remaps_entry_t mk_simple_remap(long offset)
{
	remaps_entry_t ret = { .offset = offset << REMAP_SIMPLE_SHIFT };

	ret.raw |= REMAP_IS_SIMPLE;
	return ret;
}

bool same_remaps(remaps_entry_t a, remaps_entry_t b)
{
	return a.raw == b.raw;
}

static bool is_empty_remaps(remaps_entry_t remaps)
{
	remaps_entry_t copy = remaps;

	copy.raw &= ~REMAP_ENTRY_MASK;
	return !copy.raw;
}

int nr_remaps(remaps_entry_t remaps)
{
	if (is_empty_remaps(remaps))
		return 0;
	if (is_simple_remap_entry(remaps))
		return 1;
	return dynarray_nr(get_multi_remaps(remaps));
}

long get_remap(remaps_entry_t remaps, int index)
{
	struct dynarray *arr;

	if (is_simple_remap_entry(remaps))
		return get_simple_remap(remaps);

	arr = get_multi_remaps(remaps);
	return __dynarray_get_t(long, index, arr);
}

void set_multi_remap(remaps_entry_t remaps, int index,
		     long offset)
{
	struct dynarray *arr = get_multi_remaps(remaps);

	__dynarray_set(index, arr, offset);
}

void free_remaps(remaps_entry_t remaps)
{
	if (is_empty_remaps(remaps))
		return;
	if (is_simple_remap_entry(remaps))
		return;
	if (is_remap_entry_on_stack(remaps))
		return;

	dynarray_free_rcu(get_multi_remaps(remaps));
}

void store_new_remaps_entry(struct ma_state *mas_remaps, pgoff_t pgoff,
			    pgoff_t pgoff_last, remaps_entry_t remaps)
{
	mas_set_range(mas_remaps, pgoff, pgoff_last);

	mas_lock(mas_remaps);
	mas_store_gfp(mas_remaps, remaps.entry, GFP_KERNEL);
	mas_unlock(mas_remaps);
}

void store_new_simple_remap(struct ma_state *mas_remaps, pgoff_t pgoff,
			    pgoff_t pgoff_last, long new_offset)
{
	remaps_entry_t remaps = mk_simple_remap(new_offset);

	store_new_remaps_entry(mas_remaps, pgoff, pgoff_last, remaps);
}

static struct cow_context *delete_child_from_parent(struct cow_context *context)
{
	/*
	 * Safe to access, set on context allocation and child refcount
	 * pins parent in place until freed.
	 */
	struct cow_context *parent = context->parent;

	if (!parent)
		return NULL;

	spin_lock(&parent->list_write_lock);
	list_del_rcu(&context->siblings);
	spin_unlock(&parent->list_write_lock);

	if (!refcount_dec_and_test(&parent->refcnt))
		return NULL;

	return parent;
}

void __put_cow_context(struct cow_context *context)
{
	struct cow_context *curr, *parent;

	/* Iteratively cascade to parents. */
	for (curr = context; curr; curr = parent) {
		VM_WARN_ON_ONCE(!list_empty(&context->children));
		parent = delete_child_from_parent(curr);
		kfree_rcu(curr, rcu);
	}
}

void drop_cow_context(struct mm_struct *mm)
{
	struct cow_context *context = mm->cow_context;

	WRITE_ONCE(context->mm, NULL);
	put_cow_context(context);
}

void dup_cow_context(struct mm_struct *mm, struct mm_struct *oldmm)
{
	struct cow_context *parent = oldmm->cow_context;
	struct cow_context *context = mm->cow_context;

	context->parent = parent;

	spin_lock(&parent->list_write_lock);
	list_add_rcu(&context->siblings, &parent->children);
	spin_unlock(&parent->list_write_lock);

	get_cow_context(parent);
}

void mm_init_cow_context(struct mm_struct *mm)
{
	struct cow_context *context = kzalloc_obj(*context, GFP_KERNEL);

	context->mm = mm;
	mt_init_flags(&context->remap_mt, MM_MT_FLAGS);
	refcount_set(&context->refcnt, 1); /* Referenced by the mm. */
	INIT_LIST_HEAD(&context->children);
	INIT_LIST_HEAD(&context->siblings);
	spin_lock_init(&context->list_write_lock);
	mm->cow_context = context;
}
