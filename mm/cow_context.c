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

struct walk_context_control {
	void *arg;
	bool (*walk_one)(struct folio *folio, struct vm_area_struct *vma,
			 unsigned long addr, void *arg);
};

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
void dynarray_shrink(struct dynarray *arr);

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

/* How many remaps we keep on the stack before allocating on walk. */
#define WALK_DEFAULT_STACK_SIZE 16

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

bool find_and_remap_existing(struct cow_context *context,
			     pgoff_t pgoff, unsigned long nr_pages,
			     long old_offset, long new_offset);
void find_and_unmap_existing(struct cow_context *context, pgoff_t pgoff,
			     unsigned long nr_pages, long old_offset);
void add_new_remap(struct cow_context *context, pgoff_t pgoff,
		   unsigned long nr_pages, long new_offset, gfp_t gfp);

bool should_track_remap(struct vm_area_struct *vma);

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

void dynarray_shrink(struct dynarray *arr)
{
	const int nr = __dynarray_nr(arr);

	VM_WARN_ON_ONCE(nr == 1);

	/* We are the exclusive writer so no chance of a tear. */
	dynarray_set_nr(arr, nr - 1);
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

/*
 * Reduce the count of remaps by 1, freeing the entry in the remaps maple tree
 * if reduced to zero.
 *
 * RCU read lock must be held.
 */
static void shrink_remaps(struct cow_context *context,
			  struct ma_state *mas_remaps, pgoff_t pgoff,
			  pgoff_t pgoff_last, remaps_entry_t remaps)
{
	int nr;
	remaps_entry_t new_remaps;
	pgoff_t pgoff_next;

	lockdep_assert_in_rcu_read_lock();

	/* mmap_downgrade() makes life hard. */
	spin_lock(&context->concurrent_unmap_lock);

	nr = nr_remaps(remaps);

	/* If multi, simple. */
	if (nr > 2) {
		dynarray_shrink(get_multi_remaps(remaps));
		spin_unlock(&context->concurrent_unmap_lock);
		return;
	}

	if (nr == 2)
		new_remaps = mk_simple_remap(get_remap(remaps, 0));
	else /* nr == 1 */
		new_remaps = EMPTY_REMAPS_ENTRY;

	pgoff_next = mas_remaps->last + 1;

	spin_unlock(&context->concurrent_unmap_lock);
	/* Now we're allocating so must drop the lock. RCU readers are safe. */
	rcu_read_unlock();
	store_new_remaps_entry(mas_remaps, pgoff, pgoff_last, new_remaps);
	free_remaps(remaps);
	mas_set(mas_remaps, pgoff_next);
	rcu_read_lock();
}

static remaps_entry_t append_remap_offset(remaps_entry_t remaps,
					  long new_offset, gfp_t gfp)
{
	remaps_entry_t ret;

	if (is_empty_remaps(remaps))
		return mk_simple_remap(new_offset);

	if (is_simple_remap_entry(remaps)) {
		DECLARE_STACK_DYNARRAY_SIMPLE_REMAP(arr, remaps);

		/*
		 * We declare on the stack as a cheap way of expanding ->
		 * allocated :)
		 */
		ret.multi = __dynarray_append_stack(arr, (void *)new_offset, gfp);
	} else if (is_remap_entry_on_stack(remaps)) {
		struct dynarray *arr, *new;

		arr = get_multi_remaps(remaps);
		new = __dynarray_append_stack(arr, (void *)new_offset, gfp);
		ret.multi = new;
		if (arr == new)
			ret.raw |= REMAP_IS_ON_STACK;
	} else {
		struct dynarray *arr = get_multi_remaps(remaps);

		ret.multi = __dynarray_append(arr, (void *)new_offset, gfp);
	}

	return ret;
}

static remaps_entry_t dup_remaps(remaps_entry_t remaps, gfp_t gfp)
{
	remaps_entry_t ret;

	if (is_empty_remaps(remaps) || is_simple_remap_entry(remaps))
		ret = remaps;
	else
		ret.multi = dynarray_dup(remaps.multi, gfp);
	return ret;
}

static remaps_entry_t get_remaps(struct ma_state *mas_remaps)
{
	remaps_entry_t ret;

	rcu_read_lock();
	ret.entry = mas_walk(mas_remaps);
	rcu_read_unlock();

	return ret;
}

static void split_remap(struct cow_context *context, pgoff_t pgoff,
			pgoff_t pgoff_split, long new_offset, gfp_t gfp)
{
	MA_STATE(mas_remaps, &context->remap_mt, pgoff, pgoff);
	remaps_entry_t remaps, dup;

	remaps = get_remaps(&mas_remaps);
	VM_WARN_ON_ONCE(is_empty_remaps(remaps));

	/* Non-overlapping portion. */
	dup = dup_remaps(remaps, gfp);
	VM_WARN_ON_ONCE(pgoff_split == pgoff);
	if (pgoff_split < pgoff) {
		VM_WARN_ON_ONCE(mas_remaps.index != pgoff_split);
		mas_set_range(&mas_remaps, pgoff_split, pgoff -1);
		mas_lock(&mas_remaps);
		mas_store_gfp(&mas_remaps, dup.entry, gfp);
		mas_unlock(&mas_remaps);
	} else {
		/* Split after. */
		VM_WARN_ON_ONCE(mas_remaps.last != pgoff_split);
		mas_set_range(&mas_remaps, pgoff + 1, pgoff_split);
		mas_lock(&mas_remaps);
		mas_store_gfp(&mas_remaps, dup.entry, gfp);
		mas_unlock(&mas_remaps);
	}
}

/*
 * When remapping, there are two possibilities:
 *
 * 1. No existing remap exists for this pgoff mapping to old_offset.
 * 2. >=1 remap for this pgoff exists (possibly split due to other remaps).
 *
 * Returns true if we found existing remaps and updated them, or false
 * otherwise.
 */
bool find_and_remap_existing(struct cow_context *context,
			     pgoff_t pgoff, unsigned long nr_pages,
			     long old_offset, long new_offset)
{
	const pgoff_t pgoff_last = pgoff + nr_pages - 1;
	MA_STATE(mas_remaps, &context->remap_mt, pgoff, pgoff_last);
	remaps_entry_t remaps;
	bool found = false;
	long curr_offset;
	int i;

	mmap_assert_write_locked(context->mm);

	remaps_for_each(i, &mas_remaps, remaps, curr_offset, pgoff_last) {
		if (curr_offset != old_offset)
			continue;

		found = true;
		if (is_simple_remap_entry(remaps))
			store_new_simple_remap(&mas_remaps, mas_remaps.index,
					       mas_remaps.last, new_offset);
		else
			set_multi_remap(remaps, i, new_offset);
	}

	return found;
}

void find_and_unmap_existing(struct cow_context *context, pgoff_t pgoff,
			     unsigned long nr_pages, long old_offset)
{
	const pgoff_t pgoff_last = pgoff + nr_pages - 1;
	MA_STATE(mas_remaps, &context->remap_mt, pgoff, pgoff_last);
	remaps_entry_t remaps;
	long curr_offset;
	int i;

	rcu_read_lock();
	remaps_for_each_entry(&mas_remaps, remaps, pgoff_last) {
		bool unmapping = false;

		remaps_for_each_entry_offset(i, remaps, curr_offset) {
			if (unmapping) {
				/* Here i must be >1 so we know it's multi. */
				VM_WARN_ON_ONCE(is_simple_remap_entry(remaps));
				set_multi_remap(remaps, i - 1, curr_offset);
			} else if (curr_offset == old_offset) {
				unmapping = true;
			}
		}

		if (unmapping)
			shrink_remaps(context, &mas_remaps, mas_remaps.index,
				      mas_remaps.last, remaps);
	}
	rcu_read_unlock();
}

/*
 * Iterate through [pgoff, pgoff_last], filling in any gaps and
 * appending to any existing entries.
 */
static void add_overlapping_remap(struct ma_state *mas_remaps,
				  pgoff_t pgoff, pgoff_t pgoff_last,
				  long new_offset)
{
	pgoff_t pgoff_prev = pgoff;
	remaps_entry_t remaps;

	mas_set_range(mas_remaps, pgoff, pgoff_last);
	remaps_for_each_entry(mas_remaps, remaps, pgoff_last) {
		const pgoff_t pgoff_left = mas_remaps->index;
		const pgoff_t pgoff_right = mas_remaps->last;
		remaps_entry_t new_remaps;

		/*
		 * Fill in a gap if it exists:
		 *
		 * |--------|.     Gap      |-----------|
		 * |        |<------------->|           |
		 * |--------|.              |-----------|
		 *      pgoff_prev      pgoff_left  pgoff_right
		 */
		if (pgoff_left > pgoff_prev)
			store_new_simple_remap(mas_remaps, pgoff_prev,
					       pgoff_left - 1, new_offset);

		/*
		 * Append new offset to existing entry:
		 *
		 *                         |-----------|
		 *                         |           |
		 *                         |-----------|
		 *                    pgoff_left  pgoff_right
		 */
		new_remaps = append_remap_offset(remaps, new_offset, GFP_KERNEL);
		if (!same_remaps(new_remaps, remaps))
			store_new_remaps_entry(mas_remaps, pgoff_left, pgoff_right,
					       new_remaps);

		pgoff_prev = pgoff_right + 1;
		mas_set(mas_remaps, pgoff_prev);
	}

	/* Trailing gap. */
	if (pgoff_prev <= pgoff_last)
		store_new_simple_remap(mas_remaps, pgoff_prev, pgoff_last,
				       new_offset);
}

void add_new_remap(struct cow_context *context, pgoff_t pgoff,
		   unsigned long nr_pages, long new_offset, gfp_t gfp)
{
	const pgoff_t pgoff_last = pgoff + nr_pages - 1;
	MA_STATE(mas_remaps, &context->remap_mt, pgoff, pgoff_last);

	/*
	 * If there is an overlapping entry that extends past the start of the
	 * range, we need to split LEFT:
	 *
	 *  pgoff_first_range  pgoff
	 *         .             .
	 *         .<----------->.
	 *         |-------------|------|
	 *         |    remap ent|y     |
	 *         |-------------|------|
	 *                     split
	 */
	if (!is_empty_remaps(get_remaps(&mas_remaps))) {
		const pgoff_t pgoff_first_range = mas_remaps.index;

		if (pgoff_first_range < pgoff)
			split_remap(context, pgoff, pgoff_first_range,
				    new_offset, gfp);
	}

	/*
	 * If there is an overlapping entry that extends past the end of the
	 * range, we need to split RIGHT:
	 *
	 *           pgoff_last  pgoff_last_range
	 *                .             .
	 *                .<----------->.
	 *         |------|-------------|
	 *         |    re|ap entry     |
	 *         |------|-------------|
	 *              split
	 */
	mas_set(&mas_remaps, pgoff_last);
	if (!is_empty_remaps(get_remaps(&mas_remaps))) {
		const pgoff_t pgoff_last_range = mas_remaps.last;

		if (pgoff_last_range > pgoff_last)
			split_remap(context, pgoff_last, pgoff_last_range,
				    new_offset, gfp);
	}

	add_overlapping_remap(&mas_remaps, pgoff, pgoff_last, new_offset);
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

static void delete_remaps(struct cow_context *context)
{
	MA_STATE(mas_remaps, &context->remap_mt, 0, 0);
	remaps_entry_t remaps;

	/* By now nothing references the remaps so we're safe to do this. */
	remaps_for_each_entry(&mas_remaps, remaps, UINT_MAX)
		free_remaps(remaps);

	mtree_destroy(&context->remap_mt);
}

void __put_cow_context(struct cow_context *context)
{
	struct cow_context *curr, *parent;

	/* Iteratively cascade to parents. */
	for (curr = context; curr; curr = parent) {
		VM_WARN_ON_ONCE(!list_empty(&context->children));

		parent = delete_child_from_parent(curr);

		rcu_read_lock();
		delete_remaps(curr);
		rcu_read_unlock();

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
	spin_lock_init(&context->concurrent_unmap_lock);
	mm->cow_context = context;
}

static bool should_track_anon_remap(struct vm_area_struct *vma)
{
	const pgoff_t pgoff_moved = vma->vm_start >> PAGE_SHIFT;
	const pgoff_t pgoff = vma->vm_pgoff;

	if (!vma_is_anonymous(vma))
		return false;
	if (!vma->anon_vma)
		return false;

	return pgoff != pgoff_moved;
}

static bool should_track_map_private_remap(struct vm_area_struct *vma)
{
	const pgoff_t pgoff_moved = vma->vm_start >> PAGE_SHIFT;
	const pgoff_t pgoff = vma->vm_pgoff;

	if (vma_is_anonymous(vma))
		return false;
	if (vma_flags_test_any(&vma->flags, VMA_SHARED_BIT, VMA_MAYSHARE_BIT))
		return false;
	if (!vma->anon_vma)
		return false;

	return pgoff != pgoff_moved;
}

bool should_track_remap(struct vm_area_struct *vma)
{
	if (should_track_anon_remap(vma))
		return true;
	if (should_track_map_private_remap(vma))
		return true;

	return false;
}

void cow_context_vma_unmap(struct vm_area_struct *vma)
{
	const pgoff_t pgoff_moved = vma->vm_start >> PAGE_SHIFT;
	struct mm_struct *mm = vma->vm_mm;
	struct cow_context *context = mm->cow_context;
	const unsigned long nr_pages = vma_pages(vma);
	const pgoff_t pgoff = vma->vm_pgoff;
	const long old_offset = pgoff_moved - pgoff;

	if (!should_track_remap(vma))
		return;

	find_and_unmap_existing(context, pgoff, nr_pages, old_offset);
}

void cow_context_vma_adjust(struct vm_area_struct *vma, unsigned long start,
			    unsigned long end)
{
	struct mm_struct *mm = vma->vm_mm;
	struct cow_context *context = mm->cow_context;
	const pgoff_t pgoff = vma->vm_pgoff;
	const long offset = (vma->vm_start >> PAGE_SHIFT) - pgoff;
	const long start_delta = (long)(start - vma->vm_start) >> PAGE_SHIFT;
	const unsigned long nr_pages = vma_pages(vma);
	const unsigned long new_nr_pages = (end - start) >> PAGE_SHIFT;

	if (!should_track_remap(vma))
		return;

	/* First unmap existing. */
	find_and_unmap_existing(context, pgoff, nr_pages, offset);
	/* Then add in the newly adjusted entry. */
	add_new_remap(context, pgoff + start_delta, new_nr_pages,
		      offset, GFP_KERNEL);
}

/* vma is _after_ the remap. */
static void __cow_context_do_remap(struct vm_area_struct *vma, long pgoff_orig,
				   bool is_remap)
{
	const pgoff_t pgoff = vma->vm_pgoff;
	const pgoff_t pgoff_moved = vma->vm_start >> PAGE_SHIFT;
	const unsigned long nr_pages = vma_pages(vma);
	const long old_offset = pgoff_orig - pgoff;
	const long new_offset = pgoff_moved - pgoff;
	struct mm_struct *mm = vma->vm_mm;
	struct cow_context *context = mm->cow_context;

	/* We need the exclusive write lock to avoid races on the dynarray. */
	mmap_assert_write_locked(mm);

	if (!should_track_remap(vma))
		return;
	if (!old_offset && !new_offset)
		return;
	if (!new_offset) {
		find_and_unmap_existing(context, pgoff, nr_pages, old_offset);
		return;
	}

	/* If we're not forking, we might only need to update remaps. */
	if (is_remap && find_and_remap_existing(context, pgoff, nr_pages,
						old_offset, new_offset))
		return;
	add_new_remap(context, pgoff, nr_pages, new_offset, GFP_KERNEL);
}

void cow_context_do_remap(struct vm_area_struct *vma, unsigned long orig_addr)
{
	__cow_context_do_remap(vma, orig_addr >> PAGE_SHIFT, /*is_remap=*/true);
}

void cow_context_do_fork(struct vm_area_struct *vma, struct vm_area_struct *pvma)
{
	__cow_context_do_remap(vma, vma->vm_start >> PAGE_SHIFT, /*is_remap=*/false);
	/* Parent MAP_PRIVATE-file backed mappings need to be duplicated too. */
	cow_context_do_map_private_cow(pvma);
}

void cow_context_do_map_private_cow(struct vm_area_struct *vma)
{
	const pgoff_t pgoff_moved = vma->vm_start >> PAGE_SHIFT;
	const unsigned long nr_pages = vma_pages(vma);
	const pgoff_t pgoff = vma->vm_pgoff;
	const long offset = pgoff_moved - pgoff;
	struct mm_struct *mm = vma->vm_mm;
	struct cow_context *context = mm->cow_context;

	mmap_assert_locked(mm);
	if (!should_track_map_private_remap(vma))
		return;

	add_new_remap(context, pgoff, nr_pages, offset, GFP_KERNEL);
}

static bool walk_context_remap(struct cow_context *context, struct folio *folio,
			       struct walk_context_control *wcc, long offset)
{
	const unsigned long nr_pages = folio_nr_pages(folio);
	const pgoff_t pgoff_folio = folio_pgoff(folio);
	const pgoff_t pgoff_start = pgoff_folio + offset;
	const pgoff_t pgoff_end = pgoff_start + nr_pages;
	const unsigned long range_start = pgoff_start << PAGE_SHIFT;
	const unsigned long range_end = pgoff_end << PAGE_SHIFT;
	VMA_ITERATOR(vmi, context->mm, range_start);
	struct vm_area_struct *vma;

	lockdep_assert_in_rcu_read_lock();

	for_each_vma_range(vmi, vma, range_end) {
		/* TODO: VMA is not stabilised... */
		const unsigned long start = READ_ONCE(vma->vm_start);
		const pgoff_t pgoff_vma = READ_ONCE(vma->vm_pgoff);
		const pgoff_t pgoff_vma_start = start >> PAGE_SHIFT;
		const long offset_vma = (long)pgoff_vma_start - (long)pgoff_vma;

		if (offset_vma != offset)
			continue;
		/*
		 * TODO: We shouldn't be holding the RCU lock here :) need to
		 *       change how rmap walks work.
		 * TODO: VMA not locked so unsafe to access.
		 */
		if (!wcc->walk_one(folio, vma,
				   max(vma->vm_start, range_start), wcc->arg))
			return false;
	}

	return true;
}

static bool walk_context(struct cow_context *context, void *arg1, void *arg2)
{
	struct folio *folio = arg1;
	const pgoff_t pgoff = folio_pgoff(folio);
	const pgoff_t pgoff_last = pgoff + folio_nr_pages(folio) - 1;
	DECLARE_DYNARRAY(unique_multi, WALK_DEFAULT_STACK_SIZE);
	remaps_entry_t unique_remaps = mk_stack_multi_remaps(unique_multi);
	MA_STATE(mas_remaps, &context->remap_mt, pgoff, pgoff_last);
	struct mm_struct *mm = READ_ONCE(context->mm);
	struct walk_context_control *wcc = arg2;
	remaps_entry_t remaps;
	bool was_exclusive;
	long curr_offset;
	bool ret = true;
	int i;

	lockdep_assert_in_rcu_read_lock();

	if (!mm)
		return true;
	if (!mmget_not_zero(mm))
		return true;

	/*
	 * exclusive folios are moved to the correct Cow context level, so we
	 * only look once.
	 *
	 * TODO: Races?
	 */
	was_exclusive = !folio_maybe_mapped_shared(folio);
	/* Try the non-remapped case. */
	if (!walk_context_remap(context, folio, wcc, 0)) {
		ret = false;
		goto out_mmput;
	}
	/*
	 * Check again in case fork raced, therwise, abort the walk we're done.
	 * TODO: race vs. fork + unmap in parent.
	 */
	if (was_exclusive && !folio_maybe_mapped_shared(folio)) {
		ret = false;
		goto out_mmput;
	}

	/* Get unique remaps... TODO: Sketchy GFP_ATOMIC. */
	remaps_for_each(i, &mas_remaps, remaps, curr_offset, pgoff_last)
		unique_remaps = append_remap_offset(unique_remaps, curr_offset,
						    GFP_ATOMIC);
	/* ...And check each one. */
	remaps_for_each_entry_offset(i, unique_remaps, curr_offset) {
		if (!walk_context_remap(context, folio, wcc, curr_offset)) {
			ret = false;
			break;
		}
	}

	/* In case we had so many we had to allocate. */
	free_remaps(unique_remaps);
out_mmput:
	mmput_async(mm);
	return ret;
}

typedef bool (traverse_fn_t)(struct cow_context *, void *, void *);

static void traverse_contexts(struct cow_context *root, traverse_fn_t *callback,
			      void *arg1, void *arg2)
{
	struct cow_context *parent = NULL;
	struct cow_context *curr = root;
	struct cow_context *next = NULL;

	lockdep_assert_in_rcu_read_lock();

	/* Visit root first. */
	if (!callback(root, arg1, arg2))
		return;

	/*
	 * Depth-first traversal:
	 *
	 *          ..... 7......
	 *         .      v      .
	 *        . ------*------ .
	 *       . /             \ .
	 *      . *<3............ *<6
	 *     . / \ .         . / \ .
	 *    . *   * .       . *   * .
	 *   .  ^   ^  .     .  ^   ^  .
	 *  ....1...2....   ....4...5....
	 *
	 * Loop steps:
	 *
	 * 1. If not just moved to parent, try to descend to left-most child of
	 *    current node.
	 * 2. Visit current node. If it is the root node, abort.
	 * 3. Try to traverse to next sibling. If cannot, traverse parent.
	 */
	for (; ; curr = next) {
		if (curr != parent) {
			next = list_first_or_null_rcu(&curr->children,
					struct cow_context, siblings);
			if (next)
				continue;
		}

		if (curr == root || !callback(curr, arg1, arg2))
			break;

		parent = curr->parent;
		next = list_next_or_null_rcu(&parent->children,
					     &curr->siblings, struct cow_context,
					     siblings) ?: parent;
	}
}

static bool rwc_walk_one(struct folio *folio, struct vm_area_struct *vma,
			 unsigned long addr, void *arg)
{
	struct rmap_walk_control *rwc = arg;

	if (!vma)
		return true;
	if (rwc->invalid_vma && rwc->invalid_vma(vma, rwc->arg))
		return true;
	if (!rwc->rmap_one(folio, vma, addr, rwc->arg))
		return false;
	if (rwc->done && rwc->done(folio))
		return false;

	return true;
}

void cow_context_walk(struct folio *folio, struct rmap_walk_control *rwc)
{
	struct walk_context_control wcc = {
		.walk_one = rwc_walk_one,
		.arg = rwc,
	};

	rcu_read_lock();
	traverse_contexts(folio->cow_context, walk_context, folio, &wcc);
	rcu_read_unlock();
}

bool cow_context_verify_vma(struct folio *folio, struct vm_area_struct *vma)
{
	struct mm_struct *mm = vma->vm_mm;
	struct cow_context *context = mm->cow_context;
	const pgoff_t pgoff = folio_pgoff(folio);
	const pgoff_t pgoff_last = pgoff + folio_nr_pages(folio) - 1;
	const pgoff_t pgoff_moved = vma->vm_start >> PAGE_SHIFT;
	const long offset = (long)pgoff_moved - (long)vma->vm_pgoff;
	MA_STATE(mas, &context->remap_mt, pgoff, pgoff_last);
	remaps_entry_t remaps;
	bool found = false;
	long curr_offset;
	int i;

	lockdep_assert_in_rcu_read_lock();

	if (!folio->cow_context)
		return false;

	/* Can the VMA's CoW context be reached by the folio's? */
	for (; context; context = context->parent) {
		if (context != folio->cow_context)
			continue;

		found = true;
		break;
	}
	if (!found)
		return false;

	/* Non-remapped case. */
	if (!offset)
		return true;

	/* If remapped, then make sure we have the correct remap. */
	remaps_for_each(i, &mas, remaps, curr_offset, pgoff_last)
		if (curr_offset == offset)
			return true;

	return false;
}
