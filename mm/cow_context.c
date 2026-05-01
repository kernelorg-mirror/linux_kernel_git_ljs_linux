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
