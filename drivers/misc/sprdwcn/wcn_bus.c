// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Unisoc Communications Inc.
 *
 * Filename : wcn_bus.c
 * Abstract : This file is a implementation for wcn sdio hal function
 */

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <misc/wcn_bus.h>

#include "bus_common.h"

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "WCN BUS: " fmt

struct buffer_pool_t {
	unsigned int size;
	unsigned int free;
	unsigned int payload;
	void *head;
	char *mem;
	spinlock_t lock;
};

struct chn_info_t {
	struct mchn_ops_t *ops[CHN_MAX_NUM];
	struct mutex callback_lock[CHN_MAX_NUM];
	struct buffer_pool_t pool[CHN_MAX_NUM];
};

static struct sprdwcn_bus_ops *wcn_bus_ops;
/*
 * Permanent pointer to the ops registered by the very first
 * module_ops_register() call. Never cleared, so that prebuilt vendor
 * modules using the inlined sprdwcn_bus_* helpers can never observe a
 * NULL bus ops (which would make those helpers "succeed" silently and
 * crash the caller). See module_ops_register() for details.
 */
static struct sprdwcn_bus_ops *wcn_bus_ops_permanent;

static struct chn_info_t g_chn_info;
static struct chn_info_t *chn_info(void)
{
	return &g_chn_info;
}

static int buf_list_check(struct buffer_pool_t *pool, struct mbuf_t *head,
			  struct mbuf_t *tail, int num)
{
	int i;
	struct mbuf_t *mbuf;

	if (num == 0)
		return 0;
	if (head == NULL)
		return 0;

	for (i = 0, mbuf = head; i < num; i++) {
		if ((i == (num - 1)) && (mbuf != tail)) {
			pr_err("%s(0x%lx, 0x%lx, %d), err 1\n", __func__,
			       (unsigned long)virt_to_phys(head),
			       (unsigned long)virt_to_phys(tail), num);
			WARN_ON_ONCE(1);
		}
		WARN_ON_ONCE(!mbuf);
		WARN_ON_ONCE((char *)mbuf < pool->mem ||
			(char *)mbuf > pool->mem + ((sizeof(struct mbuf_t)
			+ pool->payload) * pool->size));
		if (mbuf != NULL)
			mbuf = mbuf->next;
		else
			return 0;
	}

	if (tail->next != NULL) {
		pr_err("%s(0x%lx, 0x%lx, %d), err 2\n", __func__,
		       (unsigned long)virt_to_phys(head),
		       (unsigned long)virt_to_phys(tail), num);
		WARN_ON_ONCE(1);
	}

	return 0;
}

static int buf_pool_check(struct buffer_pool_t *pool)
{
	int i;
	struct mbuf_t *mbuf;

	if (pool->head == NULL)
		return 0;

	for (i = 0, mbuf = pool->head; i < (int)pool->free; i++) {
		WARN_ON_ONCE(!mbuf);
		WARN_ON_ONCE((char *)mbuf < pool->mem ||
			(char *)mbuf > pool->mem + ((sizeof(struct mbuf_t)
			+ pool->payload) * pool->size));
		if (mbuf != NULL)
			mbuf = mbuf->next;
		else
			return 0;
	}

	if (mbuf != NULL) {
		pr_err("%s(0x%p) err\n", __func__, pool);
		WARN_ON_ONCE(1);
	}

	return 0;
}

/* mbuf init and list, current payload is zero */
static int buf_pool_init(struct buffer_pool_t *pool, int size, int payload)
{
	int i;
	struct mbuf_t *mbuf, *next;

	pool->size = size;
	pool->payload = payload;
	spin_lock_init(&(pool->lock));
	pool->mem = kzalloc((sizeof(struct mbuf_t) + payload) * size,
			    GFP_KERNEL);
	if (!pool->mem)
		return -ENOMEM;

	pr_debug("mbuf_pool->mem:0x%lx\n",
		 (unsigned long)virt_to_phys(pool->mem));
	pool->head = (struct mbuf_t *) (pool->mem);
	for (i = 0, mbuf = (struct mbuf_t *)(pool->head);
	     i < (size - 1); i++) {
		mbuf->seq = i;
		pr_debug("%s mbuf[%d]:{0x%lx, 0x%lx}\n", __func__, i,
			 (unsigned long)mbuf,
			 (unsigned long)virt_to_phys(mbuf));
		next = (struct mbuf_t *)((char *)mbuf +
			sizeof(struct mbuf_t) + payload);
		mbuf->buf = (char *)mbuf + sizeof(struct mbuf_t);
		mbuf->len = payload;
		mbuf->next = next;
		mbuf = next;
	}
	pr_debug("%s mbuf[%d]:{0x%lx, 0x%lx}\n", __func__, i,
		 (unsigned long)mbuf,
		 (unsigned long)virt_to_phys(mbuf));
	mbuf->seq = i;
	mbuf->buf = (char *)mbuf + sizeof(struct mbuf_t);
	mbuf->len = payload;
	mbuf->next = NULL;
	pool->free = size;

	return 0;
}

static int buf_pool_deinit(struct buffer_pool_t *pool)
{
	memset(pool->mem, 0x00,
	       (sizeof(struct mbuf_t) + pool->payload) * pool->size);
	kfree(pool->mem);
	pool->mem = NULL;

	return 0;
}

/* take mbuf from pool list */
int buf_list_alloc(int chn, struct mbuf_t **head,
		   struct mbuf_t **tail, int *num)
{
	int i;
	struct buffer_pool_t *pool;
	struct mbuf_t *temp_tail;
	struct chn_info_t *chn_inf = chn_info();

	pool = &(chn_inf->pool[chn]);

	if ((*num <= 0) || (pool->free <= 0)) {
		pr_err("[+]%s err, num %d, free %d)\n",
		       __func__, *num, pool->free);
		*num = 0;
		*head = *tail = NULL;
		return -1;
	}

	spin_lock_bh(&(pool->lock));
	buf_pool_check(pool);
	if (*num > (int)pool->free)
		*num = pool->free;

	for (i = 1, temp_tail = pool->head; i < *num; i++)
		temp_tail = temp_tail->next;

	*head = pool->head;
	*tail = temp_tail;
	pool->head = temp_tail->next;
	temp_tail->next = NULL;
	pool->free -= *num;
	buf_list_check(pool, *head, *tail, *num);
	spin_unlock_bh(&(pool->lock));

	return 0;
}

int buf_list_is_empty(int chn)
{
	struct buffer_pool_t *pool;
	struct chn_info_t *chn_inf = chn_info();

	pool = &(chn_inf->pool[chn]);
	return pool->free <= 0;
}

int buf_list_is_full(int chn)
{
	struct buffer_pool_t *pool;
	struct chn_info_t *chn_inf = chn_info();

	pool = &(chn_inf->pool[chn]);
	return pool->free == pool->size;
}

int buf_list_free(int chn, struct mbuf_t *head, struct mbuf_t *tail, int num)
{
	struct buffer_pool_t *pool;
	struct chn_info_t *chn_inf = chn_info();

	if ((head == NULL) || (tail == NULL) || (num == 0)) {
		pr_err("%s(%d, 0x%lx, 0x%lx, %d)\n", __func__, chn,
		       (unsigned long)virt_to_phys(head),
		       (unsigned long)virt_to_phys(tail), num);
		return -1;
	}

	pool = &(chn_inf->pool[chn]);
	spin_lock_bh(&(pool->lock));
	buf_list_check(pool, head, tail, num);
	tail->next = pool->head;
	pool->head = head;
	pool->free += num;
	buf_pool_check(pool);
	spin_unlock_bh(&(pool->lock));

	return 0;
}

int bus_chn_init(struct mchn_ops_t *ops, int hif_type)
{
	int ret = 0;
	struct chn_info_t *chn_inf = chn_info();

	pr_info("[+]%s(%d, %d)\n", __func__, ops->channel, ops->hif_type);
	if (chn_inf->ops[ops->channel] != NULL) {
		pr_err("%s err, hif_type %d\n", __func__, ops->hif_type);
		WARN_ON_ONCE(1);
		return -1;
	}

	mutex_init(&chn_inf->callback_lock[ops->channel]);
	mutex_lock(&chn_inf->callback_lock[ops->channel]);
	ops->hif_type = hif_type;
	chn_inf->ops[ops->channel] = ops;
	if (ops->pool_size > 0)
		ret = buf_pool_init(&(chn_inf->pool[ops->channel]),
				    ops->pool_size, 0);
	mutex_unlock(&chn_inf->callback_lock[ops->channel]);

	pr_info("[-]%s(%d)\n", __func__, ops->channel);

	return ret;
}

int bus_chn_deinit(struct mchn_ops_t *ops)
{
	int ret = 0;
	struct chn_info_t *chn_inf = chn_info();

	pr_info("[+]%s(%d, %d)\n", __func__, ops->channel, ops->hif_type);
	if (chn_inf->ops[ops->channel] == NULL) {
		pr_err("%s err\n", __func__);
		return -1;
	}

	mutex_lock(&chn_inf->callback_lock[ops->channel]);
	if (ops->pool_size > 0)
		ret = buf_pool_deinit(&(chn_inf->pool[ops->channel]));
	chn_inf->ops[ops->channel] = NULL;
	mutex_unlock(&chn_inf->callback_lock[ops->channel]);
	mutex_destroy(&chn_inf->callback_lock[ops->channel]);

	pr_info("[-]%s(%d)\n", __func__, ops->channel);

	return ret;
}

struct mchn_ops_t *chn_ops(int channel)
{
	if (channel >= CHN_MAX_NUM || channel < 0)
		return NULL;

	return g_chn_info.ops[channel];
}

int module_ops_register(struct sprdwcn_bus_ops *ops)
{
	if (wcn_bus_ops) {
		/*
		 * Idempotent re-registration of the SAME table.
		 *
		 * sdiohal_if.c registers sdiohal_bus_ops from an early
		 * subsys_initcall so that platform drivers probing before
		 * marlin (notably the prebuilt sprdbt_tty.ko, whose
		 * mtty_probe silently skips sprdwcn_bus_chn_init() when
		 * get_wcn_bus_ops() is NULL) can already register their
		 * channels. marlin_probe later calls this again through
		 * wcn_bus_init(); that second call must neither warn nor
		 * fail, or it would leave no bus ops at all.
		 */
		if (wcn_bus_ops == ops)
			return 0;

		WARN_ON_ONCE(1);
		return -EBUSY;
	}

	wcn_bus_ops = ops;
	/*
	 * Keep a permanent copy of the first (and only) registered ops.
	 * The vendor modules (sprdbt_tty.ko / sprdwl_ng.ko / sprd_fm.ko)
	 * are prebuilt against <misc/wcn_bus.h> and inline the
	 * sprdwcn_bus_* helpers. Those inline stubs return 0 ("pretend
	 * success") without filling the head/tail output params when
	 * get_wcn_bus_ops() returns NULL. Callers (e.g. sdio_data_transmit
	 * in sprdbt_tty) then dereference the untouched NULL head pointer:
	 *     head = NULL; ret = sprdwcn_bus_list_alloc(...);
	 *     if (ret) goto err;   // ret==0 -> treated as success
	 *     head->next = NULL;   // Oops: NULL pointer dereference
	 * Never let get_wcn_bus_ops() go back to NULL, so the vendor code
	 * always reaches the real bus ops and gets a proper error code.
	 */
	wcn_bus_ops_permanent = ops;

	pr_info("%s: bus_ops %pK registered\n", __func__, ops);

	return 0;
}

void module_ops_unregister(void)
{
	/*
	 * Deliberately do NOT clear wcn_bus_ops.
	 *
	 * See module_ops_register(): the prebuilt vendor modules inline the
	 * sprdwcn_bus_* helpers, which silently return 0 when
	 * get_wcn_bus_ops() is NULL and leave the head/tail output params
	 * untouched. That turns an "unregistered bus" into a NULL pointer
	 * the vendor driver (observed as sdio_data_transmit+0x10c Oops).
	 *
	 * The ops tables themselves (sdiohal_bus_ops) are static and stay
	 * valid for the whole kernel lifetime, and their list_alloc
	 * (buf_list_alloc) safely returns -1 when the channel pool is not
	 * initialized, which the vendor code handles correctly.
	 */
	pr_info("%s: keep bus_ops %pK (not clearing)\n", __func__, wcn_bus_ops);
}

struct sprdwcn_bus_ops *get_wcn_bus_ops(void)
{
	if (!wcn_bus_ops)
		return wcn_bus_ops_permanent;

	return wcn_bus_ops;
}
EXPORT_SYMBOL_GPL(get_wcn_bus_ops);
