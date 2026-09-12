// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Unisoc Communications Inc.
 *
 * Filename : sdiohal_if.c
 * Abstract : This file is a implementation for wcn sdio hal function
 */

#include "sdiohal.h"

static int sdio_preinit(void)
{
	return sdiohal_init();
}

static void sdio_preexit(void)
{
	sdiohal_exit();
}

static int sdio_buf_list_alloc(int chn, struct mbuf_t **head,
			       struct mbuf_t **tail, int *num)
{
	return buf_list_alloc(chn, head, tail, num);
}

static int sdio_buf_list_free(int chn, struct mbuf_t *head,
			      struct mbuf_t *tail, int num)
{
	return buf_list_free(chn, head, tail, num);
}

static int sdio_list_push(int chn, struct mbuf_t *head,
			  struct mbuf_t *tail, int num)
{
	return sdiohal_list_push(chn, head, tail, num);
}

static int sdio_list_push_direct(int chn, struct mbuf_t *head,
				 struct mbuf_t *tail, int num)
{
	return sdiohal_list_direct_write(chn, head, tail, num);
}

static int sdio_chn_init(struct mchn_ops_t *ops)
{
	return bus_chn_init(ops, HW_TYPE_SDIO);
}

static int sdio_chn_deinit(struct mchn_ops_t *ops)
{
	return bus_chn_deinit(ops);
}

static int sdio_direct_read(unsigned int addr,
			    void *buf, unsigned int len)
{
	return sdiohal_dt_read(addr, buf, len);
}

static int sdio_direct_write(unsigned int addr,
			     void *buf, unsigned int len)
{
	return sdiohal_dt_write(addr, buf, len);
}

static int sdio_readbyte(unsigned int addr, unsigned char *val)
{
	return sdiohal_aon_readb(addr, val);
}

static int sdio_writebyte(unsigned int addr, unsigned char val)
{
	return sdiohal_aon_writeb(addr, val);
}

static unsigned int sdio_get_carddump_status(void)
{
	return sdiohal_get_carddump_status();
}

static void sdio_set_carddump_status(unsigned int flag)
{
	return sdiohal_set_carddump_status(flag);
}

static unsigned long long sdio_get_rx_total_cnt(void)
{
	return sdiohal_get_rx_total_cnt();
}

static int sdio_runtime_get(void)
{
	return sdiohal_runtime_get();
}

static int sdio_runtime_put(void)
{
	return sdiohal_runtime_put();
}

static int sdio_rescan(void *wcn_dev)
{
	return sdiohal_scan_card(wcn_dev);
}

static void sdio_register_rescan_cb(void *func)
{
	return sdiohal_register_scan_notify(func);
}

static void sdio_remove_card(void *wcn_dev)
{
	return sdiohal_remove_card(wcn_dev);
}

static enum wcn_hard_intf_type sdio_get_hwintf_type(void)
{
	return HW_TYPE_SDIO;
}

static struct sprdwcn_bus_ops sdiohal_bus_ops = {
	.preinit = sdio_preinit,
	.deinit = sdio_preexit,
	.chn_init = sdio_chn_init,
	.chn_deinit = sdio_chn_deinit,
	.list_alloc = sdio_buf_list_alloc,
	.list_free = sdio_buf_list_free,
	.push_list = sdio_list_push,
	.push_list_direct = sdio_list_push_direct,
	.direct_read = sdio_direct_read,
	.direct_write = sdio_direct_write,
	.readbyte = sdio_readbyte,
	.writebyte = sdio_writebyte,
	.read_l = sdiohal_readl,
	.write_l = sdiohal_writel,

	.get_carddump_status = sdio_get_carddump_status,
	.set_carddump_status = sdio_set_carddump_status,
	.get_rx_total_cnt = sdio_get_rx_total_cnt,

	.runtime_get = sdio_runtime_get,
	.runtime_put = sdio_runtime_put,
	.get_hwintf_type = sdio_get_hwintf_type,

	.register_rescan_cb = sdio_register_rescan_cb,
	.rescan = sdio_rescan,
	.remove_card = sdio_remove_card,
};

void module_bus_init(void)
{
	module_ops_register(&sdiohal_bus_ops);
}
EXPORT_SYMBOL_GPL(module_bus_init);

void module_bus_deinit(void)
{
	module_ops_unregister();
}
EXPORT_SYMBOL_GPL(module_bus_deinit);

/*
 * Register the SDIO HAL bus ops as early as possible.
 *
 * The prebuilt vendor modules register their data channels from their own
 * platform probe. sprdbt_tty.ko's mtty_probe, for instance, does:
 *
 *     mov  x0, #0x30 / #0x80   ; &bt_sdio_rx_ops / &bt_sdio_tx_ops
 *     bl   get_wcn_bus_ops
 *     cbz  x0, <skip>          ; bus_ops == NULL -> silently skip!
 *     ldr  x1, [x0,#16]        ; chn_init
 *     blr  x1                  ; register channel 17 / 3
 *
 * mtty_probe can run BEFORE the marlin platform device probes (which is what
 * calls wcn_bus_init() -> module_bus_init() -> module_ops_register()). When
 * that happens the vendor code observes a NULL bus ops, skips channel
 * registration, and BT channels 3 (TX) and 17 (RX) end up with uninitialized
 * buffer pools. sprdwcn_bus_list_alloc() then fails with
 * "buf_list_alloc err, num 1, free 0", HCI commands are never transmitted,
 * and the BT stack aborts with hci_timeout_abort() in a tight loop that also
 * drags down the whole system.
 *
 * sdiohal_bus_ops is a static, self-describing table; the only thing the
 * early registration does is publish it, so every later probe gets a valid
 * pointer. The module_bus_init() call from marlin_probe is then an
 * idempotent no-op (see module_ops_register()).
 */
static int __init sdiohal_bus_ops_early_init(void)
{
	module_bus_init();

	return 0;
}
subsys_initcall(sdiohal_bus_ops_early_init);
