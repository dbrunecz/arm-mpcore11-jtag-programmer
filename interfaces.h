/* Copyright (C) 2020 David Brunecz. Subject to GPL 2.0 */

#ifndef __INTERFACE_H__
#define __INTERFACE_H__
/******************************************************************************/

struct gpio_intf {

	void *priv;

	int (*set_gpio) (struct gpio_intf *, u32 pin, u32 val);
	int (*set_gpio_dir) (struct gpio_intf *, u32 pin, u32 output, u32 val);
	int (*get_gpio) (struct gpio_intf *, u32 * val_bits);
	int (*get_gpio_dir) (struct gpio_intf *, u32 * dir_bits);
	int (*count) (struct gpio_intf *);
};

struct jtag_tap_info {
	char *desc;
	u32 chain_index;
	u32 idcode;
	u32 ir_sz;
	u32 bs_sz;
#ifdef JTAG_DEBUG
	char *(*ir_desc) (u8 *, u32 sz);
#endif
};

#define JTAG_ENTER_SHIFTDR     (1 << 0)
#define JTAG_EXIT_SHIFTDR      (1 << 1)

struct jtag_intf {
	int debug;
	void (*debug_ir) (u8 * ir, int bytes);

	int (*probe) (struct jtag_intf *,
		      int (*probe_cb) (void *, struct jtag_tap_info *),
		      void *probe_ctx);
	int (*set_current_tap) (struct jtag_intf *, u32 tap_id);
	int (*txrx) (struct jtag_intf *, u8 * ir, u32 ir_sz,
		     u8 * tdi, u8 * tdo, u32 bit_cnt);

	int (*ir) (struct jtag_intf *, u8 * ir, u32 ir_sz);
	int (*tditdo) (struct jtag_intf *, u8 * tdi, u8 * tdo, u32 bit_cnt);
	int (*rti) (struct jtag_intf *);

	struct jtag_tap_info *(*tap_info) (struct jtag_intf *, u32 tap);
};

struct jtag_ctrl_intf {

	/*
	 *   tms: bits to clock out
	 *   tdo: bit0 will contain din(dout of chain) after tms clocking
	 *   tdi: value to present on dout(din of chain) while clocking tms
	 *   cnt: bit-count(== clock-cycles) to clock
	 */
	int (*clock_tms) (struct jtag_ctrl_intf * ji, u32 * tms,
			  u8 * tdo, u32 tdi, u32 cnt);

	/*
	 *   tms: value to present on tms at final data bit clock
	 *   tdi: bits to clock out to chain
	 *   tdo: bits clocked in from chain
	 *   cnt: bit-count(== clock-cycles) to clock
	 */
	int (*clock_din_dout) (struct jtag_ctrl_intf * ji, u32 tms,
			       u8 * tdi, u8 * tdo, u32 cnt);
};

/******************************************************************************/
#endif //__INTERFACE_H__
