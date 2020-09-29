/* Copyright (C) 2020 David Brunecz. Subject to GPL 2.0 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "dbg.h"
#include "interfaces.h"

#define PAD			0
#define NO_PAD		1

struct priv {
	u32 tap_count;
	struct jtag_tap_info *taps;
	struct jtag_tap_info *cur_tap;
	u32 state;

	struct jtag_ctrl_intf *ctrl;
	struct jtag_intf ji;
};

struct state_bit_stream {
	u32 tms;
	u32 cnt;
};

#define TEST_LOGIC_RESET	0
#define SELECT_DR_SCAN		1
#define SELECT_IR_SCAN		2
#define RUN_TEST_IDLE		3
#define CAPTURE_DR		4
#define CAPTURE_IR		5
#define SHIFT_DR		6
#define SHIFT_IR		7
#define EXIT1_DR		8
#define EXIT1_IR		9
#define PAUSE_DR		10
#define PAUSE_IR		11
#define EXIT2_DR		12
#define EXIT2_IR		13
#define UPDATE_DR		14
#define UPDATE_IR		15

static char *tap_state_desc[16] = {
	[TEST_LOGIC_RESET] = "TEST_LOGIC_RESET",
	[SELECT_DR_SCAN] = "SELECT_DR_SCAN",
	[SELECT_IR_SCAN] = "SELECT_IR_SCAN",
	[RUN_TEST_IDLE] = "RUN_TEST_IDLE",
	[CAPTURE_DR] = "CAPTURE_DR",
	[CAPTURE_IR] = "CAPTURE_IR",
	[SHIFT_DR] = "SHIFT_DR",
	[SHIFT_IR] = "SHIFT_IR",
	[EXIT1_DR] = "EXIT1_DR",
	[EXIT1_IR] = "EXIT1_IR",
	[PAUSE_DR] = "PAUSE_DR",
	[PAUSE_IR] = "PAUSE_IR",
	[EXIT2_DR] = "EXIT2_DR",
	[EXIT2_IR] = "EXIT2_IR",
	[UPDATE_DR] = "UPDATE_DR",
	[UPDATE_IR] = "UPDATE_IR",
};

static u32 tms_0_lookup[16] = {
	[TEST_LOGIC_RESET] = RUN_TEST_IDLE,
	[SELECT_DR_SCAN] = CAPTURE_DR,
	[SELECT_IR_SCAN] = CAPTURE_IR,
	[RUN_TEST_IDLE] = RUN_TEST_IDLE,
	[CAPTURE_DR] = SHIFT_DR,
	[CAPTURE_IR] = SHIFT_IR,
	[SHIFT_DR] = SHIFT_DR,
	[SHIFT_IR] = SHIFT_IR,
	[EXIT1_DR] = PAUSE_DR,
	[EXIT1_IR] = PAUSE_IR,
	[PAUSE_DR] = PAUSE_DR,
	[PAUSE_IR] = PAUSE_IR,
	[EXIT2_DR] = SHIFT_DR,
	[EXIT2_IR] = SHIFT_IR,
	[UPDATE_DR] = RUN_TEST_IDLE,
	[UPDATE_IR] = RUN_TEST_IDLE,
};

static u32 tms_1_lookup[16] = {
	[TEST_LOGIC_RESET] = TEST_LOGIC_RESET,
	[SELECT_DR_SCAN] = SELECT_IR_SCAN,
	[SELECT_IR_SCAN] = TEST_LOGIC_RESET,
	[RUN_TEST_IDLE] = SELECT_DR_SCAN,
	[CAPTURE_DR] = EXIT1_DR,
	[CAPTURE_IR] = EXIT1_IR,
	[SHIFT_DR] = EXIT1_DR,
	[SHIFT_IR] = EXIT1_IR,
	[EXIT1_DR] = UPDATE_DR,
	[EXIT1_IR] = UPDATE_IR,
	[PAUSE_DR] = EXIT2_DR,
	[PAUSE_IR] = EXIT2_IR,
	[EXIT2_DR] = UPDATE_DR,
	[EXIT2_IR] = UPDATE_IR,
	[UPDATE_DR] = SELECT_DR_SCAN,
	[UPDATE_IR] = SELECT_DR_SCAN,
};

static u32* tms_lookup[] = {
	tms_0_lookup, tms_1_lookup,
};

u32 jtag_tap_state_next(u32 current, u32 tms)
{
	return tms_lookup[tms & 1][current & 0xf];
}

void dump_data_shift(struct jtag_intf *ji, u8 *tdi, u8 *tdo, u32 cnt, u32 is_pad)
{
	struct priv *p = container_of(ji, struct priv, ji);
	u32 i, bytes, bits_remain, mask;

	bytes = cnt / 8;
	bits_remain = cnt % 8;
	mask = (1 << bits_remain) - 1;

	is_pad ? GREY_TXT() : WHITE_TXT();
	BOLD_TXT();
	printf("TDI %d bits: ", cnt);
	RESET_BOLD_TXT();

	for (i = 0; i < bytes; i++)
		printf("%02x ", tdi[i]);
	if (bits_remain)
		printf("%2x ", tdi[i] & mask);

	if (ji->debug_ir && p->state == SHIFT_IR)
		ji->debug_ir(tdi, bytes);

	RESET_TXT_CLR();
	printf(" => ");

	is_pad ? GREY_TXT() : WHITE_TXT();
	BOLD_TXT();
	printf("TDO %d bits: ", cnt);
	RESET_BOLD_TXT();
	for (i = 0; i < bytes; i++)
		printf("%02x ", tdo[i]);
	if (bits_remain) {
		printf("%2x ", tdo[i] & mask);
	}
	RESET_TXT_CLR();
	printf("\n");
}

char *jtag_tap_state_desc(u32 state)
{
	if (state >= DIM(tap_state_desc))
		return "bad state";
	return tap_state_desc[state];
}

static void fsm_change(struct jtag_intf *ji, u32 tms, u32 bits)
{
	struct priv *p = container_of(ji, struct priv, ji);
	u32 i = 0;

	if (ji->debug)
		CYAN_TXT();

	for (i = 0; i < bits; i++) {
		p->state = jtag_tap_state_next(p->state, (tms >> i) & 1);

		if (ji->debug) {
			if (p->state == TEST_LOGIC_RESET)
				RED_TXT();
			if (p->state == RUN_TEST_IDLE)
				YELLOW_TXT();
			printf(" > %s", jtag_tap_state_desc(p->state));
			CYAN_TXT();
		}
	}
	if (ji->debug) {
		RESET_TXT_CLR();
		printf("\n");
	}
}

static u32 jtag_clock_tap_state(struct jtag_intf *ji, u32 tdi_bit,
				struct state_bit_stream *bs)
{
	struct priv *p = container_of(ji, struct priv, ji);
	struct jtag_ctrl_intf *jci = p->ctrl;

	fsm_change(ji, bs->tms, bs->cnt);
	jci->clock_tms(jci, &bs->tms, NULL, tdi_bit, bs->cnt);
	return 0;
}

// tap 0 closest to TDI
// tap n closest to TDO
// when clocking bits for whole chain, must start with dev n's bits
static u32 get_nthbit(u8 *buf, u32 n)
{
	u32 byte = n / 8;
	u32 bit = n % 8;

	return (buf[byte] >> bit) & 1;
}

static void set_nthbit(u8 *buf, u32 n, u32 val)
{
	u32 byte = n / 8;
	u32 bit = n % 8;

	buf[byte] &= ~(1 << bit);
	buf[byte] |= ((val & 1) << bit);
}

void setbits(u8 *dst, u8 *src, u32 idx, u32 wd)
{
	u32 i, val;

	for (i = 0; i < wd; i++) {
		val = (src == NULL) ? 1 : get_nthbit(src, i);
		set_nthbit(dst, idx + i, val);
	}
}

void copybits(u8 *dst, u32 dst_idx, u8 *src, u32 src_idx, u32 wd)
{
	u32 i, val;

	for (i = 0; i < wd; i++) {
		val = get_nthbit(src, src_idx + i);
		set_nthbit(dst, dst_idx + i, val);
	}
}

static int chain_ir_len(struct jtag_intf *ji)
{
	struct priv *p = container_of(ji, struct priv, ji);
	int i, len = 0;

	if (p->tap_count == 0)
		return -1;
	for (i = 0; i < p->tap_count; i++)
		len += p->taps[i].ir_sz;
	return len;
}

static int _jtag_clock_data_bits(struct jtag_intf *ji, u8 *tdi, u8 *tdo, u32 cnt,
				 u32 pad, u32 tms)
{
	struct priv *p = container_of(ji, struct priv, ji);
	struct jtag_ctrl_intf *jci = p->ctrl;
	u32 idx, out_idx = 0, alloc_sz, wd, tapcnt;
	u8 *_tdi = NULL, *_tdo = NULL;
	struct jtag_tap_info *ti;
	u32 len = 0;
	int rv;
	int i;

	tapcnt = p->tap_count;
	if (pad != NO_PAD) {
		if (p->state == SHIFT_IR) {
			len = chain_ir_len(ji);
			alloc_sz = 1 + ((len - 1) / 8);
		} else {
			len = cnt + tapcnt - 1;
			alloc_sz = 1 + ((len - 1) / 8);
			wd = 1;
		}
		_tdi = malloc(alloc_sz);
		_tdo = malloc(alloc_sz);
		idx = 0;
		for (i = tapcnt-1; i > p->cur_tap->chain_index; i--) {
			if (p->state == SHIFT_IR) {
				ti = ji->tap_info(ji, i);
				wd = ti->ir_sz;
			}
			setbits(_tdi, NULL, idx, wd);
			idx += wd;
		}

		out_idx = idx;
		setbits(_tdi, tdi, idx, cnt);
		idx += cnt;

		i = p->cur_tap->chain_index - 1;
		for (; i >= 0; i--) {
			if (p->state == SHIFT_IR) {
				ti = ji->tap_info(ji, i);
				wd = ti->ir_sz;
			}
			setbits(_tdi, NULL, idx, wd);
			idx += wd;
		}
	}

	rv = jci->clock_din_dout(jci, tms,
				(_tdi == NULL) ? tdi : _tdi,
				(_tdo == NULL) ? tdo : _tdo,
				 (len == 0) ? cnt : len);
	if (ji->debug)
		dump_data_shift(ji,
				(_tdi == NULL) ? tdi : _tdi,
				(_tdo == NULL) ? tdo : _tdo,
				(len == 0) ? cnt : len, 0);
	fsm_change(ji, tms, 1);

	if (pad != NO_PAD) {
		copybits(tdo, 0, _tdo, out_idx, cnt);
	}
	if (_tdi) free(_tdi);
	if (_tdo) free(_tdo);
	return rv;
}

static int jtag_clock_data_bits_raw(struct jtag_intf *ji, u8 *tdi, u8 *tdo, u32 cnt)
{
	return _jtag_clock_data_bits(ji, tdi, tdo, cnt, NO_PAD, 0);
}

static int jtag_clock_data_bits_exit_shift(struct jtag_intf *ji, u8 *tdi, u8 *tdo,
					   u32 cnt)
{
	return _jtag_clock_data_bits(ji, tdi, tdo, cnt, PAD, 1);
}

static u32 jtag_run_idle_to_shift_dr(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream bs = { 0x15, 6 };

	return jtag_clock_tap_state(ji, tdi_bit, &bs);
}

static u32 jtag_select_to_run_idle(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream bs = { 0x6, 4 };

	return jtag_clock_tap_state(ji, tdi_bit, &bs);
}

static u32 jtag_select_to_capture_to_shift(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream bs = { 0, 2 };

	return jtag_clock_tap_state(ji, tdi_bit, &bs);
}

static u32 jtag_run_idle_to_shift_ir(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream bs = { 0x2b, 7 };

	return jtag_clock_tap_state(ji, tdi_bit, &bs);
}

u32 jtag_exit1_to_update(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream bs = { 0x06, 3 };

	return jtag_clock_tap_state(ji, tdi_bit, &bs);
}

static u32 jtag_exit1_to_update_to_run_idle(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream bs = { 0x06, 6/*4*/ };
	return jtag_clock_tap_state(ji, tdi_bit, &bs);
}

static u32 jtag_update_to_shift_dr(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream bs = { 0x15, 6 };

	return jtag_clock_tap_state(ji, tdi_bit, &bs);
}

static u32 jtag_update_to_select_dr(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream bs = { 0x1, 1 };

	return jtag_clock_tap_state(ji, tdi_bit, &bs);
}

static u32 jtag_exit1_to_idle_to_select_dr(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream bs = { 0x5, 3 };

	return jtag_clock_tap_state(ji, tdi_bit, &bs);
}

static u32 jtag_exit1_to_update_to_select_dr(struct jtag_intf *ji, u32 tdi_bit)
{
	u32 tdo_bit = jtag_exit1_to_update(ji, tdi_bit);

	return jtag_update_to_select_dr(ji, tdo_bit);
}

static u32 jtag_exit1_to_update_to_shift_data(struct jtag_intf *ji, u32 tdi_bit)
{
	u32 tdo_bit = jtag_exit1_to_update(ji, tdi_bit);

	return jtag_update_to_shift_dr(ji, tdo_bit);
}

static u32 jtag_shift_to_update_to_run_idle(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream bs = { 0x0d, 5 };

	return jtag_clock_tap_state(ji, tdi_bit, &bs);
}

static int jtag_set_ir_from_idle(struct jtag_intf *ji, u8 *ir, u32 ir_sz)
{
	u8 tdo[8];

	memset(tdo, 0xff, sizeof(tdo));
	jtag_run_idle_to_shift_ir(ji, 1);
	jtag_clock_data_bits_exit_shift(ji, ir, tdo, ir_sz);
	jtag_exit1_to_update_to_run_idle(ji, 1);
	return 0;
}

static int jtag_set_ir_from_idle_pass_to_select_dr(struct jtag_intf *ji, u8 *ir,
						   u32 ir_sz)
{
	u8 tdo[8];

	memset(tdo, 0xff, sizeof(tdo));
	jtag_run_idle_to_shift_ir(ji, 1);
	jtag_clock_data_bits_exit_shift(ji, ir, tdo, ir_sz);
	jtag_exit1_to_update_to_select_dr(ji, 1);
	return 0;
}

static int jtag_set_ir_from_idle_pass_to_shift_dr(struct jtag_intf *ji, u8 *ir,
						  u32 ir_sz)
{
	u8 tdo[8];

	memset(tdo, 0xff, sizeof(tdo));
	jtag_run_idle_to_shift_ir(ji, 1);
	jtag_clock_data_bits_exit_shift(ji, ir, tdo, ir_sz);
	jtag_exit1_to_update_to_shift_data(ji, 1);
	return 0;
}

static u32 jtag_reset_tap_state(struct jtag_intf *ji, u32 tdi_bit, u32 force)
{
	struct priv *p = container_of(ji, struct priv, ji);
	struct state_bit_stream rst_bs = { 0x1f, 5 };
	u32 rv = 0;

	if (force || p->state != TEST_LOGIC_RESET)
		rv = jtag_clock_tap_state(ji, tdi_bit, &rst_bs);
	return rv;
}

static u32 jtag_set_tap_state_shift_ir(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream sir_bs = { 0x56, 8 };

	jtag_reset_tap_state(ji, tdi_bit, 0);
	return jtag_clock_tap_state(ji, tdi_bit, &sir_bs);
}

static u32 jtag_set_tap_state_shift_dr(struct jtag_intf *ji, u32 tdi_bit)
{
	struct state_bit_stream sir_bs = { 0x2a, 7 };

	jtag_reset_tap_state(ji, tdi_bit, 0);
	return jtag_clock_tap_state(ji, tdi_bit, &sir_bs);
}

static int verify_bypass_length(struct jtag_intf *ji)
{
	/* TODO: fix max 32 TAP limitation */
	struct priv *p = container_of(ji, struct priv, ji);
	u32 bypass_len = p->tap_count;
	u32 magic = 0xf5a5a5a0;
	u32 mask;
	u32 tdo;

	jtag_run_idle_to_shift_dr(ji, 1);
	jtag_clock_data_bits_raw(ji, (u8 *)&magic, (u8 *)&tdo, 32);
	jtag_shift_to_update_to_run_idle(ji, 1);

	mask = (1 << (32 - bypass_len)) - 1;
	tdo >>= bypass_len;
	if ((mask & tdo) != (mask & magic)) {
		DBG("TAP_CNT:%d MGC:0x%08x  OUT:0x%08x", bypass_len, magic, tdo);
		return -1;
	}
	return 0;
}

static int jtag_bypass(struct jtag_intf *ji, u32 ir_len)
{
	struct priv *p = container_of(ji, struct priv, ji);
	u8 *tdi, *tdo;
	u32 alloc_sz;

	alloc_sz = 1 + (ir_len/8);
	tdi = malloc(alloc_sz);
	tdo = malloc(alloc_sz);
	if (!tdi || !tdo) {
		DBG();
		return -1;
	}
	memset(tdi, 0xff, alloc_sz);
	jtag_set_tap_state_shift_ir(ji, 1);
	p->ctrl->clock_din_dout(p->ctrl, 0, tdi, tdo, alloc_sz * 8);
	jtag_shift_to_update_to_run_idle(ji, 1);

	free(tdi);
	free(tdo);
	return 0;
}

static int bypass_all(struct jtag_intf *ji)
{
	jtag_bypass(ji, chain_ir_len(ji));
	return verify_bypass_length(ji);
}

static u32 u32_from_bytestream(u8 *buf, u32 offs)
{
	u32 byte = offs / 8;
	u32 bit = offs % 8;
	u32 v, i;

	for (v = 0, i = 0; i < 4; i++)
		v |= buf[byte + i] << (i * 8);
	if (bit) {
		v >>= bit;
		v |= ((u32)buf[byte + 4] << (8 - bit)) << 24;
	}
	return v;
}

static int jtag_probe(struct jtag_intf *ji,
		      int (*probe_cb)(void *, struct jtag_tap_info *),
		      void *probe_ctx)
{
	struct priv *p = container_of(ji, struct priv, ji);
	u32 i, v, tot_dr_len, alloc_sz;
	struct jtag_tap_info *ti;
	u8 *tdo, *tdi;
	int rv = -1;

	if (p->taps)
		free(p->taps);
	p->taps = NULL;
	p->tap_count = 0;

	alloc_sz = (4 * 32);
	tdo = malloc(alloc_sz);
	tdi = malloc(alloc_sz);
	if (!tdo || !tdi) {
		DBG();
		return rv;
	}

	jtag_bypass(ji, 8 * 32);

	memset(tdi, 0x55, alloc_sz);
	tdi[0] = 0xdb;

	jtag_set_tap_state_shift_dr(ji, 1);
	p->ctrl->clock_din_dout(p->ctrl, 0, tdi, tdo, alloc_sz * 8);

	for (i = 0; i < alloc_sz * 8; i++) {
		v = u32_from_bytestream(tdo, i);
		if (v != 0x555555db)
			continue;

		tot_dr_len = i;
		if (!tot_dr_len)
			goto exit;

		if (tot_dr_len % 32)
			goto exit;

		for (i = 0; i < tot_dr_len; i += 32) {
			v = u32_from_bytestream(tdo, i);
			p->tap_count++;

			p->taps = realloc(p->taps, p->tap_count * sizeof(*p->taps));
			ti = &p->taps[p->tap_count - 1];
			memset(ti, 0, sizeof(*ti));
			ti->idcode = v;
			if (probe_cb)
				if (probe_cb(probe_ctx, ti))
					goto exit;
		}
		rv = 0;
	}
exit:
	free(tdo);
	free(tdi);
	return rv;
}

static struct jtag_tap_info* jtag_get_tap_info(struct jtag_intf *ji,u32 tap_id)
{
	struct priv *p = container_of(ji, struct priv, ji);
	int i;

	for (i = 0; i < p->tap_count; i++)
		if (p->taps[i].chain_index == tap_id)
			return &p->taps[i];
	return NULL;
}

static int jtag_set_current_tap(struct jtag_intf *ji, u32 tap_id)
{
	struct priv *p = container_of(ji, struct priv, ji);
	int i;

	p->cur_tap = NULL;
	for (i = 0; i < p->tap_count; i++) {
		if (p->taps[i].chain_index == tap_id) {
			p->cur_tap = &p->taps[i];
			break;
		}
	}
	if (p->cur_tap == NULL) {
		DBG();
		return -1;
	}
	return bypass_all(ji);
}

static int jtag_txrx(struct jtag_intf *ji, u8 *ir, u32 ir_sz, u8 *tdi, u8 *tdo,
		     u32 bit_cnt)
{
	struct priv *p = container_of(ji, struct priv, ji);

	if (!p->cur_tap) {
		DBG();
		return -1;
	}
	if (p->cur_tap->ir_sz != ir_sz)
		DBG("WARN: IR size mismatch");

	if (bit_cnt) {
		jtag_set_ir_from_idle_pass_to_shift_dr(ji, ir, ir_sz);
		jtag_clock_data_bits_exit_shift(ji, tdi, tdo, bit_cnt);
		jtag_exit1_to_update_to_run_idle(ji, 1);
	} else {
		jtag_set_ir_from_idle(ji, ir, ir_sz);
	}
	return 0;
}

static int jtag_ir(struct jtag_intf *ji, u8 *ir, u32 ir_sz)
{
	jtag_set_ir_from_idle_pass_to_select_dr(ji, ir, ir_sz);
	return 0;
}

static int jtag_tditdo(struct jtag_intf *ji, u8 *tdi, u8 *tdo, u32 bit_cnt)
{
	jtag_select_to_capture_to_shift(ji, 1);
	jtag_clock_data_bits_exit_shift(ji, tdi, tdo, bit_cnt);
	jtag_exit1_to_idle_to_select_dr(ji, 1);
	return 0;
}

static int jtag_rti(struct jtag_intf *ji)
{
	jtag_select_to_run_idle(ji, 1);
	return 0;
}

struct jtag_intf * jtag_intf_init(struct jtag_ctrl_intf *ctrl,
				  int (*probe_cb)(void *, struct jtag_tap_info *),
				  void *probe_ctx)
{
	struct jtag_intf * ji;
	struct priv *p;

	p = malloc(sizeof(*p));
	if (!p) {
		DBG();
		return NULL;
	}
	p->ctrl = ctrl;
	p->tap_count = 0;
	p->taps = NULL;
	p->cur_tap = NULL;
	p->state = TEST_LOGIC_RESET;

	ji = &p->ji;
	ji->probe = jtag_probe;
	ji->txrx = jtag_txrx;
	ji->ir = jtag_ir;
	ji->tditdo = jtag_tditdo;
	ji->rti = jtag_rti;
	ji->set_current_tap = jtag_set_current_tap;
	ji->tap_info = jtag_get_tap_info;

	ji->debug = 0;
	ji->debug_ir = NULL;

	jtag_reset_tap_state(ji, 1, 1);

	if (ji->probe(ji, probe_cb, probe_ctx)) {
		free(p);
		return NULL;
	}
	jtag_set_current_tap(ji, 0);
	bypass_all(ji);
	return ji;
}

void jtag_intf_free(struct jtag_intf *ji)
{
	struct priv *p = container_of(ji, struct priv, ji);

	free(p);
}
