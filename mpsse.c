/* Copyright (C) 2020 David Brunecz. Subject to GPL 2.0 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>

#include <ftdi.h>

#include "dbg.h"
#include "interfaces.h"
#include "mpsse.h"

#define SET_DBITS_LOW				0x80
#define SET_DBITS_HIGH				0x82
#define READ_DBITS_LOW				0x81
#define READ_DBITS_HIGH				0x83

/* TDI/TDO internal loopback */
#define ENABLE_LOOPBACK				0x84
#define DISABLE_LOOPBACK			0x85
/*
 * set clock divisor
 *
 *  VAL = ((HIGH << 8) | LOW)
 *  TCK/SK frequency = 12MHz / ((1 + VAL) * 2)
 *
 *   FT232H, FT2232H, FT4232H can multiply above by 5 with
 *	DISABLE CLOCK DIVIDE BY 5 COMMAND
 *
 */
#define SET_CLOCK_DIV				0x86
/* MPSSE/MCU common instructions */
#define SEND_IMMEDIATE				0x87
/* FT2232H, FT4232H, FT232H instructions */
#define DISABLE_CLKDIV5				0x8A
#define ENABLE_CLKDIV5				0x8B
#define ENABLE_3PHASE_CLOCK			0x8C
#define DISABLE_3PHASE_CLOCK			0x8D
#define ENABLE_ADAPTIVE_CLOCKING		0x96
#define DISABLE_ADAPTIVE_CLOCKING		0x97

#define BAD_CMD					0xFA

/******************************************************************************/
int _ftdi_read(struct ftdi_context *ftdi, u8 *buffer, u32 cnt)
{
	u32 idx = 0, remain = cnt;
	int ret;

	do {
		ret = ftdi_read_data(ftdi, &buffer[idx], remain);
		if (ret < 0) {
			DBG("ftdi_read_data() failed: %d (%s)\n",
				ret, ftdi_get_error_string(ftdi));
			exit(-1);
			return -1;
		} else if (ret > remain) {
			DBG();
			return -1;
		}
		idx += ret;
		remain -= ret;
	} while (remain != 0);
	return 0;
}

int _ftdi_write(struct ftdi_context *ftdi, u8 *buffer, u32 cnt)
{
	u32 idx = 0, remain = cnt;
	int ret;

	do {
		ret = ftdi_write_data(ftdi, &buffer[idx], remain);
		if (ret < 0) {
			DBG("ftdi_write_data() failed: %d (%s)\n",
				ret, ftdi_get_error_string(ftdi));
			exit(-1);
			return -1;
		} else if (ret > remain) {
			DBG();
			return -1;
		}
		idx += ret;
		remain -= ret;
	} while (remain != 0);
	return 0;
}

int _ftdi_mpsse_sync(struct ftdi_context *ftdi)
{
	u8 bad_cmd = 0xaa, resp[2];

	if (_ftdi_write(ftdi, &bad_cmd, 1))
		return -1;
	if (_ftdi_read(ftdi, resp, 2))
		return -1;
	return (resp[0] == BAD_CMD && resp[1] == bad_cmd) ? 0 : -1;
}

static int ftdi_latency_val(void)
{
	const char *s = getenv("FTDI_LATENCY");
	u32 v;

	if (!s)
		return 1;

	v = strtoul(s, NULL, 0);
	return (v < 1 || v > 255) ? 1 : v;
}

static struct ftdi_context* _ftdi_open(u32 vid, u32 pid, u32 dev, u32 port)
{
	u32 ftdiport[4] = {INTERFACE_A, INTERFACE_B, INTERFACE_C, INTERFACE_D };
	struct ftdi_device_list *devlist = NULL, *p;
	struct ftdi_context *ftdi;
	int err, i, intfc;
	uint8_t latency;

	if (port > 3) {
		DBG("invalid port: %d", port);
		return NULL;
	}

	ftdi = ftdi_new();
	if (!ftdi) {
		DBG("ftdi_new failed");
		return NULL;
	}

	intfc = ftdiport[port];
	err = ftdi_set_interface(ftdi, intfc);
	if (err < 0) {
		DBG("ftdi_set_interface() failed: %d (%s)",
			err, ftdi_get_error_string(ftdi));
		goto exit_on_error;
	}

	err = ftdi_usb_find_all(ftdi, &devlist, vid, pid);
	if (err < 0 || devlist == NULL) {
		DBG("ftdi_usb_find_all failed");
		goto exit_on_error;
	}

	for (i = 0, p = devlist; p; p = p->next, i++) {
		if (i != dev)
			continue;

		err = ftdi_usb_open_dev(ftdi, p->dev);
		if (err < 0) {
			DBG("ftdi_usb_open_dev() failed: %d (%s)",
				err, ftdi_get_error_string(ftdi));
			goto exit_on_error;
		}

		if (ftdi_get_latency_timer(ftdi, &latency))
			DBG("ftdi_get_latency_timer failed");

		latency = ftdi_latency_val();
		if (ftdi_set_latency_timer(ftdi, latency))
			DBG("ftdi_set_latency_timer failed");

		break;
	}
	ftdi_list_free(&devlist);
	return ftdi;

exit_on_error:
	ftdi_free(ftdi);
	return NULL;
}

struct ftdi_context* _ftdi_mpsse_init(u32 vid, u32 pid, u32 dev, u32 port)
{
	struct ftdi_context *ftdi = _ftdi_open(vid, pid, dev, port);
	int err;

	if (!ftdi) {
		DBG();
		return NULL;
	}

	err = ftdi_usb_reset(ftdi);
	if (err < 0) {
		DBG("ftdi_usb_reset() failed: %d (%s)",
			err, ftdi_get_error_string(ftdi));
		goto exit_on_error;
	}

	err = ftdi_set_bitmode(ftdi, 0x00, BITMODE_RESET);
	if (err < 0) {
		DBG("ftdi_set_bitmode() failed: %d (%s)",
			err, ftdi_get_error_string(ftdi));
		goto exit_on_error;
	}

	if (port < FTDI_PORT_C) {
		err = ftdi_set_bitmode(ftdi, 0x00, BITMODE_MPSSE);
		if (err < 0) {
			DBG("ftdi_set_bitmode() failed: %d (%s)",
				err, ftdi_get_error_string(ftdi));
			goto exit_on_error;
		}

		err = ftdi_usb_purge_buffers(ftdi);
		if (err < 0) {
			DBG("ftdi_usb_purge_buffers() failed: %d (%s)",
				err, ftdi_get_error_string(ftdi));
			goto exit_on_error;
		}

		err = _ftdi_mpsse_sync(ftdi);
		if (err < 0)
			goto exit_on_error;
	} else {
		err = ftdi_set_bitmode(ftdi, 0xfb, BITMODE_BITBANG);
		if (err < 0) {
			DBG("ftdi_set_bitmode() failed: %d (%s)",
				err, ftdi_get_error_string(ftdi));
			goto exit_on_error;
		}
	}

	return ftdi;

exit_on_error:
	ftdi_free(ftdi);
	return NULL;
}

void _ftdi_deinit(struct ftdi_context *ftdi)
{
	if (ftdi) {
		ftdi_usb_close(ftdi);
		ftdi_free(ftdi);
	}
}

/******************************************************************************/
/* assumes clock div by 5 is disabled */
u16 mpsse_hz_to_clock_divide(u32 freq, u32 three_phase)
{
	/*  TCK/SK freq = 30MHz / (1 + VAL) */
	double f;

	if (freq > MAX_FREQ) freq = MAX_FREQ;
	if (freq < MIN_FREQ) freq = MIN_FREQ;

	f = (double)freq;
	f = 30000000 / f;
	f -= 1;

	if (three_phase)
		f = (f * 2) / 3;

	return (u16)f;
}

void mpsse_purge_buffers(struct mpsse_iface *mi)
{
	int err = ftdi_usb_purge_buffers(mi->ftdi);

	if (err < 0)
		DBG("ftdi_usb_purge_buffers() failed: %d (%s)",
		    err, ftdi_get_error_string(mi->ftdi));
}

int mpsse_queue_command(struct mpsse_iface *mi, u8 *cmd, u32 size)
{
	if ((mi->cmd_buf_cnt + size) > mi->cmd_buf_size)
		return -1;

	memcpy(&mi->cmd_buf[mi->cmd_buf_cnt], cmd, size);
	mi->cmd_buf_cnt += size;
	return 0;
}

#define mpsse_queue_data		mpsse_queue_command
int mpsse_send_commands(struct mpsse_iface *mi)
{
	int err;

	err = _ftdi_write(mi->ftdi, mi->cmd_buf, mi->cmd_buf_cnt);
	mi->cmd_buf_cnt = 0;
	return err;
}

#define CMD8_VAL16_PACK(buf, cmd, val)  { buf[0] = cmd;			\
					  buf[1] = (val & 0xff);	\
					  buf[2] = ((val >> 8) & 0xff); }
int mpsse_queue_ctrl_send_immediate(struct mpsse_iface *mi)
{
	u8 cmd = SEND_IMMEDIATE;

	return mpsse_queue_command(mi, &cmd, sizeof(cmd));
}

int mpsse_queue_ctrl_loopback(struct mpsse_iface *mi, u32 enable)
{
	u8 cmd = enable ? ENABLE_LOOPBACK : DISABLE_LOOPBACK;

	return mpsse_queue_command(mi, &cmd, sizeof(cmd));
}

int mpsse_queue_ctrl_clock_divide_by5(struct mpsse_iface *mi, u32 enable)
{
	u8 cmd = enable ? ENABLE_CLKDIV5 : DISABLE_CLKDIV5;

	return mpsse_queue_command(mi, &cmd, sizeof(cmd));
}

int mpsse_queue_ctrl_adaptive_clock(struct mpsse_iface *mi, u32 enable)
{
	u8 cmd = enable ? ENABLE_ADAPTIVE_CLOCKING : DISABLE_ADAPTIVE_CLOCKING;

	return mpsse_queue_command(mi, &cmd, sizeof(cmd));
}

int mpsse_queue_ctrl_3phase_data_clock(struct mpsse_iface *mi, u32 enable)
{
	u8 cmd = enable ? ENABLE_3PHASE_CLOCK : DISABLE_3PHASE_CLOCK;

	return mpsse_queue_command(mi, &cmd, sizeof(cmd));
}

int mpsse_queue_ctrl_clock_divisor(struct mpsse_iface *mi, u16 clk_div)
{
	u8 cmd[3];

	CMD8_VAL16_PACK(cmd, SET_CLOCK_DIV, clk_div);
	return mpsse_queue_command(mi, cmd, sizeof(cmd));
}

int mpsse_queue_read_pin_value_direction(struct mpsse_iface *mi, u32 bits)
{
	u8 cmd = (bits == MPSSE_HIGHBITS) ? READ_DBITS_HIGH : READ_DBITS_LOW;

	return mpsse_queue_command(mi, &cmd, sizeof(cmd));
}

int mpsse_queue_ctrl_pin_value_direction(struct mpsse_iface *mi, u32 bits,
					 u32 value, u32 direction)
{
	u8 cmd[3];

	cmd[0] = (bits == MPSSE_HIGHBITS) ? SET_DBITS_HIGH : SET_DBITS_LOW;
	cmd[1] = value & 0xff;
	cmd[2] = direction & 0xff;
	return mpsse_queue_command(mi, cmd, sizeof(cmd));
}

int mpsse_queue_read_data_bytes(struct mpsse_iface *mi, u32 count, u8 flags)
{
	u8 cmd[3], rd_cmd;

	rd_cmd = TDO_READ | (flags & (NEG_VE_CLK_READ | LSB_MODE));
	CMD8_VAL16_PACK(cmd, rd_cmd, (count - 1));
	return mpsse_queue_command(mi, cmd, sizeof(cmd));
}

int mpsse_queue_read_data_bits(struct mpsse_iface *mi, u32 count, u8 flags)
{
	u8 cmd[2];

	cmd[0] = (TDO_READ | BIT_MODE) | (flags & (NEG_VE_CLK_READ | LSB_MODE));
	cmd[1] = count - 1;
	return mpsse_queue_command(mi, cmd, sizeof(cmd));
}

int mpsse_queue_write_data_bytes(struct mpsse_iface *mi, u32 count, u8 flags)
{
	u8 cmd[3], wr_cmd;

	wr_cmd = TDI_WRITE | (flags & (NEG_VE_CLK_WRITE | LSB_MODE));
	CMD8_VAL16_PACK(cmd, wr_cmd, (count - 1));
	return mpsse_queue_command(mi, cmd, sizeof(cmd));
}

int mpsse_queue_write_data_bits(struct mpsse_iface *mi, u32 count, u8 flags)
{
	u8 cmd[2];

	cmd[0] = (TDI_WRITE | BIT_MODE) | (flags & (NEG_VE_CLK_WRITE | LSB_MODE));
	cmd[1] = (count - 1);
	return mpsse_queue_command(mi, cmd, sizeof(cmd));
}

int mpsse_queue_write_tms_bits(struct mpsse_iface *mi, u32 count, u8 flags)
{
	u8 cmd[2];

	cmd[0] = (TMS_WRITE | BIT_MODE) | (flags & (NEG_VE_CLK_WRITE | LSB_MODE));
	cmd[1] = (count - 1);
	return mpsse_queue_command(mi, cmd, sizeof(cmd));
}

int mpsse_queue_write_tms_read_tdo_bits(struct mpsse_iface *mi, u32 count,
					u8 flags)
{
	u8 cmd[2];

	cmd[0] = (TMS_WRITE | TDO_READ | BIT_MODE) |
		 (flags & (NEG_VE_CLK_WRITE | LSB_MODE));
	cmd[1] = (count - 1);
	return mpsse_queue_command(mi, cmd, sizeof(cmd));
}

int mpsse_queue_read_write_tdo_tdi_bits(struct mpsse_iface *mi, u32 count,
					u8 flags)
{
	u8 cmd[2];

	cmd[0] = (TDI_WRITE | TDO_READ | BIT_MODE) |
		 (flags & (NEG_VE_CLK_WRITE | LSB_MODE));
	cmd[1] = (count - 1);
	return mpsse_queue_command(mi, cmd, sizeof(cmd));
}

int mpsse_queue_read_write_tdo_tdi_bytes(struct mpsse_iface *mi, u32 count,
					 u8 flags)
{
	u8 cmd[3], rw_cmd;

	rw_cmd = (TDI_WRITE | TDO_READ) |
		 (flags & (NEG_VE_CLK_WRITE | NEG_VE_CLK_READ | LSB_MODE));
	CMD8_VAL16_PACK(cmd, rw_cmd, (count - 1));
	return mpsse_queue_command(mi, cmd, sizeof(cmd));
}

static void set_bit(u32 *buf, u32 offs, u32 val)
{
	*buf &= ~(1 << offs);
	*buf |=  (val & 1) << offs;
}

int mpsse_set_gpio_dir(struct gpio_intf *gi, u32 gpio, u32 output, u32 val)
{
	struct mpsse_iface *mi = (struct mpsse_iface *)gi->priv;
	u8 bits, _gpio, _val, _dir;

	if (gpio >= mi->gpio.cnt) {
		DBG();
		return -1;
	}
	_gpio = gpio + mi->gpio.offs;

	if (!output)
		val = 0;

	set_bit(&mi->gpio.dir,	_gpio, !!output);
	set_bit(&mi->gpio.cached, _gpio, !!val);

	bits = (_gpio > 7) ? MPSSE_HIGHBITS : MPSSE_LOWBITS;
	if (bits == MPSSE_HIGHBITS) {
		_val = 0xff & (mi->gpio.cached >> 8);
		_dir = 0xff & (mi->gpio.dir	>> 8);
	} else {
		_val = 0xff & mi->gpio.cached;
		_dir = 0xff & mi->gpio.dir;
	}

	mpsse_queue_ctrl_pin_value_direction(mi, bits, _val, _dir);
	mpsse_queue_ctrl_send_immediate(mi);
	mpsse_send_commands(mi);
	return 0;
}

int mpsse_get_gpio_dir(struct gpio_intf *gi, u32 *dir)
{
	struct mpsse_iface *mi = (struct mpsse_iface *)gi->priv;

	*dir = mi->gpio.dir >> mi->gpio.offs;
	*dir = *dir & ((1 << mi->gpio.cnt) - 1);
	return 0;
}

int mpsse_set_gpio(struct gpio_intf *gi, u32 gpio, u32 val)
{
	struct mpsse_iface *mi = (struct mpsse_iface *)gi->priv;
	u32 _gpio;

	if (gpio >= mi->gpio.cnt) {
		DBG();
		return -1;
	}

	_gpio = gpio + mi->gpio.offs;
	if (!((1 << _gpio) & mi->gpio.dir)) {
		DBG("GPIO not output (%d)", _gpio);
		return -1;
	}
	return mpsse_set_gpio_dir(gi, gpio, 1, val);
}

int mpsse_get_gpio(struct gpio_intf *gi, u32 *gpio)
{
	struct mpsse_iface *mi = (struct mpsse_iface *)gi->priv;
	u8 val[2];
	int err;

	if (mpsse_queue_read_pin_value_direction(mi, MPSSE_LOWBITS))
		return -1;
	if (mpsse_queue_read_pin_value_direction(mi, MPSSE_HIGHBITS))
		return -1;

	mpsse_queue_ctrl_send_immediate(mi);
	mpsse_send_commands(mi);

	err = _ftdi_read(mi->ftdi, val, sizeof(val));
	if (err)
		return -1;
	*gpio = (val[1] << 8) | val[0];
	*gpio = *gpio >> mi->gpio.offs;
	return 0;
}

int mpsse_gpio_count(struct gpio_intf *gi)
{
	struct mpsse_iface *mi = (struct mpsse_iface *)gi->priv;

	return mi->gpio.cnt;
}

struct gpio_intf * mpsse_gpio_intf_init(struct mpsse_iface *mi, u32 cnt,
					u32 offs, u32 val, u32 dir)
{
	struct gpio_state *s = &mi->gpio;
	struct gpio_intf *intf = &s->intf;

	s->cnt = cnt;
	s->dir = dir;
	s->cached = val;
	s->offs = offs;

	intf->set_gpio = mpsse_set_gpio;
	intf->set_gpio_dir = mpsse_set_gpio_dir;
	intf->get_gpio = mpsse_get_gpio;
	intf->get_gpio_dir = mpsse_get_gpio_dir;
	intf->count = mpsse_gpio_count;

	intf->priv = mi;
	return intf;
}

struct mpsse_iface * mpsse_init(u32 vid, u32 pid, u32 dev, u32 port, u32 buf_sz)
{
	struct mpsse_iface *p;

	if (buf_sz < MIN_BUF_SZ) {
		DBG("buffer too small");
		return NULL;
	}
	p = malloc(sizeof(*p));
	if (p) {
		p->cmd_buf = malloc(buf_sz);
		if (!p->cmd_buf) {
			free(p);
			return NULL;
		}
		p->ftdi = _ftdi_mpsse_init(vid, pid, dev, port);
		if (!p->ftdi) {
			free(p->cmd_buf);
			free(p);
			return NULL;
		}
	}
	p->cmd_buf_cnt = 0;
	p->cmd_buf_size = buf_sz;
	return p;
}

void mpsse_free(struct mpsse_iface *p)
{
	if (p) {
		_ftdi_deinit(p->ftdi);
		free(p->cmd_buf);
		free(p);
	}
}

struct vidpid_info {
	int vid;
	int *pids;
	int pidcnt;
};

#define DEFAULT_STRLEN	  64
void mpsse_list(void)
{
	int pid_0403[] = { 0x6011, 0x6010, 0x6001 };
	struct vidpid_info vidpid_list[] = {
		{ 0x0403 , pid_0403, DIM(pid_0403) },
	};
	struct ftdi_device_list *devlist, *p;
	int i, j, k, err, vid, pid;
	char serial[DEFAULT_STRLEN];
	char desc[DEFAULT_STRLEN];
	char mnf[DEFAULT_STRLEN];
	int len = DEFAULT_STRLEN;
	struct ftdi_context *ftdi;

	ftdi = ftdi_new();
	if (!ftdi) {
		DBG("ftdi_new failed");
		return;
	}

	for (j = 0; j < DIM(vidpid_list); j++) {
		vid = vidpid_list[j].vid;
		for (k = 0; k < vidpid_list[j].pidcnt; k++) {
			pid = vidpid_list[j].pids[k];
			err = ftdi_usb_find_all(ftdi, &devlist, vid, pid);
			if (err < 0) {
				DBG("ftdi_usb_find_all failed");
				continue;
			}
			printf("\nVID_%04x/PID_%04x:\n", vid, pid);
			if (!devlist)
				printf("  -- NONE --\n");
			for (i = 0, p = devlist; p; i++, p = p->next) {
				err = ftdi_usb_get_strings(ftdi, p->dev, mnf,
							   len, desc, len, serial,
							   len);
				if (err < 0) {
					DBG("ftdi_usb_get_strings() failed  %d  (%s)",
						 err, ftdi_get_error_string(ftdi));
					mnf[0] = desc[0] = serial[0] = '\0';
				}
				printf(" %d: Manufacturer:%s [%s] Serial:%s\n",
						i, mnf, desc, serial);
			}
			ftdi_list_free(&devlist);
		}
	}
	printf("\n");

	ftdi_free(ftdi);
}

/* debug board */
#define DBGBOARD_VID			0x0403
#define DBGBOARD_PID			0x6011

#define DBGBOARD_RESET_DEV		"01"
#define DBGBOARD_RESET_PORT		FTDI_PORT_C

#define DBGBOARD_FSB_DEV		"00"
#define DBGBOARD_FSB_PORT		FTDI_PORT_D

#define DBGBOARD_JTAG_DEV		"00"
#define DBGBOARD_JTAG_PORT		FTDI_PORT_A

#define NEW_DBGBOARD_JTAG_DEV		"00"
#define NEW_DBGBOARD_JTAG_PORT		FTDI_PORT_A

/* cdp */
#define CDP_VID				0x0403
#define CDP_PID				0x6011

#define CDP_RESET_DEV			"01"
#define CDP_RESET_PORT			FTDI_PORT_A

#define CDP_JTAG_DEV			"01"
#define CDP_JTAG_PORT			FTDI_PORT_A

/* starfighter */
#define SF_VID				0x0403
#define SF_PID				0x6011

#define SF_RESET_DEV			"00"
#define SF_RESET_PORT			FTDI_PORT_B

#define SF_JTAG_DEV			"00"
#define SF_JTAG_PORT			FTDI_PORT_A

static int match(char *mnf, char *desc, char *ser, char *amda, char *dev)
{
	if (mnf && strcmp("", mnf))
		return -1;
	if (dev && strcmp(dev, ser))
		return -1;
	if (amda && !strstr(desc, amda))
		return -1;
	return 0;
}

static int full_amda_lookup(char *amda, u32 vid, u32 pid, char *fullamda, u32 sz)
{
	struct ftdi_device_list *devlist, *p;
	struct ftdi_context *ftdi;
	char mnf[DEFAULT_STRLEN];
	char desc[DEFAULT_STRLEN];
	char ser[DEFAULT_STRLEN];
	int len = DEFAULT_STRLEN;
	int i, err, rv = -1;
	int cnt = 0;

	ftdi = ftdi_new();
	if (!ftdi)
		return -1;

	err = ftdi_usb_find_all(ftdi, &devlist, vid, pid);
	if (err < 0)
		return -1;
	if (!devlist)
		return -1;

	fullamda[0] = '\0';
	for (i = 0, p = devlist; p; i++, p = p->next) {
		err = ftdi_usb_get_strings(ftdi, p->dev, mnf, len, desc, len,
					   ser, len);
		if (err < 0)
			break;
		if (!match(mnf, desc, ser, amda, NULL)) {
			if (strcmp(fullamda, desc))
				cnt++;
			strncpy(fullamda, desc, sz);
			fullamda[sz - 1] = '\0';
		}
	}

	if (cnt == 1)
		rv = 0;

	ftdi_list_free(&devlist);
	ftdi_free(ftdi);
	return rv;
}

static int ftdi_signature(char *amda, char *amda_dev, u32 vid, u32 pid,
			  u32 *dev_idx)
{
	struct ftdi_device_list *devlist, *p;
	struct ftdi_context *ftdi;
	char mnf[DEFAULT_STRLEN];
	char desc[DEFAULT_STRLEN];
	char ser[DEFAULT_STRLEN];
	int len = DEFAULT_STRLEN;
	int i, err, rv = -1;

	ftdi = ftdi_new();
	if (!ftdi) {
		DBG("ftdi_new failed");
		return -1;
	}

	err = ftdi_usb_find_all(ftdi, &devlist, vid, pid);
	if (err < 0) {
		DBG("ftdi_usb_find_all failed");
		return -1;
	}

	if (!devlist) {
		DBG("ftdi_usb_find_all returned no devices");
		return -1;
	}

	for (i = 0, p = devlist; p; i++, p = p->next) {
		err = ftdi_usb_get_strings(ftdi, p->dev, mnf, len, desc, len,
					   ser, len);
		if (err < 0) {
			DBG("ftdi_usb_get_strings() failed  %d  (%s)",
				err, ftdi_get_error_string(ftdi));
			break;
		}
		if (!match(mnf, desc, ser, amda, amda_dev)) {
			*dev_idx = i;
			rv = 0;
			break;
		}
	}
	ftdi_list_free(&devlist);
	ftdi_free(ftdi);
	return rv;
}

static int dbgboard_ftdi_signature(char *amda, char *amda_dev, u32 *vid,
				   u32 *pid, u32 *dev_idx)
{
	*vid = DBGBOARD_VID;
	*pid = DBGBOARD_PID;

	return ftdi_signature(amda, amda_dev, *vid, *pid, dev_idx);
}

static int sf_ftdi_signature(char *amda, char *amda_dev, u32 *vid, u32 *pid,
			     u32 *dev_idx)
{
	*vid = SF_VID;
	*pid = SF_PID;

	return ftdi_signature(amda, amda_dev, *vid, *pid, dev_idx);
}

static int cdp_ftdi_signature(char *amda, char *amda_dev, u32 *vid, u32 *pid,
			      u32 *dev_idx)
{
	*vid = CDP_VID;
	*pid = CDP_PID;

	return ftdi_signature(amda, amda_dev, *vid, *pid, dev_idx);
}

static int sf_jtag_ftdi_signature(char *amda, u32 *vid, u32 *pid, u32 *dev_idx,
				  u32 *port)
{
	int ret;

	*port = SF_JTAG_PORT;

	ret = sf_ftdi_signature(amda, SF_JTAG_DEV, vid, pid, dev_idx);
	if (!ret)
		printf("SF MPCORE JTAG SN:%s => [VID:0x%04x PID:0x%04x DEV:%d PORT:%d]\n",
		       amda, *vid, *pid, *dev_idx, *port);
	return ret;
}

static int cdp_jtag_ftdi_signature(char *amda, u32 *vid, u32 *pid, u32 *dev_idx,
				   u32 *port)
{
	int ret;

	*port = CDP_JTAG_PORT;

	ret = cdp_ftdi_signature(amda, CDP_JTAG_DEV, vid, pid, dev_idx);
	if (!ret)
		printf("CDP MPCORE JTAG SN:%s => [VID:0x%04x PID:0x%04x DEV:%d PORT:%d]\n",
		       amda, *vid, *pid, *dev_idx, *port);
	return ret;
}

static int dbgboard_jtag_ftdi_signature(char *amda, u32 *vid, u32 *pid,
					u32 *dev_idx, u32 *port)
{
	int ret;

	*port = DBGBOARD_JTAG_PORT;

	ret = dbgboard_ftdi_signature(amda, DBGBOARD_JTAG_DEV, vid, pid, dev_idx);
	if (!ret)
		printf("NIC DEBUG-BOARD SN:%s => [VID:0x%04x PID:0x%04x DEV:%d PORT:%d]\n",
		       amda, *vid, *pid, *dev_idx, *port);
	return ret;
}

static int cdp_reset_ftdi_signature(char *amda, u32 *vid, u32 *pid, u32 *dev_idx,
				    u32 *port)
{
	*port = CDP_RESET_PORT;

	return cdp_ftdi_signature(amda, CDP_RESET_DEV, vid, pid, dev_idx);
}

static int sf_reset_ftdi_signature(char *amda, u32 *vid, u32 *pid, u32 *dev_idx,
				   u32 *port)
{
	*port = SF_RESET_PORT;

	return sf_ftdi_signature(amda, SF_RESET_DEV, vid, pid, dev_idx);
}

static int dbgboard_reset_ftdi_signature(char *amda, u32 *vid, u32 *pid,
					 u32 *dev_idx, u32 *port)
{
	*port = DBGBOARD_RESET_PORT;

	return dbgboard_ftdi_signature(amda, DBGBOARD_RESET_DEV, vid, pid,
				       dev_idx);
}

static int dbgboard_fsb_ftdi_signature(char *amda, u32 *vid, u32 *pid,
				       u32 *dev_idx, u32 *port)
{
	*port = DBGBOARD_FSB_PORT;

	return dbgboard_ftdi_signature(amda, DBGBOARD_FSB_DEV, vid, pid, dev_idx);
}

int dbgboard_failsafeboot(char *amda, int en)
{
	struct ftdi_context *ftdi;
	u32 vid, pid, dev, port;
	int err;
	u8 val;

	if (dbgboard_fsb_ftdi_signature(amda, &vid, &pid, &dev, &port))
		return -1;

	ftdi = _ftdi_open(vid, pid, dev, port);
	if (!ftdi)
		return -1;

	err = ftdi_set_bitmode(ftdi, 0x80, BITMODE_BITBANG);
	if (err < 0)
		goto exit;

	val = en ? 0 : 0x80;
	err = ftdi_write_data(ftdi, &val, sizeof(val));

exit:
	ftdi_usb_close(ftdi);
	ftdi_free(ftdi);
	return err < 0 ? err : 0;
}

static int ftdi_gpio_control(u32 vid, u32 pid, u32 dev, u32 port, u8 mask,
			     u8 val1, u32 us_delay, u8 val2)
{
	struct ftdi_context *ftdi;
	int err;

	ftdi = _ftdi_open(vid, pid, dev, port);
	if (!ftdi) {
		printf("%s:%d %s()\n", __FILE__, __LINE__, __func__);
		return -1;
	}

	err = ftdi_set_bitmode(ftdi, mask, BITMODE_BITBANG);
	if (err < 0) {
		printf("%s:%d %s()\n", __FILE__, __LINE__, __func__);
		goto exit;
	}

	val1 &= mask;
	err = ftdi_write_data(ftdi, &val1, sizeof(val1));
	if (err < 0) {
		printf("%s:%d %s()\n", __FILE__, __LINE__, __func__);
		goto exit;
	}

	usleep(us_delay);

	val2 &= mask;
	err = ftdi_write_data(ftdi, &val2, sizeof(val2));
	if (err < 0) {
		printf("%s:%d %s()\n", __FILE__, __LINE__, __func__);
		goto exit;
	}

exit:
	ftdi_usb_close(ftdi);
	ftdi_free(ftdi);
	return err < 0 ? err : 0;
}

int dbgboard_reset(char *amda)
{
	u32 vid, pid, dev, port;
	char fullamda[64];

	if (!full_amda_lookup(amda, 0x0403, 0x6011, fullamda, sizeof(fullamda)))
		amda = fullamda;

	if (strstr(amda, "AMDA0058")) {
		if (sf_reset_ftdi_signature(amda, &vid, &pid, &dev, &port))
			return -1;
	} else if (strstr(amda, "AMDA0057")) {
		if (cdp_reset_ftdi_signature(amda, &vid, &pid, &dev, &port)) {
			printf("%s:%d %s()\n", __FILE__, __LINE__, __func__);
			return -1;
		} else {
			printf("cdp reset\n");
		}
	} else {
		if (dbgboard_reset_ftdi_signature(amda, &vid, &pid, &dev, &port))
			return -1;
	}

	return ftdi_gpio_control(vid, pid, dev, port, 0xff, 0, 100 * 1000, 0xff);
}

int find_ftdi_device_index(int vid, int pid, char *manufacturer, char *product,
			   char *serial)
{
	struct ftdi_device_list *dl, *p;
	struct ftdi_context *ftdi;
	char mnf[DEFAULT_STRLEN];
	char prd[DEFAULT_STRLEN];
	char ser[DEFAULT_STRLEN];
	int len = DEFAULT_STRLEN;
	int i, err, rv = -1;

	ftdi = ftdi_new();
	if (!ftdi) {
		DBG("ftdi_new failed");
		return -1;
	}

	err = ftdi_usb_find_all(ftdi, &dl, vid, pid);
	if (err < 0) {
		DBG("ftdi_usb_find_all failed");
		return -1;
	}

	if (!dl) {
		DBG("ftdi_usb_find_all returned no devices");
		return -1;
	}

	for (i = 0, p = dl; p; i++, p = p->next) {
		err = ftdi_usb_get_strings(ftdi, p->dev, mnf, len, prd, len,
					   ser, len);
		if (err < 0) {
			DBG("ftdi_usb_get_strings() failed  %d  (%s)",
				err, ftdi_get_error_string(ftdi));
			break;
		}
		if (!strncmp(manufacturer, mnf, strlen(manufacturer))	&&
		    !strncmp(product, prd, strlen(product))		&&
		    !strncmp(serial, ser, strlen(serial))) {
			rv = i;
			goto _exit;
		}
	}
_exit:
	ftdi_list_free(&dl);
	ftdi_free(ftdi);
	return rv;
}

#define MPJTAG_LBITS_DEFAULT_VAL		 0x08 /* TMS, TDO, TDI, TCLK */
#define MPJTAG_LBITS_DEFAULT_DIR		 0x0b /* TMS, TDO, TDI, TCLK */
#define MPJTAG_HBITS_DEFAULT_VAL		 0x00
#define MPJTAG_HBITS_DEFAULT_DIR		 0x00

struct priv {
	struct jtag_ctrl_intf ji;
	struct mpsse_iface *mi;
	void (*deinit)(int);
	int dev_idx;
};

static int mpsse_jtag_clock_tms(struct jtag_ctrl_intf *ji, u32 *tms, u8 *tdo,
				u32 tdi, u32 cnt)
{
	struct priv *priv = container_of(ji, struct priv, ji);
	struct mpsse_iface *mi = priv->mi;
	u8 sz, tmsbits;
	u32 t = *tms;

	while (cnt) {
		sz = (cnt >= 6) ? 5 : cnt;
		tmsbits = (u8)(t & 0x1f);
		tmsbits |= (t & (1 << (sz-1))) ? (1 << sz) : 0;
		tmsbits |= (tdi & 1) << 7;
		t = t >> sz;

		mpsse_queue_write_tms_read_tdo_bits(mi, sz,
						NEG_VE_CLK_WRITE|LSB_MODE);
		mpsse_queue_data(mi, &tmsbits, 1);
		cnt -= sz;
	}

	mpsse_purge_buffers(mi);
	mpsse_send_commands(mi);
	if (tdo) {
		if (_ftdi_read(mi->ftdi, tdo, 1)) {
			DBG("read failed");
			return -1;
		}
	}
	mpsse_purge_buffers(mi);
	return 0;
}

/*
 *   tms: value to present on tms at final data bit clock
 *   tdi: bits to clock out to chain
 *   tdo: bits clocked in from chain
 *   cnt: bit-count(== clock-cycles) to clock
 */
static int mpsse_jtag_clock_din_dout(struct jtag_ctrl_intf *ji, u32 tms, u8 *tdi,
				     u8 *tdo, u32 cnt)
{
	struct priv *priv = container_of(ji, struct priv, ji);
	u32 i, bit_cnt, byte_cnt, _cnt, bytes_to_read;
	u8 tdi_tms, last_bit_out, last_bit_in;
	struct mpsse_iface *mi = priv->mi;
	u8 *_tdo;
	u8 *p;

	mpsse_purge_buffers(mi);
	bytes_to_read = byte_cnt = bit_cnt = 0;
	_cnt = (cnt > 0) ? cnt - 1 : 0; /* bit count prior to final bit to clock */
	if (_cnt > 0) {
		byte_cnt = _cnt / 8;
		bit_cnt = _cnt % 8;

		if (byte_cnt) {
			bytes_to_read = byte_cnt;
			mpsse_queue_read_write_tdo_tdi_bytes(mi, byte_cnt,
						NEG_VE_CLK_WRITE|LSB_MODE);
			for (i = 0; i < byte_cnt; i++)
				mpsse_queue_data(mi, &tdi[i], 1);
		}

		if (bit_cnt) { /* the bits read back will be contained in their
				   own data byte, the last bit read below with
				   the TMS clocking will be contained in it's own
				   data byte */
			bytes_to_read++;
			mpsse_queue_read_write_tdo_tdi_bits(mi, bit_cnt,
						NEG_VE_CLK_WRITE|LSB_MODE);
			mpsse_queue_data(mi, &tdi[byte_cnt], 1);
		}
	}

	bytes_to_read++;
	_tdo = malloc(bytes_to_read);
	if (!_tdo) {
		DBG();
		return -1;
	}

	/* clock last bit */
	tdi_tms = tms ? 0x3 : 0x0;
	last_bit_in = tdi[(cnt - 1) / 8] >> ((cnt - 1) % 8);
	tdi_tms |= last_bit_in << 7;
	mpsse_queue_write_tms_read_tdo_bits(mi, 1, NEG_VE_CLK_WRITE|LSB_MODE);
	mpsse_queue_data(mi, &tdi_tms, 1);

	mpsse_send_commands(mi);
	if (_ftdi_read(mi->ftdi, _tdo, bytes_to_read)) {
		DBG("read failed");
		return -1;
	}

	last_bit_out = (_tdo[bytes_to_read - 1] >> 7) & 1;
	if (bit_cnt % 8) {
		p = &_tdo[_cnt / 8];
		*p = *p >> (8 - (bit_cnt % 8));
		*p |= last_bit_out << (bit_cnt % 8);
	} else {
		/*
		  bit_cnt being a multiple of 8 implies our last bit
		  lives in it own final byte anyways, no need to pack into
		  previous data byte, still need to shift to bottom of byte
		*/
		_tdo[bytes_to_read - 1] >>= 7;
	}
	memcpy(tdo, _tdo, 1 + ((cnt-1) / 8));
	free(_tdo);

	return 0;
}

static struct jtag_ctrl_intf * mpsse_jtag_init(u32 vid, u32 pid, u32 dev_id,
					       u32 port, u32 frequency,
					       u32 buffer_size,
					       struct gpio_intf **gpio_intf)
{
	struct priv *p;

	p = malloc(sizeof(*p));
	if (!p) {
		DBG();
		return NULL;
	}
	memset(p, 0, sizeof(*p));

	p->mi = mpsse_init(vid, pid, dev_id, port, buffer_size);
	if (!p->mi) {
		DBG();
		goto exit_err;
	}

	mpsse_queue_ctrl_clock_divide_by5(p->mi, DISABLE);
	mpsse_queue_ctrl_adaptive_clock(p->mi, DISABLE);
	mpsse_queue_ctrl_3phase_data_clock(p->mi, DISABLE);

	mpsse_queue_ctrl_pin_value_direction(p->mi, MPSSE_LOWBITS,
					     MPJTAG_LBITS_DEFAULT_VAL,
					     MPJTAG_LBITS_DEFAULT_DIR);

	mpsse_queue_ctrl_pin_value_direction(p->mi, MPSSE_HIGHBITS,
					     MPJTAG_HBITS_DEFAULT_VAL,
					     MPJTAG_HBITS_DEFAULT_DIR);

	mpsse_queue_ctrl_clock_divisor(p->mi,
				       mpsse_hz_to_clock_divide(frequency, 0));
	mpsse_send_commands(p->mi);

	mpsse_queue_ctrl_loopback(p->mi, DISABLE);
	mpsse_send_commands(p->mi);

	mpsse_gpio_intf_init(p->mi, 4, 4, 0xf8, 0xfb);
	if (gpio_intf)
		*gpio_intf = &p->mi->gpio.intf;

	p->ji.clock_tms	= mpsse_jtag_clock_tms;
	p->ji.clock_din_dout = mpsse_jtag_clock_din_dout;

	return &p->ji;

exit_err:
	if (p->mi) mpsse_free(p->mi);
	free(p);
	return NULL;
}

static void new_dbgboard_deinit(int dev_idx)
{
	if (ftdi_gpio_control(0x0403, 0x6011, dev_idx, FTDI_PORT_C, 0x20, 0, 100, 0))
		DBG();
}

static int new_dbgboard_jtag_ftdi_signature(char *amda, u32 *vid, u32 *pid,
					    u32 *dev_idx, u32 *port)
{
	u32 mask = 0x20, val = 0x20;
	int ret;

	*port = NEW_DBGBOARD_JTAG_PORT;

	ret = dbgboard_ftdi_signature(amda, NEW_DBGBOARD_JTAG_DEV, vid, pid, dev_idx);
	if (!ret)
		printf("NIC (new) DEBUG-BOARD SN:%s => "
		       "[VID:0x%04x PID:0x%04x DEV:%d PORT:%d]\n",
		       amda, *vid, *pid, *dev_idx, *port);

	if (ftdi_gpio_control(0x0403, 0x6011, *dev_idx, FTDI_PORT_C, mask,
			      val, 100, val))
		return -1;

	return ret;
}

struct jtag_ctrl_intf * nic_dbgboard_jtag_init(char *dbgboard_amda,
					       u32 frequency_hz, u32 buffer_size)
{
	u32 vid, pid, dev_idx, port;
	struct jtag_ctrl_intf *jci;
	struct gpio_intf *gi;
	char fullamda[64];
	struct priv *p;
	int ret;

	if (!full_amda_lookup(dbgboard_amda, 0x0403, 0x6011, fullamda,
			      sizeof(fullamda)))
		dbgboard_amda = fullamda;

	if (strstr(dbgboard_amda, "AMDA0057"))
		ret = cdp_jtag_ftdi_signature(dbgboard_amda, &vid, &pid,
					      &dev_idx, &port);
	else if (strstr(dbgboard_amda, "AMDA0058"))
		ret = sf_jtag_ftdi_signature(dbgboard_amda, &vid, &pid,
					     &dev_idx, &port);
	else if (strstr(dbgboard_amda, "AMDA0132"))
		ret = new_dbgboard_jtag_ftdi_signature(dbgboard_amda, &vid, &pid,
						       &dev_idx, &port);
	else
		ret = dbgboard_jtag_ftdi_signature(dbgboard_amda, &vid, &pid,
						   &dev_idx, &port);

	if (ret) {
		DBG("failed to find signature for SN:%s", dbgboard_amda);
		return NULL;
	}

	jci = mpsse_jtag_init(vid, pid, dev_idx, port, frequency_hz, buffer_size,
			      &gi);
	if (jci && strstr(dbgboard_amda, "AMDA0058"))
		gi->set_gpio(gi, 0, 1);
	if (jci && strstr(dbgboard_amda, "AMDA0132")) {
		p = container_of(jci, struct priv, ji);
		p->deinit = new_dbgboard_deinit;
		p->dev_idx = dev_idx;
	}
	return jci;
}

void nic_dbgboard_jtag_deinit(struct jtag_ctrl_intf *ji)
{
	struct priv *p = container_of(ji, struct priv, ji);

	if (p->deinit)
		p->deinit(p->dev_idx);
	mpsse_free(p->mi);
	free(p);
}
