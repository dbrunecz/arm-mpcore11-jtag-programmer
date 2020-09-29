/* Copyright (C) 2020 David Brunecz. Subject to GPL 2.0 */

#ifndef _MPSSE_H_
#define _MPSSE_H_
/******************************************************************************/

#define ENABLE                  1
#define DISABLE                 0

#define FTDI_PORT_A             0
#define FTDI_PORT_B             1
#define FTDI_PORT_C             2
#define FTDI_PORT_D             3

/* HZ */
#define MAX_FREQ                30000000
#define MIN_FREQ                458

/*  effective buffer size for large R/W transactions is
 *   the allocated buffer size for queued MPSSE commands
 *   minus the I2C/MPSSE command overhead
 */
#define MIN_BUF_SZ              256
#define DEFAULT_CMD_OVERHEAD    64

struct mpsse_iface {
    void *ftdi;
    u8 *cmd_buf;
    u32 cmd_buf_cnt;  /* count of valid data to transmit */
    u32 cmd_buf_size; /* allocated size */
    void *user;

    /*
     *  x's represent available GPIO
     *
     * BUS-CFG   HIGH-BYTE  LOW-BYTE
     *  ____     _________  _________
     *  I2C      xxxx_xxxx  xxxx_xxDC,   Data/Clock
     *  SPI      xxxx_xxxx  xxxx_xIOC,   DI/DO/CLK
     *  JTAG     xxxx_xxxx  xxxx_IOMC,   TDI/TDO/TMS/TCK
     */
    struct gpio_state {
        int cnt;      /* number of available GPIO, (12 - 14) */
        u32 dir;      /* direction IN/OUT mask */
        u32 cached;   /* cached value */
        u32 offs;     /* bit offset of GPIO of MPSSE port (2-4) */

        struct gpio_intf intf;
    } gpio;
};

struct ftdi_context* _ftdi_mpsse_init(u32 vid, u32 pid, u32 dev, u32 port);
void _ftdi_deinit    (struct ftdi_context *ftdi);
int  _ftdi_read      (struct ftdi_context *ftdi, u8 *buffer, u32 cnt);
int  _ftdi_write     (struct ftdi_context *ftdi, u8 *buffer, u32 cnt);
int  _ftdi_mpsse_sync(struct ftdi_context *ftdi);

struct mpsse_iface * mpsse_init(u32 vid, u32 pid, u32 dev, u32 port,
                                    u32 buffer_size);
void                 mpsse_free(struct mpsse_iface *p);

u16 mpsse_hz_to_clock_divide(u32 freq, u32 three_phase);
void mpsse_purge_buffers(struct mpsse_iface *mi);
int mpsse_queue_ctrl_send_immediate(struct mpsse_iface *mi);
int mpsse_queue_ctrl_loopback(struct mpsse_iface *mi, u32 enable);
int mpsse_queue_ctrl_clock_divide_by5(struct mpsse_iface *mi, u32 enable);
int mpsse_queue_ctrl_adaptive_clock(struct mpsse_iface *mi, u32 enable);
int mpsse_queue_ctrl_3phase_data_clock(struct mpsse_iface *mi, u32 enable);
int mpsse_queue_ctrl_clock_divisor(struct mpsse_iface *mi, u16 clk_div);
#define MPSSE_HIGHBITS      0
#define MPSSE_LOWBITS       1
int mpsse_queue_read_pin_value(struct mpsse_iface *mi, u32 bits);
int mpsse_queue_ctrl_pin_value_direction(struct mpsse_iface *mi, u32 bits,
                                          u32 value, u32 direction);

int mpsse_set_gpio     (struct gpio_intf *gi, u32 pin, u32 val);
int mpsse_get_gpio     (struct gpio_intf *gi, u32 *gpio);
int mpsse_set_gpio_dir (struct gpio_intf *gi, u32 pin, u32 output, u32 val);
int mpsse_get_gpio_dir (struct gpio_intf *gi, u32 *gpio);
int mpsse_gpio_count   (struct gpio_intf *gi);

#define MPSSE_PORT_PIN_COUNT    16
struct gpio_intf * mpsse_gpio_intf_init(struct mpsse_iface *mi,
                                        u32 cnt, u32 offs, u32 val, u32 dir);

/* flags are the relevent MPSSE shift data in/out command bits */
/* MPSSE data shift command bits */
#define NEG_VE_CLK_WRITE    (1 << 0)
#define BIT_MODE            (1 << 1)
#define NEG_VE_CLK_READ     (1 << 2)
#define LSB_MODE            (1 << 3)
#define TDI_WRITE           (1 << 4)
#define TDO_READ            (1 << 5)
#define TMS_WRITE           (1 << 6)
int mpsse_queue_read_data_bytes(struct mpsse_iface *mi, u32 count, u8 flags);
int mpsse_queue_read_data_bits(struct mpsse_iface *mi, u32 count, u8 flags);
int mpsse_queue_write_data_bytes(struct mpsse_iface *mi, u32 count, u8 flags);
int mpsse_queue_write_data_bits(struct mpsse_iface *mi, u32 count, u8 flags);

int mpsse_queue_write_tms_bits(struct mpsse_iface *mi, u32 count, u8 flags);
int mpsse_queue_write_tms_read_tdo_bits(struct mpsse_iface *mi, u32 count, u8 flags);
int mpsse_queue_read_write_tdo_tdi_bits(struct mpsse_iface *mi, u32 count, u8 flags);
int mpsse_queue_read_write_tdo_tdi_bytes(struct mpsse_iface *mi, u32 count, u8 flags);

int mpsse_queue_command(struct mpsse_iface *mi, u8 *cmd, u32 size);
#define mpsse_queue_data mpsse_queue_command

int mpsse_send_commands(struct mpsse_iface *mi);

void mpsse_list(void);

struct jtag_ctrl_intf * nic_dbgboard_jtag_init(char *serial_num, u32 freq_hz,
						u32 buffer_size);
void nic_dbgboard_jtag_deinit(struct jtag_ctrl_intf *ji);

int dbgboard_reset(char *amda);
int dbgboard_failsafeboot(char *amda, int en);
/******************************************************************************/

#endif //  _MPSSE_H_
