#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <errno.h>

#include "dbg.h"
#include "interfaces.h"
#include "mpsse.h"
#include "jtag.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

struct filebuf {
	u32 size;
	u8 ptr[];
};

static struct filebuf *loadfile(const char *name)
{
	struct filebuf *fb;
	struct stat st;
	FILE *f;

	if (stat(name, &st)) {
		DBG("stat: \"%s\" (%d) %s", name, errno, strerror(errno));
		return NULL;
	}

	fb = malloc(sizeof(*fb) + st.st_size);
	if (!fb) {
		DBG();
		return NULL;
	}

	f = fopen(name, "rb");
	if (!f) {
		DBG("fopen: \"%s\" (%d) %s", name, errno, strerror(errno));
		return NULL;
	}

	if (st.st_size != fread(fb->ptr, 1, st.st_size, f)) {
		fclose(f);
		DBG();
		return NULL;
	}
	fclose(f);

	fb->size = st.st_size;
	return fb;
}

#define ARM_MPCORE11_IDCODE				0x07b37477
#define MP11_IR_LEN					5

#define MPCORE_IR_EXTEST				0x00
#define MPCORE_IR_SCAN					0x02
#define MPCORE_IR_RESTART				0x04
#define MPCORE_IR_HALT					0x08
#define MPCORE_IR_INTEST				0x0c
#define MPCORE_IR_IDCODE				0x1e
#define MPCORE_IR_ITRSEL				0x1d

#define MPCORE_SCANCHAIN_DEBUG_ID			0
#define MPCORE_SCANCHAIN_DEBUG_SCR			1
#define MPCORE_SCANCHAIN_DEBUG_ITR			4
#define MPCORE_SCANCHAIN_DEBUG_DTR			5
#define MPCORE_SCANCHAIN_DEBUG_BRP_WRP			7 //VCR PC BRP WRP

/* scan chain 7 */
#define NULL7						0
#define VCR						7
#define PC						8
#define BVR_BASE					64
#define BCR_BASE					80
#define BVR_CNT						6
#define WVR_BASE					96
#define WCR_BASE					112
#define WVR_CNT						2

#define COND_EQ						0x0
#define COND_NE						0x1
#define COND_HS						0x2
#define COND_LO						0x3
#define COND_MI						0x4
#define COND_PL						0x5
#define COND_VS						0x6
#define COND_VC						0x7
#define COND_HI						0x8
#define COND_LS						0x9
#define COND_GE						0xa
#define COND_LT						0xb
#define COND_GT						0xc
#define COND_LE						0xd
#define COND_AL						0xe

#define STR(base, src)  (0xe5800000 | (base & 0xf) << 16 | (src & 0xf) << 12)
#define LDR(base, src)  (0xe5900000 | (base & 0xf) << 16 | (src & 0xf) << 12)
#define STRB(base, src) (0xe5c00000 | (base & 0xf) << 16 | (src & 0xf) << 12)
#define LDRB(base, src) (0xe5d00000 | (base & 0xf) << 16 | (src & 0xf) << 12)
#define STRD(base, src) (0xe1c000f0 | (base & 0xf) << 16 | (src & 0xf) << 12)
#define LDRD(base, src) (0xe1c000d0 | (base & 0xf) << 16 | (src & 0xf) << 12)

#define CP_XFER(cond, cpnum, op1, l, crn, rd, op2, crm)		\
						((cond & 0xf)  << 28	 | \
						 (op1 & 0x7)   << 21	 | \
						 (l & 0x1)     << 20	 | \
						 (crn & 0xf)   << 16	 | \
						 (rd & 0xf)    << 12	 | \
						 (cpnum & 0xf) <<  8	 | \
						 (op2 & 0x7)   <<  5	 | \
						 (crm & 0xf)   <<  0	 | \
						 0x0e000010 )
#define MCR(cpnum, op1, crn, rd, op2, crm) CP_XFER(COND_AL, cpnum, op1, 0, \
							crn, rd, op2, crm)
#define MRC(cpnum, op1, crn, rd, op2, crm) CP_XFER(COND_AL, cpnum, op1, 1, \
							crn, rd, op2, crm)

#define ARM_LOAD_STORE_COPROC(cpnum, crd, rn, p, u, n, w, l, offs)	  \
						 (0xec000000		  | \
						((	p & 0x1) << 24) | \
						((	u & 0x1) << 23) | \
						((	n & 0x1) << 22) | \
						((	w & 0x1) << 21) | \
						((	l & 0x1) << 20) | \
						((   rn & 0xf) << 16) | \
						((  crd & 0xf) << 12) | \
						((cpnum & 0xf) <<  8) | \
						((offs >> 2) & 0xff))

#define ARM_STC(cpnum, crd, rn, offs)   ARM_LOAD_STORE_COPROC(cpnum, crd, rn,\
							0, 1, 0, 1, 0, offs)

#define DATAPROCESS(cond, op, s, rn, rs, oper)	  ((cond & 0xf) << 28 | \
						 (  op & 0xf) << 21 | \
						 (   s & 0x1) << 20 | \
						 (  rn & 0xf) << 16 | \
						 (  rs & 0xf) << 12 | \
						 (oper & 0xfff) << 0 | \
						 0x01000000)

#define ARM_BAL(offs)	  (0xea000000 | ((offs >> 2) & 0xffffff))
#define ARM_HANG()		  ARM_BAL(-8)
#define ARM_MOV(rd, rs)   DATAPROCESS(COND_AL, 0x5, 0, 0, rd, rs)
#define ARM_NOP()		 ARM_MOV(REG(0), REG(0))
#define ARM_SWI(IMMED24)  (0xef000000 | (IMMED24 & 0xffffff))
#define ARM_BRKP(IMMED16) (0xe1200070 | ((IMMED16 & 0xfff0) << 4) | \
					 (IMMED16 & 0x000f))

#define Rd(x)		(x)
#define Rm(x)		(x)
#define Rn(x)		(x)

#define CRn(x)		(x)
#define CRm(x)		(x)
#define OP1(x)		(x)
#define OP2(x)		(x)

#define REG(x)		(x)

#define RESETBOLD	""
#define _BOLD		"\033[1m"

#define RESETCOLOR	"\033[0m"
#define _RED		"\033[31m"
#define _YELLOW		"\033[33m"
#define _WHITE		"\033[37m"
static char *psr_mode(int m)
{
	switch (m) {
	case 16 + 0: return _BOLD _WHITE "user" RESETCOLOR RESETBOLD;
	case 16 + 1: return _BOLD _YELLOW "fiq" RESETCOLOR RESETBOLD;
	case 16 + 2: return _BOLD _WHITE "irq" RESETCOLOR RESETBOLD;
	case 16 + 3: return _BOLD _WHITE "super" RESETCOLOR RESETBOLD;
	case 16 + 7: return _BOLD _YELLOW "ABORT" RESETCOLOR RESETBOLD;
	case 16 + 11: return _BOLD _RED "UNDEFINED" RESETCOLOR RESETBOLD;
	case 16 + 15: return _BOLD _YELLOW "system" RESETCOLOR RESETBOLD;
	}
	return _BOLD _RED "ERR" RESETCOLOR RESETBOLD;
}

#define DSCR_RDTRFULL		30
#define DSCR_WDTRFULL		29

#define PSR_JAZELLE		24
#define PSR_THUMB		5

static void display_psr(u32 psr)
{
	printf("cpsr:0x%08x   ", psr);
	printf("%s%s%s%s%s%s%s%s%s%s%s%s\n"
		, psr & BIT(31) ? "neg " : ""
		, psr & BIT(30) ? "zero " : ""
		, psr & BIT(29) ? "carry " : ""
		, psr & BIT(28) ? "ovflw " : ""
		, psr & BIT(27) ? "stky-ovflw " : ""
		, psr & BIT(PSR_JAZELLE) ? "jazell " : ""
		//, psr & BIT(9) ? "blE " : ""
		, psr & BIT(9) ? "end " : ""
		, psr & BIT(8) ? "imp_abort " : ""
		, psr & BIT(7) ? "irq_dis " : ""
		, psr & BIT(6) ? "fiq_dis " : ""
		, psr & BIT(PSR_THUMB) ? "thumb " : ""
		, psr_mode(psr & 0x1f)
		);
}

#define BFV(wd, offs, v)	((v >> offs) & ((1 << wd) - 1))

#define CR_U	BIT(22)
#define CR_I	BIT(12)
#define CR_C	BIT(2)
#define CR_A	BIT(1)
#define CR_M	BIT(0)
static void display_ctrl(u32 v)
{
	printf("%s%s%s%s""%s%s%s%s""%s%s%s%s""%s%s%s\n"
		, v & BIT(29) ? "Force_AP " : ""
		, v & BIT(28) ? "TEX_remap " : ""
		, v & BIT(27) ? "NMFI " : ""
		, v & BIT(25) ? "EE " : ""
		, v & BIT(23) ? "XP " : ""
		, v & BIT(22) ? "U " : ""
		, v & BIT(15) ? "L4 " : ""
		, v & BIT(13) ? "V " : ""
		, v & BIT(12) ? "I " : ""
		, v & BIT(11) ? "Z " : ""
		, v & BIT(9) ? "R " : ""
		, v & BIT(8) ? "S " : ""
		, v & BIT(2) ? "C " : ""
		, v & BIT(1) ? "A " : ""
		, v & BIT(0) ? "M " : ""
		);
}

static void display_auxctrl(u32 v)
{
	printf("%s%s%s%s""%s%s%s\n"
		, v & BIT(6) ? "L1_parity_check " : ""
		, v & BIT(5) ? "S(A)MP_mode " : ""
		, v & BIT(4) ? "EXCL " : ""
		, v & BIT(3) ? "F " : ""
		, v & BIT(2) ? "SB " : ""
		, v & BIT(1) ? "DB " : ""
		, v & BIT(0) ? "RS " : ""
		);
}

static void display_dscr(u32 dscr)
{
	char *dscr_entry[] = {
		"HALT_DBGTAP ", "breakpoint ", "watchpoint ", "BKPT ", "EDBGRQ ",
		"vector_catch ", "data_side_abort ", "instr_side_abort " };

	printf("DSCR [%08x] : ", dscr);
	printf("%s%s%s%s%s%s%s%s%s%s%s%s""%s%s%s""%s"
		, dscr & BIT(30) ? "rDTRfull " : ""
		, dscr & BIT(29) ? "wDTRfull " : ""
		, dscr & BIT(15) ? "Monitor_mode_en " : ""
		, dscr & BIT(14) ? "Halt_mode_en " : ""
		, dscr & BIT(13) ? "ARM " : ""
		, dscr & BIT(12) ? "Comms " : ""
		, dscr & BIT(11) ? "Interrupts " : ""
		, dscr & BIT(10) ? "DbgAck " : ""
		, dscr & BIT(9) ? "DBGNOPWRDWN " : ""
		, dscr & BIT(8) ? "" : "" // UNP/SBPZ
		, dscr & BIT(7) ? "sticky_imprecise_abort " : ""
		, dscr & BIT(6) ? "sticky_precise_abort " : ""
		, dscr & BIT(0) ? _BOLD : ""
		, dscr_entry[(dscr >> 2) & 0x7]
		, dscr & BIT(0) ? RESETBOLD : ""
		, dscr & BIT(1) ? "restarted " : ""
		);

	printf(_BOLD "%s" RESETBOLD RESETCOLOR"\n", dscr & BIT(0) ? "halted "
								  : "");
}

static void display_mainid(u32 v)
{
	printf( "%11s |"
		" Impl:0x%x"
		" Var:0x%x"
		" Arch:0x%x"
		" PartNo:0x%x"
		" Rev:0x%x\n"
		, "Main ID"
		, BFV( 8, 24, v)
		, BFV( 4, 20, v)
		, BFV( 4, 16, v)
		, BFV(12,  4, v)
		, BFV( 4,  0, v)
		);
}

static void display_cachetype(u32 v)
{
	printf( "%11s |"
		" Ctype:0x%x"
		" S:%u"

		" Dp:%u"
		" Dsize:%u"
		" Dassoc:%u"
		" Dm:%u"
		" Dlen:%u"

		" Ip:%u"
		" Isize:%u"
		" Iassoc:%u"
		" Im:%u"
		" Ilen:%u\n"
		, "Cache Type"
		, BFV( 4, 25, v)
		, BFV( 1, 24, v)

		, BFV( 1, 23, v)
		, BFV( 4, 18, v)
		, BFV( 3, 15, v)
		, BFV( 1, 14, v)
		, BFV( 2, 12, v)

		, BFV( 1, 11, v)
		, BFV( 4,  6, v)
		, BFV( 3,  3, v)
		, BFV( 1,  2, v)
		, BFV( 2,  0, v)
		);
}

static void display_tlbtype(u32 v)
{
	printf( "%11s |"
		" ILSize:%u"
		" DLSize:%u"
		" U:%u\n"
		, "TLB Type"
		, BFV( 8, 16, v)
		, BFV( 8,  8, v)
		, BFV( 1,  0, v)
		);
}

static void display_cpuid(u32 v)
{
	printf( "%11s |"
		" ClusterId:%u"
		" CPUId:%u\n"
		, "CPU Id"
		, BFV( 4,  8, v)
		, BFV( 4,  0, v)
		);
}

/*static*/ void display_didr(uint64_t didr)
{
	printf("DIDR [%010lx] ", didr);
	printf("impl:0x%x WRP:%d BRP:%d CTXT:%d VER:%d VAR:%d REV:%d\n"
		, (int)((didr >> 32) & 0xff)
		, (int)((didr >> 28) & 0xf) + 1
		, (int)((didr >> 24) & 0xf) + 1
		, (int)((didr >> 20) & 0xf) + 1
		, (int)((didr >> 16) & 0xf)
		, (int)((didr >> 4) & 0xf)
		, (int)((didr >> 0) & 0xf)
		);
}

static int set_ir_with_data(struct jtag_intf *ji, u8 *ir, u32 ir_len,
			    u8 *tdo, u8 *tdi, u32 sz_in_bits, u32 tdi_bit)
{
	u8 *_tdi = NULL;
	u8 *_tdo = NULL;
	uint64_t buf;
	int rv;

	if (!tdo && sz_in_bits)
		_tdo = (void *)&buf;

	if (!tdi && sz_in_bits) {
		buf = tdi_bit ? ~0 : 0;
		_tdi = (void *)&buf;
	}

	rv = ji->txrx(ji, ir, ir_len,
				  !tdi ? _tdi : tdi,
				  !tdo ? _tdo : tdo,
				  sz_in_bits);
	if (rv)
		DBG();

	return rv;
}

static int mpcore_ir_rw(struct jtag_intf *ji, u8 ir,
						u8 *tdo, u8 *tdi, u32 dr_len)
{
	return set_ir_with_data(ji, &ir, MP11_IR_LEN, tdo, tdi, dr_len, 0);
}

static int mpcore_set_scanchain(struct jtag_intf *ji, u8 chain)
{
	return mpcore_ir_rw(ji, MPCORE_IR_SCAN, NULL, &chain, 5);
}

static int mpcore_chain_length(u32 chain)
{
	switch (chain) {
	case MPCORE_SCANCHAIN_DEBUG_ID:	return 40;
	case MPCORE_SCANCHAIN_DEBUG_SCR: return 32;
	case MPCORE_SCANCHAIN_DEBUG_ITR: return 33;
	case MPCORE_SCANCHAIN_DEBUG_DTR: return 34;
	case MPCORE_SCANCHAIN_DEBUG_BRP_WRP: return 40;
	default: return -1;
	}
	return -1;
}

static int mpcore_get_dscr(struct jtag_intf *ji, u32 *dscr)
{
	u8 tdo[4];

	if (mpcore_set_scanchain(ji, MPCORE_SCANCHAIN_DEBUG_SCR)) {
		DBG();
		return -1;
	}
	if (mpcore_ir_rw(ji, MPCORE_IR_INTEST, tdo, NULL,
			 mpcore_chain_length(MPCORE_SCANCHAIN_DEBUG_SCR)))
		return -1;
	*dscr = tdo[0] | tdo[1] << 8 | tdo[2] << 16 | tdo[3] << 24;
	*dscr &= ~(BIT(31)|BIT(8)|(0x1fff << 16));
	return 0;
}

static int mpcore_stop(struct jtag_intf *ji)
{
	u32 dscr;

	if (mpcore_ir_rw(ji, MPCORE_IR_HALT, NULL, NULL, 0))
		return -1;
	if (mpcore_get_dscr(ji, &dscr))
		return -1;
	return (dscr & 1) ? 0 : -1;
}

/*static*/ int mpcore_read_didr(struct jtag_intf *ji, uint64_t *id)
{
	u8 tdo[5];

	if (mpcore_set_scanchain(ji, MPCORE_SCANCHAIN_DEBUG_ID))
		return -1;
	if (mpcore_ir_rw(ji, MPCORE_IR_INTEST, tdo, NULL,
			 mpcore_chain_length(MPCORE_SCANCHAIN_DEBUG_ID)))
		return -1;
	*id = tdo[0] | tdo[1] << 8 | tdo[2] << 16 | tdo[3] << 24 |
			(uint64_t)tdo[4] << 32;
	return 0;
}

int quiet_mmu_disable = 0;
static int arm_exec(struct jtag_intf *ji, u32 instr)
{
	u8 tdo[5];
	u8 tdi[5];

	tdi[0] = (instr >>  0) & 0xff;
	tdi[1] = (instr >>  8) & 0xff;
	tdi[2] = (instr >> 16) & 0xff;
	tdi[3] = (instr >> 24) & 0xff;
	tdi[4] = 1;

	if (mpcore_set_scanchain(ji, MPCORE_SCANCHAIN_DEBUG_ITR))
		return -1;
	if (mpcore_ir_rw(ji, MPCORE_IR_EXTEST, tdo, tdi,
			 mpcore_chain_length(MPCORE_SCANCHAIN_DEBUG_ITR)))
		return -1;
	if (!(tdo[4] & 1)) {
		if (!quiet_mmu_disable)
			DBG();
		quiet_mmu_disable = 0;
		return -1;
	}
	return 0;
}

#define WDTR_READY		1
#define WDTR_VALID		2

static int mpcore_get_wdtr(struct jtag_intf *ji, u32 *val)
{
	u8 td[5];
	int rv;

	if (mpcore_set_scanchain(ji, MPCORE_SCANCHAIN_DEBUG_DTR))
		return -1;

	memset(td, 0, sizeof(td));
	rv = mpcore_ir_rw(ji, MPCORE_IR_INTEST, td, td,
			  mpcore_chain_length(MPCORE_SCANCHAIN_DEBUG_DTR));
	if (rv) {
		DBG();
		return -1;
	}
	if (td[4] & BIT(WDTR_VALID))
		return -1;

	*val = td[0] | td[1] << 8 | td[2] << 16 | td[3] << 24;
	return 0;
}

static int mpcore_write_dtr(struct jtag_intf *ji, u32 *val, u32 cnt)
{
	u32 retry_cnt = 0;
	u8 tdi[5];
	u8 tdo[5];
	int i, rv = 0;

	if (mpcore_set_scanchain(ji, MPCORE_SCANCHAIN_DEBUG_DTR))
		return -1;

	for (i = 0; i < cnt; i++) {

		tdi[0] = (val[i] >>  0) & 0xff;
		tdi[1] = (val[i] >>  8) & 0xff;
		tdi[2] = (val[i] >> 16) & 0xff;
		tdi[3] = (val[i] >> 24) & 0xff;

		retry_cnt = 0;
		for ( ;; ) {
			tdi[4] = 0;
			rv = mpcore_ir_rw(ji, MPCORE_IR_EXTEST, tdo, tdi,
					mpcore_chain_length(MPCORE_SCANCHAIN_DEBUG_DTR));
			if (rv)
				DBG();

			if (tdo[4] & BIT(WDTR_READY))
				break;

			printf(".");
			if (retry_cnt++ > 5) {
				DBG("%d %d", i, cnt);
				return -1;
			}
		}
	}
	return rv;
}

static int quiet = 0;

static int mpcore_read_dtr(struct jtag_intf *ji, u32 *val)
{
	u32 retry_cnt = 0;
	u8 tdo[5];
	int rv;

	if (mpcore_set_scanchain(ji, MPCORE_SCANCHAIN_DEBUG_DTR))
		return -1;

	for ( ;; ) {
		rv = mpcore_ir_rw(ji, MPCORE_IR_INTEST, tdo, NULL,
				mpcore_chain_length(MPCORE_SCANCHAIN_DEBUG_DTR));
		if (rv) {
			DBG();
			return -1;
		}
		if (tdo[4] & BIT(WDTR_READY)) {
			break;
		}
		printf(",");

		if (retry_cnt++ > 5) {
			DBG();
			break;
		}
	}
	*val = tdo[0] | tdo[1] << 8 | tdo[2] << 16 | tdo[3] << 24;
	return rv;
}

static int mpcore_ldr_from_dtr(struct jtag_intf *ji, u32 rd)
{
	return arm_exec(ji, MRC(14, 0, CRn(0), rd, 0, CRm(5)));
}

static int mpcore_str_to_dtr(struct jtag_intf *ji, u32 rd)
{
	return arm_exec(ji, MCR(14, 0, CRn(0), rd, 0, CRm(5)));
}

static int mpcore_set_register(struct jtag_intf *ji, u32 reg, u32 v)
{
	if (mpcore_ldr_from_dtr(ji, reg))
		return -1;
	if (mpcore_write_dtr(ji, &v, 1))
		return -1;
	return 0;
}

static int mpcore_get_register(struct jtag_intf *ji, u32 reg, u32 *v)
{
	if (mpcore_str_to_dtr(ji, reg))
		return -1;
	return mpcore_read_dtr(ji, v);
}

static int mpcore_wr32(struct jtag_intf *ji, u32 a, u32 v)
{
	if (a & 3) {
		DBG();
		return -1;
	}
	if (mpcore_set_register(ji, REG(9), a))
		return -1;
	if (mpcore_set_register(ji, REG(10), v))
		return -1;
	if (arm_exec(ji, STR(9, 10)))
		return -1;
	return 0;
}

static int mpcore_rd32(struct jtag_intf *ji, u32 a, u32 *v)
{
	if (a & 3) {
		DBG("addr:0x%08x", a);
		return -1;
	}
	if (mpcore_set_register(ji, REG(9), a))
		return -1;
	if (arm_exec(ji, LDR(REG(9), REG(10))))
		return -1;
	if (mpcore_get_register(ji, REG(10), v))
		return -1;
	return 0;
}

static int get_dtr(struct jtag_intf *ji, u32 *v)
{
	u32 dscr;

	if (mpcore_get_dscr(ji, &dscr)) {
		DBG();
		return -1;
	}

	if (!(dscr & BIT(DSCR_WDTRFULL)))
		return -1;

	if (mpcore_read_dtr(ji, v)) {
		DBG();
		return -1;
	}

	return 0;
}

/******************************************************************************/

u32 tickcount_ms(void)
{
	struct timespec t;

	if (clock_gettime(CLOCK_MONOTONIC, &t)) {
		DBG();
		return 0;
	}
	return t.tv_sec * 1000 + (t.tv_nsec/1000000);
}

static int transfer(struct jtag_intf *ji, u32 address, void *ptr, u32 size)
{
	u32 sz = (size + 3) & ~3;
	u32 *buf = ptr;
	u32 ms;

	ms = tickcount_ms();

	if (mpcore_set_register(ji, REG(0), address)) {
		DBG();
		return -1;
	}

	if (arm_exec(ji, ARM_STC(14, 5, REG(0), 4))) {
		DBG();
		return -1;
	}

	if (mpcore_write_dtr(ji, &buf[1], (sz - 4) / sizeof(u32))) {
		DBG();
		return -1;
	}

	if (arm_exec(ji, ARM_NOP())) {
		DBG();
		return -1;
	}

	if (mpcore_wr32(ji, address, buf[0])) {
		DBG();
		return -1;
	}

	ms = tickcount_ms() - ms;
	return ms;
}

static int mpcore_set_dscr_bits(struct jtag_intf *ji, u32 val, u32 mask)
{
	u32 dscr;
	u8 tdi[4];

	if (val & ~mask)
		DBG();

	if (mpcore_get_dscr(ji, &dscr))
		return -1;
	dscr &= ~mask;
	dscr |= val;

	tdi[0] = (dscr >>  0) & 0xff;
	tdi[1] = (dscr >>  8) & 0xff;
	tdi[2] = (dscr >> 16) & 0xff;
	tdi[3] = (dscr >> 24) & 0xff;

	return mpcore_ir_rw(ji, MPCORE_IR_EXTEST, NULL, tdi, 32);
}

#define HALT_MODE_EN_OFFS	14
int mpcore_haltmode_debug_control(struct jtag_intf *ji, int en)
{
	u32 mask, val;

	mask = BIT(HALT_MODE_EN_OFFS);
	val = en ? BIT(HALT_MODE_EN_OFFS) : 0;
	return mpcore_set_dscr_bits(ji, val, mask);
}

#define ARM_INSTR_EN_OFFS	13
static int mpcore_arm_instr_enabled(struct jtag_intf *ji)
{
	u32 dscr;

	if (mpcore_get_dscr(ji, &dscr))
		return -1;
	return dscr & BIT(ARM_INSTR_EN_OFFS) ? 1 : 0;
}

static int arm_exec_control(struct jtag_intf *ji, u32 en)
{
	u32 dscr, mask;
	int rv;

	mask = BIT(ARM_INSTR_EN_OFFS);
	rv = mpcore_set_dscr_bits(ji, en ? BIT(ARM_INSTR_EN_OFFS) : 0, mask);
	if (rv) {
		DBG();
		return rv;
	}
	rv = mpcore_get_dscr(ji, &dscr);
	if (rv) {
		DBG();
		return rv;
	}
	rv = (((dscr >> ARM_INSTR_EN_OFFS) & 1) == (en & 1)) ? 0 : -1;
	if (rv)
		DBG();
	return rv;
}

#define SPSR_BIT		22
#define MSR(spsr, rm)		(COND_AL << 28		| \
				(spsr & 1) << SPSR_BIT	| \
				(rm & 0xf) << 0		| \
				0x0129f000 )
static int mpcore_move_reg_to_cpsr(struct jtag_intf *ji, u32 rd)
{
	return arm_exec(ji, MSR(0, rd));
}

#define MRS(spsr, rd)		(COND_AL << 28		| \
				(spsr & 1) << SPSR_BIT	| \
				(rd & 0xf) << 12	| \
				0x010f0000 )
static int mpcore_move_cpsr_to_reg(struct jtag_intf *ji, u32 rd)
{
	return arm_exec(ji, MRS(0, rd));
}

static int coproc_set(struct jtag_intf *ji, int coproc_num,
		      int crn, int op1, int crm, int op2, u32 val)
{
	if (mpcore_set_register(ji, REG(9), val))
		return -1;

	return arm_exec(ji, MCR(coproc_num, op1, crn, REG(9), op2, crm));
}

static int coproc_get(struct jtag_intf *ji, int coproc_num,
		      int crn, int op1, int crm, int op2, u32 *val)
{
	if (arm_exec(ji, MRC(coproc_num, op1, crn, REG(9), op2, crm)))
		return -1;

	return mpcore_get_register(ji, REG(9), val);
}

static int cp_set_bits(struct jtag_intf *ji, int coproc_num,
		       int crn, int op1, int crm, int op2, u32 mask, u32 val)
{
	u32 v;

	if (coproc_get(ji, coproc_num, crn, op1, crm, op2, &v))
		return -1;

	v &= ~mask;
	v |= val & mask;

	return coproc_set(ji, coproc_num, crn, op1, crm, op2, v);
}

int tlb_debug = 0;
static int tlb_debug_control(struct jtag_intf *ji, int enable)
{
	return cp_set_bits(ji, 15, CRn(15), OP1(7), CRm(1), OP2(0),
			   (3 << 4), (enable ? 0 : 3) << 4);
}

static int drain_write_buffer(struct jtag_intf *ji)
{
	u32 dscr;

	/* data sync barrier */
	for ( ;; ) {
		if (arm_exec(ji, MCR(15, 0, CRn(7), REG(0), 4, CRm(10))))
			return -1;
		if (mpcore_get_dscr(ji, &dscr))
			return -1;
		if (!(dscr & BIT(7)))
			break;
	}
	if (arm_exec(ji, ARM_NOP()))
		return -1;
	if (mpcore_get_dscr(ji, &dscr))
		return -1;
	return 0;
}

static int mpcore_get_cpsr(struct jtag_intf *ji, u32 *cpsr)
{
	if (mpcore_move_cpsr_to_reg(ji, REG(9))) {
		DBG();
		return -1;
	}
	if (mpcore_get_register(ji, REG(9), cpsr)) {
		DBG();
		return -1;
	}
	return 0;
}

static int mpcore_set_cpsr(struct jtag_intf *ji, u32 cpsr)
{
	if (mpcore_set_register(ji, REG(9), cpsr)) {
		DBG();
		return -1;
	}
	if (mpcore_move_reg_to_cpsr(ji, REG(9))) {
		DBG();
		return -1;
	}
	return 0;
}

static int mpcore_get_pc(struct jtag_intf *ji, u32 *pc)
{
	if (arm_exec(ji, ARM_MOV(REG(9), REG(15)))) {
		DBG();
		return -1;
	}
	if (mpcore_get_register(ji, REG(9), pc)) {
		DBG();
		return -1;
	}
	return 0;
}

/******************************************************************************/

static struct mpcore_state {
	u32 regs[16];
	u32 cpsr;

	u32 mainid;
	u32 cachetype;
	u32 tlbtype;
	u32 cpuid;
	u32 features;

	u32 ctrl;
	u32 auxctrl;

	u32 dfsr;
	u32 ifsr;
	u32 far;
	u32 wfar;

	u32 dscr;
	u32 wdtr;
	u32 rdtr;

	uint64_t didr;

	u32 debug_trace_hangpoint;
	u32 debug_trace_hangpoint_valid;
} mpstate;

static char *regdesc(int reg)
{
	switch (reg) {
	case 13: return "(sp)";
	case 14: return "(lr)";
	case 15: return "(pc)";
	}
	return "";
}

int display_registers(struct mpcore_state *s)
{
	int i;

	printf("REGISTERS:\n ");
	for (i = 0; i < DIM(s->regs); i++) {
		if (i && !(i % 4))
			printf("\n ");
		if (i == 13)
			WHITE_TXT();
		printf("%08x ", s->regs[i]);
	}
	printf("\n    	    %s     %s     %s\n",
		regdesc(13), regdesc(14), regdesc(15));
	RESET_TXT_CLR();
	printf("dfsr:0x%08x\nifsr:0x%08x\nwfar:0x%08x\n",
		s->dfsr, s->ifsr, s->wfar);
	return 0;
}

static int display_current_instruction(struct jtag_intf *ji)
{
	struct mpcore_state *s = &mpstate;
	int thumb = s->cpsr & BIT(PSR_THUMB);
	u32 offs, val;

	offs = s->regs[15];

	if (mpcore_rd32(ji, offs & ~3, &val))
		DBG();

	if (thumb) {
		if (offs & 3)
			val >>= 16;
		val &= 0xffff;
	}

	WHITE_TXT();
	printf("INSTR: 0x%0*x\n", thumb ? 4 : 8, val);
	RESET_TXT_CLR();
	fflush(NULL);
	return 0;
}

static int mpcore_restore_state(struct jtag_intf *ji, struct mpcore_state *s)
{
	u32 i;

	if (mpcore_arm_instr_enabled(ji) == 1)
		arm_exec_control(ji, 1);

	if ((s->cpsr & 0x1f) != (16 + 3)) {
		s->cpsr &= ~0x1f;
		s->cpsr |= (16 + 3);
	}

	if (mpcore_set_cpsr(ji, s->cpsr)) {
		DBG();
		return -1;
	}

	for (i = 1; i < DIM(s->regs) - 1; i++)
		if (mpcore_set_register(ji, i, s->regs[i])) {
			DBG();
			return -1;
		}

	if (s->wdtr != 0x0badf00d) {
		if (coproc_set(ji, 14, CRn(0), OP1(0), CRm(5), OP2(0),
			       s->wdtr != 0x0badf00d ? s->wdtr : 0)) {
			DBG();
			return -1;
		}
	}

	if (mpcore_set_register(ji, REG(0), s->regs[15])) {
		DBG();
		return -1;
	}

	if (arm_exec(ji, ARM_MOV(REG(15), REG(0)))) {
		DBG();
		return -1;
	}

	if (mpcore_set_register(ji, REG(0), s->regs[0])) {
		DBG();
		return -1;
	}

	if (tlb_debug && tlb_debug_control(ji, 1)) {
		DBG();
		return -1;
	}

	if (s->rdtr != 0x0badf00d) {
		if (mpcore_write_dtr(ji, &s->rdtr, 1)) {
			DBG();
			return -1;
		}
	}

	if (mpcore_set_dscr_bits(ji, s->dscr, 0xffffffff)) {
		DBG();
		return -1;
	}

	arm_exec_control(ji, 0);
	return 0;
}

static int mpcore_save_state(struct jtag_intf *ji, struct mpcore_state *s)
{
	static int first_halt = 1;
	u32 i;

	if (mpcore_get_dscr(ji, &s->dscr))
		return -1;

	/* save wDTR if valid */
	if (!(s->dscr & BIT(DSCR_WDTRFULL)) || mpcore_get_wdtr(ji, &s->wdtr))
		s->wdtr = 0x0badf00d;

	arm_exec_control(ji, 1);

	if (drain_write_buffer(ji))
		return -1;

	if (mpcore_get_register(ji, 0, &s->regs[0]))
		return -1;

	if (s->dscr & BIT(DSCR_RDTRFULL)) {
		if (mpcore_ldr_from_dtr(ji, 0)) {
			DBG();
			return -1;
		}
		if (mpcore_get_register(ji, 0, &s->rdtr)) {
			DBG();
			return -1;
		}
		DBG("rdtr: %08x", s->rdtr);
		if (!quiet)
			printf("rDTR : 0x%08x\n", s->rdtr);
	} else {
		s->rdtr = 0x0badf00d;
	}

	if (mpcore_get_cpsr(ji, &s->cpsr)) {
		DBG();
		return -1;
	}

	if (mpcore_get_pc(ji, &s->regs[15])) {
		DBG();
		return -1;
	}

	if (!(s->cpsr & BIT(PSR_JAZELLE)))
		s->regs[15] -= (s->cpsr & BIT(PSR_THUMB)) ? 4 : 8;

	for (i = 1; i < DIM(s->regs) - 1; i++)
		if (mpcore_get_register(ji, i, &s->regs[i]))
			return -1;

	if (first_halt) {
		first_halt = 0;
		if (coproc_get(ji, 15, CRn(0), OP1(0), CRm(0), OP2(0), &s->mainid))
				return -1;
		if (coproc_get(ji, 15, CRn(0), OP1(0), CRm(0), OP2(1), &s->cachetype))
				return -1;
		if (coproc_get(ji, 15, CRn(0), OP1(0), CRm(0), OP2(3), &s->tlbtype))
				return -1;
		if (coproc_get(ji, 15, CRn(0), OP1(0), CRm(0), OP2(5), &s->cpuid))
				return -1;

		if (!quiet) {
			printf("\n");
			display_mainid(s->mainid);
			display_cachetype(s->cachetype);
			display_tlbtype(s->tlbtype);
			display_cpuid(s->cpuid);
			printf("\n");
		}
	}

	if (coproc_get(ji, 15, CRn(1), OP1(0), CRm(0), OP2(0), &s->ctrl))
			return -1;
	if (coproc_get(ji, 15, CRn(1), OP1(0), CRm(0), OP2(1), &s->auxctrl))
			return -1;

	if (coproc_get(ji, 15, CRn(5), OP1(0), CRm(0), OP2(0), &s->dfsr))
			return -1;
	if (coproc_get(ji, 15, CRn(5), OP1(0), CRm(0), OP2(1), &s->ifsr))
			return -1;
	if (coproc_get(ji, 15, CRn(6), OP1(0), CRm(0), OP2(1), &s->wfar))
			return -1;

	if (tlb_debug && tlb_debug_control(ji, 0)) {
		DBG();
		return -1;
	}

	if (!quiet) {
		display_dscr(s->dscr);
		printf("\n");
		display_ctrl(s->ctrl);
		display_auxctrl(s->auxctrl);
		printf("\n");
		display_registers(s);
		display_psr(s->cpsr);
		display_current_instruction(ji);
	}

	return 0;
}

static int mpcore_restart(struct jtag_intf *ji)
{
	u32 dscr;

	if (mpcore_ir_rw(ji, MPCORE_IR_RESTART, NULL, NULL, 0))
		return -1;
	if (mpcore_get_dscr(ji, &dscr))
		return -1;
	if (dscr & 1) {
		if (mpcore_save_state(ji, &mpstate)) {
			DBG();
			return -1;
		}
	}
	return (dscr & 1) ? -1 : 0;
}

char *debug_board_amda;

static int mpcore_continue(struct jtag_intf *ji, int done)
{
	u32 v;

	for ( ;!get_dtr(ji, &v); )
		;

	if (mpcore_restore_state(ji, &mpstate)) {
		DBG();
		return -1;
	}

	if (done)
		if (mpcore_haltmode_debug_control(ji, 0)) {
			DBG();
			return -1;
		}

	if (mpcore_restart(ji)) {
		DBG();
		return -1;
	}

	return 0;
}

static int mpcore_halt(struct jtag_intf *ji, int ms_reset_delay)
{
	if (ms_reset_delay >= 0) {
		if (dbgboard_reset(debug_board_amda))
			DBG();
		printf("reset debug board %s (%u ms delay)\n", debug_board_amda,
				ms_reset_delay);
		if (ms_reset_delay)
			usleep(1000 * ms_reset_delay);
	}

	if (mpcore_haltmode_debug_control(ji, 1)) {
		DBG();
		return -1;
	}

	if (mpcore_stop(ji)) {
		DBG();
		return -1;
	}

	if (mpcore_save_state(ji, &mpstate)) {
		DBG();
		return -1;
	}

	return 0;
}

#define THUMB_SWI(IMMED8)	(0xdf00 | (IMMED8 & 0xff))
#define THUMB_MOV3(Rm, Rd)	(0x4600 | ((Rd & 4) << 3) | ((Rd & 7) << 0) \
					  ((Rm & 4) << 4) | ((Rm & 7) << 3))
#define THUMB_NOP()		(0x46c0) //THUMB_MOV(REG(8), REG(8))
#define THUMB_BRKP(x)		(0xbe00 | (x & 0xff))
#define THUMB_LDR1(IMMED5, Rn, Rd)	(0x6800 | (IMMED5 & 0x1f) << 6 | \
						(Rn & 7) << 3 | Rd & 7)
#define THUMB_STR1(IMMED5, Rn, Rd)	(0x6000 | (IMMED5 & 0x1f) << 6 | \
						(Rn & 7) << 3 | (Rd & 7))
#define THUMB_LSL1(IMMED5, Rm, Rd)	(0x0000 | (IMMED5 & 0x1f) << 6 | \
						(Rm & 7) << 3 | (Rd & 7))
#define THUMB_MOV1(Rd, IMMED8)	(0x2000 | ((Rd & 7) << 8) | (IMMED8 & 0xff))
#define THUMB_BAL(offs)		(0xe000 | ((offs >> 1) & 0x7ff))
#define THUMB_HANG()		THUMB_BAL(-4)
#define THUMB_BX(Rm)		(0x4700 | (Rm & 0xf) << 3)
#define THUMB_ORR(Rn, Rd)	(0x4300 | (Rn & 7) << 3 | (Rd & 7))
#define THUMB_ADD2(Rd, IMMED8)	(0x3000 | (Rd & 7) << 8 | (IMMED8 & 0xff))

#define BYTE(w, b)  (((w) >> ((b) * 8)) & 0xff)
static int thumb_ldr_r0_opcodes(struct jtag_intf *ji, u32 offs, u32 val)
{
	u16 opcodes[] = {
		THUMB_MOV1( REG(0), 0),
		THUMB_MOV1( REG(1), BYTE(val, 3)),
		THUMB_LSL1(    24,  REG(1),   REG(0)),
		THUMB_MOV1( REG(1), BYTE(val, 2)),
		THUMB_LSL1(    16,  REG(1),   REG(1)),
		THUMB_ORR(  REG(1), REG(0)),
		THUMB_MOV1( REG(1), BYTE(val, 1)),
		THUMB_LSL1(     8,  REG(1),   REG(1)),
		THUMB_ORR(  REG(1), REG(0)),
		THUMB_MOV1( REG(1), BYTE(val, 0)),
		THUMB_ORR(  REG(1), REG(0)),
		THUMB_BX(   REG(0)),
		THUMB_HANG(),
		THUMB_HANG(),
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(opcodes); i += 2, offs += 4) {
		if (mpcore_wr32(ji, offs, opcodes[i] | opcodes[i + 1] << 16)) {
			DBG();
			return -1;
		}
	}
	return 0;
}

static int overwrote_prev_exec(u32 offs, u32 size)
{
	struct mpcore_state *s = &mpstate;

	if (s->regs[15] >= offs && s->regs[15] < (offs + size))
		return 1;
	return 0;
}

static int set_pc(struct jtag_intf *ji, u32 offs, u32 size)
{
	struct mpcore_state *s = &mpstate;
	u32 thumb2arm_offs;

	if (s->cpsr & BIT(PSR_THUMB)) {
		thumb2arm_offs = (offs + size + 7) & ~7;
		if (thumb_ldr_r0_opcodes(ji, thumb2arm_offs, offs)) {
			DBG();
			return -1;
		}
		offs = thumb2arm_offs;
	}
	s->regs[15] = offs;
	return 0;
}

static int clean_entire_data_cache(struct jtag_intf *ji)
{
	return coproc_set(ji, 15, CRn(7), OP1(0), CRm(14), OP2(0), 0);
}

static int invalidate_entire_instr_cache(struct jtag_intf *ji)
{
	return coproc_set(ji, 15, CRn(7), OP1(0), CRm(5), OP2(0), 0);
}

static int disable_mmu_and_caches(struct jtag_intf *ji)
{
	if (clean_entire_data_cache(ji)) {
		DBG();
		return -1;
	}

	usleep(1 * 1000);

	if (cp_set_bits(ji, 15, CRn(1), OP1(0), CRm(0), OP2(0), CR_I | CR_C, 0)) {
		//DBG();
		return -1;
	}

	if (invalidate_entire_instr_cache(ji)) {
		DBG();
		return -1;
	}

	usleep(1 * 1000);

	if (cp_set_bits(ji, 15, CRn(1), OP1(0), CRm(0), OP2(0), CR_M, 0)) {
		DBG();
		return -1;
	}

	return 0;
}

/******************************************************************************/

struct jtag_prog_intf_data {
	u32 scan_chain_index;
	struct jtag_tap_info *taps;
	u32 tap_count;
	u32 matching_probed_tap_count;
};

struct prgintf {
	void *intf;
	void *data;
	struct jtag_ctrl_intf *jci;
};

static int jtag_probe_callback(void *context, struct jtag_tap_info *ti)
{
	u32 tap_idx;
	struct jtag_prog_intf_data *id = (struct jtag_prog_intf_data *)context;

	tap_idx = id->matching_probed_tap_count;

	if (id->taps[tap_idx].idcode != ti->idcode) {
		DBG("0x%08x != 0x%08x", id->taps[tap_idx].idcode, ti->idcode);
		return -1;
	}
	fprintf(stdout, "found expected tap %d ID:0x%08x\n", tap_idx, ti->idcode);

	ti->desc = strdup(id->taps[tap_idx].desc);
	ti->chain_index = id->taps[tap_idx].chain_index;
	ti->ir_sz = id->taps[tap_idx].ir_sz;
	ti->bs_sz = id->taps[tap_idx].bs_sz;

	id->matching_probed_tap_count++;
	return 0;
}

static int jtag_add_expected_tap(struct jtag_prog_intf_data *pi, u32 index,
				 u32 ir_sz, u32 idcode, char *desc)
{
	struct jtag_tap_info *t;

	pi->tap_count++;
	pi->taps = realloc(pi->taps, pi->tap_count * sizeof(*pi->taps));
	if (!pi->taps) {
		DBG();
		return -1;
	}
	t = &pi->taps[pi->tap_count - 1];
	t->chain_index = index;
	t->ir_sz  = ir_sz;
	t->idcode = idcode;
	t->desc   = strdup(desc);
	return 0;
}

static struct prgintf * init_jtag_intf(char *serial_num, u32 hz)
{
	struct prgintf *pi = NULL;
	struct jtag_ctrl_intf *jci = NULL;
	struct jtag_intf *ji = NULL;
	struct jtag_prog_intf_data *id = NULL;

	jci = nic_dbgboard_jtag_init(serial_num, hz, 0x100);
	if (!jci) {
		DBG();
		goto exit_err;
	}

	pi = malloc(sizeof(*pi));
	if (!pi) {
		DBG();
		goto exit_err;
	}

	id = malloc(sizeof(*id));
	if (!id) {
		DBG();
		goto exit_err;
	}
	id->taps = NULL;
	id->tap_count = 0;
	id->matching_probed_tap_count = 0;
	id->scan_chain_index = 0;

	jtag_add_expected_tap(id,
		0, MP11_IR_LEN, ARM_MPCORE11_IDCODE, "ARM MPCORE11");

	ji = jtag_intf_init(jci, jtag_probe_callback, id);
	if (!ji) {
		DBG();
		goto exit_err;
	}
	if (id->matching_probed_tap_count != id->tap_count) {
		DBG();
		goto exit_err;
	}
	ji->set_current_tap(ji, 0);

	pi->intf = ji;
	pi->data = id;
	pi->jci = jci;
	return pi;

exit_err:
	if (id)  free(id);
	if (pi)  free(pi);
	if (jci) nic_dbgboard_jtag_deinit(jci);
	if (ji)  jtag_intf_free(ji);
	return NULL;
}

static void free_jtag_intf(struct prgintf *pi)
{
	nic_dbgboard_jtag_deinit(pi->jci);
	jtag_intf_free(pi->intf);
	free(pi->data);
	free(pi);
}

#define MIN_JTAG_HZ		(1000)
#define MAX_JTAG_HZ		(50 * 1000 * 1000)
#define DEFAULT_JTAG_HZ		(7 * 1000 * 1000)
static u32 jtag_frequency(void)
{
	const char *s;
	int hz;

	s = getenv("JTAG_ARMBIN_LOAD_HZ");
	if (!s)
		return DEFAULT_JTAG_HZ;

	hz = strtoul(s, NULL, 0);
	if (hz < MIN_JTAG_HZ || hz > MAX_JTAG_HZ)
		hz = DEFAULT_JTAG_HZ;

	return hz;
}

static void print_version_info(void)
{
	printf( "date : %s\n"
		"host : %s\n"
		"user : %s\n"
		"mach : %s\n"
		"kern : %s\n"
		"dist : %s\n"
		"  cc : %s\n"
		"\n"
		"git rev: %s\n"
		,DATESTR
		,HOSTNAME
		,USERNAME
		,MACH
		,KERN
		,DISTVER
		,CCVER
		,GITREV
		);
}

int main(int argc, char *argv[])
{
	char *program = argv[0];
	struct filebuf *armbin;
	struct jtag_intf *ji;
	struct prgintf *pi;
	const char *fname;
	int c, option;
	int reset = -1;
	int exec = 0;
	u32 offs;
	int ret;
	int hz;

	const struct option options[] = {
		{.name = "version", .has_arg = 0, .flag = NULL, .val = 'v'},
		{.name = "exec", .has_arg = 0, .flag = NULL, .val = 'e'},
		{.name = "reset", .has_arg = 1, .flag = NULL, .val = 'r'},
	};

	if (argc == 1) {
		mpsse_list();
		return 0;
	}

	for ( ;; ) {
		c = getopt_long(argc, argv, "ver:", options, &option);
		if (c < 0)
			break;

		switch (c) {
		case 'v':
			print_version_info();
			exit(0);
			break;
		case 'e':
			exec = 1;
			break;
		case 'r':
			reset = strtoul(optarg, NULL, 0);
			break;
		default:
			DBG("%d (%c)", c, c);
			return -1;
		}
	}

	argc -= optind;
	argv += optind;

	if (argc != 3) {
		printf("usage: %s debug-board-serial-number address /path/to/arm.bin\n", program);
		return -1;
	}

	debug_board_amda = argv[0];

	offs = strtoul(argv[1], NULL, 0);

	fname = argv[2];
	armbin = loadfile(fname);
	if (!armbin)
		return -1;

	hz = jtag_frequency();
	printf("%d Hz\n", hz);

	pi = init_jtag_intf(debug_board_amda, hz);
	if (!pi) {
		printf("%s:%d %s()\n", __FILE__, __LINE__, __func__);
		return -1;
	}

	ji = pi->intf;

	if (mpcore_halt(ji, reset)) {
		usleep(1000 * 200);
		if (mpcore_halt(ji, 30)) {
			printf(" halt failed\n");
			goto exit;
		}
	}

	if (getenv("PROG_MMU")) {
		quiet_mmu_disable = 1;
		if (disable_mmu_and_caches(ji)) {
			printf("%s:%d %s()\n", __FILE__, __LINE__, __func__);
			usleep(1000 * 200);
			if (mpcore_halt(ji, 30)) {
				printf(" halt failed\n");
				goto exit;
			}
		}
	}

	printf("transferring \"%s\" %u bytes to address 0x%x...\n", fname,
			armbin->size, offs);
	fflush(stdout);
	ret = transfer(ji, offs, armbin->ptr, armbin->size);
	if (ret < 0) {
		printf(" transfer failed\n");
		goto exit;
	}
	printf(" %u ms\n", ret);

	if (exec)
		set_pc(ji, offs, armbin->size);


	if (exec || (!exec && !overwrote_prev_exec(offs, armbin->size)))
		mpcore_continue(ji, 1);

exit:
	free_jtag_intf(pi);
	return 0;
}
