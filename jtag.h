/* Copyright (C) 2020 David Brunecz. Subject to GPL 2.0 */

#ifndef __JTAG_H__
#define __JTAG_H__
/******************************************************************************/

struct jtag_intf * jtag_intf_init(struct jtag_ctrl_intf *,
                                int (*probe_cb)(void *, struct jtag_tap_info *),
                                void *probe_ctxt);
void               jtag_intf_free(struct jtag_intf *);

/******************************************************************************/
#endif // __JTAG_H__
