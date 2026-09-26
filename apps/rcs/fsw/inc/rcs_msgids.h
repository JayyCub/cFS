#ifndef RCS_MSGIDS_H
#define RCS_MSGIDS_H

/*
** RCS (reaction control system manager) message IDs — topics 0x99 and 0xA2
** (allocation table in sim_io_msgids.h).
*/

/* Commands / scheduling received by rcs */
#define RCS_CMD_MID     0x1899 /* ground commands                   */
#define RCS_SEND_HK_MID 0x18A2 /* SCH_LAB 1 Hz housekeeping request */

/* Messages published by rcs (THRUSTER_CMD is SIM_IO_THRUSTER_CMD_MID) */
#define RCS_HK_TLM_MID 0x0899

/* Published by gnc_app, consumed by rcs: the per-cycle impulse request */
#define RCS_ACT_REQ_MID 0x08A2

#endif /* RCS_MSGIDS_H */
