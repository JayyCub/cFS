#ifndef NAV_MSGIDS_H
#define NAV_MSGIDS_H

/*
** NAV (navigation) message IDs — topics 0xD0-0xD1 (allocation table in
** sim_io_msgids.h). Default non-EDS encoding: cmd = 0x1800|topic,
** tlm = 0x0800|topic.
*/

/* Commands / scheduling received by nav */
#define NAV_CMD_MID     0x18D0 /* ground commands                                  */
#define NAV_SEND_HK_MID 0x18D1 /* SCH_LAB 1 Hz housekeeping request (+ table manage) */

/* Messages published by nav */
#define NAV_HK_TLM_MID   0x08D0 /* 1 Hz housekeeping                                   */
#define NAV_SOLUTION_MID 0x08D1 /* one per SIM_STATE — the GNC cycle trigger (lock-step) */

#endif /* NAV_MSGIDS_H */
