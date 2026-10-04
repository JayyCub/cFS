#ifndef SIM_IO_MSGIDS_H
#define SIM_IO_MSGIDS_H

/*
** SIM_IO message IDs (default non-EDS encoding: cmd = 0x1800|topic, tlm = 0x0800|topic).
**
** Topic allocation for the docking mission apps:
**   0x93-0x95  gnc_app
**   0x96-0x98  sim_io   (0x98 = THRUSTER_CMD, published by the rcs app and
**                        consumed here — the "valve driver interface")
**   0x99,0xA2  rcs      (see rcs_msgids.h)
**   0xD0-0xD1  nav      (see nav_msgids.h)
*/

/* Commands / scheduling received by sim_io */
#define SIM_IO_CMD_MID     0x1896 /* ground commands                     */
#define SIM_IO_SEND_HK_MID 0x1897 /* SCH_LAB 1 Hz housekeeping request   */

/* Messages published by sim_io */
#define SIM_IO_HK_TLM_MID    0x0896 /* housekeeping                                     */
#define SIM_IO_SIM_STATE_MID 0x0897 /* one per Unity GNC cycle — also the NAV wakeup    */

/* Published by the rcs app, consumed by sim_io (forwarded to Unity) and by
** gnc_app (actuator-health check against the impulse actually commanded) */
#define SIM_IO_THRUSTER_CMD_MID 0x0898

#endif /* SIM_IO_MSGIDS_H */
