#ifndef SIM_IO_MSGIDS_H
#define SIM_IO_MSGIDS_H

/*
** SIM_IO message IDs (default non-EDS encoding: cmd = 0x1800|topic, tlm = 0x0800|topic).
**
** Topic allocation for the docking mission apps:
**   0x93-0x95  gnc_app
**   0x96-0x98  sim_io   (0x98 = GNC actuator command, published by gnc_app and
**                        consumed here — the "vehicle actuator interface")
**   0x99       reserved for the RCS/actuator-management app (realism roadmap phase 2)
*/

/* Commands / scheduling received by sim_io */
#define SIM_IO_CMD_MID     0x1896 /* ground commands                     */
#define SIM_IO_SEND_HK_MID 0x1897 /* SCH_LAB 1 Hz housekeeping request   */

/* Messages published by sim_io */
#define SIM_IO_HK_TLM_MID    0x0896 /* housekeeping                                     */
#define SIM_IO_SIM_STATE_MID 0x0897 /* one per Unity GNC cycle — also the GNC wakeup    */

/* Published by gnc_app, consumed by sim_io and forwarded to Unity */
#define SIM_IO_WRENCH_CMD_MID 0x0898

#endif /* SIM_IO_MSGIDS_H */
