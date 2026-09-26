#ifndef GNC_APP_MSGIDS_H
#define GNC_APP_MSGIDS_H

/*
** GNC App Message IDs
**
** cFS MID encoding for the default (non-EDS) pc-linux build:
**   Command MIDs  = 0x1800 | topic_id
**   Telemetry MIDs = 0x0800 | topic_id
**
** Topic IDs 0x93-0x95 are reserved for gnc_app (confirmed unused in this bundle).
*/

/* Command and scheduling MIDs (received by gnc_app) */
#define GNC_APP_CMD_MID     0x1893   /* ground commands to gnc_app         */
#define GNC_APP_SEND_HK_MID 0x1894   /* SCH_LAB 1 Hz HK request.
                                      ** The control cycle is NOT scheduled here —
                                      ** it runs on SIM_IO_SIM_STATE_MID (lock-step). */

/* Telemetry MIDs (published by gnc_app) */
#define GNC_APP_HK_TLM_MID    0x0893 /* 1 Hz housekeeping (GNC_APP_HkTlm_t)       */
#define GNC_APP_STATE_TLM_MID 0x0895 /* per-cycle GNC state (GNC_APP_StateTlm_t) */

#endif /* GNC_APP_MSGIDS_H */
