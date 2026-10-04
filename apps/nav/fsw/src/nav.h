#ifndef NAV_H
#define NAV_H

/*
** NAV — relative navigation.
**
** Turns raw sensor readings (SIM_IO SIM_STATE: IMU, star tracker, LIDAR) into
** the navigation solution GNC flies on (NAV_SOLUTION), one per lock-step
** cycle:
**
**   SIM_IO --SIM_STATE--> NAV --NAV_SOLUTION--> GNC --ACT_REQ--> RCS --THRUSTER_CMD--> SIM_IO
**
** Before realism phase 4, Unity computed range, closing speed, lateral offset
** and attitude errors from truth and sent them to GNC directly. The estimation
** itself is in nav_filter.c (no cFE dependencies); this file is the cFS app
** around it: SB plumbing, the configuration table, events, telemetry.
*/

#include "cfe.h"
#include "nav_msgids.h"
#include "nav_msg.h"
#include "nav_tbl.h"
#include "nav_filter.h"
#include "sim_io_msgids.h"
#include "sim_io_msg.h"

#define NAV_PIPE_NAME  "NAV_PIPE"
#define NAV_PIPE_DEPTH 16

/* Same cycle-length sanity range GNC applies */
#define NAV_MIN_CYCLE_DT_S 0.01f
#define NAV_MAX_CYCLE_DT_S 1.0f

/* Event IDs */
#define NAV_INIT_INF_EID      1
#define NAV_PIPE_ERR_EID      2
#define NAV_TBL_ERR_EID       3
#define NAV_TBL_VAL_ERR_EID   4
#define NAV_NOOP_INF_EID      5
#define NAV_RESET_INF_EID     6
#define NAV_CMD_ERR_EID       7
#define NAV_MSG_ERR_EID       8
#define NAV_FILTER_INF_EID    9  /* filter initialised from the LIDAR               */
#define NAV_FILTER_ERR_EID    10 /* divergence recovery / non-finite state reset    */
#define NAV_RELNAV_INF_EID    11 /* relative nav became valid / invalid             */
#define NAV_ATT_INF_EID       12 /* attitude became valid / invalid                 */
#define NAV_RPS_INF_EID       13 /* LIDAR acquired / lost the target                */
#define NAV_SENSOR_ERR_EID    14 /* malformed sensor reading discarded              */
#define NAV_BAD_DT_EID        15 /* SIM_STATE CycleDt_s out of range                */

#define NAV_TBL_VALIDATE_ERR (0xc0000013)

typedef struct
{
    uint32          RunStatus;
    CFE_SB_PipeId_t CmdPipe;
    NAV_HkTlm_t     HkTlm;

    CFE_TBL_Handle_t CfgTblHandle;

    NAV_State_t       State;
    NAV_SolutionMsg_t SolutionMsg;

    /* Previous cycle's status and events, for transition-only EVS messages */
    uint32 PrevStatus;
    bool   PrevRpsAvailable;
    bool   SensorRejectLatched;
} NAV_Data_t;

extern NAV_Data_t NAV_Data;

void         NAV_Main(void);
CFE_Status_t NAV_Init(void);

#endif /* NAV_H */
