#ifndef RCS_H
#define RCS_H

/*
** RCS — reaction control system manager.
**
** Flight-software half of the thruster chain: GNC publishes WHAT impulse it
** wants each cycle (RCS_ACT_REQ); this app decides HOW to get it:
**   1. allocation — non-negative least squares over the enabled thrusters in the
**      RCS table (unidirectional thrusters: push only)
**   2. PWM        — convert each thruster's impulse into a valve on-time at rated
**      thrust, scale the whole solution down if any pulse would overrun the
**      cycle, and enforce the minimum impulse bit
** and publishes the on-times as SIM_IO_THRUSTER_CMD for the valve drivers
** (SIM_IO -> Unity). Unity only opens and closes valves.
*/

#include "cfe.h"
#include "rcs_msgids.h"
#include "rcs_msg.h"
#include "rcs_tbl.h"
#include "sim_io_msgids.h"
#include "sim_io_msg.h"

#define RCS_PIPE_NAME  "RCS_PIPE"
#define RCS_PIPE_DEPTH 16

/* Event IDs */
#define RCS_INIT_INF_EID    1
#define RCS_PIPE_ERR_EID    2
#define RCS_TBL_ERR_EID     3
#define RCS_TBL_VAL_ERR_EID 4
#define RCS_NOOP_INF_EID    5
#define RCS_RESET_INF_EID   6
#define RCS_CMD_ERR_EID     7
#define RCS_REQ_ERR_EID     8

#define RCS_TBL_VALIDATE_ERR (0xc0000012)

/* Result of one allocation + PWM pass */
typedef struct
{
    float  OnTime_s[RCS_MAX_THRUSTERS];
    float  AchievedLin_Ns[3];
    float  AchievedAng_Nms[3];
    float  ResidualNorm;
    float  MaxOnTime_s;
    bool   Saturated;
    uint32 Drops;
    uint32 Rounds;
    uint16 FiredMask;
} RCS_AllocResult_t;

typedef struct
{
    uint32          RunStatus;
    CFE_SB_PipeId_t CmdPipe;
    RCS_HkTlm_t     HkTlm;

    CFE_TBL_Handle_t ThrTblHandle;

    SIM_IO_ThrusterCmdMsg_t ThrCmdMsg;
} RCS_Data_t;

extern RCS_Data_t RCS_Data;

void         RCS_Main(void);
CFE_Status_t RCS_Init(void);

/* rcs_alloc.c */
void  RCS_Allocate(const RCS_ThrTbl_t *Tbl, const float Lin[3], const float Ang[3], RCS_AllocResult_t *Out);
int32 RCS_ValidateThrTbl(void *TblPtr);

#endif /* RCS_H */
