#include <string.h>
#include "rcs.h"

RCS_Data_t RCS_Data;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static bool RCS_VerifyLength(const CFE_MSG_Message_t *MsgPtr, size_t Expected)
{
    size_t Actual = 0;
    CFE_MSG_GetSize(MsgPtr, &Actual);
    return Actual == Expected;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* One GNC request -> allocation + PWM -> THRUSTER_CMD                      */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void RCS_ProcessActReq(const RCS_ActReqMsg_t *Req)
{
    SIM_IO_ThrusterCmdMsg_t *Out = &RCS_Data.ThrCmdMsg;
    RCS_ThrTbl_t            *Tbl = NULL;
    RCS_AllocResult_t        Res;
    CFE_Status_t             Status;
    uint32                   i;

    /* Hold the table only for the duration of this solve so a ground table
    ** load can be activated by CFE_TBL_Manage between cycles. */
    Status = CFE_TBL_GetAddress((void **)&Tbl, RCS_Data.ThrTblHandle);
    if (Status != CFE_SUCCESS && Status != CFE_TBL_INFO_UPDATED)
    {
        CFE_EVS_SendEvent(RCS_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "RCS: no thruster table (RC=0x%08lX) — commanding all valves closed",
                          (unsigned long)Status);
        memset(&Res, 0, sizeof(Res));
    }
    else
    {
        RCS_Allocate(Tbl, Req->LinImpulse_Ns, Req->AngImpulse_Nms, &Res);
        CFE_TBL_ReleaseAddress(RCS_Data.ThrTblHandle);
    }

    RCS_Data.HkTlm.RequestCount++;
    if (Res.Saturated)
        RCS_Data.HkTlm.SaturatedCount++;
    RCS_Data.HkTlm.MinImpulseDrops += Res.Drops;
    RCS_Data.HkTlm.MinImpulseRounds += Res.Rounds;
    RCS_Data.HkTlm.LastResidualNorm = Res.ResidualNorm;
    RCS_Data.HkTlm.LastMaxOnTime_s  = Res.MaxOnTime_s;
    RCS_Data.HkTlm.LastFiredMask    = Res.FiredMask;

    Out->Seq              = Req->Seq;
    Out->Cmd.NumThrusters = SIMLINK_NUM_THRUSTERS;
    Out->Cmd.GncPhase     = Req->GncPhase;
    for (i = 0; i < SIMLINK_NUM_THRUSTERS; i++)
        Out->Cmd.OnTime_s[i] = (i < RCS_MAX_THRUSTERS) ? Res.OnTime_s[i] : 0.0f;
    for (i = 0; i < 3; i++)
    {
        Out->AchievedLinImpulse_Ns[i]  = Res.AchievedLin_Ns[i];
        Out->AchievedAngImpulse_Nms[i] = Res.AchievedAng_Nms[i];
    }

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(Out->TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(Out->TelemetryHeader), true);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void RCS_ProcessGroundCmd(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t Fc = 0;
    CFE_MSG_GetFcnCode(&BufPtr->Msg, &Fc);

    if (!RCS_VerifyLength(&BufPtr->Msg, sizeof(RCS_NoArgsCmd_t)))
    {
        RCS_Data.HkTlm.CmdErrCount++;
        CFE_EVS_SendEvent(RCS_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "RCS: cmd length error, fc %u",
                          (unsigned int)Fc);
        return;
    }

    switch (Fc)
    {
        case RCS_NOOP_CC:
            RCS_Data.HkTlm.CmdCount++;
            CFE_EVS_SendEvent(RCS_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "RCS: NOOP");
            break;

        case RCS_RESET_COUNTERS_CC:
            RCS_Data.HkTlm.CmdCount         = 0;
            RCS_Data.HkTlm.CmdErrCount      = 0;
            RCS_Data.HkTlm.RequestCount     = 0;
            RCS_Data.HkTlm.SaturatedCount   = 0;
            RCS_Data.HkTlm.MinImpulseDrops  = 0;
            RCS_Data.HkTlm.MinImpulseRounds = 0;
            CFE_EVS_SendEvent(RCS_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "RCS: counters reset");
            break;

        default:
            RCS_Data.HkTlm.CmdErrCount++;
            CFE_EVS_SendEvent(RCS_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "RCS: invalid command code %u",
                              (unsigned int)Fc);
            break;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void RCS_SendHk(void)
{
    /* Table management happens between control cycles, never mid-solve */
    CFE_TBL_Manage(RCS_Data.ThrTblHandle);

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(RCS_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(RCS_Data.HkTlm.TelemetryHeader), true);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
CFE_Status_t RCS_Init(void)
{
    CFE_Status_t Status;

    memset(&RCS_Data, 0, sizeof(RCS_Data));
    RCS_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("RCS: CFE_EVS_Register failed, RC=0x%08lX\n", (unsigned long)Status);
        return Status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(RCS_Data.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(RCS_HK_TLM_MID),
                 sizeof(RCS_Data.HkTlm));
    CFE_MSG_Init(CFE_MSG_PTR(RCS_Data.ThrCmdMsg.TelemetryHeader), CFE_SB_ValueToMsgId(SIM_IO_THRUSTER_CMD_MID),
                 sizeof(RCS_Data.ThrCmdMsg));

    Status = CFE_SB_CreatePipe(&RCS_Data.CmdPipe, RCS_PIPE_DEPTH, RCS_PIPE_NAME);
    if (Status == CFE_SUCCESS)
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(RCS_ACT_REQ_MID), RCS_Data.CmdPipe);
    if (Status == CFE_SUCCESS)
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(RCS_SEND_HK_MID), RCS_Data.CmdPipe);
    if (Status == CFE_SUCCESS)
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(RCS_CMD_MID), RCS_Data.CmdPipe);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(RCS_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "RCS: pipe create/subscribe failed, RC=0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    Status = CFE_TBL_Register(&RCS_Data.ThrTblHandle, RCS_THR_TBL_NAME, sizeof(RCS_ThrTbl_t), CFE_TBL_OPT_DEFAULT,
                              RCS_ValidateThrTbl);
    if (Status == CFE_SUCCESS)
        Status = CFE_TBL_Load(RCS_Data.ThrTblHandle, CFE_TBL_SRC_FILE, RCS_THR_TBL_FILE);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(RCS_TBL_ERR_EID, CFE_EVS_EventType_ERROR, "RCS: thruster table register/load failed, RC=0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    CFE_EVS_SendEvent(RCS_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "RCS initialized: NNLS allocation + PWM over table %s", RCS_THR_TBL_FILE);
    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void RCS_Main(void)
{
    CFE_Status_t     Status;
    CFE_SB_Buffer_t *BufPtr;
    CFE_SB_MsgId_t   MsgId;

    if (RCS_Init() != CFE_SUCCESS)
        RCS_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;

    while (CFE_ES_RunLoop(&RCS_Data.RunStatus))
    {
        Status = CFE_SB_ReceiveBuffer(&BufPtr, RCS_Data.CmdPipe, CFE_SB_PEND_FOREVER);
        if (Status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(RCS_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "RCS: SB receive error 0x%08lX, exiting",
                              (unsigned long)Status);
            RCS_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
            continue;
        }

        CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId))
        {
            case RCS_ACT_REQ_MID:
                if (RCS_VerifyLength(&BufPtr->Msg, sizeof(RCS_ActReqMsg_t)))
                    RCS_ProcessActReq((const RCS_ActReqMsg_t *)BufPtr);
                else
                    CFE_EVS_SendEvent(RCS_REQ_ERR_EID, CFE_EVS_EventType_ERROR, "RCS: ACT_REQ length error");
                break;

            case RCS_SEND_HK_MID:
                RCS_SendHk();
                break;

            case RCS_CMD_MID:
                RCS_ProcessGroundCmd(BufPtr);
                break;

            default:
                break;
        }
    }

    CFE_ES_ExitApp(RCS_Data.RunStatus);
}
