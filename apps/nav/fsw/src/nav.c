#include <string.h>
#include "nav.h"

NAV_Data_t NAV_Data;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static bool NAV_VerifyLength(const CFE_MSG_Message_t *MsgPtr, size_t Expected)
{
    size_t Actual = 0;
    CFE_MSG_GetSize(MsgPtr, &Actual);
    return Actual == Expected;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* CFE_TBL validator: a bad image is rejected and the last good one kept    */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static int32 NAV_ValidateCfgTbl(void *TblPtr)
{
    const char *Why = NAV_CheckConfig((const NAV_CfgTbl_t *)TblPtr);

    if (Why != NULL)
    {
        CFE_EVS_SendEvent(NAV_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR, "NAV: table rejected — %s", Why);
        return NAV_TBL_VALIDATE_ERR;
    }
    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Transition-only events: what changed in the nav state this cycle         */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void NAV_ReportTransitions(const NAV_Output_t *Out, uint32 Seq)
{
    uint32 Changed = Out->Status ^ NAV_Data.PrevStatus;

    if (Out->Events & NAV_EVT_FILTER_INIT)
        CFE_EVS_SendEvent(NAV_FILTER_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "NAV: translation filter initialised from LIDAR at range %.2f m (Seq %lu)",
                          (double)Out->Range_m, (unsigned long)Seq);

    if (Out->Events & NAV_EVT_FILTER_REINIT)
        CFE_EVS_SendEvent(NAV_FILTER_ERR_EID, CFE_EVS_EventType_ERROR,
                          "NAV: filter disagreed with LIDAR (innovation %.1f sigma) — re-initialised (Seq %lu)",
                          (double)Out->InnovSigma, (unsigned long)Seq);

    if (Out->Events & NAV_EVT_SENSOR_REJECT)
    {
        if (!NAV_Data.SensorRejectLatched)
            CFE_EVS_SendEvent(NAV_SENSOR_ERR_EID, CFE_EVS_EventType_ERROR,
                              "NAV: malformed sensor reading discarded (Seq %lu)", (unsigned long)Seq);
        NAV_Data.SensorRejectLatched = true;
    }
    else
    {
        NAV_Data.SensorRejectLatched = false;
    }

    if (Changed & NAV_STATUS_ATT_VALID)
        CFE_EVS_SendEvent(NAV_ATT_INF_EID,
                          (Out->Status & NAV_STATUS_ATT_VALID) ? CFE_EVS_EventType_INFORMATION
                                                               : CFE_EVS_EventType_ERROR,
                          "NAV: attitude %s", (Out->Status & NAV_STATUS_ATT_VALID) ? "VALID" : "LOST");

    if (Out->RpsAvailable != NAV_Data.PrevRpsAvailable)
        CFE_EVS_SendEvent(NAV_RPS_INF_EID, CFE_EVS_EventType_INFORMATION,
                          Out->RpsAvailable ? "NAV: LIDAR tracking the target port" : "NAV: LIDAR lost the target port");

    if (Changed & NAV_STATUS_RELNAV_VALID)
        CFE_EVS_SendEvent(NAV_RELNAV_INF_EID,
                          (Out->Status & NAV_STATUS_RELNAV_VALID) ? CFE_EVS_EventType_INFORMATION
                                                                  : CFE_EVS_EventType_ERROR,
                          "NAV: relative nav %s — range %.2f m, sigma pos %.3f m vel %.4f m/s",
                          (Out->Status & NAV_STATUS_RELNAV_VALID) ? "VALID" : "INVALID", (double)Out->Range_m,
                          (double)Out->PosSigma_m, (double)Out->VelSigma_ms);

    NAV_Data.PrevStatus       = Out->Status;
    NAV_Data.PrevRpsAvailable = Out->RpsAvailable;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* One lock-step cycle: SIM_STATE in, NAV_SOLUTION out                      */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void NAV_ProcessSimState(const SIM_IO_SimStateMsg_t *Msg)
{
    NAV_SolutionMsg_t *Sol = &NAV_Data.SolutionMsg;
    NAV_CfgTbl_t      *Cfg = NULL;
    NAV_Output_t       Out;
    CFE_Status_t       Status;
    float              dt = Msg->State.CycleDt_s;

    memset(&Out, 0, sizeof(Out));
    NAV_Data.HkTlm.CycleCount++;

    Status = CFE_TBL_GetAddress((void **)&Cfg, NAV_Data.CfgTblHandle);
    if (Status != CFE_SUCCESS && Status != CFE_TBL_INFO_UPDATED)
    {
        CFE_EVS_SendEvent(NAV_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "NAV: no configuration table (RC=0x%08lX) — solution invalid", (unsigned long)Status);
    }
    else if (!(dt >= NAV_MIN_CYCLE_DT_S && dt <= NAV_MAX_CYCLE_DT_S))
    {
        CFE_EVS_SendEvent(NAV_BAD_DT_EID, CFE_EVS_EventType_ERROR,
                          "NAV: SIM_STATE Seq %lu CycleDt_s %g out of range — solution invalid",
                          (unsigned long)Msg->Seq, (double)dt);
        CFE_TBL_ReleaseAddress(NAV_Data.CfgTblHandle);
    }
    else
    {
        NAV_Step(&NAV_Data.State, Cfg, &Msg->State, &Out);
        CFE_TBL_ReleaseAddress(NAV_Data.CfgTblHandle);

        if (Out.Status & NAV_STATUS_RPS_USED)
            NAV_Data.HkTlm.RpsAccepted++;
        if (Out.Status & NAV_STATUS_RPS_REJECTED)
            NAV_Data.HkTlm.RpsRejected++;
        if (!Out.RpsAvailable)
            NAV_Data.HkTlm.RpsUnavailable++;
        if (Out.Events & (NAV_EVT_FILTER_INIT | NAV_EVT_FILTER_REINIT))
            NAV_Data.HkTlm.FilterInits++;
        if (Out.Events & NAV_EVT_SENSOR_REJECT)
            NAV_Data.HkTlm.SensorRejects++;
        if (Out.RpsAvailable)
            NAV_Data.HkTlm.LastInnovSigma = Out.InnovSigma;
    }

    NAV_ReportTransitions(&Out, Msg->Seq);

    Sol->SimTime_s = Msg->SimTime_s;
    Sol->Seq       = Msg->Seq;
    Sol->CycleDt_s = dt;
    Sol->Status    = Out.Status;
    memcpy(Sol->AttQuat_L, Out.AttQuat_L, sizeof(Sol->AttQuat_L));
    memcpy(Sol->AngRate_B, Out.AngRate_B, sizeof(Sol->AngRate_B));
    memcpy(Sol->AttErr_B, Out.AttErr_B, sizeof(Sol->AttErr_B));
    memcpy(Sol->CmPos_L, Out.CmPos_L, sizeof(Sol->CmPos_L));
    memcpy(Sol->CmVel_L, Out.CmVel_L, sizeof(Sol->CmVel_L));
    memcpy(Sol->PortPos_D, Out.PortPos_D, sizeof(Sol->PortPos_D));
    memcpy(Sol->PortVel_D, Out.PortVel_D, sizeof(Sol->PortVel_D));
    Sol->Range_m         = Out.Range_m;
    Sol->ClosingSpeed_ms = Out.ClosingSpeed_ms;
    Sol->LateralOffset_m = Out.LateralOffset_m;
    memcpy(Sol->DeltaV_B, Out.DeltaV_B, sizeof(Sol->DeltaV_B));
    Sol->PosSigma_m  = Out.PosSigma_m;
    Sol->VelSigma_ms = Out.VelSigma_ms;

    NAV_Data.HkTlm.Status      = Out.Status;
    NAV_Data.HkTlm.PosSigma_m  = Out.PosSigma_m;
    NAV_Data.HkTlm.VelSigma_ms = Out.VelSigma_ms;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(Sol->TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(Sol->TelemetryHeader), true);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void NAV_ProcessGroundCmd(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t Fc = 0;
    CFE_MSG_GetFcnCode(&BufPtr->Msg, &Fc);

    if (!NAV_VerifyLength(&BufPtr->Msg, sizeof(NAV_NoArgsCmd_t)))
    {
        NAV_Data.HkTlm.CmdErrCount++;
        CFE_EVS_SendEvent(NAV_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "NAV: cmd length error, fc %u",
                          (unsigned int)Fc);
        return;
    }

    switch (Fc)
    {
        case NAV_NOOP_CC:
            NAV_Data.HkTlm.CmdCount++;
            CFE_EVS_SendEvent(NAV_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "NAV: NOOP");
            break;

        case NAV_RESET_COUNTERS_CC:
            NAV_Data.HkTlm.CmdCount       = 0;
            NAV_Data.HkTlm.CmdErrCount    = 0;
            NAV_Data.HkTlm.CycleCount     = 0;
            NAV_Data.HkTlm.RpsAccepted    = 0;
            NAV_Data.HkTlm.RpsRejected    = 0;
            NAV_Data.HkTlm.RpsUnavailable = 0;
            NAV_Data.HkTlm.FilterInits    = 0;
            NAV_Data.HkTlm.SensorRejects  = 0;
            CFE_EVS_SendEvent(NAV_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "NAV: counters reset");
            break;

        case NAV_RESET_FILTER_CC:
            NAV_Data.HkTlm.CmdCount++;
            NAV_ResetFilter(&NAV_Data.State);
            CFE_EVS_SendEvent(NAV_RESET_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "NAV: translation filter reset — re-initialises on the next LIDAR fix");
            break;

        default:
            NAV_Data.HkTlm.CmdErrCount++;
            CFE_EVS_SendEvent(NAV_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "NAV: invalid command code %u",
                              (unsigned int)Fc);
            break;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void NAV_SendHk(void)
{
    /* Table management happens between cycles, never mid-step */
    CFE_TBL_Manage(NAV_Data.CfgTblHandle);

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(NAV_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(NAV_Data.HkTlm.TelemetryHeader), true);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
CFE_Status_t NAV_Init(void)
{
    CFE_Status_t Status;

    memset(&NAV_Data, 0, sizeof(NAV_Data));
    NAV_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;
    NAV_Reset(&NAV_Data.State);

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("NAV: CFE_EVS_Register failed, RC=0x%08lX\n", (unsigned long)Status);
        return Status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(NAV_Data.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(NAV_HK_TLM_MID),
                 sizeof(NAV_Data.HkTlm));
    CFE_MSG_Init(CFE_MSG_PTR(NAV_Data.SolutionMsg.TelemetryHeader), CFE_SB_ValueToMsgId(NAV_SOLUTION_MID),
                 sizeof(NAV_Data.SolutionMsg));

    Status = CFE_SB_CreatePipe(&NAV_Data.CmdPipe, NAV_PIPE_DEPTH, NAV_PIPE_NAME);
    if (Status == CFE_SUCCESS)
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(SIM_IO_SIM_STATE_MID), NAV_Data.CmdPipe);
    if (Status == CFE_SUCCESS)
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(NAV_SEND_HK_MID), NAV_Data.CmdPipe);
    if (Status == CFE_SUCCESS)
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(NAV_CMD_MID), NAV_Data.CmdPipe);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(NAV_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "NAV: pipe create/subscribe failed, RC=0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    Status = CFE_TBL_Register(&NAV_Data.CfgTblHandle, NAV_CFG_TBL_NAME, sizeof(NAV_CfgTbl_t), CFE_TBL_OPT_DEFAULT,
                              NAV_ValidateCfgTbl);
    if (Status == CFE_SUCCESS)
        Status = CFE_TBL_Load(NAV_Data.CfgTblHandle, CFE_TBL_SRC_FILE, NAV_CFG_TBL_FILE);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(NAV_TBL_ERR_EID, CFE_EVS_EventType_ERROR, "NAV: config table register/load failed, RC=0x%08lX",
                          (unsigned long)Status);
        return Status;
    }

    CFE_EVS_SendEvent(NAV_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "NAV initialized: IMU + star tracker + LIDAR, CW Kalman filter, table %s", NAV_CFG_TBL_FILE);
    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void NAV_Main(void)
{
    CFE_Status_t     Status;
    CFE_SB_Buffer_t *BufPtr;
    CFE_SB_MsgId_t   MsgId;

    if (NAV_Init() != CFE_SUCCESS)
        NAV_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;

    while (CFE_ES_RunLoop(&NAV_Data.RunStatus))
    {
        Status = CFE_SB_ReceiveBuffer(&BufPtr, NAV_Data.CmdPipe, CFE_SB_PEND_FOREVER);
        if (Status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(NAV_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "NAV: SB receive error 0x%08lX, exiting",
                              (unsigned long)Status);
            NAV_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
            continue;
        }

        CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId))
        {
            case SIM_IO_SIM_STATE_MID:
                if (NAV_VerifyLength(&BufPtr->Msg, sizeof(SIM_IO_SimStateMsg_t)))
                    NAV_ProcessSimState((const SIM_IO_SimStateMsg_t *)BufPtr);
                else
                    CFE_EVS_SendEvent(NAV_MSG_ERR_EID, CFE_EVS_EventType_ERROR, "NAV: SIM_STATE length error");
                break;

            case NAV_SEND_HK_MID:
                NAV_SendHk();
                break;

            case NAV_CMD_MID:
                NAV_ProcessGroundCmd(BufPtr);
                break;

            default:
                break;
        }
    }

    CFE_ES_ExitApp(NAV_Data.RunStatus);
}
