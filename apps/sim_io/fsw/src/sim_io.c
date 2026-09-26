#include <string.h>
#include "sim_io.h"

SIM_IO_Data_t SIM_IO_Data;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static int32 SIM_IO_ValidateCfgTbl(void *TblPtr)
{
    const SIM_IO_CfgTbl_t *t = (const SIM_IO_CfgTbl_t *)TblPtr;

    if (t->ListenPort == 0 || t->DestPort == 0)
    {
        CFE_EVS_SendEvent(SIM_IO_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SIM_IO: table rejected — zero port (listen %u dest %u)",
                          (unsigned int)t->ListenPort, (unsigned int)t->DestPort);
        return SIM_IO_TBL_VALIDATE_ERR;
    }
    if (memchr(t->DestHost, '\0', sizeof(t->DestHost)) == NULL || t->DestHost[0] == '\0')
    {
        CFE_EVS_SendEvent(SIM_IO_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SIM_IO: table rejected — DestHost empty or unterminated");
        return SIM_IO_TBL_VALIDATE_ERR;
    }
    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static bool SIM_IO_VerifyLength(const CFE_MSG_Message_t *MsgPtr, size_t Expected)
{
    size_t Actual = 0;
    CFE_MSG_GetSize(MsgPtr, &Actual);
    return Actual == Expected;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void SIM_IO_ProcessGroundCmd(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t Fc = 0;
    CFE_MSG_GetFcnCode(&BufPtr->Msg, &Fc);

    if (!SIM_IO_VerifyLength(&BufPtr->Msg, sizeof(SIM_IO_NoArgsCmd_t)))
    {
        SIM_IO_Data.HkTlm.CmdErrCount++;
        CFE_EVS_SendEvent(SIM_IO_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SIM_IO: cmd length error, fc %u", (unsigned int)Fc);
        return;
    }

    switch (Fc)
    {
        case SIM_IO_NOOP_CC:
            SIM_IO_Data.HkTlm.CmdCount++;
            CFE_EVS_SendEvent(SIM_IO_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "SIM_IO: NOOP (SimLink v%u)", (unsigned int)SIMLINK_VERSION);
            break;

        case SIM_IO_RESET_COUNTERS_CC:
            SIM_IO_Data.HkTlm.CmdCount      = 0;
            SIM_IO_Data.HkTlm.CmdErrCount   = 0;
            SIM_IO_Data.HkTlm.RxGood        = 0;
            SIM_IO_Data.HkTlm.RxBadFrame    = 0;
            SIM_IO_Data.HkTlm.RxBadCrc      = 0;
            SIM_IO_Data.HkTlm.RxSeqGaps     = 0;
            SIM_IO_Data.HkTlm.RxSeqRestarts = 0;
            SIM_IO_Data.HkTlm.TxCount       = 0;
            SIM_IO_Data.HkTlm.TxErrors      = 0;
            SIM_IO_Data.HkTlm.TxStaleSeq    = 0;
            CFE_EVS_SendEvent(SIM_IO_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "SIM_IO: counters reset");
            break;

        default:
            SIM_IO_Data.HkTlm.CmdErrCount++;
            CFE_EVS_SendEvent(SIM_IO_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                              "SIM_IO: invalid command code %u", (unsigned int)Fc);
            break;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void SIM_IO_SendHk(void)
{
    CFE_TBL_Manage(SIM_IO_Data.CfgTblHandle);

    OS_MutSemTake(SIM_IO_Data.LastRxMutex);
    SIM_IO_Data.HkTlm.LastRxSeq     = SIM_IO_Data.LastRxSeq;
    SIM_IO_Data.HkTlm.LastSimTime_s = (float)SIM_IO_Data.LastRxSimTime_s;
    OS_MutSemGive(SIM_IO_Data.LastRxMutex);

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(SIM_IO_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(SIM_IO_Data.HkTlm.TelemetryHeader), true);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
CFE_Status_t SIM_IO_Init(void)
{
    CFE_Status_t Status;
    int32        OsStatus;

    memset(&SIM_IO_Data, 0, sizeof(SIM_IO_Data));
    SIM_IO_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("SIM_IO: CFE_EVS_Register failed, RC=0x%08lX\n", (unsigned long)Status);
        return Status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(SIM_IO_Data.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(SIM_IO_HK_TLM_MID),
                 sizeof(SIM_IO_Data.HkTlm));
    CFE_MSG_Init(CFE_MSG_PTR(SIM_IO_Data.SimStateMsg.TelemetryHeader), CFE_SB_ValueToMsgId(SIM_IO_SIM_STATE_MID),
                 sizeof(SIM_IO_Data.SimStateMsg));

    Status = CFE_SB_CreatePipe(&SIM_IO_Data.CmdPipe, SIM_IO_PIPE_DEPTH, SIM_IO_PIPE_NAME);
    if (Status == CFE_SUCCESS)
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(SIM_IO_CMD_MID), SIM_IO_Data.CmdPipe);
    if (Status == CFE_SUCCESS)
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(SIM_IO_SEND_HK_MID), SIM_IO_Data.CmdPipe);
    if (Status == CFE_SUCCESS)
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(SIM_IO_THRUSTER_CMD_MID), SIM_IO_Data.CmdPipe);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(SIM_IO_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SIM_IO: pipe create/subscribe failed, RC=0x%08lX", (unsigned long)Status);
        return Status;
    }

    Status = CFE_TBL_Register(&SIM_IO_Data.CfgTblHandle, SIM_IO_CFG_TBL_NAME, sizeof(SIM_IO_CfgTbl_t),
                              CFE_TBL_OPT_DEFAULT, SIM_IO_ValidateCfgTbl);
    if (Status == CFE_SUCCESS)
        Status = CFE_TBL_Load(SIM_IO_Data.CfgTblHandle, CFE_TBL_SRC_FILE, SIM_IO_CFG_TBL_FILE);
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_TBL_GetAddress((void **)&SIM_IO_Data.Cfg, SIM_IO_Data.CfgTblHandle);
        if (Status == CFE_TBL_INFO_UPDATED)
            Status = CFE_SUCCESS;
    }
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(SIM_IO_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SIM_IO: config table register/load failed, RC=0x%08lX", (unsigned long)Status);
        return Status;
    }

    OsStatus = OS_MutSemCreate(&SIM_IO_Data.LastRxMutex, "SIM_IO_RX_MTX", 0);
    if (OsStatus != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(SIM_IO_TASK_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SIM_IO: OS_MutSemCreate failed, RC=%d", (int)OsStatus);
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    /* A tx failure is logged but not fatal — telemetry can still flow in, and
    ** HK TxErrors makes the problem visible from the ground. */
    SIM_IO_OpenTx();

    Status = CFE_ES_CreateChildTask(&SIM_IO_Data.RxTaskId, SIM_IO_RX_TASK_NAME, SIM_IO_RxTask,
                                    CFE_ES_TASK_STACK_ALLOCATE, SIM_IO_RX_STACK_SIZE, SIM_IO_RX_TASK_PRIO, 0);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(SIM_IO_TASK_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SIM_IO: CFE_ES_CreateChildTask failed, RC=0x%08lX", (unsigned long)Status);
        return Status;
    }

    CFE_EVS_SendEvent(SIM_IO_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "SIM_IO initialized: SimLink v%u, rx port %u, tx %s:%u", (unsigned int)SIMLINK_VERSION,
                      (unsigned int)SIM_IO_Data.Cfg->ListenPort, SIM_IO_Data.Cfg->DestHost,
                      (unsigned int)SIM_IO_Data.Cfg->DestPort);
    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void SIM_IO_Main(void)
{
    CFE_Status_t     Status;
    CFE_SB_Buffer_t *BufPtr;
    CFE_SB_MsgId_t   MsgId;

    if (SIM_IO_Init() != CFE_SUCCESS)
        SIM_IO_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;

    while (CFE_ES_RunLoop(&SIM_IO_Data.RunStatus))
    {
        Status = CFE_SB_ReceiveBuffer(&BufPtr, SIM_IO_Data.CmdPipe, CFE_SB_PEND_FOREVER);
        if (Status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(SIM_IO_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "SIM_IO: SB receive error 0x%08lX, exiting", (unsigned long)Status);
            SIM_IO_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
            continue;
        }

        CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId))
        {
            case SIM_IO_THRUSTER_CMD_MID:
                if (SIM_IO_VerifyLength(&BufPtr->Msg, sizeof(SIM_IO_ThrusterCmdMsg_t)))
                    SIM_IO_SendThrusterCmd((const SIM_IO_ThrusterCmdMsg_t *)BufPtr);
                else
                    SIM_IO_Data.HkTlm.TxErrors++;
                break;

            case SIM_IO_SEND_HK_MID:
                SIM_IO_SendHk();
                break;

            case SIM_IO_CMD_MID:
                SIM_IO_ProcessGroundCmd(BufPtr);
                break;

            default:
                break;
        }
    }

    CFE_ES_ExitApp(SIM_IO_Data.RunStatus);
}
