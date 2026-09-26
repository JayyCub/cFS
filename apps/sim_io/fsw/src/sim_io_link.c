/*
** SIM_IO link layer — SimLink framing over UDP.
**
** Host-name resolution uses POSIX getaddrinfo (no OSAL equivalent). On an
** RTEMS/VxWorks target this would be a table-provided dotted-quad instead,
** which OS_SocketAddrFromString handles directly.
*/
#include <string.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "sim_io.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void SIM_IO_RxDeleteHandler(void)
{
    /* Unblocks OS_SocketRecvFrom so the task can be torn down during shutdown */
    OS_close(SIM_IO_Data.RxSocketId);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Validate one received datagram; on success publish it as SIM_STATE.      */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void SIM_IO_HandleFrame(const uint8 *Buf, uint32 Len)
{
    static bool       BadFrameReported = false;
    SIMLINK_Header_t  Hdr;
    SIMLINK_Trailer_t Trl;
    const char       *Why = NULL;

    if (Len != SIMLINK_SIM_STATE_FRAME_BYTES)
    {
        Why = "length";
    }
    else
    {
        memcpy(&Hdr, Buf, sizeof(Hdr));
        memcpy(&Trl, Buf + Len - sizeof(Trl), sizeof(Trl));

        if (Hdr.Sync != SIMLINK_SYNC_WORD)
            Why = "sync word";
        else if (Hdr.Version != SIMLINK_VERSION)
            Why = "version";
        else if (Hdr.Type != SIMLINK_TYPE_SIM_STATE)
            Why = "type";
        else if (Hdr.Length != Len)
            Why = "header length";
        else if (SIMLINK_Crc16(Buf, Len - (uint32)sizeof(Trl)) != Trl.Crc)
        {
            SIM_IO_Data.HkTlm.RxBadCrc++;
            if (!BadFrameReported)
                CFE_EVS_SendEvent(SIM_IO_RX_FRAME_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "SIM_IO: SIM_STATE CRC mismatch — frame dropped");
            BadFrameReported = true;
            return;
        }
    }

    if (Why != NULL)
    {
        SIM_IO_Data.HkTlm.RxBadFrame++;
        /* Report the first bad frame of a run of them, not every one. */
        if (!BadFrameReported)
            CFE_EVS_SendEvent(SIM_IO_RX_FRAME_ERR_EID, CFE_EVS_EventType_ERROR,
                              "SIM_IO: rejected %u-byte frame (bad %s) — expecting SimLink v%u SIM_STATE",
                              (unsigned int)Len, Why, (unsigned int)SIMLINK_VERSION);
        BadFrameReported = true;
        return;
    }
    BadFrameReported = false;

    /* Sequence accounting. Lock-step means Unity never advances without our
    ** answer, so a gap signals a dropped datagram or Unity running free after
    ** a link timeout; a backwards step is a Unity play-mode restart. */
    if (SIM_IO_Data.HaveRxSeq)
    {
        if (Hdr.Seq == SIM_IO_Data.SimStateMsg.Seq + 1)
        {
            /* nominal */
        }
        else if (Hdr.Seq > SIM_IO_Data.SimStateMsg.Seq + 1)
        {
            SIM_IO_Data.HkTlm.RxSeqGaps++;
        }
        else
        {
            SIM_IO_Data.HkTlm.RxSeqRestarts++;
            CFE_EVS_SendEvent(SIM_IO_RX_SEQ_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "SIM_IO: SimLink Seq restarted (%lu -> %lu) — new Unity session",
                              (unsigned long)SIM_IO_Data.SimStateMsg.Seq, (unsigned long)Hdr.Seq);
        }
    }
    SIM_IO_Data.HaveRxSeq = true;

    SIM_IO_Data.SimStateMsg.Seq       = Hdr.Seq;
    SIM_IO_Data.SimStateMsg.SimTime_s = Hdr.SimTime_s;
    memcpy(&SIM_IO_Data.SimStateMsg.State, Buf + sizeof(Hdr), sizeof(SIM_IO_Data.SimStateMsg.State));

    OS_MutSemTake(SIM_IO_Data.LastRxMutex);
    SIM_IO_Data.LastRxSeq       = Hdr.Seq;
    SIM_IO_Data.LastRxSimTime_s = Hdr.SimTime_s;
    OS_MutSemGive(SIM_IO_Data.LastRxMutex);

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(SIM_IO_Data.SimStateMsg.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(SIM_IO_Data.SimStateMsg.TelemetryHeader), true);
    SIM_IO_Data.HkTlm.RxGood++;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Child task: own the receive socket for the life of the app.              */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void SIM_IO_RxTask(void)
{
    uint8         Buf[SIMLINK_MAX_FRAME_BYTES];
    OS_SockAddr_t BindAddr;
    OS_SockAddr_t FromAddr;
    int32         OsStatus;

    OS_TaskInstallDeleteHandler(SIM_IO_RxDeleteHandler);

    /* Outer loop re-opens the socket after a fatal receive error */
    while (true)
    {
        OsStatus = OS_SocketOpen(&SIM_IO_Data.RxSocketId, OS_SocketDomain_INET, OS_SocketType_DATAGRAM);
        if (OsStatus == OS_SUCCESS)
        {
            OS_SocketAddrInit(&BindAddr, OS_SocketDomain_INET);
            OS_SocketAddrSetPort(&BindAddr, SIM_IO_Data.Cfg->ListenPort);
            OsStatus = OS_SocketBind(SIM_IO_Data.RxSocketId, &BindAddr);
            if (OsStatus != OS_SUCCESS)
                OS_close(SIM_IO_Data.RxSocketId);
        }
        if (OsStatus != OS_SUCCESS)
        {
            CFE_EVS_SendEvent(SIM_IO_SOCKET_ERR_EID, CFE_EVS_EventType_ERROR,
                              "SIM_IO: rx socket open/bind on port %u failed RC=%d — retry in 1s",
                              (unsigned int)SIM_IO_Data.Cfg->ListenPort, (int)OsStatus);
            OS_TaskDelay(1000);
            continue;
        }

        CFE_EVS_SendEvent(SIM_IO_RX_READY_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "SIM_IO: listening for SimLink frames on port %u",
                          (unsigned int)SIM_IO_Data.Cfg->ListenPort);

        while (true)
        {
            OsStatus = OS_SocketRecvFrom(SIM_IO_Data.RxSocketId, Buf, sizeof(Buf), &FromAddr, OS_PEND);
            if (OsStatus < 0)
            {
                CFE_EVS_SendEvent(SIM_IO_SOCKET_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "SIM_IO: rx error RC=%d — reopening socket", (int)OsStatus);
                OS_close(SIM_IO_Data.RxSocketId);
                OS_TaskDelay(1000);
                break;
            }
            SIM_IO_HandleFrame(Buf, (uint32)OsStatus);
        }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
CFE_Status_t SIM_IO_OpenTx(void)
{
    struct addrinfo  Hints;
    struct addrinfo *Res = NULL;
    char             IpStr[INET_ADDRSTRLEN];
    int32            OsStatus;

    SIM_IO_Data.TxReady = false;

    memset(&Hints, 0, sizeof(Hints));
    Hints.ai_family   = AF_INET;
    Hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(SIM_IO_Data.Cfg->DestHost, NULL, &Hints, &Res) != 0 || Res == NULL)
    {
        CFE_EVS_SendEvent(SIM_IO_SOCKET_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SIM_IO: cannot resolve %s — commands to Unity disabled", SIM_IO_Data.Cfg->DestHost);
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    inet_ntop(AF_INET, &((struct sockaddr_in *)Res->ai_addr)->sin_addr, IpStr, sizeof(IpStr));
    freeaddrinfo(Res);

    OsStatus = OS_SocketOpen(&SIM_IO_Data.TxSocketId, OS_SocketDomain_INET, OS_SocketType_DATAGRAM);
    if (OsStatus != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(SIM_IO_SOCKET_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SIM_IO: tx socket open failed RC=%d", (int)OsStatus);
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    OS_SocketAddrInit(&SIM_IO_Data.TxAddr, OS_SocketDomain_INET);
    OS_SocketAddrFromString(&SIM_IO_Data.TxAddr, IpStr);
    OS_SocketAddrSetPort(&SIM_IO_Data.TxAddr, SIM_IO_Data.Cfg->DestPort);
    SIM_IO_Data.TxReady = true;
    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Frame one GNC WRENCH_CMD and send it to Unity.                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void SIM_IO_SendWrenchCmd(const SIM_IO_WrenchCmdMsg_t *Msg)
{
    uint8             Buf[SIMLINK_WRENCH_CMD_FRAME_BYTES];
    SIMLINK_Header_t  Hdr;
    SIMLINK_Trailer_t Trl;
    uint32            LastSeq;
    double            LastSimTime;
    int32             OsStatus;

    if (!SIM_IO_Data.TxReady)
    {
        SIM_IO_Data.HkTlm.TxErrors++;
        return;
    }

    OS_MutSemTake(SIM_IO_Data.LastRxMutex);
    LastSeq     = SIM_IO_Data.LastRxSeq;
    LastSimTime = SIM_IO_Data.LastRxSimTime_s;
    OS_MutSemGive(SIM_IO_Data.LastRxMutex);

    /* Still forwarded — Unity discards it if it has moved on — but counted,
    ** because in lock-step a stale answer means GNC fell behind. */
    if (Msg->Seq != LastSeq)
        SIM_IO_Data.HkTlm.TxStaleSeq++;

    Hdr.Sync      = SIMLINK_SYNC_WORD;
    Hdr.Version   = SIMLINK_VERSION;
    Hdr.Type      = SIMLINK_TYPE_WRENCH_CMD;
    Hdr.Seq       = Msg->Seq;
    Hdr.Length    = sizeof(Buf);
    Hdr.SimTime_s = (Msg->Seq == LastSeq) ? LastSimTime : 0.0;

    memcpy(Buf, &Hdr, sizeof(Hdr));
    memcpy(Buf + sizeof(Hdr), &Msg->Cmd, sizeof(Msg->Cmd));
    Trl.Crc   = SIMLINK_Crc16(Buf, sizeof(Buf) - sizeof(Trl));
    Trl.Spare = 0;
    memcpy(Buf + sizeof(Buf) - sizeof(Trl), &Trl, sizeof(Trl));

    OsStatus = OS_SocketSendTo(SIM_IO_Data.TxSocketId, Buf, sizeof(Buf), &SIM_IO_Data.TxAddr);
    if (OsStatus != (int32)sizeof(Buf))
    {
        SIM_IO_Data.HkTlm.TxErrors++;
        CFE_EVS_SendEvent(SIM_IO_TX_ERR_EID, CFE_EVS_EventType_ERROR, "SIM_IO: send to Unity failed RC=%d",
                          (int)OsStatus);
        return;
    }
    SIM_IO_Data.HkTlm.TxCount++;
}
