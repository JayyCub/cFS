#include "gnc_app.h"

/*
** POSIX headers for hostname resolution.
** getaddrinfo / inet_ntop are not part of the OSAL API — they are Linux/POSIX specific.
** On a real RTEMS or VxWorks target this block would be replaced with the platform's
** equivalent resolver or a hardcoded IP address loaded from a cFS table.
*/
#include <netdb.h>
#include <arpa/inet.h>

/*
** Delete callback — OSAL calls this when the task is killed during cFS shutdown.
** Closing the socket causes any blocking OS_SocketRecvFrom to return immediately,
** letting the task exit cleanly rather than being cancelled mid-recv.
*/
static void GNC_APP_UdpDeleteCallback(void)
{
    /* Closing the socket unblocks OS_SocketRecvFrom so the task exits cleanly.
       The outer retry loop would reopen it, but cFS is shutting down so that
       doesn't matter — the task will simply not be rescheduled. */
    OS_printf("GNC_APP: UDP receive task shutdown, closing socket.\n");
    OS_close(GNC_APP_Data.SocketId);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_UdpRecvTask — background task, runs for the life of the app      */
/*                                                                           */
/* Opens a UDP socket on GNC_APP_UDP_LISTEN_PORT and blocks on              */
/* OS_SocketRecvFrom. When a correctly-sized packet arrives it copies it     */
/* into GNC_APP_Data.LatestTlm under the mutex and sets TlmFresh so the     */
/* 1 Hz wakeup handler knows new data is ready.                              */
/*                                                                           */
/* Uses the OSAL socket abstraction (same as ci_lab) rather than raw POSIX  */
/* calls so the code would compile unchanged for RTEMS or VxWorks targets.  */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void GNC_APP_UdpRecvTask(void)
{
    int32              OsStatus;
    OS_SockAddr_t      BindAddr;
    OS_SockAddr_t      SenderAddr;
    GNC_APP_UnityTlm_t RxBuf;

    OS_TaskInstallDeleteHandler(&GNC_APP_UdpDeleteCallback);

    /*
    ** Outer retry loop — reopens the socket if it ever dies (recv error,
    ** transient OS signal, Docker networking hiccup).  Each iteration
    ** represents one complete socket lifetime: open → bind → receive loop.
    ** A 1-second delay between attempts prevents a spin-loop on persistent
    ** bind failures while still recovering quickly from transient errors.
    */
    while (true)
    {
        OsStatus = OS_SocketOpen(&GNC_APP_Data.SocketId, OS_SocketDomain_INET, OS_SocketType_DATAGRAM);
        if (OsStatus != OS_SUCCESS)
        {
            CFE_EVS_SendEvent(GNC_APP_UDP_ERR_EID, CFE_EVS_EventType_ERROR,
                              "GNC_APP UDP: OS_SocketOpen failed RC=%d — retry in 1s", (int)OsStatus);
            OS_TaskDelay(1000);
            continue;
        }

        OS_SocketAddrInit(&BindAddr, OS_SocketDomain_INET);
        OS_SocketAddrSetPort(&BindAddr, GNC_APP_UDP_LISTEN_PORT);

        OsStatus = OS_SocketBind(GNC_APP_Data.SocketId, &BindAddr);
        if (OsStatus != OS_SUCCESS)
        {
            CFE_EVS_SendEvent(GNC_APP_UDP_ERR_EID, CFE_EVS_EventType_ERROR,
                              "GNC_APP UDP: OS_SocketBind port %d failed RC=%d — retry in 1s",
                              GNC_APP_UDP_LISTEN_PORT, (int)OsStatus);
            OS_close(GNC_APP_Data.SocketId);
            OS_TaskDelay(1000);
            continue;
        }

        CFE_EVS_SendEvent(GNC_APP_UDP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "GNC_APP UDP: listening on port %d", GNC_APP_UDP_LISTEN_PORT);

        /* Inner receive loop — runs until a fatal recv error */
        while (true)
        {
            OsStatus = OS_SocketRecvFrom(GNC_APP_Data.SocketId,
                                         &RxBuf, sizeof(RxBuf),
                                         &SenderAddr, OS_PEND);

            if (OsStatus == (int32)sizeof(RxBuf))
            {
                OS_MutSemTake(GNC_APP_Data.TlmMutex);
                memcpy(&GNC_APP_Data.LatestTlm, &RxBuf, sizeof(RxBuf));
                GNC_APP_Data.TlmFresh = true;
                OS_MutSemGive(GNC_APP_Data.TlmMutex);
                GNC_APP_Data.HkTlm.UdpPacketsReceived++;
            }
            else if (OsStatus < 0)
            {
                /* Socket closed (normal shutdown) or OS error — break to outer
                   loop which will reopen the socket and try again. */
                CFE_EVS_SendEvent(GNC_APP_UDP_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "GNC_APP UDP: recv error RC=%d — reopening socket", (int)OsStatus);
                OS_close(GNC_APP_Data.SocketId);
                OS_TaskDelay(1000);
                break;  /* break inner → continue outer */
            }
            else
            {
                CFE_EVS_SendEvent(GNC_APP_UDP_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "GNC_APP UDP: wrong packet size %d (expected %d) — discarding",
                                  (int)OsStatus, (int)sizeof(RxBuf));
            }
        }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_OpenCmdSocket — called once from GNC_APP_Init                    */
/*                                                                           */
/* Resolves GNC_APP_CMD_TARGET_HOST to a dotted-decimal IP address using    */
/* the POSIX getaddrinfo API (no OSAL equivalent exists), then opens a UDP  */
/* datagram socket and stores the destination address in GNC_APP_Data so    */
/* GNC_APP_SendCommand can reuse it without re-connecting.                  */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void GNC_APP_OpenCmdSocket(void)
{
    int32           OsStatus;
    struct addrinfo hints;
    struct addrinfo *res = NULL;
    char             ip_str[INET_ADDRSTRLEN];

    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    if (getaddrinfo(GNC_APP_CMD_TARGET_HOST, NULL, &hints, &res) != 0 || res == NULL)
    {
        CFE_EVS_SendEvent(GNC_APP_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: getaddrinfo(%s) failed — command output disabled",
                          GNC_APP_CMD_TARGET_HOST);
        GNC_APP_Data.CmdSocketReady = false;
        return;
    }

    inet_ntop(AF_INET,
              &((struct sockaddr_in *)res->ai_addr)->sin_addr,
              ip_str, sizeof(ip_str));
    freeaddrinfo(res);

    /* Open a UDP datagram socket (no bind needed — we are the sender) */
    OsStatus = OS_SocketOpen(&GNC_APP_Data.CmdSocketId, OS_SocketDomain_INET, OS_SocketType_DATAGRAM);
    if (OsStatus != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(GNC_APP_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: OS_SocketOpen (cmd) failed, RC=%d", (int)OsStatus);
        GNC_APP_Data.CmdSocketReady = false;
        return;
    }

    /* Build the destination address once; reused every wakeup */
    OS_SocketAddrInit(&GNC_APP_Data.CmdAddr, OS_SocketDomain_INET);
    OS_SocketAddrFromString(&GNC_APP_Data.CmdAddr, ip_str);
    OS_SocketAddrSetPort(&GNC_APP_Data.CmdAddr, GNC_APP_CMD_PORT);

    GNC_APP_Data.CmdSocketReady = true;

    CFE_EVS_SendEvent(GNC_APP_CMD_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "GNC_APP: command socket ready → %s:%d",
                      ip_str, GNC_APP_CMD_PORT);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_SendCommand — sends an 8-byte timed thruster command to Unity    */
/*                                                                           */
/* Packet layout matches UdpCommandReceiver.cs exactly:                     */
/*   bytes [0-3]  int32  ThrusterMask   little-endian                       */
/*   bytes [4-7]  float  BurnDuration_s IEEE 754 little-endian              */
/*                                                                           */
/* Unity fires each active thruster for exactly BurnDuration_s seconds,     */
/* then auto-cuts off — it does NOT wait for a "stop" command.              */
/* Sending mask=0 / duration=0.0 is a heartbeat that resets the cFS         */
/* command timeout without firing any thrusters (coast command).            */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void GNC_APP_SendCommand(int32 mask, float duration_s)
{
    int32 OsStatus;
    uint8 buf[8];

    /* Union lets us reinterpret the float bits as uint32 without UB */
    union { float f; uint32 u; } dur;

    if (!GNC_APP_Data.CmdSocketReady)
        return;

    dur.f = duration_s;

    /* bytes [0-3]: thruster bitmask, little-endian int32 */
    buf[0] = (uint8)( mask        & 0xFF);
    buf[1] = (uint8)((mask >>  8) & 0xFF);
    buf[2] = (uint8)((mask >> 16) & 0xFF);
    buf[3] = (uint8)((mask >> 24) & 0xFF);

    /* bytes [4-7]: burn duration, little-endian IEEE 754 float */
    buf[4] = (uint8)( dur.u        & 0xFF);
    buf[5] = (uint8)((dur.u >>  8) & 0xFF);
    buf[6] = (uint8)((dur.u >> 16) & 0xFF);
    buf[7] = (uint8)((dur.u >> 24) & 0xFF);

    OsStatus = OS_SocketSendTo(GNC_APP_Data.CmdSocketId,
                               buf, sizeof(buf),
                               &GNC_APP_Data.CmdAddr);
    if (OsStatus != (int32)sizeof(buf))
    {
        CFE_EVS_SendEvent(GNC_APP_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: OS_SocketSendTo failed, RC=%d", (int)OsStatus);
    }
}
