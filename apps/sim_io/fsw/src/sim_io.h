#ifndef SIM_IO_H
#define SIM_IO_H

/*
** SIM_IO — the device-interface app between cFS and the Unity simulation.
**
** In a flight system this is the layer that talks to hardware (1553/SpaceWire/
** UART device drivers). Here the "hardware" is Unity over UDP. It is the only
** app that owns sockets: everything downstream sees plain Software Bus
** messages, so GNC (and later NAV/RCS) are identical whether the vehicle is a
** simulation or real avionics.
**
** Task layout:
**   main task  — SB pipe: WRENCH_CMD from GNC (-> UDP to Unity), SEND_HK, ground cmds
**   rx child   — blocks on the UDP socket, validates SimLink frames, publishes
**                SIM_STATE to the SB (which in turn schedules GNC — lock-step)
*/

#include "cfe.h"
#include "osapi.h"
#include "sim_io_msgids.h"
#include "sim_io_msg.h"
#include "sim_io_tbl.h"

#define SIM_IO_PIPE_NAME  "SIM_IO_PIPE"
#define SIM_IO_PIPE_DEPTH 16

#define SIM_IO_RX_TASK_NAME  "SIM_IO_RX"
#define SIM_IO_RX_STACK_SIZE 16384
#define SIM_IO_RX_TASK_PRIO  60 /* just below the main apps: a frame should reach the SB promptly */

/* Event IDs */
#define SIM_IO_INIT_INF_EID     1
#define SIM_IO_PIPE_ERR_EID     2
#define SIM_IO_TBL_ERR_EID      3
#define SIM_IO_SOCKET_ERR_EID   4
#define SIM_IO_RX_READY_INF_EID 5
#define SIM_IO_RX_FRAME_ERR_EID 6
#define SIM_IO_RX_SEQ_INF_EID   7
#define SIM_IO_TX_ERR_EID       8
#define SIM_IO_NOOP_INF_EID     9
#define SIM_IO_RESET_INF_EID    10
#define SIM_IO_CMD_ERR_EID      11
#define SIM_IO_TBL_VAL_ERR_EID  12
#define SIM_IO_TASK_ERR_EID     13

#define SIM_IO_TBL_VALIDATE_ERR (0xc0000011)

typedef struct
{
    uint32          RunStatus;
    CFE_SB_PipeId_t CmdPipe;
    SIM_IO_HkTlm_t  HkTlm;

    CFE_TBL_Handle_t       CfgTblHandle;
    const SIM_IO_CfgTbl_t *Cfg;

    /* rx child task */
    CFE_ES_TaskId_t      RxTaskId;
    osal_id_t            RxSocketId;
    SIM_IO_SimStateMsg_t SimStateMsg; /* owned by the rx task */
    bool                 HaveRxSeq;

    /* Most recent SIM_STATE identity, written by rx task, read by main task
    ** (SimTime_s is a double, so it's guarded rather than assumed atomic). */
    osal_id_t LastRxMutex;
    uint32    LastRxSeq;
    double    LastRxSimTime_s;

    /* tx to Unity */
    osal_id_t     TxSocketId;
    OS_SockAddr_t TxAddr;
    bool          TxReady;
} SIM_IO_Data_t;

extern SIM_IO_Data_t SIM_IO_Data;

void         SIM_IO_Main(void);
CFE_Status_t SIM_IO_Init(void);

/* sim_io_link.c */
void         SIM_IO_RxTask(void);
CFE_Status_t SIM_IO_OpenTx(void);
void         SIM_IO_SendWrenchCmd(const SIM_IO_WrenchCmdMsg_t *Msg);

#endif /* SIM_IO_H */
