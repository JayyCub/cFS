#ifndef SIM_IO_MSG_H
#define SIM_IO_MSG_H

/*
** Software Bus messages that cross the SIM_IO boundary. Other apps include this
** header to consume SIM_STATE or publish THRUSTER_CMD; none of them touch sockets.
*/

#include "cfe.h"
#include "simlink_icd.h"

/*
** SIM_STATE — raw sensor readings, published once per received Unity frame.
** Its arrival schedules the NAV cycle, and NAV's solution schedules GNC
** (lock-step): there is no wall-clock GNC wakeup. GNC also reads the hand
** controllers and mechanism switches from it directly.
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint32                    Seq;       /* Unity cycle counter; THRUSTER_CMD echoes it */
    uint32                    Spare;
    double                    SimTime_s; /* simulation time at this cycle boundary      */
    SIMLINK_SimState_t        State;
} SIM_IO_SimStateMsg_t;

/*
** THRUSTER_CMD — published by the RCS app each cycle (GNC impulse request ->
** allocation -> PWM). SIM_IO forwards Cmd to Unity tagged with Seq so Unity can
** release the lock-step cycle. Achieved* is the impulse these on-times deliver
** according to RCS's thruster table (after saturation scaling and minimum-
** impulse-bit rounding) — what GNC should expect to see, not what it asked for.
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint32                    Seq; /* Seq of the SIM_STATE this command answers */
    uint32                    Spare;
    SIMLINK_ThrusterCmd_t     Cmd;
    float                     AchievedLinImpulse_Ns[3];  /* body frame */
    float                     AchievedAngImpulse_Nms[3]; /* body frame */
} SIM_IO_ThrusterCmdMsg_t;

/* Ground command codes */
#define SIM_IO_NOOP_CC           0
#define SIM_IO_RESET_COUNTERS_CC 1

typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
} SIM_IO_NoArgsCmd_t;

/* Housekeeping */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint32                    CmdCount;
    uint32                    CmdErrCount;
    uint32                    RxGood;         /* valid SIM_STATE frames published            */
    uint32                    RxBadFrame;     /* sync / version / type / length rejects      */
    uint32                    RxBadCrc;
    uint32                    RxSeqGaps;      /* frames that skipped one or more Seq values   */
    uint32                    RxSeqRestarts;  /* Seq went backwards (Unity scenario restart)  */
    uint32                    TxCount;        /* THRUSTER_CMD frames sent to Unity            */
    uint32                    TxErrors;
    uint32                    TxStaleSeq;     /* commands answering an older SIM_STATE        */
    uint32                    LastRxSeq;
    float                     LastSimTime_s;
} SIM_IO_HkTlm_t;

#endif /* SIM_IO_MSG_H */
