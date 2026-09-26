#ifndef RCS_MSG_H
#define RCS_MSG_H

#include "cfe.h"

/*
** ACT_REQ — GNC's actuator request for one control cycle, published in response
** to each SIM_STATE. GNC states WHAT it wants (impulse to impart this cycle, body
** frame); the RCS app decides HOW (which thrusters, for how long).
**
** Impulse rather than force: GNC's control law naturally produces a velocity /
** rate change for this cycle (m·Δv, I·Δω); how long the valves stay open to
** deliver it is an actuator-management decision that depends on thruster
** geometry, rated thrust and the minimum impulse bit — all RCS table data.
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint32                    Seq;                   /* SIM_STATE Seq being answered   */
    int32                     GncPhase;              /* passed through for HUD display */
    float                     LinImpulse_Ns[3];      /* body frame, N·s                */
    float                     AngImpulse_Nms[3];     /* body frame, N·m·s              */
} RCS_ActReqMsg_t;

/* Ground command codes */
#define RCS_NOOP_CC           0
#define RCS_RESET_COUNTERS_CC 1

typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
} RCS_NoArgsCmd_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint32                    CmdCount;
    uint32                    CmdErrCount;
    uint32                    RequestCount;     /* ACT_REQ messages processed                       */
    uint32                    SaturatedCount;   /* requests scaled down to fit MaxOnTime_s          */
    uint32                    MinImpulseDrops;  /* thruster pulses dropped below the impulse bit    */
    uint32                    MinImpulseRounds; /* thruster pulses rounded UP to the impulse bit    */
    float                     LastResidualNorm; /* |request - achieved| of the last solve (mixed    */
                                                /* N·s / N·m·s units — trend it, don't compare it)  */
    float                     LastMaxOnTime_s;
    uint16                    LastFiredMask;    /* bit i = thruster i commanded open last cycle     */
    uint16                    Spare;
} RCS_HkTlm_t;

#endif /* RCS_MSG_H */
