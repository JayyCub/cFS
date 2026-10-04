#ifndef NAV_MSG_H
#define NAV_MSG_H

#include <stddef.h>
#include "cfe.h"
#include "nav_status.h" /* NAV_STATUS_* bits for Status */

/*
** NAV app public messages: command codes, the per-cycle navigation solution
** GNC flies on, and 1 Hz housekeeping.
**
** Frames (see nav_tbl.h for how they are defined from the table):
**   B  chaser body — RCS-table axes, origin at the CoM
**   L  LVLH — Unity world axes: +Y radial, -Z along-track, +X cross-track
**   D  docking frame — the axes the chaser docking port must have at capture
**      (+Z points at the target port, X/Y are lateral), origin at the target
**      port face
**
** ground_console.py and tools/fdr_decode.py decode these with fixed struct
** formats; the _Static_asserts catch a layout change that would break them.
*/

/* Ground command codes (NAV_CMD_MID) */
#define NAV_NOOP_CC           0
#define NAV_RESET_COUNTERS_CC 1
#define NAV_RESET_FILTER_CC   2 /* drop the translation filter; it re-initialises on the next LIDAR fix */

typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
} NAV_NoArgsCmd_t;

/*
** Navigation solution (NAV_SOLUTION_MID), one per SIM_STATE.
**
** Published every cycle, valid or not, because GNC answers each one and
** Unity's lock-step waits for that answer. Check Status before using a group.
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    double                    SimTime_s; /* of the SIM_STATE this solution is for            */
    uint32                    Seq;       /* SIM_STATE Seq                                    */
    float                     CycleDt_s; /* sim time since the previous cycle                 */
    uint32                    Status;    /* NAV_STATUS_*                                      */
    uint16                    ConsecRpsRejects; /* LIDAR fixes rejected in a row (LC watches it) */
    uint16                    Spare;
    float                     AttQuat_L[4];    /* q_L_B (x, y, z, w), attitude filter estimate */
    float                     AngRate_B[3];    /* body rate, gyro minus estimated bias (rad/s) */
    float                     AttErr_B[3];     /* rotation vector, body axes, that takes the
                                               ** body to the docking attitude (rad)          */
    float                     CmPos_L[3];      /* chaser CoM - target reference point (m)      */
    float                     CmVel_L[3];      /* (m/s)                                        */
    float                     PortPos_D[3];    /* chaser port - target port, docking frame (m):
                                               ** X/Y lateral, Z < 0 while short of the port   */
    float                     PortVel_D[3];    /* port point velocity incl. ω×r (m/s)          */
    float                     Range_m;         /* |PortPos_D|                                  */
    float                     ClosingSpeed_ms; /* PortVel_D[2]; + = closing                    */
    float                     LateralOffset_m; /* |PortPos_D[0..1]|                            */
    float                     DeltaV_B[3];     /* IMU non-gravitational Δv over the cycle that
                                               ** just ended, estimated bias removed (m/s)     */
    float                     PosSigma_m;      /* largest position 1-sigma (m)                 */
    float                     VelSigma_ms;     /* largest velocity 1-sigma (m/s)               */
    float                     AttSigma_rad;    /* largest attitude 1-sigma (rad)               */
    float                     DockFrameQuat_L[4]; /* q_L_D: docking frame axes in LVLH (table
                                               ** geometry; GNC rotates its commands with it)  */
    float                     Spare2;
} NAV_SolutionMsg_t;

/* Housekeeping (NAV_HK_TLM_MID), 1 Hz */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint32                    CmdCount;
    uint32                    CmdErrCount;
    uint32                    CycleCount;     /* SIM_STATEs processed                              */
    uint32                    RpsAccepted;    /* LIDAR fixes used by the filter                    */
    uint32                    RpsRejected;    /* LIDAR fixes that failed the innovation gate       */
    uint32                    RpsUnavailable; /* cycles with no LIDAR reading (FOV / range limits) */
    uint32                    FilterInits;    /* initialisations, divergence recoveries included   */
    uint32                    SensorRejects;  /* malformed readings discarded (non-finite, bad quat) */
    uint32                    Status;         /* latest NAV_STATUS_*                               */
    float                     LastInnovSigma; /* Mahalanobis distance of the last LIDAR fix        */
    float                     PosSigma_m;
    float                     VelSigma_ms;
    float                     AttSigma_rad;
    float                     GyroBias_rads[3];  /* attitude filter's gyro bias estimate          */
    float                     AccelBias_mss[3];  /* translation filter's accelerometer bias est.  */
    uint32                    AttRejected;       /* attitude fixes that failed the gate           */
} NAV_HkTlm_t;

_Static_assert(sizeof(NAV_SolutionMsg_t) == 184, "NAV SOLUTION layout changed - update LC WDT and ground tools");
_Static_assert(offsetof(NAV_SolutionMsg_t, SimTime_s) == 16, "NAV SOLUTION layout changed - update ground tools");
_Static_assert(sizeof(NAV_HkTlm_t) == 96, "NAV HK layout changed - update ground tools");

#endif /* NAV_MSG_H */
