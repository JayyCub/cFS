#ifndef GNC_APP_H
#define GNC_APP_H

#include <math.h>
#include "cfe.h"
#include "osapi.h"
#include "gnc_app_msgids.h"
#include "gnc_app_msg.h"
#include "gnc_app_tbl.h"
#include "sim_io_msgids.h"
#include "sim_io_msg.h"
#include "nav_msgids.h"
#include "nav_msg.h"
#include "rcs_msgids.h"
#include "rcs_msg.h"
#include "lc_msgids.h"
#include "lc_msg.h"

/*
** Pipe configuration
**
** GNC owns no sockets, no sensors and knows nothing about individual
** thrusters: the vehicle state is the NAV app's solution (NAV_SOLUTION_MID),
** and each cycle GNC publishes the impulse it wants (RCS_ACT_REQ_MID) for the
** RCS app to allocate to thrusters. NAV_SOLUTION arrival IS the control-cycle
** trigger (lock-step: SIM_IO -> NAV -> GNC). From SIM_IO's SIM_STATE GNC reads
** only the crew hand controllers and the docking-mechanism switches, plus its
** arrival for the sim-link watchdog.
*/
#define GNC_APP_PIPE_NAME  "GNC_CMD_PIPE"
#define GNC_APP_PIPE_DEPTH 32

/*
** GNC controller gain defaults — documented here for reference only.
** At runtime the control law reads from the loaded GNC_ParamTbl_t (gnc_app_tbl.h).
**
** AXIS CONVENTION (approach along world +Z, ship upright):
**   Approach axis  : chaser body +Z = world +Z (transform.forward toward target)
**   Lateral X axis : body +X = world +X (transform.right, left-right)
**   Lateral Y axis : body +Y = world +Y (transform.up, up-down)
**
** RelVel_Z is the approach (axial) velocity — do NOT use it in the lateral channel.
**
** ACTUATOR MODEL (realism phase 2):
**   GNC outputs the impulse to impart this cycle — P = m·Δv (N·s) and
**   L = I·Δω (N·m·s), body frame. The RCS app turns that into per-thruster valve
**   on-times from its own thruster table (allocation + PWM + minimum impulse
**   bit). GNC needs only mass properties; thruster force, geometry and burn
**   timing are no longer GNC parameters.
*/
#define GNC_AXIAL_KP          0.02f   /* target closing speed = KP * range (m/s per m) */
#define GNC_MIN_CLOSE_SPEED   0.02f   /* minimum approach speed (m/s) */
#define GNC_MAX_CLOSE_SPEED   0.30f   /* maximum approach speed (m/s) */

/* Hand-controller deflection (either stick, any axis) that hands control to the crew */
#define GNC_HC_TAKEOVER_THRESHOLD 0.5f

/*
** Lateral position controller gains
**
** The lateral channel commands a target lateral velocity proportional to the
** lateral position error, then fires burns to track that velocity — identical
** in structure to the axial channel. The lateral errors are NAV's docking-port
** offsets from the docking axis (LatOffset_X/Y, docking frame).
*/
#define GNC_LAT_KP           0.05f   /* commanded lateral speed = KP_LAT * position error (m/s per m) */
#define GNC_MAX_LAT_SPEED    0.10f   /* lateral speed cap — prevents aggressive cross-track burns (m/s) */

/*
** Phase gate thresholds — hysteresis pair that prevents rapid phase bouncing.
**
** The GNC uses LateralOffset_m (NAV's port distance from the docking axis) as
** the gate signal.  Hysteresis is intentional: once the
** approach phase is entered the chaser will not revert to lateral-correct unless
** the offset grows significantly, avoiding nuisance re-entries on small transients.
*/
#define GNC_LAT_APPROACH_GATE  0.50f /* m — enter APPROACH when lateral offset drops below this */
#define GNC_LAT_CORRECT_GATE   1.00f /* m — enter LATERAL_CORRECT when offset rises above this  */

/*
** Attitude PD controller gains
**
** AttKp maps attitude error (radians) to a target angular rate (rad/s).
** The D term is provided implicitly by the gyro rate AngVel — same structure
** as the lateral channels (omega_tgt = Kp × error; omega_err = omega_tgt − AngVel).
** MaxAttRate caps the commanded angular velocity on each axis.
*/
#define GNC_ATT_KP       0.50f   /* (rad/s)/rad — attitude proportional gain     */
#define GNC_MAX_ATT_RATE 0.20f   /* rad/s — cap on commanded attitude rate        */

/*
** Orbital mechanics constant for CW feedforward (Phase 5E)
**
** ISS mean motion n = sqrt(mu / a³), a ≈ 6778 km (400 km LEO).
** Used only in ComputeControl to cancel the differential gravity that
** ClohessyWiltshire.cs applies each FixedUpdate.  Not a control gain —
** changing it affects fidelity, not aggressiveness.
*/
#define GNC_CW_MEAN_MOTION  0.00113f  /* rad/s */

#define GNC_DEG2RAD  0.01745329f  /* π / 180 — shared by attitude and corridor-angle math */

/*
** GNC cycle period — no longer a compile-time constant. Each SIM_STATE carries
** CycleDt_s (the sim time one lock-step cycle covers, nominally 0.2 s / 5 Hz,
** set by Unity's UdpTelemetrySender.gncCycleSec). Samples outside this range
** are rejected rather than fed to the CW feedforward.
*/
#define GNC_MIN_CYCLE_DT_S 0.01f
#define GNC_MAX_CYCLE_DT_S 1.0f

/*
** LC actionpoint rate group sampled by GNC at the end of every control cycle.
**
** LC evaluates watchpoints as packets arrive but actionpoints only when asked
** (LC_SAMPLE_AP_MID). On a flight vehicle the scheduler sends that request in
** the minor-frame slot right after GNC; here the lock-step SIM_STATE *is* the
** minor frame, so GNC closes its own cycle with the request. That keeps the
** per-cycle monitors in sim time — LC's persistence counts cycles, not wall
** seconds, even when the sim runs faster than real time. The wall-clock group
** (link loss, which must evaluate when no SIM_STATE arrives) is sampled by
** SCH_LAB instead. Indices must match sample_defs/cpu1/tables/lc_def_adt.c.
*/
#define GNC_APP_LC_CYCLE_AP_FIRST 1
#define GNC_APP_LC_CYCLE_AP_LAST  5

/* Seconds of silence after which a returning SIM_STATE is worth an event */
#define GNC_LINK_RESTORED_EVT_SEC 2

/*
** Event IDs
*/
#define GNC_APP_INIT_INF_EID      1
#define GNC_APP_WAKEUP_INF_EID    2
#define GNC_APP_PIPE_ERR_EID      3
#define GNC_APP_SUB_ERR_EID       4
#define GNC_APP_UDP_INIT_INF_EID  5
#define GNC_APP_UDP_ERR_EID       6
#define GNC_APP_MUTEX_ERR_EID     7
#define GNC_APP_TASK_ERR_EID      8
#define GNC_APP_CMD_INIT_INF_EID  9
#define GNC_APP_CMD_ERR_EID       10
#define GNC_APP_PHASE_INF_EID     11  /* GNC phase transition                      */
#define GNC_APP_NOOP_INF_EID      12  /* NOOP command accepted                      */
#define GNC_APP_RST_INF_EID       13  /* RESET_COUNTERS command accepted            */
#define GNC_APP_HOLD_INF_EID      14  /* HOLD command accepted                      */
#define GNC_APP_GO_INF_EID        15  /* GO command accepted                        */
#define GNC_APP_ABORT_INF_EID     16  /* ABORT command accepted (CRITICAL severity) */
#define GNC_APP_CMD_LEN_ERR_EID   17  /* command rejected — wrong packet length     */
#define GNC_APP_CMD_CODE_ERR_EID  18  /* command rejected — unknown function code   */
#define GNC_APP_TBL_UPD_INF_EID   19  /* parameter table successfully reloaded      */
#define GNC_APP_TBL_ERR_EID       20  /* parameter table load or access error       */
#define GNC_APP_HOLDPT1_INF_EID   21  /* autonomous hold point 1 reached            */
#define GNC_APP_HOLDPT2_INF_EID   22  /* autonomous hold point 2 reached            */
#define GNC_APP_TBL_VAL_ERR_EID   23  /* parameter table failed load-time validation */
#define GNC_APP_BAD_CTRL_EID      24  /* non-finite control output sanitized to coast */
/* 25, 26 retired in realism phase 3: link-loss auto-abort and the actuator
** anomaly event moved to LC actionpoints (LC emits the event, SC responds). */
#define GNC_APP_BAD_DT_EID        27  /* SIM_STATE CycleDt_s out of range — cycle skipped */
#define GNC_APP_LINK_INF_EID      28  /* sim link restored after a stale period            */
#define GNC_APP_MANUAL_INF_EID    29  /* crew hand-controller takeover                     */
#define GNC_APP_NAV_INF_EID       30  /* NAV relative solution lost / restored             */
#define GNC_APP_CAM_INF_EID       31  /* collision-avoidance manoeuvre started / complete   */

/* DEPART gives up after this many cycles even if the burn isn't done (60 s) */
#define GNC_CAM_MAX_CYCLES 300
/* While burning on for passive safety, request this much retreat Δv per cycle */
#define GNC_CAM_STEP_MS 0.02f
/* Free-drift prediction grid step (s) */
#define GNC_DRIFT_STEP_S 20.0f

/* Ground command function codes: gnc_app_msg.h (shared with the SC RTS tables) */

/*
** CFE_TBL validator return code — GNC_APP_ValidateParamTbl returns this (rather
** than CFE_SUCCESS) when a loaded table image fails a sanity check, so
** CFE_TBL_Load/Manage rejects it and keeps running on the last-known-good table.
*/
#define GNC_APP_TBL_VALIDATE_ERR (0xc0000010)

/*
** Ground command message structures.
** Each command carries only a CCSDS primary + secondary header (no payload).
** Individual typedefs are kept separate so sizeof() checks remain unambiguous
** and payloads can be added per-command in a future update without refactoring.
*/
typedef GNC_APP_NoArgsCmd_t GNC_APP_NoopCmd_t;
typedef GNC_APP_NoArgsCmd_t GNC_APP_ResetCountersCmd_t;
typedef GNC_APP_NoArgsCmd_t GNC_APP_HoldCmd_t;
typedef GNC_APP_NoArgsCmd_t GNC_APP_GoCmd_t;
typedef GNC_APP_NoArgsCmd_t GNC_APP_AbortCmd_t;

/*
** GNC operational phase — drives which control law is active each cycle.
**
** Modelled after real proximity-ops guidance state machines (e.g. Dragon RPOD):
**   IDLE    — no valid telemetry, or ABORT latched. All thrust inhibited.
**             Auto-promotes to CORRECT on first telemetry (unless AbortLatch set).
**   CORRECT — lateral offset exceeds gate; station-keep axially while
**             driving Pos_X/Y to zero before committing to an approach.
**   APPROACH— on-axis; proportional axial guidance + lateral position hold.
**   DOCKED  — contact confirmed; all thrust inhibited.
**   HOLD    — ground-commanded hold; station-keep (no axial closure).
**             Entered only by GNC_APP_HOLD_CC. Released only by GNC_APP_GO_CC.
**
**   MANUAL  — crew flying on the hand controllers (THC accel command, RHC rate
**             command with rate hold). Entered on any stick deflection from any
**             undocked phase; released only by GO (-> CORRECT) or ABORT.
**   DEPART  — abort: collision-avoidance manoeuvre (realism phase 5). Retreat
**             along the docking axis until the free drift is passively safe,
**             then IDLE. GO is refused until it completes; the crew can still
**             take over on the sticks.
**
** ABORT command sets AbortLatch and starts DEPART (IDLE if already stopped);
** after the manoeuvre all thrust is inhibited until GO.
**
** Autonomous transitions (CORRECT ↔ APPROACH) are managed by GNC_APP_SelectPhase()
** with a hysteresis pair (GNC_LAT_APPROACH_GATE / GNC_LAT_CORRECT_GATE).
** Every transition generates a GNC_APP_PHASE_INF_EID EVS event for ground visibility.
*/
typedef enum
{
    GNC_PHASE_IDLE     = GNC_APP_PHASE_IDLE,     /* awaiting telemetry / abort latched      */
    GNC_PHASE_CORRECT  = GNC_APP_PHASE_CORRECT,  /* lateral correction + axial station-keep */
    GNC_PHASE_APPROACH = GNC_APP_PHASE_APPROACH, /* final approach — axial + lateral hold   */
    GNC_PHASE_DOCKED   = GNC_APP_PHASE_DOCKED,   /* docked — all thrust inhibited           */
    GNC_PHASE_HOLD     = GNC_APP_PHASE_HOLD,     /* ground-commanded hold — await GO        */
    GNC_PHASE_MANUAL   = GNC_APP_PHASE_MANUAL,   /* crew hand-controller flight — await GO  */
    GNC_PHASE_DEPART   = GNC_APP_PHASE_DEPART    /* abort retreat (CAM) in progress         */
} GNC_Phase_t;

/*
** Per-cycle GNC input, assembled in GNC_APP_ProcessNavSolution from the NAV
** solution (vehicle state) and the latest SIM_STATE (crew and mechanism
** hardware). Field names predate phase 4, when Unity sent these values
** computed from truth; the control law reads them unchanged.
**
**   docking frame D: X/Y lateral, +Z toward the target port (= world axes in
**                    this scene, nav_cfg_tbl.c)
**   LVLH:            Unity world axes, +Y radial, -Z along-track, +X cross-track
*/
typedef struct
{
    /* NAV solution */
    float Range_m;                   /* port to port (m)                                         */
    float ClosingSpeed_ms;           /* port closing rate along D +Z, + = closing (m/s)          */
    float LateralOffset_m;           /* port distance from the docking axis (m)                  */
    float LatOffset_X, LatOffset_Y;  /* signed port lateral offset, D X/Y (m)                    */
    float PortPos_D[3];              /* chaser port - target port, docking frame (m)             */
    float LatVel_X, LatVel_Y;        /* lateral velocity the lateral channel steers on (m/s):
                                     ** port point (D) or CoM (LVLH), per LatVelAtPort          */
    float RelPos_X, RelPos_Y, RelPos_Z; /* chaser CoM - target ref, LVLH (m) — CW feed-forward  */
    float RelVel_X, RelVel_Y, RelVel_Z; /* (m/s)                                                */
    float AngVel_X, AngVel_Y, AngVel_Z; /* gyro body rate (rad/s)                               */
    float PitchError_deg;            /* attitude error, body X/Y/Z rotation to the docking       */
    float YawError_deg;              /* attitude (quaternion based, NAV)                         */
    float RollError_deg;
    float DeltaV_B[3];               /* IMU-measured Δv over the cycle that just ended (m/s)     */
    float AttQuat_L[4];              /* q_L_B, NAV attitude estimate                             */
    float DockQuat_L[4];             /* q_L_D, docking frame axes in LVLH                        */
    bool  RelNavValid;               /* relative position/velocity usable                        */
    bool  AttValid;                  /* attitude error usable                                    */
    bool  RateValid;                 /* AngVel usable                                            */
    bool  DvValid;                   /* DeltaV_B usable                                          */

    /* SIM_STATE hardware */
    float Thc[3];   /* crew translation hand controller, body X/Y/Z, -1..+1 */
    float Rhc[3];   /* crew rotation hand controller, pitch/yaw/roll, -1..+1 */
    bool  Captured; /* soft-capture latches engaged                          */
} GNC_APP_Input_t;

/* HK and per-cycle STATE telemetry packets: gnc_app_msg.h */

/*
** Application global data
*/
typedef struct
{
    uint32            RunStatus;
    CFE_SB_PipeId_t   CmdPipe;
    GNC_APP_HkTlm_t   HkTlm;

    /* GNC phase state — persists across wakeup cycles to support hysteresis */
    GNC_Phase_t        Phase;
    bool               AbortLatch;   /* set by ABORT cmd; inhibits IDLE→CORRECT until GO */
    bool               HoldPt1Armed; /* true until hold point 1 fires; re-armed by ABORT+GO */
    bool               HoldPt2Armed; /* true until hold point 2 fires; re-armed by ABORT+GO */
    float              HoldRange_m;  /* Range_m captured at HOLD entry; axial position target */
    uint32             SettleCounter; /* consecutive cycles the CORRECT->APPROACH settle gate
                                       ** (position + velocity + attitude rate) has held; reset
                                       ** to 0 on any cycle where a condition fails */
    bool               BadCtrlLatched; /* true while the last ComputeControl output was non-finite;
                                        ** suppresses repeat GNC_APP_BAD_CTRL_EID events for the same episode */

    /* Axial actuator-health monitor — compares the previous cycle's commanded
    ** axial burn against the body-axis Δv the IMU measured this cycle. GNC
    ** only measures (UnderDeliveryStreak, published in STATE); LC decides when
    ** that is an anomaly and SC responds — see GNC_APP_CheckActuatorHealth. */
    float              PrevAchievedPz_Ns;   /* axial impulse RCS actually commanded last cycle (N·s) */
    bool               PrevBurnValid;       /* PrevAchievedPz_Ns is valid for the previous Seq       */
    uint32             UnderDeliveryStreak; /* consecutive cycles where the measured axial Δv came in
                                             ** well under what PrevAchievedPz_Ns predicted */

    /* Latest inputs (single-task ownership — no mutex needed) */
    GNC_APP_Input_t    LatestIn;           /* input of the most recent control cycle         */
    bool               HaveTlm;            /* at least one SIM_STATE received                */
    bool               TlmSinceLastHk;     /* SIM_STATE seen since the previous 1 Hz HK tick */
    bool               InCorridor;         /* port offset inside the approach cone (NAV)      */
    bool               NavInvalidLatched;  /* NAV relative solution unusable — reported once  */

    /* Phase 5: guidance monitors and abort manoeuvre */
    float              FfAccum_D[3];       /* CW feed-forward Δv accumulated, docking frame    */
    uint32             OverspeedStreak;    /* cycles above the approach envelope               */
    uint32             CorridorStreak;     /* cycles outside the cone inside CorridorCheckRange */
    float              DriftSafeRange_m;   /* predicted closest free-drift approach (STATE)    */
    bool               DriftSafe;          /* DriftSafeRange_m >= PassiveSafeRange_m           */
    float              CamDv_ms;           /* DEPART: retreat Δv delivered                     */
    uint32             CamCycles;          /* DEPART: cycles so far                            */
    float              CamLastCmd_ms;      /* DEPART: retreat Δv requested last cycle — counted
                                           ** as delivered when the IMU can't measure it      */

    /* Hardware readings from the latest SIM_STATE, matched to the NAV solution by Seq */
    uint32             HwSeq;
    float              HwThc[3];
    float              HwRhc[3];
    uint32             HwMechFlags;
    uint32             HwSeqMismatches;    /* NAV solutions with no SIM_STATE of the same Seq */

    uint32             PrevSeq;            /* Seq of the previous cycle (continuity check)   */
    bool               HavePrevSeq;

    /* Actuator command output (SB) */
    RCS_ActReqMsg_t    ActReqMsg;

    /* Per-cycle state telemetry and the LC sample request that closes each cycle */
    GNC_APP_StateTlm_t StateTlm;
    LC_SampleAPCmd_t   LcSampleCmd;

    /* CFE_TBL parameter table */
    CFE_TBL_Handle_t        ParamTblHandle;
    const GNC_ParamTbl_t   *ParamTblPtr;    /* valid after Init; refreshed each wakeup */
} GNC_APP_Data_t;

extern GNC_APP_Data_t GNC_APP_Data;

/*
** Control output — impulse to impart this cycle, body frame (realism phase 2).
** Computed by GNC_APP_ComputeControl, published by GNC_APP_PublishCommand.
*/
typedef struct
{
    float Px, Py, Pz;   /* linear impulse  (N·s)   = VehicleMass × Δv          */
    float Lx, Ly, Lz;   /* angular impulse (N·m·s) = Inertia × Δω per axis     */
} GNC_Control_t;

/*
** Function prototypes
*/
void         GNC_APP_Main(void);
CFE_Status_t GNC_APP_Init(void);
void         GNC_APP_ProcessSimState(const SIM_IO_SimStateMsg_t *Msg);
void         GNC_APP_ProcessNavSolution(const NAV_SolutionMsg_t *Msg);
void         GNC_APP_ProcessThrusterCmd(const SIM_IO_ThrusterCmdMsg_t *Msg);
void         GNC_APP_SendHk(void);
void         GNC_APP_ProcessCmd(CFE_SB_Buffer_t *MsgBuf);
/* Publishes the impulse request as RCS_ACT_REQ_MID, tagged with the
** Seq of the SIM_STATE it answers so Unity can release that lock-step cycle. */
void         GNC_APP_PublishCommand(const GNC_Control_t *ctrl, uint32 Seq);

#endif /* GNC_APP_H */
