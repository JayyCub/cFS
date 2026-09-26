#ifndef GNC_APP_H
#define GNC_APP_H

#include <math.h>
#include "cfe.h"
#include "osapi.h"
#include "gnc_app_msgids.h"
#include "gnc_app_tbl.h"
#include "sim_io_msgids.h"
#include "sim_io_msg.h"

/*
** Pipe configuration
**
** GNC owns no sockets: vehicle state arrives as SIM_IO_SIM_STATE_MID and the
** actuator command leaves as SIM_IO_WRENCH_CMD_MID, both on the Software Bus.
** SIM_STATE arrival IS the control-cycle trigger (lock-step with the sim).
*/
#define GNC_APP_PIPE_NAME  "GNC_CMD_PIPE"
#define GNC_APP_PIPE_DEPTH 32

/*
** GNC controller gain defaults — documented here for reference.
** At runtime the control law reads from the loaded GNC_ParamTbl_t (gnc_app_tbl.h).
** These #defines are used only by gnc_param_tbl.c to populate the default table.
**
** AXIS CONVENTION (approach along world +Z, ship upright):
**   Approach axis  : chaser body +Z = world +Z (transform.forward toward target)
**                    bit 4 (+Z) closes range; bit 5 (-Z) brakes.
**   Lateral X axis : body +X = world +X (transform.right, left-right)
**                    bit 0 = +X force;  bit 1 = -X force
**   Lateral Y axis : body +Y = world +Y (transform.up, up-down)
**                    bit 2 = +Y force;  bit 3 = -Y force
**
** Vel_Z is the approach (axial) velocity — do NOT use it in the lateral channel.
** True lateral drift axes are Vel_X (world X) and Vel_Y (world Y).
**
** BURN DURATION MODEL:
**   Rather than holding a thruster ON for a full 1-second cycle (binary mode),
**   the GNC computes a proportional burn duration each cycle:
**       duration = delta_v_needed / thruster_accel
**   Unity fires each commanded thruster for exactly that many seconds, then coasts.
**   The minimum-burn threshold acts as the implicit dead-band: velocity errors
**   too small to warrant a GNC_MIN_BURN_DURATION pulse are left to coast.
**
**   COUPLING WARNING: GNC_THRUSTER_FORCE and GNC_VEHICLE_MASS must always match
**   the values set in the Unity Inspector (RCSModel.thrusterForce and the
**   Rigidbody mass on the chaser GameObject).  If they drift the GNC will
**   systematically under- or over-shoot every burn.
*/
#define GNC_AXIAL_KP          0.02f   /* target closing speed = KP * range (m/s per m) */
#define GNC_MIN_CLOSE_SPEED   0.02f   /* minimum approach speed (m/s) */
#define GNC_MAX_CLOSE_SPEED   0.30f   /* maximum approach speed (m/s) */

#define GNC_THRUSTER_FORCE    10.0f   /* N per thruster — must match RCSModel.thrusterForce */
#define GNC_VEHICLE_MASS     200.0f   /* kg — must match Rigidbody mass in Unity inspector */
#define GNC_ROT_ACCEL          0.30f  /* rad/s² per attitude thruster — tune to match Unity */
#define GNC_RCS_MOMENT_ARM     1.5f   /* m — thruster pod radius used in mask→wrench conversion; must match Unity scene */

#define GNC_MIN_BURN_DURATION  0.050f /* s — pulses shorter than this are skipped (coast) */
#define GNC_MAX_BURN_DURATION  0.950f /* s — caps burn so thruster stops before next 1 Hz tick */

/*
** Lateral position controller gains
**
** The lateral channel commands a target lateral velocity proportional to the
** lateral position error, then fires burns to track that velocity — identical
** in structure to the axial channel.  Pos_X and Pos_Y serve as lateral errors
** under the assumption that the target's docking axis runs through the world
** origin (valid for this fixed-target simulation).  A moving-target upgrade
** would replace these with port-relative lateral offsets in the telemetry.
*/
#define GNC_LAT_KP           0.05f   /* commanded lateral speed = KP_LAT * position error (m/s per m) */
#define GNC_MAX_LAT_SPEED    0.10f   /* lateral speed cap — prevents aggressive cross-track burns (m/s) */

/*
** Phase gate thresholds — hysteresis pair that prevents rapid phase bouncing.
**
** The GNC uses LateralOffset_m (Unity-computed perpendicular distance from the
** docking axis) as the gate signal.  Hysteresis is intentional: once the
** approach phase is entered the chaser will not revert to lateral-correct unless
** the offset grows significantly, avoiding nuisance re-entries on small transients.
*/
#define GNC_LAT_APPROACH_GATE  0.50f /* m — enter APPROACH when lateral offset drops below this */
#define GNC_LAT_CORRECT_GATE   1.00f /* m — enter LATERAL_CORRECT when offset rises above this  */

/*
** Attitude PD controller gains
**
** AttKp maps attitude error (radians) to a target angular rate (rad/s).
** The D term is provided implicitly by AngVel in the telemetry — same structure
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
#define GNC_APP_TLM_LOSS_EID      25  /* telemetry loss exceeded timeout — auto-abort */
#define GNC_APP_ANOMALY_EID       26  /* actuator authority anomaly (commanded burn underdelivering) */
#define GNC_APP_BAD_DT_EID        27  /* SIM_STATE CycleDt_s out of range — cycle skipped */
#define GNC_APP_LINK_INF_EID      28  /* sim link restored after a stale period            */

/*
** Ground command function codes
**
** NOOP            — heartbeat; no state change; increments CmdCount.
** RESET_COUNTERS  — zero CmdCount, CmdErrCount, UdpPacketsReceived.
** HOLD            — immediately enter HOLD phase (station-keep, await GO).
** GO              — release HOLD or ABORT latch; resume guidance at CORRECT.
** ABORT           — inhibit all thrust immediately; latch IDLE until GO received.
*/
#define GNC_APP_NOOP_CC            0
#define GNC_APP_RESET_COUNTERS_CC  1
#define GNC_APP_HOLD_CC            2
#define GNC_APP_GO_CC              3
#define GNC_APP_ABORT_CC           4

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
typedef struct { CFE_MSG_CommandHeader_t CmdHeader; } GNC_APP_NoopCmd_t;
typedef struct { CFE_MSG_CommandHeader_t CmdHeader; } GNC_APP_ResetCountersCmd_t;
typedef struct { CFE_MSG_CommandHeader_t CmdHeader; } GNC_APP_HoldCmd_t;
typedef struct { CFE_MSG_CommandHeader_t CmdHeader; } GNC_APP_GoCmd_t;
typedef struct { CFE_MSG_CommandHeader_t CmdHeader; } GNC_APP_AbortCmd_t;

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
** ABORT command sets IDLE + AbortLatch; all thrust inhibited until GO received.
**
** Autonomous transitions (CORRECT ↔ APPROACH) are managed by GNC_APP_SelectPhase()
** with a hysteresis pair (GNC_LAT_APPROACH_GATE / GNC_LAT_CORRECT_GATE).
** Every transition generates a GNC_APP_PHASE_INF_EID EVS event for ground visibility.
*/
typedef enum
{
    GNC_PHASE_IDLE     = 0,  /* awaiting telemetry / abort latched      */
    GNC_PHASE_CORRECT  = 1,  /* lateral correction + axial station-keep */
    GNC_PHASE_APPROACH = 2,  /* final approach — axial + lateral hold   */
    GNC_PHASE_DOCKED   = 3,  /* docked — all thrust inhibited           */
    GNC_PHASE_HOLD     = 4   /* ground-commanded hold — await GO        */
} GNC_Phase_t;

/*
** Vehicle state input: SIMLINK_SimState_t (sim_io/fsw/inc/simlink_icd.h),
** delivered in SIM_IO_SimStateMsg_t. RelPos/RelVel are chaser-minus-target in
** Unity world axes (+Y radial, -Z along-track, +X cross-track) — CW feedforward
** input only; lateral control steers on the port-relative LatOffset_X/Y.
*/
typedef SIMLINK_SimState_t GNC_APP_UnityTlm_t;

/*
** Housekeeping telemetry packet (published to SB each wakeup)
**
** NOTE: struct layout must remain word-aligned.  Spare[] pads Phase (uint8)
** to a full 32-bit word so downstream ground tools and COSMOS/OpenMCT field
** offsets stay deterministic.
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint32                    WakeupCount;
    uint32                    CmdCount;
    uint32                    CmdErrCount;
    uint32                    UdpPacketsReceived; /* SIM_STATE messages received (name kept for ground-tool offsets) */
    uint8                     Phase;     /* current GNC_Phase_t value (0=IDLE 1=CORRECT 2=APPROACH 3=DOCKED) */
    uint8                     Spare[3];  /* explicit alignment padding — do not reuse without updating tools */
    /* Phase 5D — LC watchpoint fields (offsets 32, 36, 40) */
    float                     ClosingSpeed_ms;   /* latest Unity closing speed; mirrored for LC overspeed WP */
    float                     LateralOffset_m;   /* latest Unity lateral offset; mirrored for LC corridor WP */
    uint32                    TlmStaleSec;       /* consecutive 1 Hz HK periods with no SIM_STATE from
                                                  ** SIM_IO (now genuinely seconds of wall time). Drives the
                                                  ** sim-link-loss auto-abort in GNC_APP_SendHk. */
    uint32                    ActuatorAnomalyCount; /* times the axial actuator-health monitor flagged a
                                                     ** commanded burn as significantly underdelivering        */
    /* Attitude — previously visible only in the (often line-truncated) EVS
    ** wakeup log text, not as an actual telemetry point a ground tool could
    ** trend or alarm on. Mirrors the latest Unity telemetry each wakeup cycle,
    ** same pattern as ClosingSpeed_ms/LateralOffset_m above. */
    float                     PitchError_deg;
    float                     YawError_deg;
    float                     RollError_deg;
} GNC_APP_HkTlm_t;

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

    /* Axial actuator-health monitor (Phase 2) — compares the previous cycle's
    ** commanded axial burn against the closing-speed change actually observed
    ** this cycle, to catch a thruster-delivery regression (like the allocator
    ** under-delivery bug) automatically instead of relying on someone reading
    ** the EVS log. Observational only — see GNC_APP_CheckActuatorHealth. */
    float              PrevCtrlFz;          /* previous cycle's commanded axial force (N)        */
    float              PrevCtrlDuration_s;  /* previous cycle's burn duration (s); 0 = nothing to check */
    float              PrevClosingSpeed_ms; /* ClosingSpeed_ms at the time PrevCtrlFz was computed */
    uint32             UnderDeliveryStreak; /* consecutive cycles where the observed axial Δv came in
                                             ** well under what PrevCtrlFz/PrevCtrlDuration_s predicted */

    /* Latest vehicle state (single-task ownership — no mutex needed) */
    GNC_APP_UnityTlm_t LatestTlm;          /* most recent SIM_STATE payload                 */
    bool               HaveTlm;            /* at least one SIM_STATE received                */
    bool               TlmSinceLastHk;     /* SIM_STATE seen since the previous 1 Hz HK tick */
    bool               LinkLossLatched;    /* auto-abort fired for the current outage        */

    uint32             PrevSeq;            /* Seq of the previous SIM_STATE (continuity check) */
    bool               HavePrevSeq;

    /* Actuator command output (SB) */
    SIM_IO_WrenchCmdMsg_t WrenchMsg;

    /* CFE_TBL parameter table */
    CFE_TBL_Handle_t        ParamTblHandle;
    const GNC_ParamTbl_t   *ParamTblPtr;    /* valid after Init; refreshed each wakeup */
} GNC_APP_Data_t;

extern GNC_APP_Data_t GNC_APP_Data;

/*
** Control output — mask of thrusters to fire plus how long to fire them.
** Computed by GNC_APP_ComputeControl, published by GNC_APP_PublishCommand.
*/
typedef struct
{
    float Fx, Fy, Fz;   /* body-frame force  (N)   — positive = +axis direction */
    float Tx, Ty, Tz;   /* body-frame torque (N·m) — positive = +axis direction */
    float duration_s;    /* seconds each fired thruster fires; 0.0 = coast        */
} GNC_Control_t;

/*
** Function prototypes
*/
void         GNC_APP_Main(void);
CFE_Status_t GNC_APP_Init(void);
void         GNC_APP_ProcessSimState(const SIM_IO_SimStateMsg_t *Msg);
void         GNC_APP_SendHk(void);
void         GNC_APP_ProcessCmd(CFE_SB_Buffer_t *MsgBuf);
/* Publishes the body-frame wrench as SIM_IO_WRENCH_CMD_MID, tagged with the
** Seq of the SIM_STATE it answers so Unity can release that lock-step cycle. */
void         GNC_APP_PublishCommand(const GNC_Control_t *ctrl, uint32 Seq);

#endif /* GNC_APP_H */
