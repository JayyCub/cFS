#ifndef GNC_APP_TBL_H
#define GNC_APP_TBL_H

#include "cfe.h"

/*
** GNC_APP parameter table name and default load path.
** The cFS table service installs compiled table binaries to /cf/ at build time.
** This path must match the CFE_TBL_FILEDEF() declaration in gnc_param_tbl.c.
*/
#define GNC_APP_PARAM_TBL_NAME "ParamTbl"
#define GNC_APP_PARAM_TBL_FILE "/cf/gnc_param_tbl.tbl"

/*
** GNC parameter table — all gains and physical constants in one loadable struct.
**
** Moving these values out of #define constants into a CFE_TBL struct means
** ground operators can uplink a new gain set at any time without recompiling cFS.
** The table service validates the incoming image, then swaps it in on the next
** GNC wakeup cycle.  This is the standard cFS mechanism for on-orbit gain tuning.
**
** COUPLING WARNING: ThrusterForce and VehicleMass must always match the values
** set in the Unity Inspector (RCSModel.thrusterForce and Rigidbody mass).
** If they drift, every burn duration will be systematically wrong.
**
** Layout: 24 × float = 96 bytes. Naturally 4-byte aligned; no padding required.
*/
typedef struct
{
    /*
    ** Axial guidance channel
    */
    float AxialKp;          /* target closing speed = KP × range  (m/s per m) */
    float MinCloseSpeed;    /* floor on approach speed             (m/s)        */
    float MaxCloseSpeed;    /* outer cap on approach speed, before HoldPoint1_m is passed (m/s) */
    float MaxCloseSpeed_Inner; /* tighter cap once HoldPoint1_m has fired (m/s) — models the
                                ** real-world profile of a faster outer approach speed and a
                                ** slower, more cautious speed once inside the first hold point. */

    /*
    ** Physical model — must match Unity Inspector values exactly
    */
    float ThrusterForce;    /* N per thruster                                   */
    float VehicleMass;      /* kg                                               */
    float RotAccel;         /* rad/s² per attitude thruster                     */

    /*
    ** Burn duration limits
    */
    float MinBurnDuration;  /* s — pulses shorter than this coast (dead-band)   */
    float MaxBurnDuration;  /* s — cap so thruster stops before next 1 Hz tick  */

    /*
    ** Lateral position controller
    */
    float LatKp;            /* lateral speed = KP × position error (m/s per m) */
    float MaxLatSpeed;      /* lateral speed cap                   (m/s)        */

    /*
    ** Phase gate hysteresis pair (LateralOffset_m thresholds)
    */
    float LatApproachGate;  /* m — enter APPROACH when lateral offset < this   */
    float LatCorrectGate;   /* m — enter LATERAL_CORRECT when offset > this    */

    /*
    ** Autonomous hold-point waypoints (range thresholds, approach axis).
    ** SelectPhase transitions to HOLD when range drops to or below each
    ** threshold during APPROACH.  Each hold point fires at most once per
    ** approach sequence; re-armed only when an ABORT+GO is received.
    ** Set to 0.0 to disable a hold point.
    */
    float HoldPoint1_m;     /* m — outer proximity-ops waypoint (e.g. 10 m)    */
    float HoldPoint2_m;     /* m — inner proximity-ops waypoint (e.g.  3 m)    */

    /*
    ** Attitude PD controller
    ** AttKp: target angular rate = AttKp × attitude error (rad).
    ** MaxAttRate: clamp on commanded angular rate (rad/s) per axis.
    */
    float AttKp;            /* (rad/s)/rad — attitude proportional gain         */
    float MaxAttRate;       /* rad/s — max commanded angular rate per axis      */

    /*
    ** Attitude deadband
    ** Attitude corrections are suppressed on all axes when ALL three errors
    ** (pitch, yaw, roll) are within this threshold in degrees AND the vehicle
    ** is not spinning fast.  This breaks the limit cycle caused by lateral
    ** correction burns disturbing attitude, which then triggers corrective burns
    ** that themselves couple back into lateral — the core feedback loop.
    ** Set to 0.0 to disable (always correct attitude, old behaviour).
    */
    float AttDeadband_deg;  /* deg — skip attitude correction below this error  */

    /*
    ** Lateral velocity deadband
    ** Only fire a lateral correction burn when the velocity error exceeds this
    ** threshold.  Without it, 400 N thrusters at 1 Hz create a bang-bang
    ** limit cycle: each impulse overshoots the target lateral velocity, the
    ** next cycle fires the opposite direction, and so on — visible as F_x
    ** alternating sign every cycle.  A small deadband lets the vehicle coast
    ** through tiny velocity errors instead of chasing them.
    */
    float LatVelDeadband_ms; /* m/s — ignore lateral velocity errors below this */

    /*
    ** Braking deceleration constants — calibrated to actual Unity thruster geometry.
    **
    ** T00-T03 are excluded from docking; T04-T07 are approach-only (+Z).
    ** The docking brake thrusters are split into two groups:
    **   Light: T08-T11 (Brake-Yaw group)  — total -Z force ≈ 27.8 N at thrusterForce=10
    **   Hard:  T08-T15 (both brake groups) — total -Z force ≈ 57.8 N at thrusterForce=10
    **
    ** These accel values are used by SelectPhase (brake distance lookahead) and
    ** ComputeControl (HOLD axial braking).  They must match the Unity sim geometry —
    ** if thrusterForce or thruster positions change, recalibrate by summing the
    ** -Z components of the relevant group and dividing by VehicleMass.
    **
    ** Unity (RCSModel.SoftBrakeThreshold_N = 938 N) selects group by |Fz|:
    **   |Fz| < 938 N  →  light brake (T08-T11 only)  [BrakeAccel_Light_mss × mass = 612 N]
    **   |Fz| ≥ 938 N  →  hard brake  (T08-T15)        [BrakeAccel_Hard_mss  × mass = 1265 N]
    */
    float BrakeAccel_Hard_mss;  /* m/s² — deceleration from T08-T15 (hard stop)         */
    float BrakeAccel_Light_mss; /* m/s² — deceleration from T08-T11 only (soft correct)  */
    float ApproachAccel_mss;    /* m/s² — actual acceleration from approach group T04-T07  */

    /*
    ** Axial hold-position controller (HOLD phase only)
    ** Velocity-only station-keep (target closing speed = 0) cancels drift rate
    ** but never corrects accumulated range drift once it has occurred. This adds
    ** proportional position feedback toward the range captured when HOLD was
    ** entered (GNC_APP_Data.HoldRange_m), identical in structure to LatKp/MaxLatSpeed.
    */
    float AxialHoldKp;       /* target closing speed = KP × (Range_m - HoldRange_m) (m/s per m) */
    float MaxHoldSpeed;      /* cap on hold-correction closing speed                (m/s)        */

} GNC_ParamTbl_t;

#endif /* GNC_APP_TBL_H */
