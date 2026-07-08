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
** Layout: 28 × float = 112 bytes. Naturally 4-byte aligned; no padding required.
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
    float MaxBurnDuration;  /* s — cap so thruster stops before next GNC wakeup */

    /*
    ** Lateral position controller
    */
    float LatKp;            /* lateral speed = KP × position error (m/s per m) */
    float MaxLatSpeed;      /* lateral speed cap                   (m/s)        */

    /*
    ** APPROACH lateral position gain.
    **
    ** APPROACH holds the centerline continuously, same as CORRECT, but with a
    ** gentler gain. The attitude deadband tightens to 0.5× in APPROACH (nose-
    ** on-port precision), leaving less headroom to absorb the attitude
    ** disturbance each lateral burn causes before it trips a correction and
    ** couples back into lateral (the limit cycle documented on AttDeadband_deg
    ** below). A softer gain means smaller, more frequent corrective nudges
    ** instead of CORRECT's stronger pull — still always tracking the axis,
    ** just less likely to kick off that coupling.
    */
    float LatKp_Approach;    /* lateral speed = KP × position error (m/s per m),
                              ** used in place of LatKp during APPROACH           */

    /*
    ** Corridor-relative phase gate (APPROACH → CORRECT revert only).
    **
    ** The physical docking corridor (Unity ApproachCorridor.cs) is a cone of
    ** half-angle ConeHalfAngle_deg from the target port, so the *absolute*
    ** lateral offset it tolerates shrinks with range: allowed_m = Range_m *
    ** tan(ConeHalfAngle_deg). A fixed-meter gate doesn't scale with that — a
    ** threshold loose enough to avoid nuisance re-entries at long range becomes
    ** far looser than the real corridor at short range. That mismatch is what
    ** let the vehicle drift to 0.86 m off-axis while still reading as "inside
    ** the gate" at a fixed 1.5 m threshold.
    **
    ** ConeHalfAngle_deg must match Unity's ApproachCorridor.coneHalfAngle.
    **
    ** This pair only governs the *revert* to CORRECT once APPROACH is already
    ** underway — the physical corridor is the right reference for "has this
    ** gotten dangerously off-axis for the current range." The CORRECT → APPROACH
    ** *entry* gate is LatEntryThreshold_m below instead: a fixed absolute value,
    ** because "is initial convergence good enough to commit to closure" is a
    ** mission-precision choice, not a function of how forgiving the corridor
    ** happens to be at long range (a corridor-relative entry gate let LAT_CORR
    ** hand off to APPROACH 4.46 m off-axis at 33 m range — technically inside
    ** half the cone, but nowhere near "converged").
    */
    float ConeHalfAngle_deg;   /* deg — must match Unity ApproachCorridor.coneHalfAngle        */
    float MinAllowedLateral_m; /* m — floor on the corridor radius so the gate doesn't vanish
                                ** to zero right at the port                                   */
    float LatEntryThreshold_m; /* m — fixed absolute offset LAT_CORR must converge to before
                                ** entering APPROACH, independent of range                     */
    float CorridorMarginOut;   /* fraction (0-1) of corridor radius — revert to CORRECT once
                                ** offset exceeds this fraction                                */

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
    ** is not spinning fast (SpinThreshold_rads below).  This breaks the limit
    ** cycle caused by lateral correction burns disturbing attitude, which then
    ** triggers corrective burns that themselves couple back into lateral — the
    ** core feedback loop.  Set to 0.0 to disable (always correct attitude, old
    ** behaviour).
    */
    float AttDeadband_deg;  /* deg — skip attitude correction below this error  */

    /*
    ** Spin-rate override.
    **
    ** Even inside AttDeadband_deg, a nonzero angular rate this small or larger
    ** still fires a correction — otherwise a small *sustained* residual rate
    ** (e.g. left over from an earlier correction's overshoot) can silently
    ** accumulate into a large angle error over many seconds before the angle
    ** deadband itself ever notices. Deliberately tighter than it looks: with
    ** angle error near zero (that's the only time this override matters — a
    ** large angle error already fires unconditionally), AttitudeAxis's own
    ** omega_tgt = AttKp × error comes out near zero too, so the resulting
    ** correction is naturally a small rate-damping nudge, not a large shove —
    ** this does not reintroduce the angle-deadband's chatter problem, it only
    ** catches drift the angle deadband was never meant to allow indefinitely.
    */
    float SpinThreshold_rads; /* rad/s — fires correction even inside AttDeadband_deg
                               ** once |AngVel| on any axis reaches this              */

    /*
    ** Lateral velocity deadband
    ** Only fire a lateral correction burn when the velocity error exceeds this
    ** threshold.  Without it, 400 N thrusters create a bang-bang limit cycle:
    ** each impulse overshoots the target lateral velocity, the next cycle
    ** fires the opposite direction, and so on — visible as F_x alternating
    ** sign every cycle.  A small deadband lets the vehicle coast through tiny
    ** velocity errors instead of chasing them.
    **
    ** MUST stay well below LatKp × (smallest lateral offset still worth
    ** actively correcting) — GNC_APP_LateralAxis's target velocity is
    ** v_tgt = -LatKp × pos, so if this deadband exceeds that product the
    ** controller never clears it and does nothing at all below
    ** pos = LatVelDeadband_ms / LatKp. At the old 0.015, that floor was
    ** 0.015/0.02 = 0.75 m — larger than LatEntryThreshold_m (0.3 m), so
    ** LAT_CORR could get stuck orbiting the edge of that dead zone forever,
    ** never converging enough to hand off to APPROACH (confirmed in
    ** telemetry: F=(0,0,0) for 80+ consecutive cycles while Lat climbed
    ** freely from 0.67 m to 0.85 m). Lowered so the dead zone (now ~0.1 m)
    ** sits safely inside the entry threshold instead of outside it.
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
