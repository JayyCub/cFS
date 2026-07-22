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
** If they drift, every burn duration will be systematically wrong. See "Key
** Coupling Constraints" in Docs/DEV_REFERENCE.md for the full list of values
** that must stay in sync between this table and the Unity scripts.
**
** Layout: 34 × float = 136 bytes. Naturally 4-byte aligned; no padding required.
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
    ** CORRECT -> APPROACH settle gate (rate + duration).
    **
    ** LatEntryThreshold_m alone is a single-sample position check: the instant
    ** LateralOffset_m crosses it, APPROACH commits and Channel 1 immediately
    ** starts axial closure — even if lateral velocity or attitude rate are
    ** still actively changing at that exact instant. These three fields add a
    ** real "stopped and steady" requirement on top of the position check:
    ** lateral speed and attitude rate must also be under their own thresholds,
    ** and all three conditions must hold simultaneously for EntrySettleCycles
    ** consecutive wakeup cycles (tracked by GNC_APP_Data.SettleCounter) before
    ** the transition actually fires. Any cycle where any condition fails resets
    ** the counter to zero — no partial credit across a blip.
    */
    float LatEntryVelMax_ms;    /* m/s — |Vel_X| and |Vel_Y| must both be below this   */
    float AttEntryRateMax_rads; /* rad/s — |AngVel_X/Y/Z| must all be below this       */
    float EntrySettleCycles;    /* consecutive cycles all conditions must hold         */

    /*
    ** APPROACH -> CORRECT attitude revert threshold.
    **
    ** The lateral corridor gate above will pull the vehicle back to CORRECT if
    ** it drifts too far off-axis, but nothing previously checked attitude
    ** during APPROACH — a vehicle that stayed laterally centered while tilting
    ** or yawing hard would sail through unchecked until DockingDetector's
    ** one-shot maxAttitudeError (10°, checked only at contact) failed capture.
    ** This mirrors the corridor check for the rotational axes: if the worst of
    ** |PitchError_deg|, |YawError_deg|, |RollError_deg| exceeds this during
    ** APPROACH, revert to CORRECT (whose wider AttDeadband_deg and station-kept
    ** axial channel give the attitude loop room to actually recover) instead of
    ** continuing to close range while pointed the wrong way. Set clearly below
    ** the 10° capture requirement so there's real margin to correct before
    ** contact, but well above AttDeadband_deg's 1° APPROACH operating band so
    ** normal correction transients don't trigger nuisance reverts.
    */
    float AttRevertThreshold_deg; /* deg — revert APPROACH->CORRECT if any axis exceeds this */

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
    ** Lateral velocity deadband — split by phase.
    **
    ** Only fire a lateral correction burn when the velocity error exceeds this
    ** threshold.  Without it, 400 N thrusters create a bang-bang limit cycle:
    ** each impulse overshoots the target lateral velocity, the next cycle
    ** fires the opposite direction, and so on — visible as F_x alternating
    ** sign every cycle.  A small deadband lets the vehicle coast through tiny
    ** velocity errors instead of chasing them.
    **
    ** MUST stay well below Kp × (smallest lateral offset still worth actively
    ** correcting) for the phase it applies to — GNC_APP_LateralAxis's target
    ** velocity is v_tgt = -Kp × pos, so if the deadband exceeds that product
    ** the controller never clears it and does nothing at all below
    ** pos = deadband / Kp. At the old shared 0.015, that floor was
    ** 0.015/0.02 = 0.75 m — larger than LatEntryThreshold_m, so LAT_CORR could
    ** get stuck orbiting the edge of that dead zone forever, never converging
    ** enough to hand off to APPROACH (confirmed in telemetry: F=(0,0,0) for
    ** 80+ consecutive cycles while Lat climbed freely from 0.67 m to 0.85 m).
    **
    ** CORRECT needs a tight deadband so it can actually converge to
    ** LatEntryThreshold_m (dead zone must sit comfortably inside that gate).
    ** APPROACH wants a looser one: LatKp_Approach is already halved so its
    ** burns stay small, but constantly re-firing on sub-centimeter noise still
    ** disturbs attitude and feeds the lateral/attitude limit cycle documented
    ** on AttDeadband_deg — a wider dead zone here lets it coast through that
    ** noise instead of chasing it. A single shared value could not satisfy
    ** both constraints at once, hence the split.
    */
    float LatVelDeadband_ms;          /* m/s — CORRECT phase deadband */
    float LatVelDeadband_Approach_ms; /* m/s — APPROACH phase deadband (looser) */

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

    /*
    ** Telemetry-loss watchdog.
    ** If no fresh Unity telemetry packet arrives for this many seconds, GNC_APP
    ** forces IDLE + AbortLatch (same effect as a ground ABORT command) rather
    ** than continuing to coast silently. This exists because the field this
    ** watchdog reads (HkTlm.TlmStaleSec) was originally scoped as an LC
    ** watchpoint, but LC_APP is not part of this target's app list — see
    ** GNC_APP_ProcessWakeup for the actual check.
    */
    float TlmLossTimeoutSec; /* s — consecutive telemetry loss before auto-abort */

} GNC_ParamTbl_t;

#endif /* GNC_APP_TBL_H */
