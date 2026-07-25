#include "cfe_tbl_filedef.h"
#include "gnc_app_tbl.h"

/*
** Default GNC parameter table.
**
** These values replicate the original #define constants from gnc_app.h.
** To change gains without recompiling the application:
**   1. Edit the values below.
**   2. Rebuild: (inside container) make native_std.install
**   3. The new gnc_param_tbl.tbl is installed to /cf/ automatically.
**   4. Restart cFS — the table is loaded from /cf/ at GNC_APP_Init time.
**
** For a running system, the table can also be uplinked via the cFS table
** service (CFE_TBL) over the ground command link without restarting cFS.
*/
GNC_ParamTbl_t GNC_ParamTbl =
{
    /* Axial guidance */
    .AxialKp         = 0.02f,   /* target closing speed = KP × range            */
    .MinCloseSpeed   = 0.10f,   /* floor on approach speed (m/s) — holds a constant
                                   ~0.1 m/s soft-capture speed for the final stretch
                                   instead of tapering toward zero (matches observed
                                   SpaceX Dragon terminal closing rate)               */
    .MaxCloseSpeed   = 0.30f,   /* outer cap, before HoldPoint1_m (m/s)          */
    .MaxCloseSpeed_Inner = 0.10f, /* inner cap, after HoldPoint1_m fires (m/s)   */
    /* Mirrors observed SpaceX Dragon docking profile: ~0.3 m/s outside the first
       hold point, dropping to ~0.1 m/s once past it and re-commanded to GO.      */

    /* Physical model — must match Unity Inspector; see "Key Coupling
       Constraints" in Docs/DEV_REFERENCE.md for the full list and both
       locations that must stay in sync. */
    .ThrusterForce   = 400.0f,  /* N per thruster — real Draco thruster          */
    .VehicleMass     = 12000.0f, /* kg — updated 2026-07-11 to match the Unity chaser
                                   Rigidbody (Scene2.unity ChaserVehicle), which was
                                   changed from 4500 without updating this table —
                                   see "Key Coupling Constraints" in Docs/DEV_REFERENCE.md. */
    .RotAccel        = 0.012f,  /* rad/s² — re-measured 2026-07-11 via ThrusterDiagnostic.cs
                                   (ThrusterDiagnostic_20260711_172942.log) against the new
                                   12000 kg mass: avg of +Tx=0.0118, +Ty=0.0123 rad/s² (1.00s
                                   burn). Roll (+Tz) measured ~2x higher (0.0244) — the control
                                   law uses one RotAccel for all three axes, so this is
                                   calibrated toward pitch/yaw (docking-alignment precision)
                                   rather than roll.                                        */

    /* Burn duration limits — scaled to the 5 Hz (0.2s) GNC_CYCLE_DT_S cycle.
       MaxBurnDuration keeps the same ~95% margin the old 1 Hz values had
       (0.95/1.0); MinBurnDuration is floored at one Unity physics tick
       (Fixed Timestep = 0.02s) rather than scaled proportionally (0.2×0.05=
       0.01s), since a shorter request can't resolve to anything finer than
       one FixedUpdate step anyway. */
    .MinBurnDuration = 0.020f,  /* s — shorter pulses are skipped (coast)        */
    .MaxBurnDuration = 0.190f,  /* s — cap so burn ends before next 5 Hz tick    */

    /* Lateral position controller */
    .LatKp           = 0.02f,   /* lateral speed = KP × position error           */
    .MaxLatSpeed     = 0.05f,   /* lateral speed cap (m/s)                       */
    /* Reduced from 0.05/0.10 — slower lateral correction means less thruster     */
    /* force per cycle, which reduces the axial-coupling drift that was causing   */
    /* the vehicle to recede from the ISS during lateral correction.              */

    /* APPROACH lateral position gain — see doc comment in gnc_app_tbl.h.
       Both phases now hold the centerline continuously; APPROACH uses a
       softer gain to keep burns small enough to avoid the lateral/attitude
       coupling limit cycle under APPROACH's tighter attitude deadband. */
    .LatKp_Approach    = 0.006f, /* softened further from 0.01 (half of LatKp) — smaller
                                    target velocity per meter of offset means shorter, gentler
                                    burns during APPROACH, reducing the attitude disturbance
                                    that was feeding the lateral/attitude limit cycle          */

    /* Corridor-relative phase gate for the APPROACH -> CORRECT revert only
       (replaces the old fixed-meter LatCorrectGate — see doc comment in
       gnc_app_tbl.h). Must track the physical corridor cone (Unity
       ApproachCorridor.coneHalfAngle) instead of a constant that goes stale
       as range changes. */
    .ConeHalfAngle_deg   = 15.0f, /* deg — must match Unity ApproachCorridor.coneHalfAngle   */
    .MinAllowedLateral_m =  0.15f,/* m — floor so the gate doesn't collapse to 0 near contact */
    .CorridorMarginOut   =  0.9f, /* revert to CORRECT just inside the actual cone edge       */

    /* CORRECT -> APPROACH entry gate: fixed absolute convergence target,
       independent of range (replaces the old CorridorMarginIn fraction, which
       let LAT_CORR hand off to APPROACH 4.46 m off-axis at 33 m range — half
       the corridor cone there, but nowhere near converged). */
    .LatEntryThreshold_m = 0.05f, /* m — LAT_CORR must reach this before entering APPROACH.
                                      Tightened from 0.3 -> 0.1 -> 0.05 over the course of
                                      2026-07-18. 0.05m sits with margin under real docking
                                      capture envelopes (IDSS/NDS-class mechanical capture
                                      tolerates several cm to ~10cm laterally, several degrees
                                      angularly — the ring's own guide petals do final
                                      centering, GNC only needs to deliver into that envelope
                                      reliably) — deliberately tighter than strictly required
                                      so APPROACH has margin against CW drift/noise, while
                                      still being clearly above the observed ~0.01-0.02m
                                      sensor/actuator noise floor so the gate stays reachable.
                                      See LatVelDeadband_ms below — tightened to match, same
                                      proportion as the 0.3->0.1 step. */

    /* CORRECT -> APPROACH settle gate — see doc comment in gnc_app_tbl.h. Added
       2026-07-18: a run that hit LatEntryThreshold_m still had attitude visibly
       drifting on all three axes at the exact cycle APPROACH committed, so axial
       closure and residual attitude motion fought each other from the first
       APPROACH cycle. These three add "stopped and steady," not just "position
       crossed a line," to the entry condition.
       LatEntryVelMax_ms / AttEntryRateMax_rads: left UNCHANGED when
         LatEntryThreshold_m tightened to 0.05 — unlike LatVelDeadband_ms, these
         aren't mathematically tied to the position threshold (they're "is it
         genuinely stopped," not "how small is the position error"), and they're
         already only ~2x the observed 0.004-0.006 rad/s / ~0.01 m/s noise floor.
         Tightening them further risked the settle gate never being satisfiable
         at all given real sensor/actuator noise — the same failure mode that
         motivated adding EntrySettleCycles in the first place.
       EntrySettleCycles: raised 10 -> 15 (3s @ 5Hz, was 2s) — a tighter position
         target benefits from a longer hold to confirm it's a real convergence
         and not one lucky low-noise sample, now that LatEntryThreshold_m gives
         less room for a spurious pass. */
    .LatEntryVelMax_ms    = 0.01f,  /* m/s — |Vel_X|,|Vel_Y| must both be below this   */
    .AttEntryRateMax_rads = 0.01f,  /* rad/s — |AngVel_X/Y/Z| must all be below this   */
    .EntrySettleCycles    = 15.0f,  /* consecutive cycles all conditions must hold     */

    /* APPROACH -> CORRECT attitude revert — see doc comment in gnc_app_tbl.h.
       6.0 sits with 4° of margin below DockingDetector's 10° capture requirement
       (Assets/DockingDetector.cs maxAttitudeError) and well above AttDeadband_deg's
       1° APPROACH operating band (2.0 x 0.5), so normal correction transients
       don't trip it. */
    .AttRevertThreshold_deg = 6.0f, /* deg — revert APPROACH->CORRECT if any axis exceeds this */

    /* Autonomous hold-point waypoints — set to 0.0 to disable */
    .HoldPoint1_m    = 20.0f,   /* m — outer waypoint; GNC pauses here for GO    */
    .HoldPoint2_m    =  3.0f,   /* m — inner waypoint; GNC pauses here for GO    */

    /* Attitude PD controller */
    .AttKp           =  0.25f,  /* (rad/s)/rad — reduced to limit overshoot      */
    .MaxAttRate      =  0.20f,  /* rad/s — cap per axis                          */

    /* Attitude deadband — breaks the lateral↔attitude coupling limit cycle      */
    /* HALVED 2026-07-25 (2.0 -> 1.0) after the RCS thruster-cant fix: the 2.0°
       baseline was sized for the ~0.5-1° lateral-burn coupling measured with the
       old (misangled) thruster geometry. Post-fix ThrusterDiagnostic data shows
       ~0 coupling on pure translation commands, so the disturbance this deadband
       was protecting against should be much smaller now. Unverified in closed-
       loop flight — if the old lateral<->attitude limit cycle (chatter every
       cycle, correction burns feeding back into lateral motion) reappears in
       AngVel_X/Y/Z telemetry, revert to 2.0. */
    .AttDeadband_deg =  1.0f,   /* deg — skip correction when all errors < this  */
    /* In APPROACH phase the code tightens this to 0.5× (0.5°) automatically so
       attitude is held more precisely as the vehicle closes on the port.          */

    /* Spin-rate override — see doc comment in gnc_app_tbl.h.
       RETUNED 2026-07-18 using the AngVel_X/Y/Z telemetry added for exactly this
       purpose. A 2026-07-18 LAT_CORR run showed sustained roll rate sitting at
       W_Z=0.001-0.003 rad/s for ~130 consecutive cycles (26s) — right on top of
       the old 0.003 threshold, so it never reliably cleared `> spin_th` and the
       roll error was left to drift on AttDeadband_deg alone, climbing from 0° to
       4° unpunished before the angle deadband finally caught it (by which point
       the correction was a large, near-saturated multi-axis burn — see the
       LAT_CORR +Fz coupling feedforward comment below for what that triggered).
       Raised to 0.006 rad/s — a clear 2x margin above the observed 0.003 rad/s
       noise floor — so a genuine sustained residual rate reliably trips the
       override instead of straddling it.

       HALVED BACK 2026-07-25 (0.006 -> 0.003) alongside AttDeadband_deg, same
       rationale: the 0.001-0.003 rad/s noise floor above was measured with the
       old thruster geometry, which fed real coupling noise into AngVel. If the
       post-fix noise floor hasn't actually dropped, this value will straddle it
       again exactly like the pre-07-18 setting did — watch for W_Z sitting flat
       at 0.001-0.003 rad/s for many consecutive cycles without ever tripping
       `> spin_th`, and revert to 0.006 if so. */
    .SpinThreshold_rads = 0.003f, /* rad/s — fires correction even inside AttDeadband_deg */

    /* Lateral velocity deadband — see doc comment in gnc_app_tbl.h. Now split
       by phase so CORRECT's convergence precision and APPROACH's anti-chatter
       margin don't have to share one value.

       CORRECT: tightened again, 0.0007 -> 0.00035, tracking LatEntryThreshold_m's
       0.1m -> 0.05m drop with the same proportion as the previous 0.3->0.1 step
       (dead zone = deadband/LatKp(0.02), so halving the deadband when the entry
       gate also halves holds the dead-zone-to-gate ratio at ~35%, the same
       comfortable margin used each time this has been tightened — see the
       LAT_CORR-stuck-at-0.75m history in the header doc comment for what
       happens when that margin isn't kept).

       APPROACH: kept at 0.002, unchanged since the original phase split —
       this tightening pass is CORRECT-only, same as last time. */
    .LatVelDeadband_ms          = 0.00035f, /* m/s — CORRECT: coast below this */
    .LatVelDeadband_Approach_ms = 0.002f,   /* m/s — APPROACH: coast below this */
    /* The limit cycle: lateral burn disturbs roll ~0.5–1°; at AttKp=0.50 that   */
    /* fires a correction every cycle; the correction couples back into lateral.  */
    /* With a 2° deadband, sub-2° perturbations coast rather than being fought.  */

    /* Braking deceleration — empirically measured from actual Unity physics.
    ** These are NOT computed from thrusterForce × Fz_sum / VehicleMass because the
    ** binary on/off thruster model combined with the 1 Hz discrete loop produces an
    ** effective acceleration roughly 0.55× the theoretical value.  The values below
    ** were derived from flight telemetry: Δv / burn_time for known thruster groups.
    **
    ** To recalibrate: run cFS, observe a full-power 0.95 s brake burn in the log,
    ** read Δv from consecutive cycle speed readings, compute Δv / 0.95.
    ** If thrusterForce or scene geometry changes, re-measure rather than recompute.
    **
    ** Re-measured 2026-07-11 via ThrusterDiagnostic.cs against VehicleMass = 12000
    ** (ThrusterDiagnostic_20260711_172942.log, 1.00s burns — body-frame log confirms
    ** zero off-axis coupling on all three, so these are clean single-axis readings):
    **   Hard  (T08-T15): dV.z = -0.1890 m/s over 1.00s = 0.189 m/s²
    **   Light (T08-T11): dV.z = -0.1332 m/s over 1.00s = 0.133 m/s²
    **   Approach (T04-T07): dV.z = +0.0313 m/s over 1.00s = 0.031 m/s² */
    .BrakeAccel_Hard_mss  = 0.189f,   /* m/s² — T08-T15, all 8 brake thrusters      */
    .BrakeAccel_Light_mss = 0.133f,   /* m/s² — T08-T11 only (Brake-Yaw group)      */
    .ApproachAccel_mss    = 0.031f,   /* m/s² — T04-T07 approach group               */

    /* Axial hold-position controller (HOLD phase) — same magnitude as LatKp/
    ** MaxLatSpeed; gentle enough not to fight the brake burn's own overshoot
    ** while still walking accumulated range drift back to HoldRange_m. */
    .AxialHoldKp     = 0.02f,   /* target closing speed = KP × range error (m/s per m) */
    .MaxHoldSpeed    = 0.05f,   /* cap on hold-correction closing speed (m/s)          */

    /* ~10 missed 5 Hz cycles (GNC_CYCLE_DT_S) before GNC_APP_ProcessWakeup forces
       an auto-abort — see doc comment in gnc_app_tbl.h. */
    .TlmLossTimeoutSec = 2.0f,  /* s */
};

/*
** CFE_TBL_FILEDEF — registers this table with the cFS table service at build time.
**
** Arguments:
**   1. C variable name of the table data struct (above)
**   2. "APP_NAME.TblName" — must match GNC_APP_PARAM_TBL_NAME with "GNC_APP." prefix
**   3. Human-readable description embedded in the .tbl file header
**   4. Output .tbl binary filename (installed to /cf/ by the build system)
*/
CFE_TBL_FILEDEF(GNC_ParamTbl, GNC_APP.ParamTbl, GNC Guidance Parameter Table, gnc_param_tbl.tbl)
