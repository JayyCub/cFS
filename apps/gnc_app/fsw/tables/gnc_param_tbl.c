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

    /* Vehicle mass properties — FSW's model of the chaser (Unity: Rigidbody mass
       12000 kg; VehicleState solid-cylinder inertia, r = 2 m, l = 6 m):
         pitch/yaw I = m(3r² + l²)/12 = 48000 kg·m²,  roll I = m r²/2 = 24000 kg·m²
       The 2:1 pitch-to-roll ratio matches the old ThrusterDiagnostic measurement
       (roll accelerated ~2x faster than pitch/yaw for the same torque), which the
       single RotAccel constant this replaced could not represent. */
    .VehicleMass     = 12000.0f,
    .Inertia_kgm2    = { 48000.0f, 48000.0f, 24000.0f },

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
    .LatKp_Approach    = 0.03f,  /* phase 5 (Monte Carlo trade, 60 runs per point): 0.006 with
                                    a 2 mm/s deadband left lateral uncontrolled within 0.33 m
                                    of the axis (deadband / Kp) — contact lateral 5.0 cm mean,
                                    9 cm worst against a 10 cm limit. With NAV port velocity and
                                    the phase-plane attitude loop the lateral/attitude limit
                                    cycle that motivated it is gone: 0.03 / 1 mm/s gives 1.1 cm
                                    mean, 2.3 cm worst, for ~5 % more propellant.             */

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
    .EntrySettleCycles    = 15,     /* consecutive cycles all conditions must hold     */

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
    .MaxAttRate      =  0.02f,  /* rad/s — cap per axis (~1.1 deg/s; was 0.20 before the
                                   phase-5 phase-plane controller bounded slews itself) */

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
    .LatVelDeadband_Approach_ms = 0.001f,   /* m/s — APPROACH: coast below this (phase 5:
                                               was 0.002, see LatKp_Approach) */
    /* The limit cycle: lateral burn disturbs roll ~0.5–1°; at AttKp=0.50 that   */
    /* fires a correction every cycle; the correction couples back into lateral.  */
    /* With a 2° deadband, sub-2° perturbations coast rather than being fought.  */

    /* Axial braking capability — guidance planning estimate for the hold-point
    ** lookahead only (no longer sizes burns). 0.189 m/s² is the brake-group
    ** deceleration last measured by ThrusterDiagnostic (2026-07-11, 1.0 s burn
    ** on T08-T15 at 12000 kg). */
    .AxialBrakeAccel_mss = 0.189f,

    /* Axial hold-position controller (HOLD phase) — same magnitude as LatKp/
    ** MaxLatSpeed; gentle enough not to fight the brake burn's own overshoot
    ** while still walking accumulated range drift back to HoldRange_m. */
    .AxialHoldKp     = 0.02f,   /* target closing speed = KP × range error (m/s per m) */
    .MaxHoldSpeed    = 0.05f,   /* cap on hold-correction closing speed (m/s)          */

    /* Crew manual flight (MANUAL phase). Full THC ≈ 240 N of net thrust at
       12000 kg; full RHC = 1 °/s. The rate-hold deadband sits above the RCS
       minimum angular impulse bit (~14 N·m·s ≈ 3e-4 rad/s in pitch/yaw, 6e-4 in
       roll) so rate hold doesn't limit-cycle one impulse bit either side of 0. */
    .ManualAccel_mss         = 0.02f,
    .ManualRate_rads         = 0.0175f,
    .ManualRateDeadband_rads = 0.0008f,

    /* Lateral channel steers on the docking-port point velocity (NAV), the
       derivative of the port offset it controls. 0 = old CoM velocity. */
    .LatVelAtPort = 1,

    /* Phase-plane attitude (phase 5). α = 0.01 rad/s² is about a third of what
       two pitch/yaw thrusters deliver on 48000 kg·m², leaving authority for
       translation. MaxAttRate (above) caps slews; the rate deadband sits just
       over the minimum angular impulse bit (~3e-4 rad/s). */
    .AttCtrlAccel_rads2   = 0.01f,
    .AttRateDeadband_rads = 0.0004f,

    /* ≈ one minimum impulse bit: 8 N·s / 12000 kg */
    .FfReleaseDv_ms = 0.0007f,

    /* Approach envelope: 0.36 m/s out to 20 m (outer cap 0.30 + margin), 0.15 m/s
       inside 10 m (inner cap 0.10 + margin); linear in between. */
    .ProfileCount         = 4,
    .ProfileRange_m       = {0.0f, 10.0f, 20.0f, 1000.0f},
    .ProfileMaxClose_ms   = {0.15f, 0.15f, 0.36f, 0.36f},
    .ProfileGuideFrac     = 0.9f,
    .CorridorCheckRange_m = 20.0f,

    /* Collision-avoidance manoeuvre. 0.1 m/s posigrade from +V-bar rises ~180 m
       and drifts behind the station: passively safe for an orbit. */
    .CamDeltaV_ms         = 0.10f,
    .CamMaxDeltaV_ms      = 0.30f,
    .PassiveSafeRange_m   = 10.0f,
    .PassiveSafeStart_s   = 120.0f,
    .PassiveSafeHorizon_s = 5600.0f, /* one ISS orbit */
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
