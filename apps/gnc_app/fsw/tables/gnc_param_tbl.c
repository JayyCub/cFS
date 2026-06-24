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
    .MinCloseSpeed   = 0.02f,   /* floor on approach speed (m/s)                 */
    .MaxCloseSpeed   = 0.30f,   /* cap on approach speed (m/s)                   */

    /* Physical model — must match Unity Inspector */
    .ThrusterForce   = 400.0f,  /* N per thruster — real Draco thruster          */
    .VehicleMass     = 4500.0f, /* kg — Dragon 2 capsule + trunk + crew          */
    .RotAccel        = 0.033f,  /* rad/s² — derived: 400 N × ~1.5 m arm / 18000 kg·m²
                                   I_pitch = m(3r²+l²)/12 = 4500×(12+36)/12 = 18000
                                   Scale from old: 0.394 × (400/10) × (38/18000) = 0.033
                                   Recalibrate empirically if rotation feels off   */

    /* Burn duration limits */
    .MinBurnDuration = 0.050f,  /* s — shorter pulses are skipped (coast)        */
    .MaxBurnDuration = 0.950f,  /* s — cap so burn ends before next 1 Hz tick    */

    /* Lateral position controller */
    .LatKp           = 0.02f,   /* lateral speed = KP × position error           */
    .MaxLatSpeed     = 0.05f,   /* lateral speed cap (m/s)                       */
    /* Reduced from 0.05/0.10 — slower lateral correction means less thruster     */
    /* force per cycle, which reduces the axial-coupling drift that was causing   */
    /* the vehicle to recede from the ISS during lateral correction.              */

    /* Phase gate hysteresis pair */
    .LatApproachGate = 1.00f,   /* m — enter APPROACH when lateral offset < this */
    .LatCorrectGate  = 1.50f,   /* m — enter CORRECT when lateral offset > this  */
    /* Increased from 0.5/1.0 — vehicle transitions to APPROACH from 1 m instead
       of 0.5 m.  In APPROACH the lateral channel switches to velocity-damping only
       (no position feedback), so the vehicle drifts naturally through the axis.
       The wider gate means the coasting phase starts sooner and the vehicle has
       more time to settle before close approach.                                   */

    /* Autonomous hold-point waypoints — set to 0.0 to disable */
    .HoldPoint1_m    = 10.0f,   /* m — outer waypoint; GNC pauses here for GO    */
    .HoldPoint2_m    =  3.0f,   /* m — inner waypoint; GNC pauses here for GO    */

    /* Attitude PD controller */
    .AttKp           =  0.25f,  /* (rad/s)/rad — reduced to limit overshoot      */
    .MaxAttRate      =  0.20f,  /* rad/s — cap per axis                          */

    /* Attitude deadband — breaks the lateral↔attitude coupling limit cycle      */
    .AttDeadband_deg =  2.0f,   /* deg — skip correction when all errors < this  */
    /* In APPROACH phase the code tightens this to 0.5× (1.0°) automatically so
       attitude is held more precisely as the vehicle closes on the port.          */

    /* Lateral velocity deadband — prevents bang-bang chatter at small errors     */
    .LatVelDeadband_ms = 0.015f, /* m/s — coast when velocity error is this small */
    /* The limit cycle: lateral burn disturbs roll ~0.5–1°; at AttKp=0.50 that   */
    /* fires a correction every cycle; the correction couples back into lateral.  */
    /* With a 2° deadband, sub-2° perturbations coast rather than being fought.  */
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
