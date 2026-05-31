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
    .ThrusterForce   = 10.0f,   /* N per thruster                                */
    .VehicleMass     = 200.0f,  /* kg                                            */
    .RotAccel        = 0.30f,   /* rad/s² per attitude thruster                  */

    /* Burn duration limits */
    .MinBurnDuration = 0.050f,  /* s — shorter pulses are skipped (coast)        */
    .MaxBurnDuration = 0.950f,  /* s — cap so burn ends before next 1 Hz tick    */

    /* Lateral position controller */
    .LatKp           = 0.05f,   /* lateral speed = KP × position error           */
    .MaxLatSpeed     = 0.10f,   /* lateral speed cap (m/s)                       */

    /* Phase gate hysteresis pair */
    .LatApproachGate = 0.50f,   /* m — enter APPROACH when lateral offset < this */
    .LatCorrectGate  = 1.00f,   /* m — enter CORRECT when lateral offset > this  */

    /* Autonomous hold-point waypoints — set to 0.0 to disable */
    .HoldPoint1_m    = 10.0f,   /* m — outer waypoint; GNC pauses here for GO    */
    .HoldPoint2_m    =  3.0f,   /* m — inner waypoint; GNC pauses here for GO    */
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
