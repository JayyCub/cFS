#include "cfe_tbl_filedef.h"
#include "rcs_tbl.h"

/*
** Dragon RCS geometry, extracted 2026-09-26 from Scene2.unity (ChaserVehicle
** RCSModel.thrusterTransforms, CoM override (0,0,-4) local), in the RCSModel
** body frame relative to the CoM — the same numbers RCSModel.BuildThrusterArray
** fed the old Unity-side allocator. Post 2026-07-25 cant fix.
**
**   T00-T03  aft-facing deorbit thrusters — disabled for proximity ops
**   T04-T07  approach group (+Z, canted)
**   T08-T11  brake-yaw group (-Z, strongly canted outboard)
**   T12-T15  brake-pitch group (-Z, canted outboard)
*/
RCS_ThrTbl_t RCS_ThrTbl = {
    .NumThrusters = 16,
    .Thrust_N     = 400.0f,  /* Draco; must match RCSModel.thrusterForce (the "hardware") */
    .MinOnTime_s  = 0.020f,  /* 8 N·s minimum impulse bit; one Unity physics step        */
    .MaxOnTime_s  = 0.190f,  /* inside the 0.2 s lock-step GNC cycle                     */
    .Thruster = {
        /*   Pos_m (x, y, z)                  Dir (x, y, z)                   Enabled */
        { { +0.6819f, +0.3990f, +3.5945f }, { +0.0000f, +0.0000f, -1.0000f }, 0 }, /* T00 */
        { { -0.6819f, +0.3990f, +3.5945f }, { +0.0000f, +0.0000f, -1.0000f }, 0 }, /* T01 */
        { { -0.6819f, -0.3990f, +3.5945f }, { +0.0000f, +0.0000f, -1.0000f }, 0 }, /* T02 */
        { { +0.6819f, -0.3990f, +3.5945f }, { +0.0000f, +0.0000f, -1.0000f }, 0 }, /* T03 */
        { { +1.1043f, +1.6561f, +0.6398f }, { -0.2241f, -0.5000f, +0.8365f }, 1 }, /* T04 */
        { { -1.1043f, +1.6561f, +0.6398f }, { +0.2241f, -0.5000f, +0.8365f }, 1 }, /* T05 */
        { { -1.1043f, -1.6561f, +0.6398f }, { +0.2241f, +0.5000f, +0.8365f }, 1 }, /* T06 */
        { { +1.1043f, -1.6561f, +0.6398f }, { -0.2241f, +0.5000f, +0.8365f }, 1 }, /* T07 */
        { { +0.8694f, +1.7694f, +0.7101f }, { -0.7071f, +0.0000f, -0.7071f }, 1 }, /* T08 */
        { { -0.8694f, +1.7694f, +0.7101f }, { +0.7071f, +0.0000f, -0.7071f }, 1 }, /* T09 */
        { { -0.8694f, -1.7694f, +0.7101f }, { +0.7071f, +0.0000f, -0.7071f }, 1 }, /* T10 */
        { { +0.8694f, -1.7694f, +0.7101f }, { -0.7071f, +0.0000f, -0.7071f }, 1 }, /* T11 */
        { { +1.0376f, +1.5997f, +0.9511f }, { +0.4330f, -0.5000f, -0.7500f }, 1 }, /* T12 */
        { { -1.0376f, +1.5997f, +0.9511f }, { -0.4330f, -0.5000f, -0.7500f }, 1 }, /* T13 */
        { { -1.0376f, -1.5997f, +0.9511f }, { -0.4330f, +0.5000f, -0.7500f }, 1 }, /* T14 */
        { { +1.0376f, -1.5997f, +0.9511f }, { +0.4330f, +0.5000f, -0.7500f }, 1 }, /* T15 */
    },
};

CFE_TBL_FILEDEF(RCS_ThrTbl, RCS.ThrTbl, RCS thruster table, rcs_thr_tbl.tbl)
