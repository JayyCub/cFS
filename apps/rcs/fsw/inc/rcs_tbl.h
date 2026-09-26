#ifndef RCS_TBL_H
#define RCS_TBL_H

#include "common_types.h"

#define RCS_THR_TBL_NAME "ThrTbl"
#define RCS_THR_TBL_FILE "/cf/rcs_thr_tbl.tbl"

#define RCS_MAX_THRUSTERS 16

/*
** One thruster, body frame (the chaser RCSModel transform: +Z toward the ISS,
** +X right, +Y up), relative to the vehicle's centre of mass.
*/
typedef struct
{
    float  Pos_m[3];   /* nozzle position relative to CoM (m)                   */
    float  Dir[3];     /* unit vector the vehicle is PUSHED (opposite exhaust)   */
    uint32 Enabled;    /* 0 = never allocated (e.g. deorbit thrusters, failed)   */
} RCS_ThrusterDef_t;

/*
** RCS configuration table — this vehicle's thruster geometry as flight software
** believes it to be. It is deliberately a separate copy from the Unity scene
** (the "real hardware"): if the two disagree, that's a realistic modelling error
** (misalignment, CoM estimate) for FSW to cope with, not a silent coupling bug.
** Regenerate from the scene with RCSModel's "Log cFS thruster table" context menu.
*/
typedef struct
{
    uint32            NumThrusters;   /* <= RCS_MAX_THRUSTERS                                 */
    float             Thrust_N;       /* rated thrust per thruster (all identical, Draco)     */
    float             MinOnTime_s;    /* minimum valve pulse — sets the minimum impulse bit   */
    float             MaxOnTime_s;    /* longest pulse per GNC cycle (must end before the next) */
    RCS_ThrusterDef_t Thruster[RCS_MAX_THRUSTERS];
} RCS_ThrTbl_t;

#endif /* RCS_TBL_H */
