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
** Layout: 14 × float = 56 bytes. Naturally 4-byte aligned; no padding required.
*/
typedef struct
{
    /*
    ** Axial guidance channel
    */
    float AxialKp;          /* target closing speed = KP × range  (m/s per m) */
    float MinCloseSpeed;    /* floor on approach speed             (m/s)        */
    float MaxCloseSpeed;    /* cap on approach speed               (m/s)        */

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

} GNC_ParamTbl_t;

#endif /* GNC_APP_TBL_H */
