#include "cfe_tbl_filedef.h"
#include "nav_tbl.h"

/*
** NAV configuration, geometry extracted 2026-10-04 from Scene2.unity
** (ChaserVehicle CoM override (0,0,-4) local, ChaserPort at (0,0,-0.07) local;
** TargetVehicle at the world origin with identity rotation). Regenerate the
** geometry rows with ChaserSensors' "Log cFS NAV table" context menu.
**
** TargetPortQuat_T: the ISS port's authored axes are rolled — its "up" is
** world -X. DockedQuat_TP undoes that, so the docking frame D lines up with
** world axes and the docking attitude is the chaser upright, facing +Z (what
** RelativeNav.rollErrorOffset = 90 used to correct for on the Unity side).
**
** Noise rows describe the sensors as FSW models them; they match the Unity
** ChaserSensors defaults (the "hardware" spec sheet).
*/
NAV_CfgTbl_t NAV_CfgTbl = {
    /* Chaser */
    .ChaserPortPos_B  = {0.0f, 0.0f, 3.93f},
    .ChaserPortQuat_B = {0.0f, 0.0f, 0.0f, 1.0f},
    .RpsPos_B         = {0.0f, 0.0f, 3.93f}, /* LIDAR at the docking port, looking along it */
    .RpsQuat_B        = {0.0f, 0.0f, 0.0f, 1.0f},

    /* Target (ISS) */
    .TargetAttQuat_L  = {0.0f, 0.0f, 0.0f, 1.0f}, /* LVLH hold */
    .TargetPortPos_T  = {0.000005485832f, -5.5613275f, -16.092f},
    .TargetPortQuat_T = {-0.70668393f, 0.7075295f, -0.00000087704393f, -0.00000014301622f},
    .DockedQuat_TP    = {0.70668393f, -0.7075295f, 0.00000087704393f, -0.00000014301622f},

    .MeanMotion_rads = 0.00113f, /* ISS, 400 km — same as ClohessyWiltshire.meanMotion */

    /* Measurement noise (1-sigma) */
    .RpsRangeSigma_m   = 0.005f,  /* 5 mm ...                     */
    .RpsRangeSigmaFrac = 0.001f,  /* ... + 0.1 % of range          */
    .RpsAngleSigma_rad = 0.0005f, /* 0.03 deg az / el              */
    .AttSigma_rad      = 0.00005f, /* ~10 arcsec star tracker       */

    /* Filter tuning */
    .AccelNoise_mss   = 0.00005f, /* unmodelled acceleration                               */
    .DeltaVNoise_ms   = 0.00002f, /* accelerometer Δv noise per cycle                      */
    .InitVelSigma_ms  = 0.1f,
    .GateSigma        = 6.0f,     /* 3-DOF chi-square: a 6-sigma fix is ~1e-7 likely       */
    .MaxConsecRejects = 10,       /* 2 s of disagreement at 5 Hz -> re-initialise          */
    .MaxPosSigma_m    = 0.5f,
    .MaxVelSigma_ms   = 0.005f,   /* half GNC's LatEntryVelMax_ms                          */
};

CFE_TBL_FILEDEF(NAV_CfgTbl, NAV.CfgTbl, NAV configuration, nav_cfg_tbl.tbl)
