#ifndef NAV_TBL_H
#define NAV_TBL_H

#include "common_types.h"

#define NAV_CFG_TBL_NAME "CfgTbl"
#define NAV_CFG_TBL_FILE "/cf/nav_cfg_tbl.tbl"

/*
** NAV configuration table: vehicle and target geometry as flight software
** believes it to be, the sensor noise model, and filter tuning.
**
** Like the RCS thruster table, this is deliberately a separate copy of the
** geometry, not read from the Unity scene (the "real hardware"). A mismatch
** is a realistic modelling error for FSW to cope with. Regenerate the
** geometry rows from the scene with ChaserSensors' "Log cFS NAV table"
** context menu (Unity Play mode).
**
** Quaternions are (x, y, z, w). q_A_B rotates B-frame vectors into frame A.
**
** The docking frame D and the docking attitude follow from the target rows:
**   q_L_D    = TargetAttQuat_L ⊗ TargetPortQuat_T ⊗ DockedQuat_TP
**   q_L_Bdes = q_L_D ⊗ conj(ChaserPortQuat_B)
** DockedQuat_TP is the petal / roll indexing. Stating it explicitly replaces
** the old Unity-side rollErrorOffset = 90 fudge (the ISS port's authored "up"
** axis is world -X).
*/
typedef struct
{
    /* Chaser, body frame B (RCS-table axes, origin at the centre of mass) */
    float ChaserPortPos_B[3];  /* docking port face centre (m)                          */
    float ChaserPortQuat_B[4]; /* q_B_CP: port axes in body                              */
    float RpsPos_B[3];         /* relative-pose sensor optical centre (m)                */
    float RpsQuat_B[4];        /* q_B_S: sensor axes (boresight +Z) in body              */

    /* Target, ISS body frame T (origin at the reference point the CW model is
    ** centred on) */
    float TargetAttQuat_L[4];  /* q_L_T: ISS attitude in LVLH (constant, LVLH hold)      */
    float TargetPortPos_T[3];  /* target docking port face centre = LIDAR reflector (m)  */
    float TargetPortQuat_T[4]; /* q_T_TP: target port axes                               */
    float DockedQuat_TP[4];    /* q_TP_CP: chaser port axes in the target port frame when
                               ** correctly docked                                       */

    float MeanMotion_rads; /* reference orbit mean motion n, CW process model (rad/s)   */

    /* Measurement noise model, 1-sigma */
    float RpsRangeSigma_m;   /* LIDAR range noise floor (m)                              */
    float RpsRangeSigmaFrac; /* plus this fraction of range                              */
    float RpsAngleSigma_rad; /* LIDAR azimuth / elevation noise (rad)                    */
    float AttSigma_rad;      /* star tracker attitude noise, per axis (rad)             */
    float RpsPoseSigma_rad;  /* LIDAR pose solution orientation noise, per axis (rad)    */

    /* Filter tuning */
    float  AccelNoise_mss;   /* unmodelled acceleration (process noise), 1-sigma (m/s²)  */
    float  DeltaVNoise_ms;   /* accelerometer Δv noise per cycle, 1-sigma (m/s)          */
    float  InitVelSigma_ms;  /* velocity uncertainty at initialisation (m/s)             */
    float  GateSigma;        /* reject a LIDAR fix whose innovation Mahalanobis distance
                             ** exceeds this                                             */
    uint32 MaxConsecRejects; /* re-initialise from the LIDAR after this many consecutive
                             ** rejected fixes (divergence recovery)                     */
    float  MaxPosSigma_m;    /* relative nav VALID only while position sigma <= this     */
    float  MaxVelSigma_ms;   /*   ... and velocity sigma <= this                         */

    /* Sensor errors the filters estimate (realism phase 4b) */
    float AccelBiasWalk_mss;      /* accelerometer bias random walk (m/s² per sqrt s)     */
    float InitAccelBiasSigma_mss; /* accelerometer bias uncertainty at initialisation     */
    float GyroNoise_rads;         /* gyro rate noise per sample, 1-sigma (rad/s)          */
    float GyroBiasWalk_rads;      /* gyro bias random walk (rad/s per sqrt s)             */
    float InitGyroBiasSigma_rads; /* gyro bias uncertainty at initialisation              */
    float MaxAttSigma_rad;        /* attitude VALID only while its sigma <= this          */
} NAV_CfgTbl_t;

#endif /* NAV_TBL_H */
