#ifndef NAV_FILTER_H
#define NAV_FILTER_H

/*
** NAV estimation core — pure math, no cFE calls, so it can be compiled into a
** stand-alone test harness (apps/nav/unit-test/nav_filter_sim.c).
**
** Two filters run each cycle (NAV_Step):
**
**   1. Attitude — multiplicative extended Kalman filter (MEKF), realism 4b.
**      State: attitude q_L_B (kept as a quaternion) + gyro bias; the filter's
**      6-state error vector is [δθ (body), δb_gyro].
**        propagate: bias-corrected gyro, trapezoidal over the cycle
**        update:    star tracker quaternion, and the LIDAR pose solution
**                   (target port orientation) turned into a body attitude
**                   through the known ISS attitude and port geometry
**      Chi-square gate on every fix; re-initialise from the star tracker
**      after MaxConsecRejects rejections.
**
**   2. Translation — 9-state Kalman filter, chaser CoM relative to the target
**      reference point in LVLH, plus the accelerometer bias (body):
**        propagate: Clohessy-Wiltshire dynamics (same equations as
**                   ClohessyWiltshire.cs and the GNC feed-forward) plus the
**                   bias-corrected IMU Δv, rotated to LVLH
**        update:    LIDAR range / azimuth / elevation to the target port
**                   reflector, turned into a CoM position fix through the
**                   attitude and the table geometry; chi-square gate;
**                   re-initialise after MaxConsecRejects rejections
**
** Then the docking-port quantities in the docking frame D: port position,
** port point velocity v_cm + ω×r, range, closing speed, lateral offset, and the
** quaternion attitude error to the docking attitude.
*/

#include "common_types.h"
#include "nav_status.h"
#include "nav_tbl.h"
#include "simlink_icd.h"

#define NAV_NX 9 /* translation filter: r_L (3), v_L (3), accelerometer bias b_a (3, body) */
#define NAV_NA 6 /* attitude filter error state: δθ (3, body), gyro bias δb (3)            */

/* Attitude uncertainty growth per second with no gyro (a slow tumble we can't see) */
#define NAV_NO_GYRO_RATE_SIGMA 0.003

/* Event bits NAV_Step reports to the app (turned into EVS events there) */
#define NAV_EVT_FILTER_INIT   0x01u /* translation: first initialisation, or after a reset command */
#define NAV_EVT_FILTER_REINIT 0x02u /* translation: divergence recovery / non-finite state reset   */
#define NAV_EVT_SENSOR_REJECT 0x04u /* a reading flagged valid was malformed and discarded         */
#define NAV_EVT_ATT_INIT      0x08u /* attitude: first initialisation                              */
#define NAV_EVT_ATT_REINIT    0x10u /* attitude: divergence recovery / non-finite state reset      */

typedef struct
{
    bool   Initialized;
    double X[NAV_NX];         /* r_L (m), v_L (m/s): chaser CoM - target ref; b_a (m/s², body) */
    double P[NAV_NX][NAV_NX]; /* covariance */
    uint32 ConsecRejects;
} NAV_Filter_t;

typedef struct
{
    bool   Initialized;
    float  Q[4];              /* q_L_B estimate                       */
    double Bias[3];           /* gyro bias estimate (rad/s)           */
    double P[NAV_NA][NAV_NA]; /* covariance of [δθ, δb]               */
    float  PrevGyro[3];       /* raw gyro of the previous cycle       */
    bool   HavePrevGyro;
    uint32 ConsecRejects;
} NAV_AttFilter_t;

typedef struct
{
    NAV_Filter_t    Filter;
    NAV_AttFilter_t Att;
} NAV_State_t;

typedef struct
{
    uint32 Status; /* NAV_STATUS_* (nav_status.h) */
    uint32 Events; /* NAV_EVT_*                   */
    float  AttQuat_L[4];
    float  AngRate_B[3];
    float  AttErr_B[3];
    float  CmPos_L[3];
    float  CmVel_L[3];
    float  PortPos_D[3];
    float  PortVel_D[3];
    float  Range_m;
    float  ClosingSpeed_ms;
    float  LateralOffset_m;
    float  DeltaV_B[3];
    float  PosSigma_m;
    float  VelSigma_ms;
    float  AttSigma_rad;
    float  DockFrameQuat_L[4]; /* q_L_D from the table geometry */
    float  GyroBias_rads[3];
    float  AccelBias_mss[3];
    float  InnovSigma;      /* Mahalanobis distance of this cycle's LIDAR fix (0 if none) */
    float  AttInnovSigma;   /* largest Mahalanobis distance of this cycle's attitude fixes */
    uint32 ConsecRpsRejects;
    bool   RpsAvailable;
} NAV_Output_t;

/* Quaternion helpers, (x, y, z, w), Hamilton product */
void NAV_QuatMul(const float a[4], const float b[4], float out[4]);
void NAV_QuatConj(const float q[4], float out[4]);
void NAV_QuatRotate(const float q[4], const float v[3], float out[3]);
bool NAV_QuatNormalize(float q[4], float tolerance);

/* Table sanity check shared by the CFE_TBL validator and the test harness;
** returns NULL when OK, otherwise a short description of the first problem. */
const char *NAV_CheckConfig(const NAV_CfgTbl_t *Cfg);

void NAV_Reset(NAV_State_t *S);
void NAV_ResetFilter(NAV_State_t *S); /* translation filter only */

/* One navigation cycle. dt has already been range-checked by the caller. */
void NAV_Step(NAV_State_t *S, const NAV_CfgTbl_t *Cfg, const SIMLINK_SimState_t *In, NAV_Output_t *Out);

#endif /* NAV_FILTER_H */
