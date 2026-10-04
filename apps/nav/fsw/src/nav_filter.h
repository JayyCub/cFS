#ifndef NAV_FILTER_H
#define NAV_FILTER_H

/*
** NAV estimation core — pure math, no cFE calls, so it can be compiled into a
** stand-alone test harness (apps/nav/unit-test/nav_filter_sim.c).
**
** Per cycle (NAV_Step):
**   1. Attitude  — star tracker quaternion when present; otherwise the last
**                  attitude propagated with the gyro (for up to
**                  NAV_MAX_ATT_PROP_CYCLES).
**   2. Translation — 6-state Kalman filter, chaser CoM relative to the target
**      reference point in LVLH:
**        propagate: Clohessy-Wiltshire dynamics (same equations as
**                   ClohessyWiltshire.cs and the GNC feed-forward) plus the
**                   IMU's measured non-gravitational Δv, rotated to LVLH
**        update:    LIDAR range / azimuth / elevation to the target port
**                   reflector, turned into a CoM position fix through the
**                   attitude and the table geometry; chi-square innovation
**                   gate; re-initialise after MaxConsecRejects rejections
**   3. Docking-port quantities in the docking frame D: port position, port
**      point velocity v_cm + ω×r, range, closing speed, lateral offset, and the
**      quaternion attitude error to the docking attitude.
**
** Attitude estimation is deliberately simple in phase 4a (no gyro-bias
** estimation, no LIDAR relative attitude). Phase 4b replaces it with a MEKF.
*/

#include "common_types.h"
#include "nav_status.h"
#include "nav_tbl.h"
#include "simlink_icd.h"

/* Attitude is reported valid for this many cycles of gyro-only propagation */
#define NAV_MAX_ATT_PROP_CYCLES 50

/* Event bits NAV_Step reports to the app (turned into EVS events there) */
#define NAV_EVT_FILTER_INIT     0x1u /* first initialisation, or after a reset command          */
#define NAV_EVT_FILTER_REINIT   0x2u /* divergence recovery: MaxConsecRejects LIDAR rejections */
#define NAV_EVT_SENSOR_REJECT   0x4u /* a reading flagged valid was malformed and discarded     */

typedef struct
{
    bool   Initialized;
    double X[6];    /* r_L (m), v_L (m/s): chaser CoM - target reference point */
    double P[6][6]; /* covariance */
    uint32 ConsecRejects;
} NAV_Filter_t;

typedef struct
{
    NAV_Filter_t Filter;
    float        Att[4];        /* last attitude q_L_B */
    bool         HaveAtt;
    uint32       AttPropCycles; /* cycles since the last star tracker fix */
} NAV_State_t;

typedef struct
{
    uint32 Status; /* NAV_STATUS_* (nav_msg.h) */
    uint32 Events; /* NAV_EVT_*                */
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
    float  InnovSigma; /* Mahalanobis distance of this cycle's LIDAR fix (0 if none) */
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
void NAV_ResetFilter(NAV_State_t *S);

/* One navigation cycle. dt has already been range-checked by the caller. */
void NAV_Step(NAV_State_t *S, const NAV_CfgTbl_t *Cfg, const SIMLINK_SimState_t *In, NAV_Output_t *Out);

#endif /* NAV_FILTER_H */
