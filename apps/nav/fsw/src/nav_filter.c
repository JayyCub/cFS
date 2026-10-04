/*
** NAV estimation core — see nav_filter.h for the algorithm overview.
**
** Frame and sign conventions match Unity's, so that what Unity measures and
** what FSW computes agree without any handedness bookkeeping: Unity composes
** quaternions with the Hamilton product, rotates vectors with q v q*, and its
** rigid-body point velocity is v + ω × r with the standard cross-product
** formula. The same algebra is used here throughout.
**
** The filter runs in double precision: position variances span ~1e-6 m²
** (LIDAR at close range) to ~1e2 m² (after a long LIDAR outage).
*/
#include <math.h>
#include <string.h>

#include "nav_filter.h"

/* ======================================================================= */
/* Small vector / quaternion helpers                                        */
/* ======================================================================= */

static void Cross(const float a[3], const float b[3], float out[3])
{
    float x = a[1] * b[2] - a[2] * b[1];
    float y = a[2] * b[0] - a[0] * b[2];
    float z = a[0] * b[1] - a[1] * b[0];
    out[0]  = x;
    out[1]  = y;
    out[2]  = z;
}

static float Norm3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static bool Finite(const float *v, int n)
{
    int i;
    for (i = 0; i < n; i++)
    {
        if (!isfinite(v[i]))
            return false;
    }
    return true;
}

void NAV_QuatMul(const float a[4], const float b[4], float out[4])
{
    float x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    float y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    float z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    out[0]  = x;
    out[1]  = y;
    out[2]  = z;
    out[3]  = w;
}

void NAV_QuatConj(const float q[4], float out[4])
{
    out[0] = -q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = q[3];
}

void NAV_QuatRotate(const float q[4], const float v[3], float out[3])
{
    /* v' = v + w t + u × t, t = 2 u × v  (u = vector part) */
    float t[3], ut[3];
    Cross(q, v, t);
    t[0] *= 2.0f;
    t[1] *= 2.0f;
    t[2] *= 2.0f;
    Cross(q, t, ut);
    out[0] = v[0] + q[3] * t[0] + ut[0];
    out[1] = v[1] + q[3] * t[1] + ut[1];
    out[2] = v[2] + q[3] * t[2] + ut[2];
}

bool NAV_QuatNormalize(float q[4], float tolerance)
{
    float n;

    if (!Finite(q, 4))
        return false;
    n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (fabsf(n - 1.0f) > tolerance)
        return false;
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
    return true;
}

/* Rotation vector (rad) of a unit quaternion, shortest way round */
static void QuatToRotVec(const float q[4], float rv[3])
{
    float s   = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
    float sgn = (q[3] < 0.0f) ? -1.0f : 1.0f;
    float k;

    if (s < 1e-9f)
    {
        k = 2.0f * sgn;
    }
    else
    {
        k = 2.0f * atan2f(s, fabsf(q[3])) / s * sgn;
    }
    rv[0] = k * q[0];
    rv[1] = k * q[1];
    rv[2] = k * q[2];
}

/* Columns of the rotation matrix of q, in double for the covariance math */
static void QuatToDcm(const float q[4], double m[3][3])
{
    static const float E[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float              c[3];
    int                i, j;

    for (j = 0; j < 3; j++)
    {
        NAV_QuatRotate(q, E[j], c);
        for (i = 0; i < 3; i++)
            m[i][j] = c[i];
    }
}

/* ======================================================================= */
/* Configuration check                                                      */
/* ======================================================================= */

static bool UnitQuat(const float q[4])
{
    float n2;
    if (!Finite(q, 4))
        return false;
    n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    return fabsf(n2 - 1.0f) < 2e-3f;
}

const char *NAV_CheckConfig(const NAV_CfgTbl_t *Cfg)
{
    if (!UnitQuat(Cfg->ChaserPortQuat_B) || !UnitQuat(Cfg->RpsQuat_B) || !UnitQuat(Cfg->TargetAttQuat_L) ||
        !UnitQuat(Cfg->TargetPortQuat_T) || !UnitQuat(Cfg->DockedQuat_TP))
        return "a geometry quaternion is not unit length";
    if (!Finite(Cfg->ChaserPortPos_B, 3) || !Finite(Cfg->RpsPos_B, 3) || !Finite(Cfg->TargetPortPos_T, 3))
        return "a geometry position is not finite";
    if (!(Cfg->MeanMotion_rads >= 0.0f && Cfg->MeanMotion_rads < 0.01f))
        return "MeanMotion_rads outside [0, 0.01)";
    if (!(Cfg->RpsRangeSigma_m > 0.0f) || !(Cfg->RpsRangeSigmaFrac >= 0.0f) || !(Cfg->RpsAngleSigma_rad > 0.0f) ||
        !(Cfg->AttSigma_rad >= 0.0f))
        return "a measurement sigma is not positive";
    if (!(Cfg->AccelNoise_mss >= 0.0f) || !(Cfg->DeltaVNoise_ms >= 0.0f) || !(Cfg->AccelNoise_mss > 0.0f || Cfg->DeltaVNoise_ms > 0.0f))
        return "process noise must be positive";
    if (!(Cfg->InitVelSigma_ms > 0.0f) || !(Cfg->GateSigma > 0.0f) || Cfg->MaxConsecRejects < 1)
        return "InitVelSigma_ms, GateSigma or MaxConsecRejects not positive";
    if (!(Cfg->MaxPosSigma_m > 0.0f) || !(Cfg->MaxVelSigma_ms > 0.0f))
        return "MaxPosSigma_m / MaxVelSigma_ms not positive";
    return NULL;
}

/* ======================================================================= */
/* Translation filter                                                       */
/* ======================================================================= */

void NAV_ResetFilter(NAV_State_t *S)
{
    memset(&S->Filter, 0, sizeof(S->Filter));
}

void NAV_Reset(NAV_State_t *S)
{
    memset(S, 0, sizeof(*S));
}

/*
** Propagate over dt: Clohessy-Wiltshire, x' = A x, mapped to world axes as in
** ClohessyWiltshire.cs (+Y radial, -Z along-track, +X cross-track):
**   a_x = -n² r_x              (cross-track)
**   a_y = 3n² r_y - 2n v_z     (radial; v_along = -v_z)
**   a_z = 2n v_y               (-a_along)
** Φ = exp(A dt) by its series to third order (n·dt ~ 2e-4 per cycle, so the
** truncation is far below float precision). The IMU's Δv over the cycle is
** added as an impulse at mid-interval: it moves the position by Δv·dt/2.
**
** Process noise: unmodelled acceleration (σ_a) and accelerometer noise (σ_dv)
** both enter like that mid-interval impulse, per axis
**   q = σ_a² dt² + σ_dv²,   Q = q [[dt²/4, dt/2], [dt/2, 1]]
*/
static void FilterPropagate(NAV_Filter_t *F, const NAV_CfgTbl_t *Cfg, double dt, const double dv_L[3])
{
    double n = Cfg->MeanMotion_rads;
    double A[6][6], M2[6][6], M3[6][6], Phi[6][6], T[6][6], X[6];
    double q;
    int    i, j, k;

    memset(A, 0, sizeof(A));
    for (i = 0; i < 3; i++)
        A[i][i + 3] = dt;
    A[3][0] = -n * n * dt;
    A[4][1] = 3.0 * n * n * dt;
    A[4][5] = -2.0 * n * dt;
    A[5][4] = 2.0 * n * dt;

    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
        {
            M2[i][j] = 0.0;
            for (k = 0; k < 6; k++)
                M2[i][j] += A[i][k] * A[k][j];
        }
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
        {
            M3[i][j] = 0.0;
            for (k = 0; k < 6; k++)
                M3[i][j] += M2[i][k] * A[k][j];
        }
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
            Phi[i][j] = (i == j ? 1.0 : 0.0) + A[i][j] + M2[i][j] / 2.0 + M3[i][j] / 6.0;

    /* State */
    for (i = 0; i < 6; i++)
    {
        X[i] = 0.0;
        for (k = 0; k < 6; k++)
            X[i] += Phi[i][k] * F->X[k];
    }
    for (i = 0; i < 3; i++)
    {
        X[i] += dv_L[i] * dt / 2.0;
        X[i + 3] += dv_L[i];
    }
    memcpy(F->X, X, sizeof(X));

    /* Covariance: Φ P Φᵀ + Q */
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
        {
            T[i][j] = 0.0;
            for (k = 0; k < 6; k++)
                T[i][j] += Phi[i][k] * F->P[k][j];
        }
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
        {
            F->P[i][j] = 0.0;
            for (k = 0; k < 6; k++)
                F->P[i][j] += T[i][k] * Phi[j][k];
        }

    q = (double)Cfg->AccelNoise_mss * Cfg->AccelNoise_mss * dt * dt +
        (double)Cfg->DeltaVNoise_ms * Cfg->DeltaVNoise_ms;
    for (i = 0; i < 3; i++)
    {
        F->P[i][i] += q * dt * dt / 4.0;
        F->P[i][i + 3] += q * dt / 2.0;
        F->P[i + 3][i] += q * dt / 2.0;
        F->P[i + 3][i + 3] += q;
    }
}

static bool Inverse3(double m[3][3], double inv[3][3])
{
    double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                 m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    if (!(fabs(det) > 1e-300) || !isfinite(det))
        return false;

    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det;
    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det;
    return true;
}

/*
** LIDAR fix -> chaser CoM position in LVLH, with its covariance.
**
**   p_S = range · (cos el sin az, sin el, cos el cos az)   reflector, sensor frame
**   p_B = RpsPos_B + q_B_S p_S                             reflector rel. CoM, body
**   z   = q_L_T TargetPortPos_T - q_L_B p_B                CoM rel. target ref., LVLH
**
** Noise: range σ_r = RangeSigma + Frac·range and angle σ_a on az and el,
** mapped to position through the Jacobian of p_S(range, az, el) — so the
** range noise lies along the actual line of sight, which is well off the
** boresight at close range or with the chaser rotated (an earlier version
** put it along boresight and the gate then rejected good fixes inside 1 m).
** Plus the star tracker error swinging the whole lever arm |p_B| (isotropic,
** slightly conservative).
*/
static void RpsFix(const NAV_CfgTbl_t *Cfg, const float q_L_B[4], float range, float az, float el, double z[3],
                   double Rm[3][3])
{
    float  p_S[3], p_B[3], p_L[3], tp_L[3], q_L_S[4];
    double M[3][3], J[3][3], G[3][3], D[3];
    double s_r = (double)Cfg->RpsRangeSigma_m + (double)Cfg->RpsRangeSigmaFrac * range;
    double s_a = Cfg->RpsAngleSigma_rad;
    double ce = cos(el), se = sin(el), ca = cos(az), sa = sin(az);
    double s_att;
    int    i, j, k;

    p_S[0] = range * cosf(el) * sinf(az);
    p_S[1] = range * sinf(el);
    p_S[2] = range * cosf(el) * cosf(az);
    NAV_QuatRotate(Cfg->RpsQuat_B, p_S, p_B);
    for (i = 0; i < 3; i++)
        p_B[i] += Cfg->RpsPos_B[i];
    NAV_QuatRotate(q_L_B, p_B, p_L);
    NAV_QuatRotate(Cfg->TargetAttQuat_L, Cfg->TargetPortPos_T, tp_L);
    for (i = 0; i < 3; i++)
        z[i] = (double)tp_L[i] - p_L[i];

    /* Jacobian columns ∂p_S/∂range, ∂p_S/∂az, ∂p_S/∂el (sensor frame) */
    J[0][0] = ce * sa;
    J[1][0] = se;
    J[2][0] = ce * ca;
    J[0][1] = range * ce * ca;
    J[1][1] = 0.0;
    J[2][1] = -range * ce * sa;
    J[0][2] = -range * se * sa;
    J[1][2] = range * ce;
    J[2][2] = -range * se * ca;

    /* G = M_L_S J; the fix z = tp_L - p_L, so its sign flips but the
    ** covariance G D Gᵀ doesn't care */
    NAV_QuatMul(q_L_B, Cfg->RpsQuat_B, q_L_S);
    QuatToDcm(q_L_S, M);
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            G[i][j] = 0.0;
            for (k = 0; k < 3; k++)
                G[i][j] += M[i][k] * J[k][j];
        }
    D[0]  = s_r * s_r;
    D[1]  = s_a * s_a;
    D[2]  = s_a * s_a;
    s_att = (double)Cfg->AttSigma_rad * Norm3(p_B);
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            Rm[i][j] = (i == j) ? s_att * s_att : 0.0;
            for (k = 0; k < 3; k++)
                Rm[i][j] += G[i][k] * D[k] * G[j][k];
        }
}

static void FilterInit(NAV_Filter_t *F, const NAV_CfgTbl_t *Cfg, const double z[3], double Rm[3][3])
{
    int i, j;

    memset(F, 0, sizeof(*F));
    for (i = 0; i < 3; i++)
    {
        F->X[i] = z[i];
        for (j = 0; j < 3; j++)
            F->P[i][j] = Rm[i][j];
        F->P[i + 3][i + 3] = (double)Cfg->InitVelSigma_ms * Cfg->InitVelSigma_ms;
    }
    F->Initialized = true;
}

/* Kalman update with a direct position measurement (H = [I 0]), Joseph form.
** Returns false (and leaves the filter alone) if the fix fails the gate. */
static bool FilterUpdate(NAV_Filter_t *F, const double z[3], double Rm[3][3], double gate, double *mahal)
{
    double S[3][3], Si[3][3], K[6][3], y[3], d2 = 0.0;
    double IKH[6][6], T[6][6], Pn[6][6];
    int    i, j, k;

    for (i = 0; i < 3; i++)
    {
        y[i] = z[i] - F->X[i];
        for (j = 0; j < 3; j++)
            S[i][j] = F->P[i][j] + Rm[i][j];
    }
    if (!Inverse3(S, Si))
    {
        *mahal = INFINITY;
        return false;
    }
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            d2 += y[i] * Si[i][j] * y[j];
    *mahal = sqrt(d2 > 0.0 ? d2 : 0.0);
    if (!(*mahal <= gate))
        return false;

    /* K = P Hᵀ S⁻¹ = P[:, 0:3] S⁻¹ */
    for (i = 0; i < 6; i++)
        for (j = 0; j < 3; j++)
        {
            K[i][j] = 0.0;
            for (k = 0; k < 3; k++)
                K[i][j] += F->P[i][k] * Si[k][j];
        }
    for (i = 0; i < 6; i++)
        for (k = 0; k < 3; k++)
            F->X[i] += K[i][k] * y[k];

    /* P = (I - KH) P (I - KH)ᵀ + K R Kᵀ */
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
            IKH[i][j] = (i == j ? 1.0 : 0.0) - (j < 3 ? K[i][j] : 0.0);
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
        {
            T[i][j] = 0.0;
            for (k = 0; k < 6; k++)
                T[i][j] += IKH[i][k] * F->P[k][j];
        }
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
        {
            Pn[i][j] = 0.0;
            for (k = 0; k < 6; k++)
                Pn[i][j] += T[i][k] * IKH[j][k];
        }
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
        {
            double krk = 0.0;
            int    a, b;
            for (a = 0; a < 3; a++)
                for (b = 0; b < 3; b++)
                    krk += K[i][a] * Rm[a][b] * K[j][b];
            Pn[i][j] += krk;
        }
    /* symmetrise against round-off */
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
            F->P[i][j] = 0.5 * (Pn[i][j] + Pn[j][i]);
    return true;
}

static bool FilterFinite(const NAV_Filter_t *F)
{
    int i, j;
    for (i = 0; i < 6; i++)
    {
        if (!isfinite(F->X[i]))
            return false;
        for (j = 0; j < 6; j++)
            if (!isfinite(F->P[i][j]))
                return false;
    }
    return true;
}

/* ======================================================================= */
/* One navigation cycle                                                     */
/* ======================================================================= */

void NAV_Step(NAV_State_t *S, const NAV_CfgTbl_t *Cfg, const SIMLINK_SimState_t *In, NAV_Output_t *Out)
{
    float  dt   = In->CycleDt_s;
    bool   imu  = false;
    bool   att  = false;
    float  rate[3] = {0.0f, 0.0f, 0.0f};
    float  dv_B[3] = {0.0f, 0.0f, 0.0f};
    int    i;

    memset(Out, 0, sizeof(*Out));

    /* --- 1. IMU --------------------------------------------------------- */
    if (In->SensorValid & SIMLINK_SENSOR_IMU)
    {
        if (Finite(In->GyroRate_B, 3) && Finite(In->DeltaV_B, 3))
        {
            imu = true;
            memcpy(rate, In->GyroRate_B, sizeof(rate));
            memcpy(dv_B, In->DeltaV_B, sizeof(dv_B));
            Out->Status |= NAV_STATUS_RATE_VALID | NAV_STATUS_DV_VALID;
        }
        else
        {
            Out->Events |= NAV_EVT_SENSOR_REJECT;
        }
    }

    /* --- 2. Attitude: star tracker, else gyro propagation ---------------- */
    if (In->SensorValid & SIMLINK_SENSOR_ST)
    {
        float q[4];
        memcpy(q, In->StQuat, sizeof(q));
        if (NAV_QuatNormalize(q, 0.01f))
        {
            memcpy(S->Att, q, sizeof(q));
            S->HaveAtt       = true;
            S->AttPropCycles = 0;
            att              = true;
            Out->Status |= NAV_STATUS_ST_USED;
        }
        else
        {
            Out->Events |= NAV_EVT_SENSOR_REJECT;
        }
    }
    if (!att && S->HaveAtt && imu && S->AttPropCycles < NAV_MAX_ATT_PROP_CYCLES)
    {
        /* q ← q ⊗ δq(ω dt), body-frame rate */
        float th = Norm3(rate) * dt;
        float dq[4];
        if (th > 1e-12f)
        {
            float s = sinf(0.5f * th) / Norm3(rate);
            dq[0]   = rate[0] * s;
            dq[1]   = rate[1] * s;
            dq[2]   = rate[2] * s;
            dq[3]   = cosf(0.5f * th);
            NAV_QuatMul(S->Att, dq, S->Att);
            (void)NAV_QuatNormalize(S->Att, 0.5f);
        }
        S->AttPropCycles++;
        att = true;
    }
    else if (!att)
    {
        S->HaveAtt = S->HaveAtt && S->AttPropCycles < NAV_MAX_ATT_PROP_CYCLES;
    }
    if (att)
        Out->Status |= NAV_STATUS_ATT_VALID;

    /* --- 3. Translation filter: propagate ------------------------------- */
    if (S->Filter.Initialized)
    {
        double dv_L[3] = {0.0, 0.0, 0.0};
        if (imu && att)
        {
            float v[3];
            NAV_QuatRotate(S->Att, dv_B, v);
            for (i = 0; i < 3; i++)
                dv_L[i] = v[i];
        }
        FilterPropagate(&S->Filter, Cfg, dt, dv_L);
    }

    /* --- 4. LIDAR update ------------------------------------------------- */
    if (In->SensorValid & SIMLINK_SENSOR_RPS)
    {
        bool ok = isfinite(In->RpsRange_m) && isfinite(In->RpsAz_rad) && isfinite(In->RpsEl_rad) &&
                  In->RpsRange_m > 0.0f;
        if (!ok)
        {
            Out->Events |= NAV_EVT_SENSOR_REJECT;
        }
        else if (att) /* the fix needs attitude to become an LVLH position */
        {
            double z[3], Rm[3][3], mahal = 0.0;

            Out->RpsAvailable = true;
            RpsFix(Cfg, S->Att, In->RpsRange_m, In->RpsAz_rad, In->RpsEl_rad, z, Rm);

            if (!S->Filter.Initialized)
            {
                FilterInit(&S->Filter, Cfg, z, Rm);
                Out->Events |= NAV_EVT_FILTER_INIT;
                Out->Status |= NAV_STATUS_RPS_USED;
            }
            else if (FilterUpdate(&S->Filter, z, Rm, Cfg->GateSigma, &mahal))
            {
                S->Filter.ConsecRejects = 0;
                Out->Status |= NAV_STATUS_RPS_USED;
            }
            else
            {
                Out->Status |= NAV_STATUS_RPS_REJECTED;
                if (++S->Filter.ConsecRejects >= Cfg->MaxConsecRejects)
                {
                    /* The filter and the sensor disagree persistently: trust the
                    ** sensor (scenario reset, missed thrust, a bad propagation). */
                    FilterInit(&S->Filter, Cfg, z, Rm);
                    Out->Events |= NAV_EVT_FILTER_REINIT;
                    Out->Status |= NAV_STATUS_RPS_USED;
                }
            }
            Out->InnovSigma = isfinite(mahal) ? (float)mahal : 1e30f;
        }
    }

    if (S->Filter.Initialized && !FilterFinite(&S->Filter))
    {
        NAV_ResetFilter(S);
        Out->Events |= NAV_EVT_FILTER_REINIT;
    }

    /* --- 5. Outputs ------------------------------------------------------ */
    memcpy(Out->AngRate_B, rate, sizeof(rate));
    memcpy(Out->DeltaV_B, dv_B, sizeof(dv_B));

    if (att)
    {
        float q_L_D[4], q_L_Bdes[4], q_tmp[4], q_e[4];

        memcpy(Out->AttQuat_L, S->Att, sizeof(S->Att));

        /* Docking frame and docking attitude from the table geometry */
        NAV_QuatMul(Cfg->TargetAttQuat_L, Cfg->TargetPortQuat_T, q_tmp);
        NAV_QuatMul(q_tmp, Cfg->DockedQuat_TP, q_L_D);
        NAV_QuatConj(Cfg->ChaserPortQuat_B, q_tmp);
        NAV_QuatMul(q_L_D, q_tmp, q_L_Bdes);

        /* Attitude error: q_e = conj(q_L_B) ⊗ q_L_Bdes, as a body-axis rotation vector */
        NAV_QuatConj(S->Att, q_tmp);
        NAV_QuatMul(q_tmp, q_L_Bdes, q_e);
        QuatToRotVec(q_e, Out->AttErr_B);

        if (S->Filter.Initialized)
        {
            const NAV_Filter_t *F = &S->Filter;
            float r_port_L[3], wxr_B[3], wxr_L[3], tp_L[3], pos_L[3], vel_L[3], q_D_L[4];
            double ps = 0.0, vs = 0.0;

            NAV_QuatRotate(S->Att, Cfg->ChaserPortPos_B, r_port_L);
            Cross(rate, Cfg->ChaserPortPos_B, wxr_B);
            NAV_QuatRotate(S->Att, wxr_B, wxr_L);
            NAV_QuatRotate(Cfg->TargetAttQuat_L, Cfg->TargetPortPos_T, tp_L);

            for (i = 0; i < 3; i++)
            {
                Out->CmPos_L[i] = (float)F->X[i];
                Out->CmVel_L[i] = (float)F->X[i + 3];
                pos_L[i]        = (float)F->X[i] + r_port_L[i] - tp_L[i];
                vel_L[i]        = (float)F->X[i + 3] + wxr_L[i];
                if (F->P[i][i] > ps)
                    ps = F->P[i][i];
                if (F->P[i + 3][i + 3] > vs)
                    vs = F->P[i + 3][i + 3];
            }
            NAV_QuatConj(q_L_D, q_D_L);
            NAV_QuatRotate(q_D_L, pos_L, Out->PortPos_D);
            NAV_QuatRotate(q_D_L, vel_L, Out->PortVel_D);

            Out->Range_m         = Norm3(Out->PortPos_D);
            Out->ClosingSpeed_ms = Out->PortVel_D[2];
            Out->LateralOffset_m = sqrtf(Out->PortPos_D[0] * Out->PortPos_D[0] + Out->PortPos_D[1] * Out->PortPos_D[1]);
            Out->PosSigma_m      = (float)sqrt(ps);
            Out->VelSigma_ms     = (float)sqrt(vs);

            Out->Status |= NAV_STATUS_FILTER_INIT;
            if (imu && Out->PosSigma_m <= Cfg->MaxPosSigma_m && Out->VelSigma_ms <= Cfg->MaxVelSigma_ms)
                Out->Status |= NAV_STATUS_RELNAV_VALID;
        }
    }
    else if (S->Filter.Initialized)
    {
        Out->Status |= NAV_STATUS_FILTER_INIT;
    }
}
