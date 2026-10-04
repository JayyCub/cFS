/*
** NAV estimation core — see nav_filter.h for the algorithm overview.
**
** Frame and sign conventions match Unity's, so that what Unity measures and
** what FSW computes agree without any handedness bookkeeping: Unity composes
** quaternions with the Hamilton product, rotates vectors with q v q*, and its
** rigid-body point velocity is v + ω × r with the standard cross-product
** formula. The same algebra is used here throughout.
**
** Attitude error convention (MEKF): the true attitude is q̂ ⊗ δq(δθ), δθ a small
** rotation in body axes. With the bias-corrected rate ω, δθ' = -ω×δθ - δb.
**
** The filters run in double precision: position variances span ~1e-6 m²
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

/* q ← q ⊗ δq(θ), θ a rotation vector in the frame q rotates from (body) */
static void QuatApplyRotVec(float q[4], double tx, double ty, double tz)
{
    double th = sqrt(tx * tx + ty * ty + tz * tz);
    float  dq[4];

    if (th < 1e-12)
        return;
    {
        double s = sin(0.5 * th) / th;
        dq[0]    = (float)(tx * s);
        dq[1]    = (float)(ty * s);
        dq[2]    = (float)(tz * s);
        dq[3]    = (float)cos(0.5 * th);
    }
    NAV_QuatMul(q, dq, q);
    (void)NAV_QuatNormalize(q, 0.5f);
}

/* Rotation matrix of q (columns = rotated basis vectors), row-major 3×3 */
static void QuatToDcm(const float q[4], double m[9])
{
    static const float E[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float              c[3];
    int                i, j;

    for (j = 0; j < 3; j++)
    {
        NAV_QuatRotate(q, E[j], c);
        for (i = 0; i < 3; i++)
            m[i * 3 + j] = c[i];
    }
}

/* ======================================================================= */
/* Matrix helpers — row-major flat arrays, n <= NAV_NX                      */
/* ======================================================================= */

/* C(n×m) = A(n×k) · B(k×m); C must not alias A or B */
static void MatMul(const double *A, const double *B, double *C, int n, int k, int m)
{
    int i, j, l;
    for (i = 0; i < n; i++)
        for (j = 0; j < m; j++)
        {
            double s = 0.0;
            for (l = 0; l < k; l++)
                s += A[i * k + l] * B[l * m + j];
            C[i * m + j] = s;
        }
}

/* C(n×m) = A(n×k) · B(m×k)ᵀ; C must not alias A or B */
static void MatMulT(const double *A, const double *B, double *C, int n, int k, int m)
{
    int i, j, l;
    for (i = 0; i < n; i++)
        for (j = 0; j < m; j++)
        {
            double s = 0.0;
            for (l = 0; l < k; l++)
                s += A[i * k + l] * B[j * k + l];
            C[i * m + j] = s;
        }
}

/* P ← Φ P Φᵀ (n×n) */
static void CovPropagate(double *P, const double *Phi, int n)
{
    double T[NAV_NX * NAV_NX];
    MatMul(Phi, P, T, n, n, n);
    MatMulT(T, Phi, P, n, n, n);
}

static bool Inverse3(const double *m, double *inv)
{
    double det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
                 m[2] * (m[3] * m[7] - m[4] * m[6]);

    if (!(fabs(det) > 1e-300) || !isfinite(det))
        return false;

    inv[0] = (m[4] * m[8] - m[5] * m[7]) / det;
    inv[1] = (m[2] * m[7] - m[1] * m[8]) / det;
    inv[2] = (m[1] * m[5] - m[2] * m[4]) / det;
    inv[3] = (m[5] * m[6] - m[3] * m[8]) / det;
    inv[4] = (m[0] * m[8] - m[2] * m[6]) / det;
    inv[5] = (m[2] * m[3] - m[0] * m[5]) / det;
    inv[6] = (m[3] * m[7] - m[4] * m[6]) / det;
    inv[7] = (m[1] * m[6] - m[0] * m[7]) / det;
    inv[8] = (m[0] * m[4] - m[1] * m[3]) / det;
    return true;
}

/*
** Kalman update for a 3-vector measurement of the first three states
** (H = [I 0]), Joseph form. x is the state (translation) or the error state
** (attitude) the correction K·y is added to. Returns false and leaves x, P
** untouched if the innovation fails the chi-square gate.
*/
static bool KalmanUpdate3(int n, double *x, double *P, const double y[3], const double Rm[9], double gate,
                          double *mahal)
{
    double S[9], Si[9], K[NAV_NX * 3], IKH[NAV_NX * NAV_NX], T[NAV_NX * NAV_NX], Pn[NAV_NX * NAV_NX];
    double KR[NAV_NX * 3], d2 = 0.0;
    int    i, j;

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            S[i * 3 + j] = P[i * n + j] + Rm[i * 3 + j];
    if (!Inverse3(S, Si))
    {
        *mahal = INFINITY;
        return false;
    }
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            d2 += y[i] * Si[i * 3 + j] * y[j];
    *mahal = sqrt(d2 > 0.0 ? d2 : 0.0);
    if (!(*mahal <= gate))
        return false;

    /* K = P Hᵀ S⁻¹ = P[:, 0:3] S⁻¹ */
    for (i = 0; i < n; i++)
        for (j = 0; j < 3; j++)
            K[i * 3 + j] = P[i * n + 0] * Si[0 * 3 + j] + P[i * n + 1] * Si[1 * 3 + j] + P[i * n + 2] * Si[2 * 3 + j];
    for (i = 0; i < n; i++)
        x[i] += K[i * 3 + 0] * y[0] + K[i * 3 + 1] * y[1] + K[i * 3 + 2] * y[2];

    /* P = (I - KH) P (I - KH)ᵀ + K R Kᵀ */
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            IKH[i * n + j] = (i == j ? 1.0 : 0.0) - (j < 3 ? K[i * 3 + j] : 0.0);
    MatMul(IKH, P, T, n, n, n);
    MatMulT(T, IKH, Pn, n, n, n);
    MatMul(K, Rm, KR, n, 3, 3);
    MatMulT(KR, K, T, n, 3, n);
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            P[i * n + j] = 0.5 * ((Pn[i * n + j] + T[i * n + j]) + (Pn[j * n + i] + T[j * n + i]));
    return true;
}

static bool AllFinite(const double *v, int n)
{
    int i;
    for (i = 0; i < n; i++)
        if (!isfinite(v[i]))
            return false;
    return true;
}

static double MaxDiagSigma(const double *P, int n, int first, int count)
{
    double m = 0.0;
    int    i;
    for (i = first; i < first + count; i++)
        if (P[i * n + i] > m)
            m = P[i * n + i];
    return sqrt(m);
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
        !(Cfg->AttSigma_rad > 0.0f) || !(Cfg->RpsPoseSigma_rad > 0.0f))
        return "a measurement sigma is not positive";
    if (!(Cfg->AccelNoise_mss >= 0.0f) || !(Cfg->DeltaVNoise_ms >= 0.0f) ||
        !(Cfg->AccelNoise_mss > 0.0f || Cfg->DeltaVNoise_ms > 0.0f) || !(Cfg->GyroNoise_rads > 0.0f) ||
        !(Cfg->AccelBiasWalk_mss >= 0.0f) || !(Cfg->GyroBiasWalk_rads >= 0.0f))
        return "process noise must be positive";
    if (!(Cfg->InitVelSigma_ms > 0.0f) || !(Cfg->InitAccelBiasSigma_mss > 0.0f) ||
        !(Cfg->InitGyroBiasSigma_rads > 0.0f))
        return "an initial sigma is not positive";
    if (!(Cfg->GateSigma > 0.0f) || Cfg->MaxConsecRejects < 1)
        return "GateSigma or MaxConsecRejects not positive";
    if (!(Cfg->MaxPosSigma_m > 0.0f) || !(Cfg->MaxVelSigma_ms > 0.0f) || !(Cfg->MaxAttSigma_rad > 0.0f))
        return "a validity sigma limit is not positive";
    return NULL;
}

/* ======================================================================= */
/* State resets                                                             */
/* ======================================================================= */

void NAV_ResetFilter(NAV_State_t *S)
{
    memset(&S->Filter, 0, sizeof(S->Filter));
}

void NAV_Reset(NAV_State_t *S)
{
    memset(S, 0, sizeof(*S));
}

/* ======================================================================= */
/* Attitude filter (MEKF)                                                   */
/* ======================================================================= */

static void AttInit(NAV_AttFilter_t *A, const NAV_CfgTbl_t *Cfg, const float q[4], double sigma, bool keepBias)
{
    double b[3] = {0.0, 0.0, 0.0};
    double sb   = Cfg->InitGyroBiasSigma_rads;
    int    i;

    if (keepBias && A->Initialized)
        memcpy(b, A->Bias, sizeof(b));
    memset(A->P, 0, sizeof(A->P));
    memcpy(A->Q, q, sizeof(A->Q));
    memcpy(A->Bias, b, sizeof(A->Bias));
    for (i = 0; i < 3; i++)
    {
        A->P[i][i]         = sigma * sigma;
        A->P[i + 3][i + 3] = sb * sb;
    }
    A->Initialized   = true;
    A->ConsecRejects = 0;
}

/*
** Propagate over dt with the bias-corrected gyro, trapezoidal between this
** cycle's and the previous cycle's sample. Error-state transition (first order):
**   Φ = [ I - [θ×]   -I·dt ]      θ = ω·dt
**       [ 0           I    ]
** Process noise: gyro noise on the angle, random walk on the bias. With no
** gyro the attitude is held and its uncertainty grows at NAV_NO_GYRO_RATE_SIGMA.
*/
static void AttPropagate(NAV_AttFilter_t *A, const NAV_CfgTbl_t *Cfg, double dt, bool imu, const float gyro[3])
{
    double Phi[NAV_NA * NAV_NA];
    double qa, qb = (double)Cfg->GyroBiasWalk_rads * Cfg->GyroBiasWalk_rads * dt;
    int    i;

    memset(Phi, 0, sizeof(Phi));
    for (i = 0; i < NAV_NA; i++)
        Phi[i * NAV_NA + i] = 1.0;

    if (imu)
    {
        double w[3], th[3];
        for (i = 0; i < 3; i++)
        {
            double g = A->HavePrevGyro ? 0.5 * ((double)A->PrevGyro[i] + gyro[i]) : gyro[i];
            w[i]     = g - A->Bias[i];
            th[i]    = w[i] * dt;
        }
        QuatApplyRotVec(A->Q, th[0], th[1], th[2]);

        /* I - [θ×] */
        Phi[0 * NAV_NA + 1] = th[2];
        Phi[0 * NAV_NA + 2] = -th[1];
        Phi[1 * NAV_NA + 0] = -th[2];
        Phi[1 * NAV_NA + 2] = th[0];
        Phi[2 * NAV_NA + 0] = th[1];
        Phi[2 * NAV_NA + 1] = -th[0];
        for (i = 0; i < 3; i++)
            Phi[i * NAV_NA + i + 3] = -dt;
        qa = (double)Cfg->GyroNoise_rads * Cfg->GyroNoise_rads * dt * dt;
    }
    else
    {
        qa = NAV_NO_GYRO_RATE_SIGMA * NAV_NO_GYRO_RATE_SIGMA * dt * dt;
    }

    CovPropagate(&A->P[0][0], Phi, NAV_NA);
    for (i = 0; i < 3; i++)
    {
        A->P[i][i] += qa;
        A->P[i + 3][i + 3] += qb;
    }
}

/* One attitude fix (star tracker, or the LIDAR pose turned into a body attitude) */
static bool AttUpdate(NAV_AttFilter_t *A, const NAV_CfgTbl_t *Cfg, const float q_meas[4], double sigma, double *mahal)
{
    float  q_e[4], q_c[4], rv[3];
    double y[3], Rm[9], dx[NAV_NA];
    int    i;

    NAV_QuatConj(A->Q, q_c);
    NAV_QuatMul(q_c, q_meas, q_e);
    QuatToRotVec(q_e, rv);
    for (i = 0; i < 3; i++)
        y[i] = rv[i];
    memset(Rm, 0, sizeof(Rm));
    Rm[0] = Rm[4] = Rm[8] = sigma * sigma;
    memset(dx, 0, sizeof(dx));

    if (!KalmanUpdate3(NAV_NA, dx, &A->P[0][0], y, Rm, Cfg->GateSigma, mahal))
        return false;

    QuatApplyRotVec(A->Q, dx[0], dx[1], dx[2]);
    for (i = 0; i < 3; i++)
        A->Bias[i] += dx[i + 3];
    return true;
}

/* ======================================================================= */
/* Translation filter                                                       */
/* ======================================================================= */

/*
** Propagate over dt: Clohessy-Wiltshire, x' = A x, mapped to world axes as in
** ClohessyWiltshire.cs (+Y radial, -Z along-track, +X cross-track):
**   a_x = -n² r_x              (cross-track)
**   a_y = 3n² r_y - 2n v_z     (radial; v_along = -v_z)
**   a_z = 2n v_y               (-a_along)
** Φ = exp(A dt) by its series to third order (n·dt ~ 2e-4 per cycle, so the
** truncation is far below float precision). The IMU's Δv over the cycle,
** less the estimated bias, is added as an impulse at mid-interval: it moves
** the position by Δv·dt/2. The bias state enters through that same path.
**
** Process noise: unmodelled acceleration (σ_a) and accelerometer noise (σ_dv)
** both enter like that mid-interval impulse, per axis
**   q = σ_a² dt² + σ_dv²,   Q = q [[dt²/4, dt/2], [dt/2, 1]]
** and the bias random-walks.
*/
static void FilterPropagate(NAV_Filter_t *F, const NAV_CfgTbl_t *Cfg, double dt, const float *q_L_B,
                            const float *dv_B)
{
    double n = Cfg->MeanMotion_rads;
    double A[36], M2[36], M3[36], Phi[NAV_NX * NAV_NX], X[NAV_NX], dv_L[3] = {0.0, 0.0, 0.0};
    double q;
    int    i, j, k;

    memset(A, 0, sizeof(A));
    for (i = 0; i < 3; i++)
        A[i * 6 + i + 3] = dt;
    A[3 * 6 + 0] = -n * n * dt;
    A[4 * 6 + 1] = 3.0 * n * n * dt;
    A[4 * 6 + 5] = -2.0 * n * dt;
    A[5 * 6 + 4] = 2.0 * n * dt;
    MatMul(A, A, M2, 6, 6, 6);
    MatMul(M2, A, M3, 6, 6, 6);

    memset(Phi, 0, sizeof(Phi));
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++)
            Phi[i * NAV_NX + j] = (i == j ? 1.0 : 0.0) + A[i * 6 + j] + M2[i * 6 + j] / 2.0 + M3[i * 6 + j] / 6.0;
    for (i = 6; i < NAV_NX; i++)
        Phi[i * NAV_NX + i] = 1.0;

    if (q_L_B != NULL && dv_B != NULL)
    {
        double R[9], dvc[3];
        QuatToDcm(q_L_B, R);
        for (i = 0; i < 3; i++)
            dvc[i] = dv_B[i] - F->X[6 + i] * dt;
        for (i = 0; i < 3; i++)
        {
            dv_L[i] = R[i * 3 + 0] * dvc[0] + R[i * 3 + 1] * dvc[1] + R[i * 3 + 2] * dvc[2];
            for (j = 0; j < 3; j++)
            {
                Phi[i * NAV_NX + 6 + j]       = -R[i * 3 + j] * dt * dt / 2.0;
                Phi[(i + 3) * NAV_NX + 6 + j] = -R[i * 3 + j] * dt;
            }
        }
    }

    /* State: the CW block with the measured impulse; the bias is constant */
    for (i = 0; i < 6; i++)
    {
        X[i] = 0.0;
        for (k = 0; k < 6; k++)
            X[i] += Phi[i * NAV_NX + k] * F->X[k];
    }
    for (i = 0; i < 3; i++)
    {
        X[i] += dv_L[i] * dt / 2.0;
        X[i + 3] += dv_L[i];
        X[i + 6] = F->X[i + 6];
    }
    memcpy(F->X, X, sizeof(X));

    CovPropagate(&F->P[0][0], Phi, NAV_NX);
    q = (double)Cfg->AccelNoise_mss * Cfg->AccelNoise_mss * dt * dt +
        (double)Cfg->DeltaVNoise_ms * Cfg->DeltaVNoise_ms;
    for (i = 0; i < 3; i++)
    {
        F->P[i][i] += q * dt * dt / 4.0;
        F->P[i][i + 3] += q * dt / 2.0;
        F->P[i + 3][i] += q * dt / 2.0;
        F->P[i + 3][i + 3] += q;
        F->P[i + 6][i + 6] += (double)Cfg->AccelBiasWalk_mss * Cfg->AccelBiasWalk_mss * dt;
    }
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
** boresight at close range or with the chaser rotated. Plus the attitude
** uncertainty swinging the whole lever arm |p_B| (isotropic, slightly
** conservative).
*/
static void RpsFix(const NAV_CfgTbl_t *Cfg, const float q_L_B[4], double attSigma, float range, float az, float el,
                   double z[3], double Rm[9])
{
    float  p_S[3], p_B[3], p_L[3], tp_L[3], q_L_S[4];
    double M[9], J[9], G[9], D[3];
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
    J[0] = ce * sa;
    J[3] = se;
    J[6] = ce * ca;
    J[1] = range * ce * ca;
    J[4] = 0.0;
    J[7] = -range * ce * sa;
    J[2] = -range * se * sa;
    J[5] = range * ce;
    J[8] = -range * se * ca;

    NAV_QuatMul(q_L_B, Cfg->RpsQuat_B, q_L_S);
    QuatToDcm(q_L_S, M);
    MatMul(M, J, G, 3, 3, 3);
    D[0]  = s_r * s_r;
    D[1]  = s_a * s_a;
    D[2]  = s_a * s_a;
    s_att = attSigma * Norm3(p_B);
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            Rm[i * 3 + j] = (i == j) ? s_att * s_att : 0.0;
            for (k = 0; k < 3; k++)
                Rm[i * 3 + j] += G[i * 3 + k] * D[k] * G[j * 3 + k];
        }
}

static void FilterInit(NAV_Filter_t *F, const NAV_CfgTbl_t *Cfg, const double z[3], const double Rm[9])
{
    double b[3] = {0.0, 0.0, 0.0};
    double sb   = Cfg->InitAccelBiasSigma_mss;
    int    i, j;

    if (F->Initialized) /* re-initialisation: the accelerometer is the same one */
        memcpy(b, &F->X[6], sizeof(b));
    memset(F, 0, sizeof(*F));
    for (i = 0; i < 3; i++)
    {
        F->X[i]     = z[i];
        F->X[i + 6] = b[i];
        for (j = 0; j < 3; j++)
            F->P[i][j] = Rm[i * 3 + j];
        F->P[i + 3][i + 3] = (double)Cfg->InitVelSigma_ms * Cfg->InitVelSigma_ms;
        F->P[i + 6][i + 6] = sb * sb;
    }
    F->Initialized = true;
}

/* ======================================================================= */
/* One navigation cycle                                                     */
/* ======================================================================= */

void NAV_Step(NAV_State_t *S, const NAV_CfgTbl_t *Cfg, const SIMLINK_SimState_t *In, NAV_Output_t *Out)
{
    NAV_AttFilter_t *A    = &S->Att;
    NAV_Filter_t    *F    = &S->Filter;
    float            dt   = In->CycleDt_s;
    bool             imu  = false;
    bool             st   = false;
    bool             pose = false;
    bool             att  = false;
    float            gyro[3] = {0.0f, 0.0f, 0.0f};
    float            dv_B[3] = {0.0f, 0.0f, 0.0f};
    float            q_st[4], q_pose[4];
    double           attSigma = 0.0;
    int              i;

    memset(Out, 0, sizeof(*Out));

    /* --- 1. Sensor sanity ----------------------------------------------- */
    if (In->SensorValid & SIMLINK_SENSOR_IMU)
    {
        if (Finite(In->GyroRate_B, 3) && Finite(In->DeltaV_B, 3))
        {
            imu = true;
            memcpy(gyro, In->GyroRate_B, sizeof(gyro));
            memcpy(dv_B, In->DeltaV_B, sizeof(dv_B));
            Out->Status |= NAV_STATUS_RATE_VALID | NAV_STATUS_DV_VALID;
        }
        else
        {
            Out->Events |= NAV_EVT_SENSOR_REJECT;
        }
    }
    if (In->SensorValid & SIMLINK_SENSOR_ST)
    {
        memcpy(q_st, In->StQuat, sizeof(q_st));
        st = NAV_QuatNormalize(q_st, 0.01f);
        if (!st)
            Out->Events |= NAV_EVT_SENSOR_REJECT;
    }
    if (In->SensorValid & SIMLINK_SENSOR_RPS_POSE)
    {
        float q_S_TP[4];
        memcpy(q_S_TP, In->RpsQuat, sizeof(q_S_TP));
        if (NAV_QuatNormalize(q_S_TP, 0.01f))
        {
            /* body attitude implied by the pose: q_L_B = q_L_TP ⊗ q_TP_S ⊗ q_S_B */
            float q_L_TP[4], q_tmp[4], q_L_S[4];
            NAV_QuatMul(Cfg->TargetAttQuat_L, Cfg->TargetPortQuat_T, q_L_TP);
            NAV_QuatConj(q_S_TP, q_tmp);
            NAV_QuatMul(q_L_TP, q_tmp, q_L_S);
            NAV_QuatConj(Cfg->RpsQuat_B, q_tmp);
            NAV_QuatMul(q_L_S, q_tmp, q_pose);
            pose = true;
        }
        else
        {
            Out->Events |= NAV_EVT_SENSOR_REJECT;
        }
    }

    /* --- 2. Attitude filter --------------------------------------------- */
    if (A->Initialized)
        AttPropagate(A, Cfg, dt, imu, gyro);

    if (st)
    {
        double mahal = 0.0;
        if (!A->Initialized)
        {
            AttInit(A, Cfg, q_st, Cfg->AttSigma_rad, false);
            Out->Events |= NAV_EVT_ATT_INIT;
            Out->Status |= NAV_STATUS_ST_USED;
        }
        else if (AttUpdate(A, Cfg, q_st, Cfg->AttSigma_rad, &mahal))
        {
            A->ConsecRejects = 0;
            Out->Status |= NAV_STATUS_ST_USED;
        }
        else
        {
            Out->Status |= NAV_STATUS_ATT_REJECTED;
            if (++A->ConsecRejects >= Cfg->MaxConsecRejects)
            {
                AttInit(A, Cfg, q_st, Cfg->AttSigma_rad, true);
                Out->Events |= NAV_EVT_ATT_REINIT;
                Out->Status |= NAV_STATUS_ST_USED;
            }
        }
        if (mahal > Out->AttInnovSigma)
            Out->AttInnovSigma = isfinite(mahal) ? (float)mahal : 1e30f;
    }
    if (pose)
    {
        double mahal = 0.0;
        if (!A->Initialized)
        {
            AttInit(A, Cfg, q_pose, Cfg->RpsPoseSigma_rad, false);
            Out->Events |= NAV_EVT_ATT_INIT;
            Out->Status |= NAV_STATUS_POSE_USED;
        }
        else if (AttUpdate(A, Cfg, q_pose, Cfg->RpsPoseSigma_rad, &mahal))
        {
            Out->Status |= NAV_STATUS_POSE_USED;
        }
        else
        {
            Out->Status |= NAV_STATUS_ATT_REJECTED;
        }
        if (mahal > Out->AttInnovSigma)
            Out->AttInnovSigma = isfinite(mahal) ? (float)mahal : 1e30f;
    }

    if (imu)
        memcpy(A->PrevGyro, gyro, sizeof(gyro));
    A->HavePrevGyro = imu;

    if (A->Initialized && (!Finite(A->Q, 4) || !AllFinite(A->Bias, 3) || !AllFinite(&A->P[0][0], NAV_NA * NAV_NA)))
    {
        memset(A, 0, sizeof(*A));
        Out->Events |= NAV_EVT_ATT_REINIT;
    }
    if (A->Initialized)
    {
        attSigma = MaxDiagSigma(&A->P[0][0], NAV_NA, 0, 3);
        att      = attSigma <= Cfg->MaxAttSigma_rad;
        Out->AttSigma_rad = (float)attSigma;
        for (i = 0; i < 3; i++)
            Out->GyroBias_rads[i] = (float)A->Bias[i];
    }
    if (att)
        Out->Status |= NAV_STATUS_ATT_VALID;

    /* --- 3. Translation filter: propagate -------------------------------- */
    if (F->Initialized)
        FilterPropagate(F, Cfg, dt, att ? A->Q : NULL, imu ? dv_B : NULL);

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
            double z[3], Rm[9], y[3], mahal = 0.0;

            Out->RpsAvailable = true;
            RpsFix(Cfg, A->Q, attSigma, In->RpsRange_m, In->RpsAz_rad, In->RpsEl_rad, z, Rm);

            if (!F->Initialized)
            {
                FilterInit(F, Cfg, z, Rm);
                Out->Events |= NAV_EVT_FILTER_INIT;
                Out->Status |= NAV_STATUS_RPS_USED;
            }
            else
            {
                for (i = 0; i < 3; i++)
                    y[i] = z[i] - F->X[i];
                if (KalmanUpdate3(NAV_NX, F->X, &F->P[0][0], y, Rm, Cfg->GateSigma, &mahal))
                {
                    F->ConsecRejects = 0;
                    Out->Status |= NAV_STATUS_RPS_USED;
                }
                else
                {
                    Out->Status |= NAV_STATUS_RPS_REJECTED;
                    if (++F->ConsecRejects >= Cfg->MaxConsecRejects)
                    {
                        /* The filter and the sensor disagree persistently: trust the
                        ** sensor (scenario reset, missed thrust, a bad propagation). */
                        FilterInit(F, Cfg, z, Rm);
                        Out->Events |= NAV_EVT_FILTER_REINIT;
                        Out->Status |= NAV_STATUS_RPS_USED;
                    }
                }
            }
            Out->InnovSigma = isfinite(mahal) ? (float)mahal : 1e30f;
        }
    }

    if (F->Initialized && (!AllFinite(F->X, NAV_NX) || !AllFinite(&F->P[0][0], NAV_NX * NAV_NX)))
    {
        NAV_ResetFilter(S);
        Out->Events |= NAV_EVT_FILTER_REINIT;
    }

    /* --- 5. Outputs ------------------------------------------------------ */
    Out->ConsecRpsRejects = F->ConsecRejects;
    for (i = 0; i < 3; i++)
    {
        Out->AngRate_B[i] = imu ? (float)(gyro[i] - (A->Initialized ? A->Bias[i] : 0.0)) : 0.0f;
        Out->DeltaV_B[i]  = imu ? (float)(dv_B[i] - (F->Initialized ? F->X[6 + i] * dt : 0.0)) : 0.0f;
        if (F->Initialized)
            Out->AccelBias_mss[i] = (float)F->X[6 + i];
    }

    if (att)
    {
        float q_L_D[4], q_L_Bdes[4], q_tmp[4], q_e[4];

        memcpy(Out->AttQuat_L, A->Q, sizeof(A->Q));

        /* Docking frame and docking attitude from the table geometry */
        NAV_QuatMul(Cfg->TargetAttQuat_L, Cfg->TargetPortQuat_T, q_tmp);
        NAV_QuatMul(q_tmp, Cfg->DockedQuat_TP, q_L_D);
        NAV_QuatConj(Cfg->ChaserPortQuat_B, q_tmp);
        NAV_QuatMul(q_L_D, q_tmp, q_L_Bdes);

        /* Attitude error: q_e = conj(q_L_B) ⊗ q_L_Bdes, as a body-axis rotation vector */
        NAV_QuatConj(A->Q, q_tmp);
        NAV_QuatMul(q_tmp, q_L_Bdes, q_e);
        QuatToRotVec(q_e, Out->AttErr_B);

        if (F->Initialized)
        {
            float r_port_L[3], wxr_B[3], wxr_L[3], tp_L[3], pos_L[3], vel_L[3], q_D_L[4];

            NAV_QuatRotate(A->Q, Cfg->ChaserPortPos_B, r_port_L);
            Cross(Out->AngRate_B, Cfg->ChaserPortPos_B, wxr_B);
            NAV_QuatRotate(A->Q, wxr_B, wxr_L);
            NAV_QuatRotate(Cfg->TargetAttQuat_L, Cfg->TargetPortPos_T, tp_L);

            for (i = 0; i < 3; i++)
            {
                Out->CmPos_L[i] = (float)F->X[i];
                Out->CmVel_L[i] = (float)F->X[i + 3];
                pos_L[i]        = (float)F->X[i] + r_port_L[i] - tp_L[i];
                vel_L[i]        = (float)F->X[i + 3] + wxr_L[i];
            }
            NAV_QuatConj(q_L_D, q_D_L);
            NAV_QuatRotate(q_D_L, pos_L, Out->PortPos_D);
            NAV_QuatRotate(q_D_L, vel_L, Out->PortVel_D);

            Out->Range_m         = Norm3(Out->PortPos_D);
            Out->ClosingSpeed_ms = Out->PortVel_D[2];
            Out->LateralOffset_m = sqrtf(Out->PortPos_D[0] * Out->PortPos_D[0] + Out->PortPos_D[1] * Out->PortPos_D[1]);
            Out->PosSigma_m      = (float)MaxDiagSigma(&F->P[0][0], NAV_NX, 0, 3);
            Out->VelSigma_ms     = (float)MaxDiagSigma(&F->P[0][0], NAV_NX, 3, 3);

            Out->Status |= NAV_STATUS_FILTER_INIT;
            if (imu && Out->PosSigma_m <= Cfg->MaxPosSigma_m && Out->VelSigma_ms <= Cfg->MaxVelSigma_ms)
                Out->Status |= NAV_STATUS_RELNAV_VALID;
        }
    }
    else if (F->Initialized)
    {
        Out->Status |= NAV_STATUS_FILTER_INIT;
    }
}
