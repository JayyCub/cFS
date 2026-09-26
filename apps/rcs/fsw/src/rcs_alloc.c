/*
** RCS allocation + PWM.
**
** Allocation is a non-negative least-squares (NNLS, Lawson-Hanson active set)
** solve of  B·J = y,  J >= 0  where
**   y      = requested [linear impulse (N·s); angular impulse (N·m·s)], body frame
**   B[:,i] = [d_i; r_i × d_i]  (unit-thrust effectiveness of thruster i)
**   J_i    = impulse thruster i must deliver (N·s)
** Thrusters are push-only, so J >= 0 is a hard constraint; solving with it (rather
** than solving unconstrained and clipping negatives) keeps the torque/force
** cancellation between thrusters exact, and when the request is outside what
** this thruster set can produce it returns the closest achievable impulse.
**
** Ported from Unity's former ThrusterAllocator.cs (same algorithm, same 1e-3
** Tikhonov term, same iteration caps) so behaviour carries over unchanged; only
** its home moved from the "physical world" into flight software.
**
** PWM: J_i / Thrust_N = valve on-time. If any on-time exceeds MaxOnTime_s the
** WHOLE solution is scaled down (direction-preserving — better a smaller correct
** impulse than a clipped, skewed one). Pulses shorter than MinOnTime_s can't be
** produced by the valve: >= half the minimum rounds up to it, less is dropped.
*/
#include <math.h>
#include <string.h>
#include "rcs.h"

#define RCS_NNLS_REG      1.0e-3 /* Tikhonov term on B·Bᵀ (unit-thrust columns, O(1-10) entries) */
#define RCS_NNLS_ADD_TOL  1.0e-6 /* don't activate a thruster whose improvement is only noise    */
#define RCS_ZERO_REQ      1.0e-9

/* Column i of B (unit thrust) */
static void RCS_Column(const RCS_ThrusterDef_t *T, double c[6])
{
    const float *r = T->Pos_m;
    const float *d = T->Dir;
    c[0] = d[0];
    c[1] = d[1];
    c[2] = d[2];
    /* r × d — same formula as Unity's Vector3.Cross, which is what the vehicle's
    ** torque sign convention was validated against. */
    c[3] = (double)r[1] * d[2] - (double)r[2] * d[1];
    c[4] = (double)r[2] * d[0] - (double)r[0] * d[2];
    c[5] = (double)r[0] * d[1] - (double)r[1] * d[0];
}

/* Solve the 6x6 system A·a = b in place (Gaussian elimination, partial pivot). */
static bool RCS_Solve6(double A[6][6], double b[6], double a[6])
{
    int i, j, k;
    for (k = 0; k < 6; k++)
    {
        int    p    = k;
        double best = fabs(A[k][k]);
        for (i = k + 1; i < 6; i++)
            if (fabs(A[i][k]) > best)
            {
                best = fabs(A[i][k]);
                p    = i;
            }
        if (best < 1e-12)
            return false;
        if (p != k)
        {
            for (j = 0; j < 6; j++)
            {
                double t = A[k][j];
                A[k][j]  = A[p][j];
                A[p][j]  = t;
            }
            double t = b[k];
            b[k]     = b[p];
            b[p]     = t;
        }
        for (i = k + 1; i < 6; i++)
        {
            double f = A[i][k] / A[k][k];
            for (j = k; j < 6; j++)
                A[i][j] -= f * A[k][j];
            b[i] -= f * b[k];
        }
    }
    for (i = 5; i >= 0; i--)
    {
        double s = b[i];
        for (j = i + 1; j < 6; j++)
            s -= A[i][j] * a[j];
        a[i] = s / A[i][i];
    }
    return true;
}

/*
** Regularized minimum-norm least squares over the passive set P:
**   z_P = B_Pᵀ (B_P B_Pᵀ + λI)⁻¹ y,   z_i = 0 for i ∉ P
** Well-defined even when |P| < 6 (under-determined) thanks to λ.
*/
static bool RCS_SolvePassive(double C[][6], const bool *P, uint32 n, const double y[6], double *z)
{
    double A[6][6] = {{0}};
    double b[6];
    double a[6] = {0};
    uint32 i;
    int    r, c;

    for (i = 0; i < n; i++)
    {
        if (!P[i])
            continue;
        for (r = 0; r < 6; r++)
            for (c = 0; c < 6; c++)
                A[r][c] += C[i][r] * C[i][c];
    }
    for (r = 0; r < 6; r++)
    {
        A[r][r] += RCS_NNLS_REG;
        b[r] = y[r];
    }
    if (!RCS_Solve6(A, b, a))
        return false;

    for (i = 0; i < n; i++)
    {
        z[i] = 0.0;
        if (P[i])
            for (r = 0; r < 6; r++)
                z[i] += C[i][r] * a[r];
    }
    return true;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void RCS_Nnls(double C[][6], const bool *Usable, uint32 n, const double y[6], double *x)
{
    bool   P[RCS_MAX_THRUSTERS] = {false};
    double z[RCS_MAX_THRUSTERS];
    uint32 i, outer, inner;
    int    k;

    for (i = 0; i < n; i++)
        x[i] = 0.0;

    for (outer = 0; outer < n; outer++)
    {
        /* Residual of the current solution; bring in the excluded thruster whose
        ** effectiveness column best reduces it (Lawson-Hanson's gradient step). */
        double res[6];
        for (k = 0; k < 6; k++)
            res[k] = y[k];
        for (i = 0; i < n; i++)
            for (k = 0; k < 6; k++)
                res[k] -= C[i][k] * x[i];

        int    best      = -1;
        double bestScore = RCS_NNLS_ADD_TOL;
        for (i = 0; i < n; i++)
        {
            if (!Usable[i] || P[i])
                continue;
            double score = 0.0;
            for (k = 0; k < 6; k++)
                score += C[i][k] * res[k];
            if (score > bestScore)
            {
                bestScore = score;
                best      = (int)i;
            }
        }
        if (best < 0)
            break; /* converged — nothing excluded would help */
        P[best] = true;

        /* Inner loop: solve over P; if any active thruster wants to go negative,
        ** step only as far as the first one reaching zero, drop it, re-solve. */
        for (inner = 0; inner < 2 * n; inner++)
        {
            if (!RCS_SolvePassive(C, P, n, y, z))
            {
                P[best] = false;
                return;
            }

            double minRatio = 1.0;
            int    drop     = -1;
            for (i = 0; i < n; i++)
            {
                if (!P[i] || z[i] >= 0.0)
                    continue;
                double denom = x[i] - z[i];
                if (denom <= 1e-12)
                    continue;
                double ratio = x[i] / denom;
                if (ratio < minRatio)
                {
                    minRatio = ratio;
                    drop     = (int)i;
                }
            }

            if (drop < 0)
            {
                for (i = 0; i < n; i++)
                    x[i] = P[i] ? z[i] : 0.0;
                break;
            }

            for (i = 0; i < n; i++)
                if (P[i])
                    x[i] += minRatio * (z[i] - x[i]);
            P[drop] = false;
            x[drop] = 0.0;
        }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void RCS_Allocate(const RCS_ThrTbl_t *Tbl, const float Lin[3], const float Ang[3], RCS_AllocResult_t *Out)
{
    double C[RCS_MAX_THRUSTERS][6];
    bool   Usable[RCS_MAX_THRUSTERS];
    double J[RCS_MAX_THRUSTERS];
    double y[6] = {Lin[0], Lin[1], Lin[2], Ang[0], Ang[1], Ang[2]};
    uint32 n    = Tbl->NumThrusters;
    uint32 i;
    int    k;
    double ynorm = 0.0;

    memset(Out, 0, sizeof(*Out));

    for (k = 0; k < 6; k++)
        ynorm += y[k] * y[k];
    if (ynorm < RCS_ZERO_REQ)
        return; /* coast */

    for (i = 0; i < n; i++)
    {
        RCS_Column(&Tbl->Thruster[i], C[i]);
        Usable[i] = Tbl->Thruster[i].Enabled != 0;
    }

    RCS_Nnls(C, Usable, n, y, J);

    /* Impulse -> on-time at rated thrust */
    double maxT = 0.0;
    double t[RCS_MAX_THRUSTERS];
    for (i = 0; i < n; i++)
    {
        t[i] = (J[i] > 0.0) ? J[i] / Tbl->Thrust_N : 0.0;
        if (t[i] > maxT)
            maxT = t[i];
    }

    /* Saturation: scale the whole solution so the longest pulse fits the cycle */
    if (maxT > Tbl->MaxOnTime_s)
    {
        double s = Tbl->MaxOnTime_s / maxT;
        for (i = 0; i < n; i++)
            t[i] *= s;
        Out->Saturated = true;
    }

    /* Minimum impulse bit */
    for (i = 0; i < n; i++)
    {
        if (t[i] <= 0.0 || t[i] >= Tbl->MinOnTime_s)
            continue;
        if (t[i] >= 0.5 * Tbl->MinOnTime_s)
        {
            t[i] = Tbl->MinOnTime_s;
            Out->Rounds++;
        }
        else
        {
            t[i] = 0.0;
            Out->Drops++;
        }
    }

    /* What these on-times will actually deliver, per the table */
    double ach[6] = {0};
    for (i = 0; i < n; i++)
    {
        Out->OnTime_s[i] = (float)t[i];
        if (t[i] <= 0.0)
            continue;
        Out->FiredMask |= (uint16)(1u << i);
        if (t[i] > Out->MaxOnTime_s)
            Out->MaxOnTime_s = (float)t[i];
        for (k = 0; k < 6; k++)
            ach[k] += C[i][k] * t[i] * Tbl->Thrust_N;
    }

    double res2 = 0.0;
    for (k = 0; k < 3; k++)
    {
        Out->AchievedLin_Ns[k]  = (float)ach[k];
        Out->AchievedAng_Nms[k] = (float)ach[k + 3];
    }
    for (k = 0; k < 6; k++)
        res2 += (y[k] - ach[k]) * (y[k] - ach[k]);
    Out->ResidualNorm = (float)sqrt(res2);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Table validation — rejects an image the allocator can't use safely.      */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 RCS_ValidateThrTbl(void *TblPtr)
{
    const RCS_ThrTbl_t *t = (const RCS_ThrTbl_t *)TblPtr;
    uint32              i;
    uint32              enabled = 0;

    if (t->NumThrusters == 0 || t->NumThrusters > RCS_MAX_THRUSTERS)
    {
        CFE_EVS_SendEvent(RCS_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR, "RCS: table rejected — NumThrusters %lu",
                          (unsigned long)t->NumThrusters);
        return RCS_TBL_VALIDATE_ERR;
    }
    if (!(t->Thrust_N > 0.0f) || !(t->MinOnTime_s > 0.0f) || !(t->MaxOnTime_s >= t->MinOnTime_s))
    {
        CFE_EVS_SendEvent(RCS_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "RCS: table rejected — Thrust %g, MinOnTime %g, MaxOnTime %g", (double)t->Thrust_N,
                          (double)t->MinOnTime_s, (double)t->MaxOnTime_s);
        return RCS_TBL_VALIDATE_ERR;
    }
    for (i = 0; i < t->NumThrusters; i++)
    {
        const float *d = t->Thruster[i].Dir;
        float        m = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (!t->Thruster[i].Enabled)
            continue;
        enabled++;
        if (fabsf(m - 1.0f) > 0.01f)
        {
            CFE_EVS_SendEvent(RCS_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                              "RCS: table rejected — T%02lu direction not unit length (%g)", (unsigned long)i,
                              (double)m);
            return RCS_TBL_VALIDATE_ERR;
        }
    }
    if (enabled < 6)
    {
        CFE_EVS_SendEvent(RCS_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "RCS: table rejected — only %lu enabled thrusters, need >= 6 for 6-DOF",
                          (unsigned long)enabled);
        return RCS_TBL_VALIDATE_ERR;
    }
    return CFE_SUCCESS;
}
