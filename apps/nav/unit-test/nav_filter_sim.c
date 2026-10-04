/*
** Stand-alone closed-loop check of the NAV estimation core (nav_filter.c)
** against a truth model that behaves like the Unity sim: 20 ms physics steps
** with semi-implicit Euler, CW gravity, thruster pulses at the start of each
** 0.2 s cycle, and sensors generated the way ChaserSensors.cs does it.
**
** Build and run (cfs-dev container, from the cFS root):
**   gcc -std=c99 -O2 -Wall -Wextra -o /tmp/nav_sim \
**       -Iapps/nav/unit-test/stubs -Iapps/nav/fsw/inc -Iapps/nav/fsw/src \
**       -Iapps/sim_io/fsw/inc -Iosal/src/os/inc \
**       apps/nav/unit-test/nav_filter_sim.c apps/nav/fsw/src/nav_filter.c -lm
**   /tmp/nav_sim
**
** Exit status 0 = every scenario met its accuracy limits.
*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nav_filter.h"
#include "../fsw/tables/nav_cfg_tbl.c" /* the real default table */

#define PHYS_DT  0.02
#define STEPS    10 /* physics steps per 0.2 s cycle */
#define MASS     12000.0
#define THRUST_N 400.0

/* ---- sensor "hardware" spec (ChaserSensors.cs defaults) --------------- */
static const double GYRO_SIGMA  = 1e-5;   /* rad/s           */
static const double DV_SIGMA    = 1e-5;   /* m/s per cycle   */
static const double ST_SIGMA    = 5e-5;   /* rad             */
static const double RNG_SIGMA0  = 0.005;  /* m               */
static const double RNG_FRAC    = 0.001;
static const double ANG_SIGMA   = 5e-4;   /* rad             */
static const double RPS_MIN_RNG = 0.3, RPS_MAX_RNG = 250.0, RPS_HALF_FOV = 35.0 * 3.14159265358979 / 180.0;
static const double POSE_SIGMA  = 3e-3;   /* rad             */
static const double POSE_MAX_RNG = 30.0;  /* m               */
static const double GYRO_BIAS[3]  = {1.5e-6, -1.0e-6, 2.0e-6}; /* rad/s */
static const double ACCEL_BIAS[3] = {2.0e-5, -1.5e-5, 3.0e-5}; /* m/s²  */

static unsigned long long Rng = 12345;
static double Uniform(void)
{
    Rng = Rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return ((Rng >> 11) + 0.5) / 9007199254740992.0;
}
static double Gauss(void)
{
    return sqrt(-2.0 * log(Uniform())) * cos(2.0 * 3.14159265358979 * Uniform());
}

typedef struct
{
    double r[3], v[3]; /* CoM rel. ISS reference, world (m, m/s) */
    float  q[4];       /* q_L_B                                    */
    double w[3];       /* body rate (rad/s)                         */
} Truth_t;

static void Rot(const float q[4], const double v[3], double out[3])
{
    float vf[3] = {(float)v[0], (float)v[1], (float)v[2]}, o[3];
    NAV_QuatRotate(q, vf, o);
    out[0] = o[0];
    out[1] = o[1];
    out[2] = o[2];
}

static void SmallRot(float q[4], double ax, double ay, double az) /* q ← q ⊗ δq(a) */
{
    double th = sqrt(ax * ax + ay * ay + az * az);
    float  dq[4] = {0, 0, 0, 1};
    if (th > 0)
    {
        double s = sin(th / 2) / th;
        dq[0]    = ax * s;
        dq[1]    = ay * s;
        dq[2]    = az * s;
        dq[3]    = cos(th / 2);
    }
    NAV_QuatMul(q, dq, q);
    NAV_QuatNormalize(q, 0.5f);
}

/* Truth docking-frame quantities, computed the way RelativeNav does from transforms */
static void TruthPort(const Truth_t *T, double pos_D[3], double vel_D[3], double atterr[3])
{
    float  q_tmp[4], q_L_D[4], q_D_L[4], q_des[4], q_e[4];
    double port_L[3], tp_L[3], wxr_B[3], wxr_L[3], p[3], v[3];
    double pB[3] = {NAV_CfgTbl.ChaserPortPos_B[0], NAV_CfgTbl.ChaserPortPos_B[1], NAV_CfgTbl.ChaserPortPos_B[2]};
    double tT[3] = {NAV_CfgTbl.TargetPortPos_T[0], NAV_CfgTbl.TargetPortPos_T[1], NAV_CfgTbl.TargetPortPos_T[2]};
    int    i;

    NAV_QuatMul(NAV_CfgTbl.TargetAttQuat_L, NAV_CfgTbl.TargetPortQuat_T, q_tmp);
    NAV_QuatMul(q_tmp, NAV_CfgTbl.DockedQuat_TP, q_L_D);
    NAV_QuatConj(q_L_D, q_D_L);

    Rot(T->q, pB, port_L);
    Rot(NAV_CfgTbl.TargetAttQuat_L, tT, tp_L);
    wxr_B[0] = T->w[1] * pB[2] - T->w[2] * pB[1];
    wxr_B[1] = T->w[2] * pB[0] - T->w[0] * pB[2];
    wxr_B[2] = T->w[0] * pB[1] - T->w[1] * pB[0];
    Rot(T->q, wxr_B, wxr_L);
    for (i = 0; i < 3; i++)
    {
        p[i] = T->r[i] + port_L[i] - tp_L[i];
        v[i] = T->v[i] + wxr_L[i];
    }
    Rot(q_D_L, p, pos_D);
    Rot(q_D_L, v, vel_D);

    NAV_QuatConj(NAV_CfgTbl.ChaserPortQuat_B, q_tmp);
    NAV_QuatMul(q_L_D, q_tmp, q_des);
    NAV_QuatConj(T->q, q_tmp);
    NAV_QuatMul(q_tmp, q_des, q_e);
    {
        double s = sqrt((double)q_e[0] * q_e[0] + (double)q_e[1] * q_e[1] + (double)q_e[2] * q_e[2]);
        double k = (s < 1e-12) ? 2.0 : 2.0 * atan2(s, fabs(q_e[3])) / s;
        if (q_e[3] < 0)
            k = -k;
        for (i = 0; i < 3; i++)
            atterr[i] = k * q_e[i];
    }
}

/* Sensors at a cycle boundary (dv_B accumulated over the cycle by the caller) */
static void Sense(const Truth_t *T, const double dv_B[3], bool rps_enabled, bool st_enabled, SIMLINK_SimState_t *s)
{
    double pB[3] = {NAV_CfgTbl.RpsPos_B[0], NAV_CfgTbl.RpsPos_B[1], NAV_CfgTbl.RpsPos_B[2]};
    double tT[3] = {NAV_CfgTbl.TargetPortPos_T[0], NAV_CfgTbl.TargetPortPos_T[1], NAV_CfgTbl.TargetPortPos_T[2]};
    double sens_L[3], tp_L[3], d[3], p_S[3];
    float  q_L_S[4], q_S_L[4];
    int    i;

    memset(s, 0, sizeof(*s));
    s->CycleDt_s   = (float)(PHYS_DT * STEPS);
    s->SensorValid = SIMLINK_SENSOR_IMU | (st_enabled ? SIMLINK_SENSOR_ST : 0u);
    for (i = 0; i < 3; i++)
    {
        s->GyroRate_B[i] = (float)(T->w[i] + GYRO_BIAS[i] + GYRO_SIGMA * Gauss());
        s->DeltaV_B[i]   = (float)(dv_B[i] + ACCEL_BIAS[i] * PHYS_DT * STEPS + DV_SIGMA * Gauss());
    }
    memcpy(s->StQuat, T->q, sizeof(s->StQuat));
    SmallRot(s->StQuat, ST_SIGMA * Gauss(), ST_SIGMA * Gauss(), ST_SIGMA * Gauss());

    Rot(T->q, pB, sens_L);
    Rot(NAV_CfgTbl.TargetAttQuat_L, tT, tp_L);
    for (i = 0; i < 3; i++)
        d[i] = tp_L[i] - (T->r[i] + sens_L[i]);
    NAV_QuatMul(T->q, NAV_CfgTbl.RpsQuat_B, q_L_S);
    NAV_QuatConj(q_L_S, q_S_L);
    Rot(q_S_L, d, p_S);
    {
        double rng = sqrt(p_S[0] * p_S[0] + p_S[1] * p_S[1] + p_S[2] * p_S[2]);
        double off = acos(p_S[2] / rng);
        if (rps_enabled && rng >= RPS_MIN_RNG && rng <= RPS_MAX_RNG && off <= RPS_HALF_FOV)
        {
            s->SensorValid |= SIMLINK_SENSOR_RPS;
            s->RpsRange_m = (float)(rng + (RNG_SIGMA0 + RNG_FRAC * rng) * Gauss());
            s->RpsAz_rad  = (float)(atan2(p_S[0], p_S[2]) + ANG_SIGMA * Gauss());
            s->RpsEl_rad  = (float)(asin(p_S[1] / rng) + ANG_SIGMA * Gauss());
            if (rng <= POSE_MAX_RNG)
            {
                /* pose: q_S_TP = conj(q_L_S) ⊗ q_L_TP, with orientation noise */
                float q_L_TP[4];
                NAV_QuatMul(NAV_CfgTbl.TargetAttQuat_L, NAV_CfgTbl.TargetPortQuat_T, q_L_TP);
                NAV_QuatMul(q_S_L, q_L_TP, s->RpsQuat);
                SmallRot(s->RpsQuat, POSE_SIGMA * Gauss(), POSE_SIGMA * Gauss(), POSE_SIGMA * Gauss());
                s->SensorValid |= SIMLINK_SENSOR_RPS_POSE;
            }
        }
    }
}

/* One 0.2 s cycle of truth: thrust (world accel) for burn_s, CW gravity throughout */
static void Physics(Truth_t *T, const double a_th[3], double burn_s, double dv_B[3])
{
    const double n = NAV_CfgTbl.MeanMotion_rads;
    int          k, i;

    dv_B[0] = dv_B[1] = dv_B[2] = 0.0;
    for (k = 0; k < STEPS; k++)
    {
        double frac = burn_s / PHYS_DT - k; /* fraction of this step the valves are open */
        double a[3], dv_L[3], dvb[3];
        float  q_B_L[4];
        if (frac > 1)
            frac = 1;
        if (frac < 0)
            frac = 0;
        a[0] = -n * n * T->r[0];
        a[1] = 3 * n * n * T->r[1] - 2 * n * T->v[2];
        a[2] = 2 * n * T->v[1];
        for (i = 0; i < 3; i++)
        {
            dv_L[i] = a_th[i] * frac * PHYS_DT;
            T->v[i] += a[i] * PHYS_DT + dv_L[i];
            T->r[i] += T->v[i] * PHYS_DT;
        }
        NAV_QuatConj(T->q, q_B_L);
        Rot(q_B_L, dv_L, dvb);
        for (i = 0; i < 3; i++)
            dv_B[i] += dvb[i];
        SmallRot(T->q, T->w[0] * PHYS_DT, T->w[1] * PHYS_DT, T->w[2] * PHYS_DT);
    }
}

typedef struct
{
    const char *name;
    double      r0[3], v0[3], w0[3], att0[3];
    int         cycles;
    double      close_speed; /* truth controller target closing speed (m/s) */
    int         dropout_from, dropout_to; /* LIDAR blanked over [from, to) */
    int         jump_at;     /* teleport (scenario reset) at this cycle, -1 = none */
    double      lim_pos, lim_vel, lim_att; /* RMS limits after convergence */
    int         st_from, st_to; /* star tracker blanked over [from, to) */
} Scenario_t;

static int Run(const Scenario_t *sc)
{
    Truth_t            T;
    NAV_State_t        S;
    NAV_Output_t       O;
    SIMLINK_SimState_t s;
    double             dv_B[3] = {0, 0, 0};
    double se_p[3] = {0}, se_v[3] = {0}, se_a = 0, max_vlat = 0, max_att_outage = 0;
    long               nstat = 0, n3sig = 0, nvalid_first = -1, inits = 0, reinits = 0, rejects = 0;
    int                c, i;

    memset(&T, 0, sizeof(T));
    memcpy(T.r, sc->r0, sizeof(T.r));
    memcpy(T.v, sc->v0, sizeof(T.v));
    memcpy(T.w, sc->w0, sizeof(T.w));
    T.q[3] = 1;
    SmallRot(T.q, sc->att0[0], sc->att0[1], sc->att0[2]);
    NAV_Reset(&S);

    for (c = 0; c < sc->cycles; c++)
    {
        double pos_D[3], vel_D[3], atterr[3], a_th[3] = {0, 0, 0}, burn = 0;

        if (c == sc->jump_at)
        {
            T.r[2] -= 15.0; /* scenario reset: chaser teleported 15 m back, at rest */
            T.v[0] = T.v[1] = T.v[2] = 0;
        }

        Sense(&T, dv_B, !(c >= sc->dropout_from && c < sc->dropout_to), !(c >= sc->st_from && c < sc->st_to), &s);
        NAV_Step(&S, &NAV_CfgTbl, &s, &O);
        TruthPort(&T, pos_D, vel_D, atterr);
        if (c >= sc->st_from && c < sc->st_to)
            for (i = 0; i < 3; i++)
                if (fabs(O.AttErr_B[i] - atterr[i]) > max_att_outage)
                    max_att_outage = fabs(O.AttErr_B[i] - atterr[i]);

        if (O.Events & NAV_EVT_FILTER_INIT)
            inits++;
        if (O.Events & NAV_EVT_FILTER_REINIT)
            reinits++;
        if (O.Status & NAV_STATUS_RPS_REJECTED)
        {
            rejects++;
            if (getenv("NAV_DEBUG"))
                printf("  %s: reject at cycle %d, range %.3f m, innovation %.1f sigma\n", sc->name, c, O.Range_m,
                       O.InnovSigma);
        }
        if ((O.Status & NAV_STATUS_RELNAV_VALID) && nvalid_first < 0)
            nvalid_first = c;

        /* statistics once converged (skip 50 cycles after any (re)start) */
        if ((O.Status & NAV_STATUS_RELNAV_VALID) && c > nvalid_first + 50 &&
            (sc->jump_at < 0 || c < sc->jump_at || c > sc->jump_at + 80))
        {
            double ep2 = 0;
            for (i = 0; i < 3; i++)
            {
                double ep = O.PortPos_D[i] - pos_D[i], ev = O.PortVel_D[i] - vel_D[i], ea = O.AttErr_B[i] - atterr[i];
                se_p[i] += ep * ep;
                se_v[i] += ev * ev;
                se_a += ea * ea;
                ep2 += ep * ep;
            }
            if (fabs(O.PortVel_D[0] - vel_D[0]) > max_vlat)
                max_vlat = fabs(O.PortVel_D[0] - vel_D[0]);
            if (fabs(O.PortVel_D[1] - vel_D[1]) > max_vlat)
                max_vlat = fabs(O.PortVel_D[1] - vel_D[1]);
            if (sqrt(ep2) <= 3.0 * sqrt(3.0) * O.PosSigma_m)
                n3sig++;
            nstat++;
        }

        /* truth controller (flies on truth so NAV errors don't feed back):
        ** hold lateral, close at close_speed, damp rates */
        {
            double dv_D[3], dv_L[3];
            float  q_tmp[4], q_L_D[4];
            dv_D[0] = -0.02 * pos_D[0] - vel_D[0];
            dv_D[1] = -0.02 * pos_D[1] - vel_D[1];
            dv_D[2] = (pos_D[2] < -1.0 ? sc->close_speed : pos_D[2] < -0.1 ? 0.03 : 0.0) - vel_D[2];
            for (i = 0; i < 3; i++)
                if (fabs(dv_D[i]) < 0.0007) /* ~min impulse bit */
                    dv_D[i] = 0;
            NAV_QuatMul(NAV_CfgTbl.TargetAttQuat_L, NAV_CfgTbl.TargetPortQuat_T, q_tmp);
            NAV_QuatMul(q_tmp, NAV_CfgTbl.DockedQuat_TP, q_L_D);
            Rot(q_L_D, dv_D, dv_L);
            {
                double mag = sqrt(dv_L[0] * dv_L[0] + dv_L[1] * dv_L[1] + dv_L[2] * dv_L[2]);
                burn = mag * MASS / (4 * THRUST_N); /* 4 thrusters' worth of authority */
                if (burn > 0.19)
                    burn = 0.19;
                if (burn > 0)
                    for (i = 0; i < 3; i++)
                        a_th[i] = dv_L[i] / mag * 4 * THRUST_N / MASS;
            }
        }
        Physics(&T, a_th, burn, dv_B);
    }

    {
        double rp = sqrt((se_p[0] + se_p[1] + se_p[2]) / (3.0 * (nstat ? nstat : 1)));
        double rv = sqrt((se_v[0] + se_v[1]) / (2.0 * (nstat ? nstat : 1)));
        double rz = sqrt(se_v[2] / (nstat ? nstat : 1));
        double ra = sqrt(se_a / (3.0 * (nstat ? nstat : 1)));
        double eg = 0, ea2 = 0;
        int    ok = nstat > 0 && rp <= sc->lim_pos && rv <= sc->lim_vel && ra <= sc->lim_att;
        for (i = 0; i < 3; i++)
        {
            if (fabs(O.GyroBias_rads[i] - GYRO_BIAS[i]) > eg)
                eg = fabs(O.GyroBias_rads[i] - GYRO_BIAS[i]);
            if (fabs(O.AccelBias_mss[i] - ACCEL_BIAS[i]) > ea2)
                ea2 = fabs(O.AccelBias_mss[i] - ACCEL_BIAS[i]);
        }
        if (sc->jump_at >= 0 && reinits < 1)
            ok = 0;
        if (eg > 7.5e-7 || ea2 > 1.5e-5) /* bias estimates: within half the injected bias */
            ok = 0;
        if (sc->st_from >= 0 && max_att_outage > 1e-3)
            ok = 0;
        printf("%-28s valid@%4ld  inits %ld reinits %ld rejects %3ld | RMS pos %.4f m  lat-vel %.5f m/s (max %.5f)"
               "  ax-vel %.5f  att %.5f rad | in 3sig %.1f%%  final sigma %.4f/%.5f\n"
               "%-28s bias error gyro %.1e rad/s accel %.1e m/s2 | att sigma %.1e rad",
               sc->name, nvalid_first, inits, reinits, rejects, rp, rv, max_vlat, rz, ra,
               100.0 * n3sig / (nstat ? nstat : 1), O.PosSigma_m, O.VelSigma_ms, "", eg, ea2, O.AttSigma_rad);
        if (sc->st_from >= 0)
            printf(" | max att error in ST outage %.1e rad", max_att_outage);
        printf("  %s\n", ok ? "PASS" : "FAIL");
        return ok ? 0 : 1;
    }
}

int main(void)
{
    const char *why = NAV_CheckConfig(&NAV_CfgTbl);
    int         fails = 0;
    static const Scenario_t sc[] = {
        /* name                        r0 (CoM rel ISS)        v0              w0 (rad/s)          att0 (rad)        cyc  close  dropout   jump  lim pos/vel/att */
        {"approach from 40 m",          {0.5, -6.0, -60.0}, {0, 0, 0},      {0, 0, 0},          {0.02, -0.01, 0.01}, 3000, 0.10, -1, -1,  -1, 0.03, 0.0005, 0.0002, -1, -1},
        {"close in from 6 m, rotating", {0.05, -5.6, -24.0}, {0, 0, 0.02},  {0.001, -0.0005, 0.002}, {0, 0, 0},     700,  0.05, -1, -1,  -1, 0.01, 0.0005, 0.0002, -1, -1},
        {"LIDAR dropout 20 s",          {0.2, -5.8, -40.0}, {0, 0, 0},      {0, 0, 0},          {0, 0, 0},       2000, 0.08, 600, 700, -1, 0.03, 0.0005, 0.0002, -1, -1},
        {"scenario reset (teleport)",   {0.2, -5.8, -40.0}, {0, 0, 0},      {0, 0, 0},          {0, 0, 0},       1500, 0.08, -1, -1, 400, 0.03, 0.0005, 0.0002, -1, -1},
        {"star tracker out 60 s",       {0.2, -5.8, -60.0}, {0, 0, 0},      {0, 0, 0},          {0.01, 0, 0},    2500, 0.08, -1, -1,  -1, 0.03, 0.0005, 0.0003, 1200, 1500},
    };
    unsigned i;

    if (why != NULL)
    {
        printf("default table rejected: %s\n", why);
        return 1;
    }
    for (i = 0; i < sizeof(sc) / sizeof(sc[0]); i++)
        fails += Run(&sc[i]);
    printf(fails ? "FAILED (%d)\n" : "PASS\n", fails);
    return fails ? 1 : 0;
}
