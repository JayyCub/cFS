/*
** dock_mc — Monte Carlo closed-loop test of the docking flight software.
**
** Flies the real flight code — gnc_app.c (phases, guidance, control law),
** nav_filter.c (NAV estimation) and rcs_alloc.c (thruster allocation + PWM),
** each with its default table — against a 6-DOF truth model that behaves like
** the Unity sim: 20 ms physics steps, Clohessy-Wiltshire gravity, thruster
** valves opening for their commanded on-times, and the ChaserSensors models.
** Lock-step order per 0.2 s cycle is the same as on the Software Bus:
**   sensors -> GNC_APP_ProcessSimState -> NAV_Step -> GNC_APP_ProcessNavSolution
**   -> RCS_Allocate -> GNC_APP_ProcessThrusterCmd -> physics
** cFE itself is replaced by cfe_stubs.c. LC/SC fault protection is not
** simulated; an operator sends GO at start and 10 s into every hold.
**
** Each run draws dispersions: start position, attitude and rates; gyro and
** accelerometer biases; per-thruster thrust (±2 %) and alignment (0.3 deg);
** the true centre of mass (2 cm) and mass / inertia (2 % / 5 %) versus what
** the FSW tables believe.
**
** Modes
**   dock   (default) approach to capture; a run passes if the docking
**          mechanism captures within its limits (DockingDetector.cs)
**   abort  an ABORT is sent at a random range on approach; a run passes if
**          the chaser never touches the station over the next 25 minutes
**
** Build and run: apps/gnc_app/unit-test/run_mc.sh [build dir] [-- dock_mc args]
**   dock_mc [-n runs] [-s seed] [-m dock|abort] [-v run#] [-p GncParam=value ...]
** -p overrides a float GNC table parameter for a trade study (see Tunables[]).
*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gnc_app.h"
#include "nav_filter.h"
#include "rcs.h"
#include "cfe_stubs.h"

/* Default tables, compiled in (cfe_tbl_filedef.h is stubbed out) */
#include "../../nav/fsw/tables/nav_cfg_tbl.c"
#include "../../rcs/fsw/tables/rcs_thr_tbl.c"
#include "../fsw/tables/gnc_param_tbl.c"

#define DT          0.2
#define STEPS       10
#define H           (DT / STEPS)
#define ISP_S       300.0 /* Draco */
#define G0          9.80665
#define PI          3.14159265358979
#define DEG         (PI / 180.0)
#define DOCK_TMAX_S 2400.0
#define ABORT_TAIL_S 1500.0

/* ------------------------------------------------------------------------ */
/* Random numbers (deterministic per run)                                    */
/* ------------------------------------------------------------------------ */
static unsigned long long Rng;
static double Uniform(void)
{
    Rng = Rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return ((Rng >> 11) + 0.5) / 9007199254740992.0;
}
static double Gauss(void)
{
    return sqrt(-2.0 * log(Uniform())) * cos(2.0 * PI * Uniform());
}
static double U(double a, double b)
{
    return a + (b - a) * Uniform();
}

/* ------------------------------------------------------------------------ */
/* Vector helpers                                                            */
/* ------------------------------------------------------------------------ */
static void Rot(const float q[4], const double v[3], double out[3])
{
    float vf[3] = {(float)v[0], (float)v[1], (float)v[2]}, o[3];
    NAV_QuatRotate(q, vf, o);
    out[0] = o[0];
    out[1] = o[1];
    out[2] = o[2];
}
static void CrossD(const double a[3], const double b[3], double o[3])
{
    double x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2], z = a[0] * b[1] - a[1] * b[0];
    o[0]     = x;
    o[1]     = y;
    o[2]     = z;
}
static void SmallRot(float q[4], double ax, double ay, double az) /* q ← q ⊗ δq(a) */
{
    double th    = sqrt(ax * ax + ay * ay + az * az);
    float  dq[4] = {0, 0, 0, 1};
    if (th > 0)
    {
        double s = sin(th / 2) / th;
        dq[0]    = (float)(ax * s);
        dq[1]    = (float)(ay * s);
        dq[2]    = (float)(az * s);
        dq[3]    = (float)cos(th / 2);
    }
    NAV_QuatMul(q, dq, q);
    NAV_QuatNormalize(q, 0.5f);
}

/* ------------------------------------------------------------------------ */
/* Truth model                                                               */
/* ------------------------------------------------------------------------ */
typedef struct
{
    double arm[3]; /* nozzle relative to the TRUE CoM, body (m) */
    double dir[3]; /* push direction, body (with misalignment)   */
    double thrust; /* N                                           */
    int    enabled;
} HwThruster_t;

typedef struct
{
    double       r[3], v[3]; /* CoM - ISS reference, LVLH (m, m/s) */
    float        q[4];       /* q_L_B                               */
    double       w[3];       /* body rate (rad/s)                   */
    double       mass, I[3];
    double       com[3];     /* true CoM - table CoM, body (m)      */
    double       gyroBias[3], accelBias[3];
    HwThruster_t thr[RCS_MAX_THRUSTERS];
} Truth_t;

/* Sensor "hardware" spec (ChaserSensors.cs defaults, biases drawn per run) */
static const double GYRO_SIGMA = 1e-5, DV_SIGMA = 1e-5, ST_SIGMA = 5e-5;
static const double RNG_SIGMA0 = 0.005, RNG_FRAC = 0.001, ANG_SIGMA = 5e-4, POSE_SIGMA = 3e-3;
static const double RPS_MIN = 0.3, RPS_MAX = 250.0, RPS_FOV = 35.0 * DEG, POSE_MAX = 30.0;

static void DockFrame(float q_L_D[4])
{
    float q_tmp[4];
    NAV_QuatMul(NAV_CfgTbl.TargetAttQuat_L, NAV_CfgTbl.TargetPortQuat_T, q_tmp);
    NAV_QuatMul(q_tmp, NAV_CfgTbl.DockedQuat_TP, q_L_D);
}

/* Truth docking-frame state of the chaser port (what RelativeNav computes) */
typedef struct
{
    double pos_D[3], vel_D[3], atterr[3];
    double gap, lateral, closing, cone_deg, roll_deg, range;
} PortTruth_t;

static void TruthPort(const Truth_t *T, PortTruth_t *P)
{
    float  q_L_D[4], q_D_L[4], q_tmp[4], q_des[4], q_e[4];
    double pB[3], tT[3], port_L[3], tp_L[3], wxr_B[3], wxr_L[3], p[3], v[3];
    int    i;

    for (i = 0; i < 3; i++)
    {
        pB[i] = NAV_CfgTbl.ChaserPortPos_B[i] - T->com[i];
        tT[i] = NAV_CfgTbl.TargetPortPos_T[i];
    }
    DockFrame(q_L_D);
    NAV_QuatConj(q_L_D, q_D_L);
    Rot(T->q, pB, port_L);
    Rot(NAV_CfgTbl.TargetAttQuat_L, tT, tp_L);
    CrossD(T->w, pB, wxr_B);
    Rot(T->q, wxr_B, wxr_L);
    for (i = 0; i < 3; i++)
    {
        p[i] = T->r[i] + port_L[i] - tp_L[i];
        v[i] = T->v[i] + wxr_L[i];
    }
    Rot(q_D_L, p, P->pos_D);
    Rot(q_D_L, v, P->vel_D);

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
            P->atterr[i] = k * q_e[i];
    }
    P->gap      = -P->pos_D[2];
    P->lateral  = sqrt(P->pos_D[0] * P->pos_D[0] + P->pos_D[1] * P->pos_D[1]);
    P->closing  = P->vel_D[2];
    P->range    = sqrt(P->lateral * P->lateral + P->gap * P->gap);
    P->cone_deg = sqrt(P->atterr[0] * P->atterr[0] + P->atterr[1] * P->atterr[1]) / DEG;
    P->roll_deg = fabs(P->atterr[2]) / DEG;
}

/* DockingDetector.cs capture conditions */
static bool CaptureOk(const PortTruth_t *P)
{
    return P->gap <= 0.05 && P->closing > 0.0 && P->closing <= 0.30 && P->lateral <= 0.10 && P->cone_deg <= 10.0 &&
           P->roll_deg <= 10.0;
}

static void Sense(const Truth_t *T, const double dv_B[3], bool captured, SIMLINK_SimState_t *s)
{
    double pB[3], tT[3], sens_L[3], tp_L[3], d[3], p_S[3];
    float  q_L_S[4], q_S_L[4];
    int    i;

    memset(s, 0, sizeof(*s));
    s->CycleDt_s   = (float)DT;
    s->SensorValid = SIMLINK_SENSOR_IMU | SIMLINK_SENSOR_ST;
    s->MechFlags   = captured ? SIMLINK_MECH_CAPTURE : 0u;
    for (i = 0; i < 3; i++)
    {
        s->GyroRate_B[i] = (float)(T->w[i] + T->gyroBias[i] + GYRO_SIGMA * Gauss());
        s->DeltaV_B[i]   = (float)(dv_B[i] + T->accelBias[i] * DT + DV_SIGMA * Gauss());
        pB[i]            = NAV_CfgTbl.RpsPos_B[i] - T->com[i];
        tT[i]            = NAV_CfgTbl.TargetPortPos_T[i];
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
        if (rng >= RPS_MIN && rng <= RPS_MAX && acos(p_S[2] / rng) <= RPS_FOV)
        {
            s->SensorValid |= SIMLINK_SENSOR_RPS;
            s->RpsRange_m = (float)(rng + (RNG_SIGMA0 + RNG_FRAC * rng) * Gauss());
            s->RpsAz_rad  = (float)(atan2(p_S[0], p_S[2]) + ANG_SIGMA * Gauss());
            s->RpsEl_rad  = (float)(asin(p_S[1] / rng) + ANG_SIGMA * Gauss());
            if (rng <= POSE_MAX)
            {
                float q_L_TP[4];
                NAV_QuatMul(NAV_CfgTbl.TargetAttQuat_L, NAV_CfgTbl.TargetPortQuat_T, q_L_TP);
                NAV_QuatMul(q_S_L, q_L_TP, s->RpsQuat);
                SmallRot(s->RpsQuat, POSE_SIGMA * Gauss(), POSE_SIGMA * Gauss(), POSE_SIGMA * Gauss());
                s->SensorValid |= SIMLINK_SENSOR_RPS_POSE;
            }
        }
    }
}

/*
** One 0.2 s cycle of physics with the commanded valve on-times; returns the
** non-gravitational Δv (body) the IMU accumulates, and adds the impulse spent.
** Calls back after every step so capture / contact can be checked.
*/
typedef bool (*StepCheck_t)(const Truth_t *T, void *ctx);

static bool Physics(Truth_t *T, const float onTime[RCS_MAX_THRUSTERS], double dv_B[3], double *impulse_Ns,
                    StepCheck_t check, void *ctx)
{
    const double n = NAV_CfgTbl.MeanMotion_rads;
    int          k, i, j;

    dv_B[0] = dv_B[1] = dv_B[2] = 0.0;
    for (k = 0; k < STEPS; k++)
    {
        double F[3] = {0, 0, 0}, tau[3] = {0, 0, 0}, a_L[3], a_cw[3], Iw[3], wxIw[3];

        for (i = 0; i < RCS_MAX_THRUSTERS; i++)
        {
            double frac = (onTime[i] - k * H) / H, f[3], t[3];
            if (!T->thr[i].enabled || frac <= 0)
                continue;
            if (frac > 1)
                frac = 1;
            for (j = 0; j < 3; j++)
                f[j] = T->thr[i].dir[j] * T->thr[i].thrust * frac;
            CrossD(T->thr[i].arm, f, t);
            for (j = 0; j < 3; j++)
            {
                F[j] += f[j];
                tau[j] += t[j];
            }
            *impulse_Ns += T->thr[i].thrust * frac * H;
        }

        /* rotation: I ω' = τ - ω × Iω (body) */
        for (j = 0; j < 3; j++)
            Iw[j] = T->I[j] * T->w[j];
        CrossD(T->w, Iw, wxIw);
        for (j = 0; j < 3; j++)
            T->w[j] += (tau[j] - wxIw[j]) / T->I[j] * H;

        /* translation: thrust + CW gravity at the CoM (semi-implicit Euler, like PhysX) */
        {
            double Fm[3] = {F[0] / T->mass, F[1] / T->mass, F[2] / T->mass};
            Rot(T->q, Fm, a_L);
            for (j = 0; j < 3; j++)
                dv_B[j] += Fm[j] * H;
        }
        a_cw[0] = -n * n * T->r[0];
        a_cw[1] = 3 * n * n * T->r[1] - 2 * n * T->v[2];
        a_cw[2] = 2 * n * T->v[1];
        for (j = 0; j < 3; j++)
        {
            T->v[j] += (a_L[j] + a_cw[j]) * H;
            T->r[j] += T->v[j] * H;
        }
        SmallRot(T->q, T->w[0] * H, T->w[1] * H, T->w[2] * H);

        if (check != NULL && check(T, ctx))
            return true;
    }
    return false;
}

/* ------------------------------------------------------------------------ */
/* Flight software plumbing                                                  */
/* ------------------------------------------------------------------------ */
static RCS_ActReqMsg_t LastActReq;
static bool            HaveActReq;
static int             Verbose;
static double          SimTime;

static void OnTransmit(const CFE_MSG_Message_t *MsgPtr)
{
    if (MsgPtr == CFE_MSG_PTR(GNC_APP_Data.ActReqMsg.TelemetryHeader))
    {
        LastActReq = GNC_APP_Data.ActReqMsg;
        HaveActReq = true;
    }
}

static void OnEvent(uint16 EventID, uint16 EventType, const char *Text)
{
    (void)EventType;
    if (Verbose && EventID != GNC_APP_WAKEUP_INF_EID)
        printf("    t=%7.1f  GNC %2u: %s\n", SimTime, (unsigned)EventID, Text);
}

static void GroundCmd(CFE_MSG_FcnCode_t fc)
{
    CFE_SB_Buffer_t buf;
    memset(&buf, 0, sizeof(buf));
    Stub_FcnCode = fc;
    Stub_MsgSize = sizeof(GNC_APP_NoArgsCmd_t);
    GNC_APP_ProcessCmd(&buf);
}

/* The NAV app's packing of its output (nav.c NAV_ProcessSimState) */
static void PackSolution(const NAV_Output_t *O, uint32 Seq, NAV_SolutionMsg_t *Sol)
{
    memset(Sol, 0, sizeof(*Sol));
    Sol->SimTime_s        = SimTime;
    Sol->Seq              = Seq;
    Sol->CycleDt_s        = (float)DT;
    Sol->Status           = O->Status;
    Sol->ConsecRpsRejects = (uint16)(O->ConsecRpsRejects > 0xFFFF ? 0xFFFF : O->ConsecRpsRejects);
    memcpy(Sol->AttQuat_L, O->AttQuat_L, sizeof(Sol->AttQuat_L));
    memcpy(Sol->AngRate_B, O->AngRate_B, sizeof(Sol->AngRate_B));
    memcpy(Sol->AttErr_B, O->AttErr_B, sizeof(Sol->AttErr_B));
    memcpy(Sol->CmPos_L, O->CmPos_L, sizeof(Sol->CmPos_L));
    memcpy(Sol->CmVel_L, O->CmVel_L, sizeof(Sol->CmVel_L));
    memcpy(Sol->PortPos_D, O->PortPos_D, sizeof(Sol->PortPos_D));
    memcpy(Sol->PortVel_D, O->PortVel_D, sizeof(Sol->PortVel_D));
    Sol->Range_m         = O->Range_m;
    Sol->ClosingSpeed_ms = O->ClosingSpeed_ms;
    Sol->LateralOffset_m = O->LateralOffset_m;
    memcpy(Sol->DeltaV_B, O->DeltaV_B, sizeof(Sol->DeltaV_B));
    Sol->PosSigma_m   = O->PosSigma_m;
    Sol->VelSigma_ms  = O->VelSigma_ms;
    Sol->AttSigma_rad = O->AttSigma_rad;
    memcpy(Sol->DockFrameQuat_L, O->DockFrameQuat_L, sizeof(Sol->DockFrameQuat_L));
}

/* ------------------------------------------------------------------------ */
/* One run                                                                   */
/* ------------------------------------------------------------------------ */
typedef struct
{
    int    pass;
    int    docked;
    int    contact;          /* touched the station without a valid capture */
    double t_end;
    double gap0, lat0;       /* start                                       */
    double cont_lat, cont_close, cont_cone, cont_roll; /* at capture / contact */
    double dv_ms, prop_kg;
    double max_lat_inside10; /* largest lateral offset inside 10 m          */
    double abort_range;      /* abort mode: range when ABORT was sent        */
    double min_range_after;  /* abort mode: closest port range 60 s+ after   */
    int    reverts;          /* APPROACH -> CORRECT transitions              */
    const char *why;
} Result_t;

typedef struct
{
    int  captured, contact;
    bool abortMode;
} CheckCtx_t;

static bool StepCheck(const Truth_t *T, void *vctx)
{
    CheckCtx_t *ctx = (CheckCtx_t *)vctx;
    PortTruth_t P;
    TruthPort(T, &P);
    if (!ctx->abortMode && CaptureOk(&P))
    {
        ctx->captured = 1;
        return true;
    }
    /* Ports pass through each other: no capture -> collision with the station */
    if (P.gap < -0.02 && P.lateral < 1.0)
    {
        ctx->contact = 1;
        return true;
    }
    return false;
}

static void InitTruth(Truth_t *T, int mode)
{
    float  q_L_D[4];
    double gap, lat[2], p_D[3], p_L[3], port_L[3], tp_L[3], pB[3], tT[3];
    int    i, j;

    memset(T, 0, sizeof(*T));
    gap    = (mode == 0) ? U(15.0, 45.0) : U(10.0, 40.0);
    lat[0] = U(-1.5, 1.5);
    lat[1] = U(-1.5, 1.5);

    T->mass = GNC_ParamTbl.VehicleMass * (1.0 + 0.02 * Gauss());
    for (i = 0; i < 3; i++)
    {
        T->I[i]         = GNC_ParamTbl.Inertia_kgm2[i] * (1.0 + 0.05 * Gauss());
        T->com[i]       = 0.02 * Gauss();
        T->w[i]         = 0.002 * Gauss();
        T->v[i]         = 0.005 * Gauss();
        T->gyroBias[i]  = 2e-6 * Gauss();
        T->accelBias[i] = 3e-5 * Gauss();
    }

    /* attitude: docking attitude, dispersed */
    DockFrame(q_L_D);
    {
        float q_tmp[4];
        NAV_QuatConj(NAV_CfgTbl.ChaserPortQuat_B, q_tmp);
        NAV_QuatMul(q_L_D, q_tmp, T->q);
        SmallRot(T->q, 3.0 * DEG * Gauss(), 3.0 * DEG * Gauss(), 3.0 * DEG * Gauss());
    }

    /* place the CoM so the chaser port sits at (lat, -gap) in the docking frame */
    p_D[0] = lat[0];
    p_D[1] = lat[1];
    p_D[2] = -gap;
    Rot(q_L_D, p_D, p_L);
    for (i = 0; i < 3; i++)
    {
        pB[i] = NAV_CfgTbl.ChaserPortPos_B[i] - T->com[i];
        tT[i] = NAV_CfgTbl.TargetPortPos_T[i];
    }
    Rot(T->q, pB, port_L);
    Rot(NAV_CfgTbl.TargetAttQuat_L, tT, tp_L);
    for (i = 0; i < 3; i++)
        T->r[i] = p_L[i] + tp_L[i] - port_L[i];

    /* thruster hardware: table geometry relative to the true CoM, dispersed */
    for (i = 0; i < (int)RCS_ThrTbl.NumThrusters && i < RCS_MAX_THRUSTERS; i++)
    {
        double d[3], ax = 0.3 * DEG * Gauss(), ay = 0.3 * DEG * Gauss(), az = 0.3 * DEG * Gauss();
        float  q_mis[4] = {0, 0, 0, 1};
        SmallRot(q_mis, ax, ay, az);
        for (j = 0; j < 3; j++)
        {
            T->thr[i].arm[j] = RCS_ThrTbl.Thruster[i].Pos_m[j] - T->com[j];
            d[j]             = RCS_ThrTbl.Thruster[i].Dir[j];
        }
        Rot(q_mis, d, T->thr[i].dir);
        T->thr[i].thrust  = RCS_ThrTbl.Thrust_N * (1.0 + 0.02 * Gauss());
        T->thr[i].enabled = RCS_ThrTbl.Thruster[i].Enabled != 0;
    }
}

static void RunOne(int mode, Result_t *R)
{
    Truth_t            T;
    NAV_State_t        nav;
    NAV_Output_t       O;
    NAV_SolutionMsg_t  sol;
    SIM_IO_SimStateMsg_t sm;
    SIMLINK_SimState_t s;
    CheckCtx_t         ctx = {0, 0, mode == 1};
    PortTruth_t        P;
    double             dv_B[3] = {0, 0, 0}, impulse = 0.0, holdTimer = 0.0, abortRange, abortTime = -1.0;
    uint32             seq = 0;
    GNC_Phase_t        prevPhase;

    memset(R, 0, sizeof(*R));
    InitTruth(&T, mode);
    TruthPort(&T, &P);
    R->gap0             = P.gap;
    R->lat0             = P.lateral;
    R->min_range_after  = 1e9;
    abortRange          = U(1.0, 25.0);
    R->abort_range      = -1.0;

    Stub_TblPtr     = &GNC_ParamTbl;
    Stub_OnTransmit = OnTransmit;
    Stub_OnEvent    = OnEvent;
    SimTime         = 0.0;
    GNC_APP_Init();
    NAV_Reset(&nav);
    prevPhase = GNC_APP_Data.Phase;

    for (;;)
    {
        RCS_AllocResult_t       res;
        SIM_IO_ThrusterCmdMsg_t tc;
        bool                    stop;

        seq++;
        Sense(&T, dv_B, ctx.captured != 0, &s);

        memset(&sm, 0, sizeof(sm));
        sm.Seq       = seq;
        sm.SimTime_s = SimTime;
        sm.State     = s;
        GNC_APP_ProcessSimState(&sm);

        NAV_Step(&nav, &NAV_CfgTbl, &s, &O);
        PackSolution(&O, seq, &sol);
        HaveActReq = false;
        GNC_APP_ProcessNavSolution(&sol);

        if (GNC_APP_Data.Phase != prevPhase)
        {
            if (prevPhase == GNC_PHASE_APPROACH && GNC_APP_Data.Phase == GNC_PHASE_CORRECT)
                R->reverts++;
            prevPhase = GNC_APP_Data.Phase;
        }

        memset(&res, 0, sizeof(res));
        if (HaveActReq)
            RCS_Allocate(&RCS_ThrTbl, LastActReq.LinImpulse_Ns, LastActReq.AngImpulse_Nms, &res);
        memset(&tc, 0, sizeof(tc));
        tc.Seq = seq;
        memcpy(tc.AchievedLinImpulse_Ns, res.AchievedLin_Ns, sizeof(tc.AchievedLinImpulse_Ns));
        memcpy(tc.AchievedAngImpulse_Nms, res.AchievedAng_Nms, sizeof(tc.AchievedAngImpulse_Nms));
        GNC_APP_ProcessThrusterCmd(&tc);

        stop = Physics(&T, res.OnTime_s, dv_B, &impulse, StepCheck, &ctx);
        SimTime += DT;
        TruthPort(&T, &P);

        if (P.range < 10.0 && P.lateral > R->max_lat_inside10 && abortTime < 0)
            R->max_lat_inside10 = P.lateral;
        if (abortTime >= 0 && SimTime > abortTime + 60.0 && P.range < R->min_range_after)
            R->min_range_after = P.range;

        if (stop)
        {
            R->cont_lat   = P.lateral;
            R->cont_close = P.closing;
            R->cont_cone  = P.cone_deg;
            R->cont_roll  = P.roll_deg;
            break;
        }

        /* operator */
        if (seq == 5)
            GroundCmd(GNC_APP_GO_CC);
        if (abortTime < 0)
        {
            if (GNC_APP_Data.Phase == GNC_PHASE_HOLD)
            {
                holdTimer += DT;
                if (holdTimer >= 10.0)
                {
                    GroundCmd(GNC_APP_GO_CC);
                    holdTimer = 0.0;
                }
            }
            else
            {
                holdTimer = 0.0;
            }
            if (mode == 1 && GNC_APP_Data.Phase == GNC_PHASE_APPROACH && P.range <= abortRange)
            {
                GroundCmd(GNC_APP_ABORT_CC);
                abortTime      = SimTime;
                R->abort_range = P.range;
            }
        }

        if (mode == 0 && SimTime > DOCK_TMAX_S)
            break;
        if (mode == 1 && ((abortTime >= 0 && SimTime > abortTime + ABORT_TAIL_S) || SimTime > DOCK_TMAX_S + ABORT_TAIL_S))
            break;
    }

    R->t_end   = SimTime;
    R->docked  = ctx.captured;
    R->contact = ctx.contact;
    R->dv_ms   = impulse / T.mass;
    R->prop_kg = impulse / (ISP_S * G0);
    if (mode == 0)
    {
        R->pass = R->docked;
        R->why  = R->docked ? "docked" : R->contact ? "CONTACT OUT OF LIMITS" : "TIMEOUT";
    }
    else
    {
        R->pass = abortTime >= 0 && !R->contact;
        R->why  = abortTime < 0 ? "never reached abort point" : R->contact ? "HIT THE STATION" : "safe";
    }
}

/* ------------------------------------------------------------------------ */
/* GNC table overrides for trade studies (-p Name=value)                     */
/* ------------------------------------------------------------------------ */
#define TUNABLE(f) {#f, &GNC_ParamTbl.f}
static const struct
{
    const char *name;
    float      *field;
} Tunables[] = {
    TUNABLE(LatKp), TUNABLE(LatKp_Approach), TUNABLE(LatVelDeadband_ms), TUNABLE(LatVelDeadband_Approach_ms),
    TUNABLE(MaxLatSpeed), TUNABLE(AttKp), TUNABLE(MaxAttRate), TUNABLE(AttDeadband_deg), TUNABLE(SpinThreshold_rads),
    TUNABLE(AttCtrlAccel_rads2), TUNABLE(AttRateDeadband_rads), TUNABLE(FfReleaseDv_ms), TUNABLE(AxialKp),
    TUNABLE(MinCloseSpeed), TUNABLE(MaxCloseSpeed), TUNABLE(MaxCloseSpeed_Inner), TUNABLE(CamDeltaV_ms),
    TUNABLE(PassiveSafeRange_m), TUNABLE(LatEntryThreshold_m),
};

static int SetTunable(const char *arg)
{
    const char *eq = strchr(arg, '=');
    unsigned    i;
    if (eq != NULL)
        for (i = 0; i < sizeof(Tunables) / sizeof(Tunables[0]); i++)
            if (strlen(Tunables[i].name) == (size_t)(eq - arg) && !strncmp(arg, Tunables[i].name, eq - arg))
            {
                *Tunables[i].field = (float)atof(eq + 1);
                printf("dock_mc: %s = %g\n", Tunables[i].name, (double)*Tunables[i].field);
                return 1;
            }
    printf("dock_mc: unknown parameter override '%s'\n", arg);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Statistics                                                                */
/* ------------------------------------------------------------------------ */
static int CmpD(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static void Stat(const char *name, double *v, int n, const char *unit)
{
    double sum = 0;
    int    i;
    if (n == 0)
        return;
    qsort(v, n, sizeof(double), CmpD);
    for (i = 0; i < n; i++)
        sum += v[i];
    printf("  %-30s mean %9.4f   p95 %9.4f   max %9.4f  %s\n", name, sum / n, v[(int)(0.95 * (n - 1))], v[n - 1],
           unit);
}

int main(int argc, char **argv)
{
    int                 runs = 100, mode = 0, verboseRun = -1, i, npass = 0, nd = 0;
    unsigned long long  seed = 1;
    Result_t           *Rs;
    double             *a, *b, *c, *d, *e, *f, *g, *h;

    for (i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-n") && i + 1 < argc)
            runs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-s") && i + 1 < argc)
            seed = strtoull(argv[++i], NULL, 10);
        else if (!strcmp(argv[i], "-m") && i + 1 < argc)
            mode = !strcmp(argv[++i], "abort") ? 1 : 0;
        else if (!strcmp(argv[i], "-v") && i + 1 < argc)
            verboseRun = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-p") && i + 1 < argc)
        {
            if (!SetTunable(argv[++i]))
                return 2;
        }
    }

    Rs = calloc(runs, sizeof(Result_t));
    a  = calloc(runs, sizeof(double));
    b  = calloc(runs, sizeof(double));
    c  = calloc(runs, sizeof(double));
    d  = calloc(runs, sizeof(double));
    e  = calloc(runs, sizeof(double));
    f  = calloc(runs, sizeof(double));
    g  = calloc(runs, sizeof(double));
    h  = calloc(runs, sizeof(double));

    printf("dock_mc: %d %s runs, seed %llu\n", runs, mode ? "abort" : "dock", seed);
    for (i = 0; i < runs; i++)
    {
        Result_t *R = &Rs[i];
        Rng         = seed * 1000003ULL + (unsigned long long)i * 7919ULL + 1;
        Verbose     = (i == verboseRun);
        RunOne(mode, R);
        npass += R->pass;
        if (mode == 0)
            printf("  run %3d  start %5.1f m (lat %4.2f)  %-22s t=%6.0f s  contact lat %.3f m close %.3f m/s "
                   "cone %.1f roll %.1f deg  dv %.2f m/s  prop %.1f kg  reverts %d\n",
                   i, R->gap0, R->lat0, R->why, R->t_end, R->cont_lat, R->cont_close, R->cont_cone, R->cont_roll,
                   R->dv_ms, R->prop_kg, R->reverts);
        else
            printf("  run %3d  abort at %5.1f m  %-26s closest after 60 s %7.1f m  dv %.2f m/s\n", i,
                   R->abort_range, R->why, R->min_range_after, R->dv_ms);
        if (R->docked)
        {
            a[nd] = R->cont_lat;
            b[nd] = R->cont_close;
            c[nd] = R->cont_cone;
            d[nd] = R->cont_roll;
            e[nd] = R->t_end;
            f[nd] = R->prop_kg;
            g[nd] = R->max_lat_inside10;
            h[nd] = R->reverts;
            nd++;
        }
    }

    printf("\n%s: %d / %d runs passed (%.1f %%)\n", mode ? "ABORT" : "DOCK", npass, runs, 100.0 * npass / runs);
    if (mode == 0 && nd > 0)
    {
        Stat("contact lateral offset", a, nd, "m    (limit 0.10)");
        Stat("contact closing speed", b, nd, "m/s  (limit 0.30)");
        Stat("contact cone angle", c, nd, "deg  (limit 10)");
        Stat("contact roll", d, nd, "deg  (limit 10)");
        Stat("time to capture", e, nd, "s");
        Stat("propellant", f, nd, "kg");
        Stat("max lateral inside 10 m", g, nd, "m");
        Stat("APPROACH->CORRECT reverts", h, nd, "");
    }
    return npass == runs ? 0 : 1;
}
