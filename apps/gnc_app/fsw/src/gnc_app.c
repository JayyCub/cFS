#include "gnc_app.h"

GNC_APP_Data_t GNC_APP_Data;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void GNC_APP_Main(void)
{
    CFE_Status_t     status;
    CFE_SB_Buffer_t *MsgBuf;
    CFE_SB_MsgId_t   MsgId;

    status = GNC_APP_Init();
    if (status != CFE_SUCCESS)
        GNC_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;

    while (CFE_ES_RunLoop(&GNC_APP_Data.RunStatus) == true)
    {
        status = CFE_SB_ReceiveBuffer(&MsgBuf, GNC_APP_Data.CmdPipe, CFE_SB_PEND_FOREVER);

        if (status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(GNC_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "GNC_APP: SB pipe read error 0x%08lX, exiting", (unsigned long)status);
            GNC_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
            continue;
        }

        CFE_MSG_GetMsgId(&MsgBuf->Msg, &MsgId);

        switch (CFE_SB_MsgIdToValue(MsgId))
        {
            case GNC_APP_SEND_HK_MID:
                GNC_APP_ProcessWakeup();
                break;
            case GNC_APP_CMD_MID:
                GNC_APP_ProcessCmd(MsgBuf);
                break;
            default:
                break;
        }
    }

    CFE_ES_ExitApp(GNC_APP_Data.RunStatus);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
CFE_Status_t GNC_APP_Init(void)
{
    CFE_Status_t status;
    int32        OsStatus;

    memset(&GNC_APP_Data, 0, sizeof(GNC_APP_Data));
    GNC_APP_Data.RunStatus    = CFE_ES_RunStatus_APP_RUN;
    GNC_APP_Data.AbortLatch   = true;  /* inhibit guidance until operator sends GO */
    GNC_APP_Data.HoldPt1Armed = true;  /* arm outer hold-point waypoint            */
    GNC_APP_Data.HoldPt2Armed = true;  /* arm inner hold-point waypoint            */

    /* 1. Register with Event Services */
    status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("GNC_APP: CFE_EVS_Register failed, RC=0x%08lX\n", (unsigned long)status);
        return status;
    }

    /* 2. Initialize HK telemetry message header */
    CFE_MSG_Init(CFE_MSG_PTR(GNC_APP_Data.HkTlm.TelemetryHeader),
                 CFE_SB_ValueToMsgId(GNC_APP_HK_TLM_MID),
                 sizeof(GNC_APP_HkTlm_t));

    /* 3. Create SB pipe and subscribe to wakeup message */
    status = CFE_SB_CreatePipe(&GNC_APP_Data.CmdPipe, GNC_APP_PIPE_DEPTH, GNC_APP_PIPE_NAME);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GNC_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: CFE_SB_CreatePipe failed, RC=0x%08lX", (unsigned long)status);
        return status;
    }

    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GNC_APP_SEND_HK_MID), GNC_APP_Data.CmdPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GNC_APP_SUB_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: subscribe to SEND_HK_MID failed, RC=0x%08lX", (unsigned long)status);
        return status;
    }

    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GNC_APP_CMD_MID), GNC_APP_Data.CmdPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GNC_APP_SUB_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: subscribe to CMD_MID failed, RC=0x%08lX", (unsigned long)status);
        return status;
    }

    /*
    ** 4. Create the mutex that guards the shared telemetry buffer.
    **
    ** In cFS, any state shared between a background thread and the main app
    ** task must be protected. OS_MutSemCreate is the OSAL API — it maps to
    ** pthread_mutex on Linux, but the same call works on RTEMS and VxWorks.
    */
    OsStatus = OS_MutSemCreate(&GNC_APP_Data.TlmMutex, "GNC_TLM_MTX", 0);
    if (OsStatus != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(GNC_APP_MUTEX_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: OS_MutSemCreate failed, RC=%d", (int)OsStatus);
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    /*
    ** 5. Spawn the UDP receive task.
    **
    ** OS_TaskCreate is the OSAL equivalent of pthread_create. The task runs
    ** GNC_APP_UdpRecvTask independently of the main app task, blocked on
    ** OS_SocketRecvFrom until a packet arrives from Unity.
    **
    ** Priority 80 is lower than the main app task (55 in the startup script),
    ** so the receive task never pre-empts the control loop.
    */
    OsStatus = OS_TaskCreate(&GNC_APP_Data.UdpTaskId,
                             GNC_APP_UDP_TASK_NAME,
                             GNC_APP_UdpRecvTask,
                             OSAL_TASK_STACK_ALLOCATE,
                             GNC_APP_UDP_STACK_SIZE,
                             GNC_APP_UDP_TASK_PRIO,
                             0);
    if (OsStatus != OS_SUCCESS)
    {
        CFE_EVS_SendEvent(GNC_APP_TASK_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: OS_TaskCreate failed, RC=%d", (int)OsStatus);
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    /* 6. Open the command send socket (cFS → Unity). */
    GNC_APP_OpenCmdSocket();

    /*
    ** 7. Register and load the GNC parameter table.
    **
    ** CFE_TBL_Register allocates a slot in the table registry.  The table name
    ** is scoped to this app: "GNC_APP.ParamTbl" as seen by ground tools.
    ** CFE_TBL_OPT_DEFAULT = single-buffer, load from file.
    ** NULL validator: no range-checking on load (add one in Phase 5B+ for safety).
    **
    ** CFE_TBL_Load fills the buffer from the compiled binary installed to /cf/.
    ** CFE_TBL_GetAddress hands us a pointer to the live buffer — this pointer
    ** remains valid until the next CFE_TBL_Manage call that swaps in a new image.
    */
    status = CFE_TBL_Register(&GNC_APP_Data.ParamTblHandle,
                               GNC_APP_PARAM_TBL_NAME,
                               sizeof(GNC_ParamTbl_t),
                               CFE_TBL_OPT_DEFAULT,
                               NULL);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GNC_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: CFE_TBL_Register failed, RC=0x%08lX", (unsigned long)status);
        return status;
    }

    status = CFE_TBL_Load(GNC_APP_Data.ParamTblHandle,
                           CFE_TBL_SRC_FILE,
                           GNC_APP_PARAM_TBL_FILE);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GNC_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: CFE_TBL_Load(%s) failed, RC=0x%08lX",
                          GNC_APP_PARAM_TBL_FILE, (unsigned long)status);
        return status;
    }

    status = CFE_TBL_GetAddress((void **)&GNC_APP_Data.ParamTblPtr,
                                 GNC_APP_Data.ParamTblHandle);
    if (status != CFE_SUCCESS && status != CFE_TBL_INFO_UPDATED)
    {
        CFE_EVS_SendEvent(GNC_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: CFE_TBL_GetAddress failed, RC=0x%08lX", (unsigned long)status);
        return status;
    }

    CFE_EVS_SendEvent(GNC_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "GNC_APP initialized. Recv port %d, Cmd port %d. "
                      "Guidance INHIBITED — send GO to start.",
                      GNC_APP_UDP_LISTEN_PORT, GNC_APP_CMD_PORT);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_SelectPhase — discrete GNC phase state machine                   */
/*                                                                           */
/* Evaluates the current telemetry snapshot against phase transition rules  */
/* and returns the phase that should be active this cycle.  Hysteresis is   */
/* built into the gate pair (GNC_LAT_APPROACH_GATE / GNC_LAT_CORRECT_GATE)  */
/* so a single noisy telemetry sample cannot flip the phase back and forth.  */
/*                                                                           */
/* Called once per wakeup cycle BEFORE ComputeControl so the control law    */
/* always operates on a consistent, up-to-date phase value.                  */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static GNC_Phase_t GNC_APP_SelectPhase(const GNC_APP_UnityTlm_t *tlm, GNC_Phase_t current)
{
    /* Docked check is unconditional — highest priority inhibit */
    if (tlm->Flags & 0x2)
        return GNC_PHASE_DOCKED;

    /* Once docked, stay docked until the scenario is reset (cFS restart) */
    if (current == GNC_PHASE_DOCKED)
        return GNC_PHASE_DOCKED;

    /*
    ** HOLD is sticky — only ground GO or ABORT commands release it.
    ** Autonomous gate transitions must not override a ground-commanded hold.
    */
    if (current == GNC_PHASE_HOLD)
        return GNC_PHASE_HOLD;

    /*
    ** AbortLatch prevents auto-promotion from IDLE until the operator sends GO.
    ** This ensures guidance does not silently resume after an abort.
    */
    if (GNC_APP_Data.AbortLatch)
        return GNC_PHASE_IDLE;

    /*
    ** IDLE → CORRECT on first usable telemetry frame.
    ** After promotion, fall through to the gate check so a vehicle that is
    ** already on-axis can skip directly to APPROACH in the same cycle.
    */
    GNC_Phase_t next = (current == GNC_PHASE_IDLE) ? GNC_PHASE_CORRECT : current;

    /*
    ** Lateral offset gate with hysteresis.
    **
    ** Entry (CORRECT -> APPROACH) uses a fixed absolute threshold: initial
    ** convergence should mean actually near the axis, not just "within some
    ** fraction of however forgiving the corridor happens to be at this range"
    ** (a corridor-relative entry gate let LAT_CORR hand off to APPROACH 4.46 m
    ** off-axis at 33 m range).
    **
    ** Revert (APPROACH -> CORRECT) stays corridor-relative: "allowed" is the
    ** radius of the real docking corridor cone at the current range, so once
    ** underway, the gate tightens automatically as the vehicle closes in
    ** instead of using one constant across the whole approach.
    */
    {
        const GNC_ParamTbl_t *p = GNC_APP_Data.ParamTblPtr;

        if (next == GNC_PHASE_CORRECT && tlm->LateralOffset_m < p->LatEntryThreshold_m)
            return GNC_PHASE_APPROACH;

        float allowed = tlm->Range_m * tanf(p->ConeHalfAngle_deg * GNC_DEG2RAD);
        if (allowed < p->MinAllowedLateral_m) allowed = p->MinAllowedLateral_m;

        if (next == GNC_PHASE_APPROACH && tlm->LateralOffset_m > allowed * p->CorridorMarginOut)
            return GNC_PHASE_CORRECT;
    }

    /*
    ** Autonomous hold-point check — fires once per approach sequence when range
    ** drops to or below each configured waypoint threshold during APPROACH.
    ** Hold points are consumed in outer→inner order; each fires at most once
    ** before being disarmed (re-armed only by ABORT+GO, which implies a reset).
    ** A threshold of 0.0 disables that hold point.
    */
    if (next == GNC_PHASE_APPROACH)
    {
        const GNC_ParamTbl_t *p     = GNC_APP_Data.ParamTblPtr;
        float                 v     = (tlm->ClosingSpeed_ms > 0.0f) ? tlm->ClosingSpeed_ms : 0.0f;
        /* Distance needed to brake to a stop from current closing speed.
        ** BrakeAccel_Hard_mss is the actual deceleration from all 8 brake thrusters
        ** (T08-T15) in Unity — calibrated from thruster geometry, not ThrusterForce.
        ** The continuous-thrust formula is v²/(2a); at the old 1 Hz cadence the
        ** discrete loop only achieved roughly half that in practice (cycle-miss +
        ** 0.95s burn cap), so empirically v²/a was used instead. That derating was
        ** calibrated for a 1 Hz cycle — now that GNC_CYCLE_DT_S is 0.2s (5 Hz),
        ** cycle-miss loss should be much smaller and this formula is likely too
        ** conservative (braking earlier than necessary). Left as v²/a pending a
        ** real test run to recalibrate against actual 5 Hz braking distance. */
        float brake_dist = (v * v) / p->BrakeAccel_Hard_mss;

        if (GNC_APP_Data.HoldPt1Armed && p->HoldPoint1_m > 0.0f &&
            tlm->Range_m <= p->HoldPoint1_m + brake_dist)
        {
            GNC_APP_Data.HoldPt1Armed = false;
            GNC_APP_Data.HoldRange_m  = tlm->Range_m;
            CFE_EVS_SendEvent(GNC_APP_HOLDPT1_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC_APP: HOLD POINT 1 — braking to %.2f m (range now %.2f m) — awaiting GO",
                              (double)p->HoldPoint1_m, (double)tlm->Range_m);
            return GNC_PHASE_HOLD;
        }

        if (GNC_APP_Data.HoldPt2Armed && p->HoldPoint2_m > 0.0f &&
            tlm->Range_m <= p->HoldPoint2_m + brake_dist)
        {
            GNC_APP_Data.HoldPt2Armed = false;
            GNC_APP_Data.HoldRange_m  = tlm->Range_m;
            CFE_EVS_SendEvent(GNC_APP_HOLDPT2_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC_APP: HOLD POINT 2 — braking to %.2f m (range now %.2f m) — awaiting GO",
                              (double)p->HoldPoint2_m, (double)tlm->Range_m);
            return GNC_PHASE_HOLD;
        }
    }

    return next;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_LateralAxis — position + velocity burn for one lateral axis      */
/*                                                                           */
/* Shared by the X and Y channels in GNC_APP_ComputeControl: compute a      */
/* target velocity from continuous position feedback (gain differs by      */
/* phase — gentler in APPROACH), then fire a proportional burn to close    */
/* the velocity error.                                                      */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void GNC_APP_LateralAxis(float pos, float vel, float ff, GNC_Phase_t phase,
                                 const GNC_ParamTbl_t *p, float accel,
                                 float *force, float *max_trans_dur)
{
    /* Both phases actively hold the centerline now — APPROACH just uses a
    ** gentler gain (LatKp_Approach, tuned well below LatKp) so the burns it
    ** fires are small enough to stay clear of the lateral/attitude coupling
    ** limit cycle documented on the attitude deadband below, rather than
    ** disengaging position feedback entirely. */
    float kp    = (phase == GNC_PHASE_APPROACH) ? p->LatKp_Approach : p->LatKp;
    float v_tgt = -kp * pos;
    if (v_tgt >  p->MaxLatSpeed) v_tgt =  p->MaxLatSpeed;
    if (v_tgt < -p->MaxLatSpeed) v_tgt = -p->MaxLatSpeed;

    float v_err = v_tgt - vel + ff;
    float db    = p->LatVelDeadband_ms;   /* m/s — coasting threshold */
    if (v_err > db)
    {
        float dur = (v_err - db) / accel;
        if (dur >= p->MinBurnDuration) { *force += p->ThrusterForce; if (dur > *max_trans_dur) *max_trans_dur = dur; }
    }
    else if (v_err < -db)
    {
        float dur = (-v_err - db) / accel;
        if (dur >= p->MinBurnDuration) { *force -= p->ThrusterForce; if (dur > *max_trans_dur) *max_trans_dur = dur; }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_AttitudeAxis — PD burn for one attitude axis (pitch/yaw/roll)    */
/*                                                                           */
/* Shared by the three Channel 3 axes: target rate = AttKp × error (rad),  */
/* rate error drives the burn duration.  kT is passed in rather than       */
/* recomputed per axis (it's the same for all three).                     */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void GNC_APP_AttitudeAxis(float err_deg, float angvel, const GNC_ParamTbl_t *p,
                                  float kT, float *torque, float *max_att_dur)
{
    const float d2r       = 0.01745329f;  /* π / 180 */
    float       omega_tgt = p->AttKp * (err_deg * d2r);
    if (omega_tgt >  p->MaxAttRate) omega_tgt =  p->MaxAttRate;
    if (omega_tgt < -p->MaxAttRate) omega_tgt = -p->MaxAttRate;

    float omega_err = omega_tgt - angvel;
    if (omega_err > 0.0f)
    {
        float dur = omega_err / p->RotAccel;
        if (dur >= p->MinBurnDuration) { *torque += kT; if (dur > *max_att_dur) *max_att_dur = dur; }
    }
    else if (omega_err < 0.0f)
    {
        float dur = -omega_err / p->RotAccel;
        if (dur >= p->MinBurnDuration) { *torque -= kT; if (dur > *max_att_dur) *max_att_dur = dur; }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_ComputeControl — phase-dispatched proportional guidance          */
/*                                                                           */
/* Three independent channels; behaviour changes with phase:                 */
/*                                                                           */
/*   Channel 1 — Axial                                                       */
/*     APPROACH : proportional guidance  v_tgt = clamp(KP * range, min, max)*/
/*                max is MaxCloseSpeed before HoldPoint1_m fires, then the   */
/*                tighter MaxCloseSpeed_Inner for the remainder of approach */
/*     CORRECT  : station-keep           v_tgt = 0 (brake if drifting in)   */
/*     HOLD     : position + velocity    v_tgt = clamp(KP_HOLD * (range -   */
/*                                        HoldRange_m), ±MaxHoldSpeed)      */
/*                                                                           */
/*   Channel 2 — Lateral (X and Y)                                          */
/*     Both phases: position + velocity controller.                          */
/*       v_lat_tgt = clamp(-KP_LAT * Pos_XY, ±MAX_LAT_SPEED)               */
/*       dur       = |v_lat_tgt - Vel_XY| / accel                           */
/*     Pos_X/Y serve as lateral errors (world-origin / fixed-target assumed).*/
/*                                                                           */
/*   Channel 3 — Attitude PD (all phases)                                    */
/*     omega_tgt = clamp(AttKp × error_rad, ±MaxAttRate)                     */
/*     dur       = |omega_tgt − AngVel| / RotAccel                           */
/*                                                                           */
/* Duration is the maximum across all active channels so the dominant        */
/* correction is exact; shorter-needed axes are slightly over-fired but self-*/
/* correct within the next wakeup cycle (same behaviour as real bang-coast-  */
/* bang proximity ops with finite-duration pulses).                          */
/*                                                                           */
/* Reads only from the caller's local copy of tlm — no shared state.        */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static GNC_Control_t GNC_APP_ComputeControl(const GNC_APP_UnityTlm_t *tlm, GNC_Phase_t phase)
{
    GNC_Control_t         ctrl         = {0};
    float                 max_trans_dur = 0.0f;  /* longest needed by translational channels */
    float                 max_att_dur   = 0.0f;  /* longest needed by attitude channels      */
    float                 dur;
    float                 ff_x, ff_y, ff_z;  /* CW feedforward delta-v (m/s) per axis */

    /* IDLE and DOCKED: all thrust inhibited */
    if (phase == GNC_PHASE_IDLE || phase == GNC_PHASE_DOCKED)
        return ctrl;

    /* Dereference the table pointer once for readability.
    ** ParamTblPtr is guaranteed non-NULL after a successful Init. */
    const GNC_ParamTbl_t *p     = GNC_APP_Data.ParamTblPtr;
    float                 accel = p->ThrusterForce / p->VehicleMass;  /* m/s² per thruster */
    float                 kF    = p->ThrusterForce;
    float                 kT    = kF * GNC_RCS_MOMENT_ARM;

    /* === CW Feedforward ============================================== */
    /* ClohessyWiltshire.cs applies the differential gravity equations    */
    /* below to the chaser every FixedUpdate (world X=radial,             */
    /* Y=along-track, Z=approach/cross-track).  Folding the equal-and-    */
    /* opposite delta-v into each channel's velocity error pre-cancels    */
    /* the drift without extra logic — the existing burn computation       */
    /* absorbs it automatically.  ax/ay/az are accelerations (m/s²); the   */
    /* ×GNC_CYCLE_DT_S converts each to the delta-v accumulated over one   */
    /* wakeup period, so the feedforward stays correct if the cycle rate   */
    /* changes (it does NOT assume a 1-second cycle the way the raw accel  */
    /* values would).                                                       */
    /*   CW: ax =  3n²rx + 2n·vy  →  ff_x = -(ax) × dt                    */
    /*       ay = -2n·vx          →  ff_y = -(ay) × dt = +2n·vx × dt      */
    /*       az = -n²rz           →  ff_z = -(az) × dt = +n²rz × dt       */
    {
        const float n  = GNC_CW_MEAN_MOTION;
        const float n2 = n * n;
        ff_x = -(3.0f * n2 * tlm->Pos_X + 2.0f * n * tlm->Vel_Y) * GNC_CYCLE_DT_S;
        ff_y =   2.0f * n  * tlm->Vel_X                          * GNC_CYCLE_DT_S;
        ff_z =   n2         * tlm->Pos_Z                          * GNC_CYCLE_DT_S;
    }

    /* === Channel 1 — Axial =========================================== */
    /*                                                                    */
    /* APPROACH: proportional guidance — target closing speed scales with */
    /*   range so the vehicle naturally slows as it closes (linear law).  */
    /* HOLD: station-keep with position feedback toward HoldRange_m — the  */
    /*   range captured when the hold was entered — so drift accumulated  */
    /*   during the hold (imperfect braking, residual CW drift) is walked */
    /*   back out instead of merely having its velocity zeroed.           */
    /* CORRECT: station-keep — target closing speed is zero (transient    */
    /*   phase driving toward APPROACH; no fixed range to hold yet).      */
    {
        float v_axial_tgt;
        if (phase == GNC_PHASE_APPROACH)
        {
            /* Outer cap before HoldPoint1_m has fired, tighter cap after —
            ** models the real-world profile of a faster outer approach speed
            ** and a slower, more cautious speed once inside the first hold. */
            float cap = GNC_APP_Data.HoldPt1Armed ? p->MaxCloseSpeed : p->MaxCloseSpeed_Inner;
            v_axial_tgt = tlm->Range_m * p->AxialKp;
            if (v_axial_tgt < p->MinCloseSpeed) v_axial_tgt = p->MinCloseSpeed;
            if (v_axial_tgt > cap) v_axial_tgt = cap;
        }
        else if (phase == GNC_PHASE_HOLD)
        {
            float range_err = tlm->Range_m - GNC_APP_Data.HoldRange_m;
            v_axial_tgt = p->AxialHoldKp * range_err;
            if (v_axial_tgt >  p->MaxHoldSpeed) v_axial_tgt =  p->MaxHoldSpeed;
            if (v_axial_tgt < -p->MaxHoldSpeed) v_axial_tgt = -p->MaxHoldSpeed;
        }
        else /* CORRECT — station-keep axially */
        {
            v_axial_tgt = 0.0f;
        }

        float v_axial_err = v_axial_tgt - tlm->ClosingSpeed_ms + ff_z;
        if (v_axial_err > 0.0f)
        {
            /* Need to speed up — fire approach group (T04-T07, +Z).  kF = ThrusterForce.
            ** Use ApproachAccel_mss (empirically measured) so the burn duration is exact
            ** rather than over-shooting by 1.8× with the theoretical ThrusterForce/Mass. */
            dur = v_axial_err / p->ApproachAccel_mss;
            if (dur >= p->MinBurnDuration) { ctrl.Fz += kF; if (dur > max_trans_dur) max_trans_dur = dur; }
        }
        else if (v_axial_err < 0.0f)
        {
            /* Need to brake — choose hard or soft based on velocity error magnitude.
            **   Hard (|err| > half MaxCloseSpeed): T08-T15, BrakeAccel_Hard_mss × mass.
            **   Soft (fine correction): T08-T11 only, BrakeAccel_Light_mss × mass.
            ** Unity selects the thruster group by comparing |Fz| against 43 N. */
            bool  hard        = ((-v_axial_err) > (p->MaxCloseSpeed * 0.5f));
            float brake_accel = hard ? p->BrakeAccel_Hard_mss : p->BrakeAccel_Light_mss;
            float brake_force = brake_accel * p->VehicleMass;
            dur = (-v_axial_err) / brake_accel;
            if (dur >= p->MinBurnDuration) { ctrl.Fz -= brake_force; if (dur > max_trans_dur) max_trans_dur = dur; }
        }
    }

    /* === Channel 2 — Lateral position + velocity (X and Y) =========== */
    /*                                                                    */
    /* Both phases hold the centerline continuously via position +      */
    /* velocity feedback:                                                */
    /*   Target velocity = clamp(-Kp × Pos_XY, ±MaxLatSpeed)            */
    /*   Velocity error drives the burn duration.                        */
    /* Kp is LatKp in CORRECT, LatKp_Approach (deliberately gentler) in  */
    /* APPROACH — see GNC_APP_LateralAxis for why: the tighter APPROACH  */
    /* attitude deadband below has less headroom to absorb the attitude  */
    /* disturbance each lateral burn causes, so APPROACH corrects with   */
    /* smaller, more frequent nudges rather than CORRECT's stronger gain.*/
    /*                                                                    */
    /* Velocity deadband: skip the burn when the velocity error is small */
    /* enough that the minimum 400 N impulse would overshoot.  Without   */
    /* this, the controller alternates ±400 N every cycle (bang-bang     */
    /* chatter) whenever it is near the target velocity.                 */
    {
        GNC_APP_LateralAxis(tlm->Pos_X, tlm->Vel_X, ff_x, phase, p, accel, &ctrl.Fx, &max_trans_dur);
        GNC_APP_LateralAxis(tlm->Pos_Y, tlm->Vel_Y, ff_y, phase, p, accel, &ctrl.Fy, &max_trans_dur);
    }

    /* === Channel 3 — Attitude PD (all active phases) =================== */
    /*                                                                      */
    /* P term: target angular rate = AttKp × attitude error (rad).         */
    /* D term: provided by AngVel — same structure as the lateral channels: */
    /*   omega_tgt = clamp(Kp × error, ±MaxAttRate)                        */
    /*   omega_err = omega_tgt - AngVel                                     */
    /*   duration  = |omega_err| / RotAccel                                 */
    /*                                                                      */
    /* Deadband: attitude corrections are skipped when all three errors are */
    /* below AttDeadband_deg AND the vehicle is not spinning faster than    */
    /* SpinThreshold_rads on any axis.  The angle deadband breaks the       */
    /* lateral↔attitude limit cycle: lateral burns disturb attitude ~0.5–1°;*/
    /* without it the controller fires corrective burns every cycle, and    */
    /* those burns couple back into lateral and roll, sustaining the        */
    /* oscillation. The separate, much tighter spin check exists because    */
    /* the angle deadband alone lets a small *sustained* residual rate      */
    /* (e.g. left over from an earlier correction's overshoot, not an       */
    /* active burn) silently accumulate into a large angle error over many  */
    /* seconds before the angle ever crosses the deadband edge. With angle  */
    /* error near zero, AttitudeAxis's own omega_tgt = AttKp × error comes  */
    /* out near zero too, so this override's correction is naturally a      */
    /* small rate-damping nudge, not a large shove — it doesn't reintroduce */
    /* the angle deadband's chatter problem, it just stops residual spin    */
    /* from going uncorrected indefinitely.                                 */
    {
        const float d2r = 0.01745329f;  /* π / 180 */

        /* Attitude deadband by phase:
        **   LAT_CORR  2× (4°): lateral burns disturb attitude ~0.5-2°; a wider
        **             deadband lets those perturbations coast rather than triggering
        **             T08-T15 corrections whose -Z coupling drives the vehicle back.
        **   APPROACH  0.5× (1°): tighten for nose-on-port precision.
        **   HOLD      1× (2°): baseline. */
        float       att_db_deg;
        if      (phase == GNC_PHASE_APPROACH) att_db_deg = p->AttDeadband_deg * 0.5f;
        else if (phase == GNC_PHASE_CORRECT)  att_db_deg = p->AttDeadband_deg * 2.0f;
        else                                   att_db_deg = p->AttDeadband_deg;
        float       db_rad   = att_db_deg * d2r;
        float       spin_th  = p->SpinThreshold_rads;
        bool        spinning = (tlm->AngVel_X >  spin_th || tlm->AngVel_X < -spin_th ||
                                tlm->AngVel_Y >  spin_th || tlm->AngVel_Y < -spin_th ||
                                tlm->AngVel_Z >  spin_th || tlm->AngVel_Z < -spin_th);
        bool        in_db    = (db_rad > 0.0f &&
                                tlm->PitchError_deg * d2r >  -db_rad &&
                                tlm->PitchError_deg * d2r <   db_rad &&
                                tlm->YawError_deg   * d2r >  -db_rad &&
                                tlm->YawError_deg   * d2r <   db_rad &&
                                tlm->RollError_deg  * d2r >  -db_rad &&
                                tlm->RollError_deg  * d2r <   db_rad);
        /* Skip attitude correction only when all errors are small AND not spinning */
        if (in_db && !spinning) goto attitude_done;

        GNC_APP_AttitudeAxis(tlm->PitchError_deg, tlm->AngVel_X, p, kT, &ctrl.Tx, &max_att_dur);
        GNC_APP_AttitudeAxis(tlm->YawError_deg,   tlm->AngVel_Y, p, kT, &ctrl.Ty, &max_att_dur);
        GNC_APP_AttitudeAxis(tlm->RollError_deg,  tlm->AngVel_Z, p, kT, &ctrl.Tz, &max_att_dur);
    }
    attitude_done:;

    /*
    ** Resolve shared burn duration.
    **
    ** Translational and attitude channels may need different burn durations.
    ** Sending a single wrench with one duration means whichever channel drives
    ** max_dur over-fires all other channels.  Scale each group's commands down
    ** so the angular/linear impulse is correct regardless of which group wins.
    **
    ** Example: lateral needs 2 s (capped to 0.95 s), attitude needs 0.58 s.
    ** Without scaling, attitude torques would fire 0.95 s and impart 1.6× too
    ** much angular velocity, causing the growing oscillation seen in telemetry.
    */
    float max_dur = (max_trans_dur > max_att_dur) ? max_trans_dur : max_att_dur;
    if (max_dur > p->MaxBurnDuration) max_dur = p->MaxBurnDuration;

    if (max_dur > 0.0f)
    {
        if (max_att_dur > 0.0f && max_att_dur < max_dur)
        {
            float scale = max_att_dur / max_dur;
            ctrl.Tx *= scale;
            ctrl.Ty *= scale;
            ctrl.Tz *= scale;
        }
        if (max_trans_dur > 0.0f && max_trans_dur < max_dur)
        {
            float scale = max_trans_dur / max_dur;
            ctrl.Fx *= scale;
            ctrl.Fy *= scale;
            ctrl.Fz *= scale;
        }
    }

    /* === Lateral/attitude-coupling +Fz feed-forward (LAT_CORR only) ====== */
    /*                                                                        */
    /* When actively translating or re-orienting in LAT_CORR, the lateral    */
    /* and attitude burns disturb the vehicle; the resulting T08-T15         */
    /* corrections produce coupled −Z that drives the vehicle backward       */
    /* faster than Channel 1 can react.  Add proactive +Fz proportional to   */
    /* the coupling-causing commands so forward thrust fires in the same     */
    /* burst rather than 1 cycle later.                                      */
    /*                                                                        */
    /* Computed here, after duration scaling, so it reacts to the wrench      */
    /* actually sent to Unity this cycle rather than the pre-scale Channel 2  */
    /* values — the original version only ever saw Fx/Fy because it ran      */
    /* before Channel 3 had set Tx/Ty/Tz, silently missing the attitude half  */
    /* of the coupling whenever a large attitude error was also being        */
    /* corrected (as it commonly is on LAT_CORR entry).                       */
    /*                                                                        */
    /* Torque is converted back to a force-equivalent (÷ moment arm) so it    */
    /* combines with the lateral force magnitude on the same footing before   */
    /* coefficient 0.4 is applied. That coefficient is still the same rough   */
    /* empirical estimate as before (T08-T15 ≈1.7× stronger in −Z than       */
    /* T04-T07 are in +Z) — re-tune against telemetry if range still drifts   */
    /* during LAT_CORR.                                                       */
    if (phase == GNC_PHASE_CORRECT)
    {
        float lat_mag      = fabsf(ctrl.Fx) + fabsf(ctrl.Fy);
        float att_mag      = (fabsf(ctrl.Tx) + fabsf(ctrl.Ty) + fabsf(ctrl.Tz)) / GNC_RCS_MOMENT_ARM;
        float coupling_mag = lat_mag + att_mag;
        if (coupling_mag > 0.5f)
            ctrl.Fz += 0.4f * coupling_mag;
    }

    ctrl.duration_s = max_dur;
    return ctrl;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_ProcessWakeup — called every GNC_CYCLE_DT_S by SCH_LAB (5 Hz)    */
/*                                                                           */
/* 1. Snapshot the latest Unity telemetry under the mutex.                  */
/* 2. Run the phase state machine to determine the active GNC mode.         */
/* 3. Run the control law for that phase to compute a timed thruster cmd.   */
/* 4. Send the command to Unity over UDP.                                    */
/* 5. Log approach state (including phase) and publish HK to the SB.        */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void GNC_APP_ProcessWakeup(void)
{
    static const char *const PHASE_NAMES[] = {"IDLE", "LAT_CORR", "APPROACH", "DOCKED", "HOLD"};

    GNC_APP_UnityTlm_t tlm;
    bool               fresh;
    GNC_Control_t      ctrl = {0};

    GNC_APP_Data.HkTlm.WakeupCount++;

    /*
    ** Manage the parameter table.
    **
    ** CFE_TBL_Manage must be called periodically so the table service can process
    ** any pending requests from the ground (validate, activate, dump).  Without it,
    ** uplinked table images would queue but never get applied.
    **
    ** After Manage, re-acquire the pointer: if a new image was activated, the old
    ** pointer is stale and GetAddress returns CFE_TBL_INFO_UPDATED with the new one.
    */
    CFE_TBL_Manage(GNC_APP_Data.ParamTblHandle);
    {
        const GNC_ParamTbl_t *new_ptr = NULL;
        CFE_Status_t          tbl_st  = CFE_TBL_GetAddress((void **)&new_ptr,
                                                             GNC_APP_Data.ParamTblHandle);
        if (tbl_st == CFE_TBL_INFO_UPDATED)
        {
            GNC_APP_Data.ParamTblPtr = new_ptr;
            CFE_EVS_SendEvent(GNC_APP_TBL_UPD_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC_APP: parameter table updated — new gains active");
        }
        else if (tbl_st == CFE_SUCCESS)
        {
            GNC_APP_Data.ParamTblPtr = new_ptr;
        }
        else
        {
            /* Keep the last valid pointer and log the error; do not abort the cycle */
            CFE_EVS_SendEvent(GNC_APP_TBL_ERR_EID, CFE_EVS_EventType_ERROR,
                              "GNC_APP: CFE_TBL_GetAddress RC=0x%08lX — using stale params",
                              (unsigned long)tbl_st);
        }
    }

    /*
    ** Snapshot the shared telemetry buffer.
    ** The mutex lock is held for the minimum time possible — just long enough
    ** to copy the struct and clear the flag.  All processing happens on the
    ** local copy 'tlm', so the recv task is never blocked waiting for us.
    */
    OS_MutSemTake(GNC_APP_Data.TlmMutex);
    fresh = GNC_APP_Data.TlmFresh;
    if (fresh)
    {
        memcpy(&tlm, &GNC_APP_Data.LatestTlm, sizeof(tlm));
        GNC_APP_Data.TlmFresh = false;
    }
    OS_MutSemGive(GNC_APP_Data.TlmMutex);

    if (fresh)
    {
        /*
        ** Phase state machine — evaluate before running the control law.
        ** Emit an EVS event on every transition for ground visibility.
        */
        GNC_Phase_t prev_phase = GNC_APP_Data.Phase;
        GNC_Phase_t new_phase  = GNC_APP_SelectPhase(&tlm, prev_phase);

        if (new_phase != prev_phase)
        {
            CFE_EVS_SendEvent(GNC_APP_PHASE_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC mode: %s → %s (lat=%.2fm rng=%.2fm)",
                              PHASE_NAMES[prev_phase], PHASE_NAMES[new_phase],
                              (double)tlm.LateralOffset_m, (double)tlm.Range_m);
            GNC_APP_Data.Phase = new_phase;
        }

        /* Update HK so ground tools always reflect the current phase */
        GNC_APP_Data.HkTlm.Phase           = (uint8)GNC_APP_Data.Phase;
        GNC_APP_Data.HkTlm.ClosingSpeed_ms = tlm.ClosingSpeed_ms;
        GNC_APP_Data.HkTlm.LateralOffset_m = tlm.LateralOffset_m;
        GNC_APP_Data.HkTlm.TlmStaleSec     = 0;

        /* Run the control law for the active phase and command Unity */
        ctrl = GNC_APP_ComputeControl(&tlm, GNC_APP_Data.Phase);
        GNC_APP_SendCommand(&ctrl);
        GNC_APP_Data.HkTlm.CmdCount++;

        int inCorridor = (tlm.Flags & 0x1) != 0;
        int docked     = (tlm.Flags & 0x2) != 0;

        /* Compact format — the previous version regularly exceeded
        ** CFE_MISSION_EVS_MAX_MESSAGE_LENGTH (122) and got silently truncated
        ** mid-field. PYR = pitch/yaw/roll error (deg); W = AngVel_X/Y/Z
        ** (rad/s), added to actually see attitude rate instead of inferring it
        ** from how PYR changes cycle to cycle; C/D = InCorridor/Docked flags. */
        CFE_EVS_SendEvent(GNC_APP_WAKEUP_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "GNC #%u [%s] Rng=%.2f Spd=%.3f Lat=%.3f "
                          "PYR=%.1f/%.1f/%.1f W=%.3f/%.3f/%.3f C%dD%d "
                          "F=%.0f/%.0f/%.0f T=%.0f/%.0f/%.0f D=%.2f",
                          (unsigned int)GNC_APP_Data.HkTlm.WakeupCount,
                          PHASE_NAMES[GNC_APP_Data.Phase],
                          (double)tlm.Range_m,
                          (double)tlm.ClosingSpeed_ms,
                          (double)tlm.LateralOffset_m,
                          (double)tlm.PitchError_deg,
                          (double)tlm.YawError_deg,
                          (double)tlm.RollError_deg,
                          (double)tlm.AngVel_X,
                          (double)tlm.AngVel_Y,
                          (double)tlm.AngVel_Z,
                          inCorridor, docked,
                          (double)ctrl.Fx, (double)ctrl.Fy, (double)ctrl.Fz,
                          (double)ctrl.Tx, (double)ctrl.Ty, (double)ctrl.Tz,
                          (double)ctrl.duration_s);
    }
    else
    {
        /* Send a zero command — Unity coasts and keeps the cFS timeout alive */
        GNC_Control_t coast = {0};
        GNC_APP_SendCommand(&coast);
        GNC_APP_Data.HkTlm.TlmStaleSec++;

        CFE_EVS_SendEvent(GNC_APP_WAKEUP_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "GNC #%u | waiting for Unity telemetry...",
                          (unsigned int)GNC_APP_Data.HkTlm.WakeupCount);
    }

    /* Publish HK telemetry to the SB (TO_LAB will forward it to any ground system) */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(GNC_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(GNC_APP_Data.HkTlm.TelemetryHeader), true);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static bool GNC_APP_VerifyCmdLength(const CFE_MSG_Message_t *MsgPtr, size_t expected)
{
    size_t actual;
    CFE_MSG_GetSize(MsgPtr, &actual);
    if (actual != expected)
    {
        CFE_EVS_SendEvent(GNC_APP_CMD_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: cmd len err — expected %zu got %zu", expected, actual);
        GNC_APP_Data.HkTlm.CmdErrCount++;
        return false;
    }
    return true;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_ProcessCmd — ground command dispatcher                            */
/*                                                                           */
/* Validates command length, increments the appropriate counter, and         */
/* dispatches to the per-command handler.  All state changes happen here     */
/* under the main-task context; no mutex is required for GNC_APP_Data fields */
/* accessed only by this task (Phase, AbortLatch, counters).                 */
/*                                                                           */
/* HOLD  — enters HOLD phase; guidance is inhibited (station-keep only)     */
/*         until GO is received.  Models Dragon-style hold-point GO/NO-GO.  */
/* GO    — releases HOLD or AbortLatch; resumes guidance from CORRECT so    */
/*         SelectPhase can re-evaluate the lateral gate on the next wakeup. */
/* ABORT — forces IDLE, sets AbortLatch, sends immediate coast to Unity.    */
/*         Guidance stays inhibited until a subsequent GO.                  */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void GNC_APP_ProcessCmd(CFE_SB_Buffer_t *MsgBuf)
{
    CFE_MSG_FcnCode_t fc = 0;
    CFE_MSG_GetFcnCode(&MsgBuf->Msg, &fc);

    switch (fc)
    {
        /* ---------------------------------------------------------------- */
        case GNC_APP_NOOP_CC:
            if (!GNC_APP_VerifyCmdLength(&MsgBuf->Msg, sizeof(GNC_APP_NoopCmd_t))) break;
            GNC_APP_Data.HkTlm.CmdCount++;
            CFE_EVS_SendEvent(GNC_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC_APP: NOOP (ver 1.0)");
            break;

        /* ---------------------------------------------------------------- */
        case GNC_APP_RESET_COUNTERS_CC:
            if (!GNC_APP_VerifyCmdLength(&MsgBuf->Msg, sizeof(GNC_APP_ResetCountersCmd_t))) break;
            GNC_APP_Data.HkTlm.CmdCount           = 0;
            GNC_APP_Data.HkTlm.CmdErrCount        = 0;
            GNC_APP_Data.HkTlm.UdpPacketsReceived  = 0;
            CFE_EVS_SendEvent(GNC_APP_RST_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC_APP: counters reset");
            break;

        /* ---------------------------------------------------------------- */
        case GNC_APP_HOLD_CC:
        {
            if (!GNC_APP_VerifyCmdLength(&MsgBuf->Msg, sizeof(GNC_APP_HoldCmd_t))) break;
            GNC_APP_Data.HkTlm.CmdCount++;
            GNC_APP_Data.Phase          = GNC_PHASE_HOLD;
            GNC_APP_Data.HkTlm.Phase    = (uint8)GNC_PHASE_HOLD;
            /* Range read without mutex — acceptable for an EVS log message and
            ** as the axial hold-position target (next wakeup cycle refines it) */
            float rng = GNC_APP_Data.LatestTlm.Range_m;
            GNC_APP_Data.HoldRange_m    = rng;
            CFE_EVS_SendEvent(GNC_APP_HOLD_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC_APP: HOLD — station-keep at %.2f m (await GO)", (double)rng);
            break;
        }

        /* ---------------------------------------------------------------- */
        case GNC_APP_GO_CC:
        {
            if (!GNC_APP_VerifyCmdLength(&MsgBuf->Msg, sizeof(GNC_APP_GoCmd_t))) break;

            bool was_hold  = (GNC_APP_Data.Phase == GNC_PHASE_HOLD);
            bool was_abort = GNC_APP_Data.AbortLatch;

            if (!was_hold && !was_abort)
            {
                GNC_APP_Data.HkTlm.CmdErrCount++;
                CFE_EVS_SendEvent(GNC_APP_GO_INF_EID, CFE_EVS_EventType_ERROR,
                                  "GNC_APP: GO rejected — guidance already active (phase=%u)",
                                  (unsigned int)GNC_APP_Data.Phase);
                break;
            }

            GNC_APP_Data.HkTlm.CmdCount++;
            GNC_APP_Data.AbortLatch     = false;
            GNC_APP_Data.Phase          = GNC_PHASE_CORRECT;  /* SelectPhase promotes if on-axis */
            GNC_APP_Data.HkTlm.Phase    = (uint8)GNC_PHASE_CORRECT;
            if (was_abort)
            {
                /* ABORT+GO implies a scenario reset — re-arm all autonomous hold points */
                GNC_APP_Data.HoldPt1Armed = true;
                GNC_APP_Data.HoldPt2Armed = true;
            }
            CFE_EVS_SendEvent(GNC_APP_GO_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC_APP: GO — guidance ENABLED, starting from CORRECT");
            break;
        }

        /* ---------------------------------------------------------------- */
        case GNC_APP_ABORT_CC:
            if (!GNC_APP_VerifyCmdLength(&MsgBuf->Msg, sizeof(GNC_APP_AbortCmd_t))) break;
            GNC_APP_Data.HkTlm.CmdCount++;
            GNC_APP_Data.Phase          = GNC_PHASE_IDLE;
            GNC_APP_Data.AbortLatch     = true;
            GNC_APP_Data.HkTlm.Phase    = (uint8)GNC_PHASE_IDLE;
            { GNC_Control_t coast = {0}; GNC_APP_SendCommand(&coast); }  /* immediate coast */
            CFE_EVS_SendEvent(GNC_APP_ABORT_INF_EID, CFE_EVS_EventType_CRITICAL,
                              "GNC_APP: *** ABORT *** all thrust inhibited — send GO to resume");
            break;

        /* ---------------------------------------------------------------- */
        default:
            GNC_APP_Data.HkTlm.CmdErrCount++;
            CFE_EVS_SendEvent(GNC_APP_CMD_CODE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "GNC_APP: invalid command code %u", (unsigned int)fc);
            break;
    }
}
