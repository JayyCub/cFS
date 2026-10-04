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
            case SIM_IO_SIM_STATE_MID:
            {
                /* Hardware readings + link watchdog; NAV's solution for the
                ** same Seq follows and runs the control cycle. */
                size_t Size = 0;
                CFE_MSG_GetSize(&MsgBuf->Msg, &Size);
                if (Size == sizeof(SIM_IO_SimStateMsg_t))
                    GNC_APP_ProcessSimState((const SIM_IO_SimStateMsg_t *)MsgBuf);
                break;
            }
            case NAV_SOLUTION_MID:
            {
                size_t Size = 0;
                CFE_MSG_GetSize(&MsgBuf->Msg, &Size);
                if (Size == sizeof(NAV_SolutionMsg_t))
                    GNC_APP_ProcessNavSolution((const NAV_SolutionMsg_t *)MsgBuf);
                break;
            }
            case SIM_IO_THRUSTER_CMD_MID:
            {
                size_t Size = 0;
                CFE_MSG_GetSize(&MsgBuf->Msg, &Size);
                if (Size == sizeof(SIM_IO_ThrusterCmdMsg_t))
                    GNC_APP_ProcessThrusterCmd((const SIM_IO_ThrusterCmdMsg_t *)MsgBuf);
                break;
            }
            case GNC_APP_SEND_HK_MID:
                GNC_APP_SendHk();
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
/*                                                                           */
/* GNC_APP_ValidateParamTbl — CFE_TBL load-time validator                    */
/*                                                                           */
/* Registered with CFE_TBL_Register so a bad table image (corrupt uplink,   */
/* hand-edited .tbl with a typo) is rejected by CFE_TBL_Load/Manage instead  */
/* of silently becoming the active table. Checks only the values every      */
/* other calculation divides by or that would make burns nonsensical if     */
/* zero/negative/inverted — not a full range check on every gain.           */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static int32 GNC_APP_ValidateParamTbl(void *TblPtr)
{
    const GNC_ParamTbl_t *p = (const GNC_ParamTbl_t *)TblPtr;

    if (p->VehicleMass <= 0.0f)
    {
        CFE_EVS_SendEvent(GNC_APP_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: table rejected — VehicleMass %g <= 0", (double)p->VehicleMass);
        return GNC_APP_TBL_VALIDATE_ERR;
    }
    {
        int i;
        for (i = 0; i < 3; i++)
        {
            if (!(p->Inertia_kgm2[i] > 0.0f))
            {
                CFE_EVS_SendEvent(GNC_APP_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "GNC_APP: table rejected — Inertia_kgm2[%d] %g <= 0", i,
                                  (double)p->Inertia_kgm2[i]);
                return GNC_APP_TBL_VALIDATE_ERR;
            }
        }
    }
    if (!(p->AxialBrakeAccel_mss > 0.0f))
    {
        CFE_EVS_SendEvent(GNC_APP_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: table rejected — AxialBrakeAccel_mss %g <= 0", (double)p->AxialBrakeAccel_mss);
        return GNC_APP_TBL_VALIDATE_ERR;
    }
    if (p->ManualAccel_mss < 0.0f || p->ManualRate_rads < 0.0f || p->ManualRateDeadband_rads < 0.0f)
    {
        CFE_EVS_SendEvent(GNC_APP_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: table rejected — negative manual-control limit");
        return GNC_APP_TBL_VALIDATE_ERR;
    }
    /* SettleCounter is published as a uint16 in the STATE packet */
    if (p->EntrySettleCycles > 0xFFFF)
    {
        CFE_EVS_SendEvent(GNC_APP_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: table rejected — EntrySettleCycles %lu > 65535",
                          (unsigned long)p->EntrySettleCycles);
        return GNC_APP_TBL_VALIDATE_ERR;
    }
    if (p->LatVelAtPort > 1)
    {
        CFE_EVS_SendEvent(GNC_APP_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: table rejected — LatVelAtPort %lu is not 0 or 1",
                          (unsigned long)p->LatVelAtPort);
        return GNC_APP_TBL_VALIDATE_ERR;
    }
    if (p->ProfileCount < 1 || p->ProfileCount > GNC_PROFILE_MAX)
    {
        CFE_EVS_SendEvent(GNC_APP_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: table rejected — ProfileCount %lu outside 1..%d",
                          (unsigned long)p->ProfileCount, GNC_PROFILE_MAX);
        return GNC_APP_TBL_VALIDATE_ERR;
    }
    {
        uint32 i;
        for (i = 0; i < p->ProfileCount; i++)
        {
            if (!(p->ProfileMaxClose_ms[i] > 0.0f) || (i > 0 && !(p->ProfileRange_m[i] > p->ProfileRange_m[i - 1])))
            {
                CFE_EVS_SendEvent(GNC_APP_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "GNC_APP: table rejected — approach profile point %lu (ranges must ascend, "
                                  "speeds be positive)", (unsigned long)i);
                return GNC_APP_TBL_VALIDATE_ERR;
            }
        }
    }
    if (!(p->AttCtrlAccel_rads2 > 0.0f) || !(p->AttRateDeadband_rads >= 0.0f) || !(p->FfReleaseDv_ms > 0.0f) ||
        !(p->ProfileGuideFrac > 0.0f && p->ProfileGuideFrac <= 1.0f))
    {
        CFE_EVS_SendEvent(GNC_APP_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: table rejected — attitude accel / rate deadband / ff release / guide fraction");
        return GNC_APP_TBL_VALIDATE_ERR;
    }
    if (!(p->CamDeltaV_ms > 0.0f) || !(p->CamMaxDeltaV_ms >= p->CamDeltaV_ms) ||
        !(p->PassiveSafeStart_s >= 0.0f) || !(p->PassiveSafeHorizon_s > p->PassiveSafeStart_s))
    {
        CFE_EVS_SendEvent(GNC_APP_TBL_VAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: table rejected — abort manoeuvre / passive-safety parameters");
        return GNC_APP_TBL_VALIDATE_ERR;
    }

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
CFE_Status_t GNC_APP_Init(void)
{
    CFE_Status_t status;

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
    CFE_MSG_Init(CFE_MSG_PTR(GNC_APP_Data.ActReqMsg.TelemetryHeader),
                 CFE_SB_ValueToMsgId(RCS_ACT_REQ_MID),
                 sizeof(GNC_APP_Data.ActReqMsg));
    CFE_MSG_Init(CFE_MSG_PTR(GNC_APP_Data.StateTlm.TelemetryHeader),
                 CFE_SB_ValueToMsgId(GNC_APP_STATE_TLM_MID),
                 sizeof(GNC_APP_Data.StateTlm));
    CFE_MSG_Init(CFE_MSG_PTR(GNC_APP_Data.LcSampleCmd.CommandHeader),
                 CFE_SB_ValueToMsgId(LC_SAMPLE_AP_MID),
                 sizeof(GNC_APP_Data.LcSampleCmd));
    GNC_APP_Data.LcSampleCmd.Payload.StartIndex = GNC_APP_LC_CYCLE_AP_FIRST;
    GNC_APP_Data.LcSampleCmd.Payload.EndIndex   = GNC_APP_LC_CYCLE_AP_LAST;
    GNC_APP_Data.LcSampleCmd.Payload.UpdateAge  = 0;

    /* 3. Create SB pipe and subscribe: NAV solution (the cycle trigger), the
    **    sim hardware readings, RCS's answer, 1 Hz HK request, ground commands. */
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

    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(SIM_IO_SIM_STATE_MID), GNC_APP_Data.CmdPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GNC_APP_SUB_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: subscribe to SIM_STATE failed, RC=0x%08lX", (unsigned long)status);
        return status;
    }

    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(SIM_IO_THRUSTER_CMD_MID), GNC_APP_Data.CmdPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GNC_APP_SUB_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: subscribe to THRUSTER_CMD failed, RC=0x%08lX", (unsigned long)status);
        return status;
    }

    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(NAV_SOLUTION_MID), GNC_APP_Data.CmdPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GNC_APP_SUB_ERR_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: subscribe to NAV_SOLUTION failed, RC=0x%08lX", (unsigned long)status);
        return status;
    }

    /*
    ** 4. Register and load the GNC parameter table.
    **
    ** CFE_TBL_Register allocates a slot in the table registry.  The table name
    ** is scoped to this app: "GNC_APP.ParamTbl" as seen by ground tools.
    ** CFE_TBL_OPT_DEFAULT = single-buffer, load from file.
    ** GNC_APP_ValidateParamTbl range-checks the values every control-law
    ** division depends on (mass, thruster force, accel constants, burn
    ** duration bounds) — a bad image is rejected rather than silently loaded.
    **
    ** CFE_TBL_Load fills the buffer from the compiled binary installed to /cf/.
    ** CFE_TBL_GetAddress hands us a pointer to the live buffer — this pointer
    ** remains valid until the next CFE_TBL_Manage call that swaps in a new image.
    */
    status = CFE_TBL_Register(&GNC_APP_Data.ParamTblHandle,
                               GNC_APP_PARAM_TBL_NAME,
                               sizeof(GNC_ParamTbl_t),
                               CFE_TBL_OPT_DEFAULT,
                               GNC_APP_ValidateParamTbl);
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
                      "GNC_APP initialized (lock-step on NAV_SOLUTION). "
                      "Guidance INHIBITED — send GO to start.");

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
static GNC_Phase_t GNC_APP_SelectPhase(const GNC_APP_Input_t *tlm, GNC_Phase_t current, float dt)
{
    /* Docked check is unconditional — highest priority inhibit. The capture
    ** latch switch is a mechanism sensor, not a navigation estimate. */
    if (tlm->Captured)
        return GNC_PHASE_DOCKED;

    /* Once docked, stay docked until the scenario is reset (cFS restart) */
    if (current == GNC_PHASE_DOCKED)
        return GNC_PHASE_DOCKED;

    /*
    ** Crew manual takeover. Any hand-controller deflection past
    ** GNC_HC_TAKEOVER_THRESHOLD hands control to the crew from ANY other
    ** undocked phase — including IDLE with the abort latch set, so the crew can
    ** always fly — and MANUAL stays latched (releasing the sticks just holds
    ** rates) until the ground sends GO (autopilot resumes at CORRECT) or ABORT.
    */
    if (current == GNC_PHASE_MANUAL)
        return GNC_PHASE_MANUAL;
    {
        int i;
        for (i = 0; i < 3; i++)
        {
            if (fabsf(tlm->Thc[i]) > GNC_HC_TAKEOVER_THRESHOLD || fabsf(tlm->Rhc[i]) > GNC_HC_TAKEOVER_THRESHOLD)
            {
                GNC_APP_Data.SettleCounter = 0;
                return GNC_PHASE_MANUAL;
            }
        }
    }

    /*
    ** DEPART (the abort manoeuvre) runs to completion: GNC_APP_ProcessNavSolution
    ** ends it once the retreat is done and the free drift is passively safe.
    ** Only the crew (above) can take it over.
    */
    if (current == GNC_PHASE_DEPART)
        return GNC_PHASE_DEPART;

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
    ** Every gate below judges NAV's relative solution. While it is unusable
    ** (LIDAR not yet converged, or lost long enough for the covariance to grow
    ** past NAV's limits) there is nothing to judge: hold the phase and restart
    ** the settle count. ComputeControl coasts translation meanwhile.
    */
    if (!tlm->RelNavValid || !tlm->AttValid || !tlm->RateValid)
    {
        GNC_APP_Data.SettleCounter = 0;
        return next;
    }

    /*
    ** Lateral offset gate with hysteresis.
    **
    ** Entry (CORRECT -> APPROACH) uses a fixed absolute threshold: initial
    ** convergence should mean actually near the axis, not just "within some
    ** fraction of however forgiving the corridor happens to be at this range"
    ** (a corridor-relative entry gate let LAT_CORR hand off to APPROACH 4.46 m
    ** off-axis at 33 m range).
    **
    ** RETUNED 2026-07-18: position-only was a single-sample check — the instant
    ** LateralOffset_m ticked under the threshold, APPROACH committed and started
    ** axial closure (Channel 1 floors at MinCloseSpeed) even though lateral
    ** velocity and attitude rate were often still actively changing at that
    ** exact moment (a 2026-07-18 run entered APPROACH at Lat=0.10m while attitude
    ** was still visibly drifting on all three axes). That's what "slow and smooth
    ** to the centerline, get still, THEN approach" was asking for and position-only
    ** couldn't deliver — closure and residual attitude motion ended up fighting
    ** each other from the first APPROACH cycle. Entry now also requires lateral
    ** speed and attitude rate under their own thresholds, held for
    ** EntrySettleCycles consecutive cycles (SettleCounter below) — a real
    ** "stopped and steady," not just "position happened to cross a line."
    **
    ** Revert (APPROACH -> CORRECT) stays corridor-relative: "allowed" is the
    ** radius of the real docking corridor cone at the current range, so once
    ** underway, the gate tightens automatically as the vehicle closes in
    ** instead of using one constant across the whole approach.
    */
    {
        const GNC_ParamTbl_t *p = GNC_APP_Data.ParamTblPtr;

        if (next == GNC_PHASE_CORRECT)
        {
            bool posOk = tlm->LateralOffset_m < p->LatEntryThreshold_m;
            bool velOk = fabsf(tlm->LatVel_X) < p->LatEntryVelMax_ms &&
                         fabsf(tlm->LatVel_Y) < p->LatEntryVelMax_ms;
            bool rateOk = fabsf(tlm->AngVel_X) < p->AttEntryRateMax_rads &&
                          fabsf(tlm->AngVel_Y) < p->AttEntryRateMax_rads &&
                          fabsf(tlm->AngVel_Z) < p->AttEntryRateMax_rads;

            if (posOk && velOk && rateOk)
            {
                GNC_APP_Data.SettleCounter++;
                if (GNC_APP_Data.SettleCounter >= p->EntrySettleCycles)
                {
                    GNC_APP_Data.SettleCounter = 0;
                    return GNC_PHASE_APPROACH;
                }
            }
            else
            {
                GNC_APP_Data.SettleCounter = 0;
            }
        }

        float allowed = tlm->Range_m * tanf(p->ConeHalfAngle_deg * GNC_DEG2RAD);
        if (allowed < p->MinAllowedLateral_m) allowed = p->MinAllowedLateral_m;

        if (next == GNC_PHASE_APPROACH && tlm->LateralOffset_m > allowed * p->CorridorMarginOut)
        {
            GNC_APP_Data.SettleCounter = 0;
            return GNC_PHASE_CORRECT;
        }
        /* Attitude revert — same idea as the corridor check above, but for
        ** rotation: nothing previously stopped a vehicle that stayed laterally
        ** centered while tilting/yawing hard from continuing to close range. */
        float maxAttErr = fabsf(tlm->PitchError_deg);
        if (fabsf(tlm->YawError_deg)  > maxAttErr) maxAttErr = fabsf(tlm->YawError_deg);
        if (fabsf(tlm->RollError_deg) > maxAttErr) maxAttErr = fabsf(tlm->RollError_deg);

        if (next == GNC_PHASE_APPROACH && maxAttErr > p->AttRevertThreshold_deg)
        {
            GNC_APP_Data.SettleCounter = 0;
            return GNC_PHASE_CORRECT;
        }
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
        /* Distance needed to brake to a stop from the current closing speed:
        ** v²/(2a) at AxialBrakeAccel_mss — a guidance-planning estimate of the
        ** axial deceleration the RCS can sustain (it sizes no burns; the RCS app
        ** does that from its thruster table) — plus two cycles of travel for the
        ** decision and the first pulse. Realism phase 5; the 1 Hz-era empirical
        ** v²/a braked early (twice the distance) at today's 5 Hz cycle. */
        float brake_dist = (v * v) / (2.0f * p->AxialBrakeAccel_mss) + 2.0f * v * dt;

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
/* Small quaternion helpers (x, y, z, w; Hamilton product, v' = q v q*) —    */
/* the same conventions as NAV and Unity.                                    */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void GNC_QuatRotate(const float q[4], const float v[3], float out[3])
{
    float t[3], ut[3];
    t[0]  = 2.0f * (q[1] * v[2] - q[2] * v[1]);
    t[1]  = 2.0f * (q[2] * v[0] - q[0] * v[2]);
    t[2]  = 2.0f * (q[0] * v[1] - q[1] * v[0]);
    ut[0] = q[1] * t[2] - q[2] * t[1];
    ut[1] = q[2] * t[0] - q[0] * t[2];
    ut[2] = q[0] * t[1] - q[1] * t[0];
    out[0] = v[0] + q[3] * t[0] + ut[0];
    out[1] = v[1] + q[3] * t[1] + ut[1];
    out[2] = v[2] + q[3] * t[2] + ut[2];
}

static void GNC_QuatConj(const float q[4], float out[4])
{
    out[0] = -q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = q[3];
}

/* Docking-frame vector -> body frame: v_B = q_B_L q_L_D v_D */
static void GNC_DockToBody(const GNC_APP_Input_t *tlm, const float v_D[3], float v_B[3])
{
    float v_L[3], q_B_L[4];
    GNC_QuatRotate(tlm->DockQuat_L, v_D, v_L);
    GNC_QuatConj(tlm->AttQuat_L, q_B_L);
    GNC_QuatRotate(q_B_L, v_L, v_B);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_ProfileLimit — approach envelope: max closing speed at a range    */
/* (piecewise linear between the table breakpoints, flat beyond the ends)    */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static float GNC_APP_ProfileLimit(float range, const GNC_ParamTbl_t *p)
{
    uint32 i, n = p->ProfileCount;

    if (range <= p->ProfileRange_m[0])
        return p->ProfileMaxClose_ms[0];
    for (i = 1; i < n; i++)
    {
        if (range <= p->ProfileRange_m[i])
        {
            float f = (range - p->ProfileRange_m[i - 1]) / (p->ProfileRange_m[i] - p->ProfileRange_m[i - 1]);
            return p->ProfileMaxClose_ms[i - 1] + f * (p->ProfileMaxClose_ms[i] - p->ProfileMaxClose_ms[i - 1]);
        }
    }
    return p->ProfileMaxClose_ms[n - 1];
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_DriftSafeRange — passive-safety check                             */
/*                                                                           */
/* Where would the docking port go if every thruster stopped now? Closed-    */
/* form Clohessy-Wiltshire (Hill) free drift from NAV's CoM state, attitude  */
/* held, sampled every GNC_DRIFT_STEP_S between PassiveSafeStart_s and       */
/* PassiveSafeHorizon_s. Returns the closest approach of the chaser port to  */
/* the target port over that window (0 if NAV can't support the estimate).  */
/*   LVLH (Hill) x radial = +Y world, y along-track = -Z world,              */
/*               z cross-track = +X world                                    */
/*   x(t) = (4-3c)x0 + (s/n)vx0 + (2/n)(1-c)vy0                              */
/*   y(t) = 6(s-nt)x0 + y0 - (2/n)(1-c)vx0 + ((4s-3nt)/n)vy0                 */
/*   z(t) = c z0 + (s/n)vz0               (s = sin nt, c = cos nt)           */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static float GNC_APP_DriftSafeRange(const GNC_APP_Input_t *tlm, const GNC_ParamTbl_t *p)
{
    const double n = GNC_CW_MEAN_MOTION;
    double       x0, y0, z0, vx, vy, vz, off[3], best = 1e30, t;
    float        port_L[3];
    int          i;

    if (!tlm->RelNavValid || !tlm->AttValid)
        return 0.0f;

    /* port relative to the target port = CoM drift + this fixed offset */
    GNC_QuatRotate(tlm->DockQuat_L, tlm->PortPos_D, port_L);
    off[0] = (double)port_L[0] - tlm->RelPos_X;
    off[1] = (double)port_L[1] - tlm->RelPos_Y;
    off[2] = (double)port_L[2] - tlm->RelPos_Z;

    x0 = tlm->RelPos_Y;
    y0 = -tlm->RelPos_Z;
    z0 = tlm->RelPos_X;
    vx = tlm->RelVel_Y;
    vy = -tlm->RelVel_Z;
    vz = tlm->RelVel_X;

    for (t = p->PassiveSafeStart_s; t <= p->PassiveSafeHorizon_s; t += GNC_DRIFT_STEP_S)
    {
        double nt = n * t, sn = sin(nt), cn = cos(nt), x, y, z, w[3], d2 = 0.0;
        x    = (4.0 - 3.0 * cn) * x0 + (sn / n) * vx + (2.0 / n) * (1.0 - cn) * vy;
        y    = 6.0 * (sn - nt) * x0 + y0 - (2.0 / n) * (1.0 - cn) * vx + ((4.0 * sn - 3.0 * nt) / n) * vy;
        z    = cn * z0 + (sn / n) * vz;
        w[0] = z;
        w[1] = x;
        w[2] = -y;
        for (i = 0; i < 3; i++)
            d2 += (w[i] + off[i]) * (w[i] + off[i]);
        if (d2 < best)
            best = d2;
    }
    return (float)sqrt(best);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_LateralAxis — velocity change wanted on one lateral axis          */
/*                                                                           */
/* Target velocity from continuous position feedback (gain differs by        */
/* phase — gentler in APPROACH), minus current velocity. Inside the velocity */
/* deadband the axis coasts; outside it, the request is the part of the      */
/* error beyond the deadband. The CW feed-forward is no longer folded in     */
/* here (phase 5): it is accumulated and released on its own in             */
/* ComputeControl, so the deadband can't swallow it.                         */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static float GNC_APP_LateralAxis(float pos, float vel, GNC_Phase_t phase, const GNC_ParamTbl_t *p)
{
    /* Both phases actively hold the centerline — APPROACH uses a gentler gain
    ** (LatKp_Approach) and a looser velocity deadband so it coasts through
    ** noise instead of feeding the lateral/attitude coupling limit cycle,
    ** while CORRECT keeps it tight to converge inside LatEntryThreshold_m. */
    float kp    = (phase == GNC_PHASE_APPROACH) ? p->LatKp_Approach : p->LatKp;
    float v_tgt = -kp * pos;
    if (v_tgt >  p->MaxLatSpeed) v_tgt =  p->MaxLatSpeed;
    if (v_tgt < -p->MaxLatSpeed) v_tgt = -p->MaxLatSpeed;

    float v_err = v_tgt - vel;
    float db    = (phase == GNC_PHASE_APPROACH) ? p->LatVelDeadband_Approach_ms
                                                  : p->LatVelDeadband_ms;
    if (v_err >  db) return v_err - db;
    if (v_err < -db) return v_err + db;
    return 0.0f;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_PhasePlaneAxis — rate change wanted on one attitude axis          */
/*                                                                           */
/* Phase-plane control (realism phase 5). With e = |error| beyond the angle  */
/* deadband, the target rate toward zero error is the slowest of            */
/*   AttKp·e           linear near the deadband edge                         */
/*   sqrt(2·α·e)       the braking curve: the fastest rate the RCS can still */
/*                     stop from at the planned deceleration α               */
/*   MaxAttRate        slew cap                                              */
/* and a rate error under AttRateDeadband_rads is left alone (no chatter on  */
/* the minimum impulse bit). Inside the angle deadband the target is zero    */
/* rate, corrected only past SpinThreshold_rads, so a slow drift walks to    */
/* the edge and is turned around there — the classic RCS limit cycle,        */
/* sized by the deadband instead of by thruster chatter. Per axis.           */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static float GNC_APP_PhasePlaneAxis(float err_deg, float rate, float db_deg, const GNC_ParamTbl_t *p)
{
    float e     = (fabsf(err_deg) - db_deg) * GNC_DEG2RAD;
    float w_tgt = 0.0f;
    float rdb   = p->SpinThreshold_rads;
    float w_err;

    if (e > 0.0f)
    {
        float w  = p->AttKp * e;
        float wb = sqrtf(2.0f * p->AttCtrlAccel_rads2 * e);
        if (wb < w)
            w = wb;
        if (w > p->MaxAttRate)
            w = p->MaxAttRate;
        w_tgt = (err_deg > 0.0f) ? w : -w;
        rdb   = p->AttRateDeadband_rads;
    }

    w_err = w_tgt - rate;
    return (fabsf(w_err) <= rdb) ? 0.0f : w_err;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_CamDirection — unit retreat direction for the abort manoeuvre     */
/*                                                                           */
/* Away from the target port along the docking axis (−Z of the docking       */
/* frame) in body axes when attitude is known; otherwise straight back      */
/* along the body docking axis (−Z body), which needs no navigation at all. */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void GNC_APP_CamDirection(const GNC_APP_Input_t *tlm, float dir_B[3])
{
    static const float BACK[3] = {0.0f, 0.0f, -1.0f};

    if (tlm->AttValid)
    {
        GNC_DockToBody(tlm, BACK, dir_B);
    }
    else
    {
        dir_B[0] = BACK[0];
        dir_B[1] = BACK[1];
        dir_B[2] = BACK[2];
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_SanitizeControl — last-chance NaN/Inf guard before a control      */
/* output is logged or sent to the RCS.                                      */
/*                                                                           */
/* Backstops GNC_APP_ValidateParamTbl: a table value passing that check can  */
/* still combine with a particular telemetry state to divide out to Inf/NaN */
/* somewhere in the control law. Rather than let a non-finite request reach  */
/* the allocator, zero the whole request for this cycle and let the next     */
/* cycle try again from fresh telemetry. Latched so a persistently bad state */
/* logs once, not every cycle.                                               */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void GNC_APP_SanitizeControl(GNC_Control_t *ctrl)
{
    bool bad = !isfinite(ctrl->Px) || !isfinite(ctrl->Py) || !isfinite(ctrl->Pz) ||
               !isfinite(ctrl->Lx) || !isfinite(ctrl->Ly) || !isfinite(ctrl->Lz);

    if (bad && !GNC_APP_Data.BadCtrlLatched)
    {
        CFE_EVS_SendEvent(GNC_APP_BAD_CTRL_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: non-finite control output — commanding coast until it clears");
    }
    GNC_APP_Data.BadCtrlLatched = bad;

    if (bad)
        memset(ctrl, 0, sizeof(*ctrl));
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_ManualControl — crew hand controllers (MANUAL phase)              */
/*                                                                           */
/*   THC (translation): acceleration command — while deflected, request     */
/*        ManualAccel_mss × deflection of Δv per cycle on that body axis;   */
/*        released = coast (no automatic translation hold).                  */
/*   RHC (rotation): rate command with rate hold — target rate =            */
/*        ManualRate_rads × deflection; released = target 0, so the vehicle */
/*        damps its own rotation (what the Unity-side RateDamping hack used  */
/*        to fake by writing Rigidbody velocities directly).                 */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void GNC_APP_ManualControl(const GNC_APP_Input_t *tlm, const GNC_ParamTbl_t *p, float dt,
                                  float dv[3], float dw[3])
{
    const float rate[3] = {tlm->AngVel_X, tlm->AngVel_Y, tlm->AngVel_Z};
    int         i;

    for (i = 0; i < 3; i++)
    {
        float thc = tlm->Thc[i];
        float rhc = tlm->Rhc[i];
        if (thc >  1.0f) thc =  1.0f;
        if (thc < -1.0f) thc = -1.0f;
        if (rhc >  1.0f) rhc =  1.0f;
        if (rhc < -1.0f) rhc = -1.0f;

        dv[i] = thc * p->ManualAccel_mss * dt;

        /* Rate command needs the gyro: without it there is no rate to hold */
        if (tlm->RateValid)
        {
            float w_err = rhc * p->ManualRate_rads - rate[i];
            dw[i]       = (fabsf(w_err) < p->ManualRateDeadband_rads) ? 0.0f : w_err;
        }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_ComputeControl — phase-dispatched guidance → impulse request      */
/*                                                                           */
/* Output is the linear and angular impulse (body frame) GNC wants imparted  */
/* this cycle: P = m·Δv, L = I·Δω. WHICH thrusters fire and for HOW LONG is  */
/* the RCS app's job (allocation + PWM against its thruster table).          */
/*                                                                           */
/* Translation is worked out in the docking frame D (X/Y lateral, +Z toward  */
/* the port) and rotated into body axes with NAV's attitude (realism phase  */
/* 5; before, body and world axes were assumed to coincide).                 */
/*   Channel 1 — Axial (D Z)                                                 */
/*     APPROACH : v_tgt = clamp(AxialKp × range, MinCloseSpeed, cap), and   */
/*                at most ProfileGuideFrac of the approach envelope;         */
/*                cap = MaxCloseSpeed before HoldPoint1_m, then _Inner       */
/*     CORRECT  : v_tgt = 0 (station-keep)                                   */
/*     HOLD     : v_tgt = clamp(AxialHoldKp × (range − HoldRange_m),         */
/*                ±MaxHoldSpeed)                                             */
/*   Channel 2 — Lateral (D X, Y): GNC_APP_LateralAxis                       */
/*   CW feed-forward — accumulated per D axis, released at FfReleaseDv_ms    */
/*   Channel 3 — Attitude (all active phases): GNC_APP_PhasePlaneAxis        */
/*   MANUAL — hand controllers only (GNC_APP_ManualControl)                  */
/*   DEPART — retreat along −Z_D (GNC_APP_CamDirection), attitude held       */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static GNC_Control_t GNC_APP_ComputeControl(const GNC_APP_Input_t *tlm, GNC_Phase_t phase, float dt)
{
    GNC_Control_t         ctrl = {0};
    const GNC_ParamTbl_t *p    = GNC_APP_Data.ParamTblPtr;
    float                 dv_D[3] = {0.0f, 0.0f, 0.0f}; /* docking-frame Δv wanted this cycle (m/s) */
    float                 dv[3]   = {0.0f, 0.0f, 0.0f}; /* body-frame Δv wanted this cycle (m/s)    */
    float                 dw[3]   = {0.0f, 0.0f, 0.0f}; /* body-frame Δω wanted this cycle (rad/s)  */
    float                 att_db_deg;
    int                   i;

    if (phase != GNC_PHASE_CORRECT && phase != GNC_PHASE_APPROACH && phase != GNC_PHASE_HOLD)
        memset(GNC_APP_Data.FfAccum_D, 0, sizeof(GNC_APP_Data.FfAccum_D));

    /* IDLE and DOCKED: all thrust inhibited */
    if (phase == GNC_PHASE_IDLE || phase == GNC_PHASE_DOCKED)
        return ctrl;

    if (phase == GNC_PHASE_MANUAL)
    {
        GNC_APP_ManualControl(tlm, p, dt, dv, dw);
        goto to_impulse;
    }

    if (phase == GNC_PHASE_DEPART)
    {
        /* Retreat: whatever is left of the nominal CAM Δv; once that's done,
        ** keep stepping while the free drift isn't passively safe (and NAV can
        ** tell), up to CamMaxDeltaV_ms. GNC_APP_ProcessNavSolution ends DEPART. */
        float want = p->CamDeltaV_ms - GNC_APP_Data.CamDv_ms;
        float dir_B[3];
        /* Never ask for less than a step: a remainder below the minimum
        ** impulse bit would be dropped by RCS and the retreat would stall */
        if ((want > 0.0f && want < GNC_CAM_STEP_MS) ||
            (want < GNC_CAM_STEP_MS && !GNC_APP_Data.DriftSafe && tlm->RelNavValid))
            want = GNC_CAM_STEP_MS;
        if (GNC_APP_Data.CamDv_ms >= p->CamMaxDeltaV_ms || want < 0.0f)
            want = 0.0f;
        GNC_APP_CamDirection(tlm, dir_B);
        for (i = 0; i < 3; i++)
            dv[i] = dir_B[i] * want;
        GNC_APP_Data.CamLastCmd_ms = want;
        goto attitude;
    }

    /* === Channel 1 — Axial =========================================== */
    {
        float v_axial_tgt;
        if (phase == GNC_PHASE_APPROACH)
        {
            /* Outer cap before HoldPoint1_m has fired, tighter cap after —
            ** models the real-world profile of a faster outer approach speed
            ** and a slower, more cautious speed once inside the first hold.
            ** Never above ProfileGuideFrac of the approach envelope. */
            float cap   = GNC_APP_Data.HoldPt1Armed ? p->MaxCloseSpeed : p->MaxCloseSpeed_Inner;
            float guide = p->ProfileGuideFrac * GNC_APP_ProfileLimit(tlm->Range_m, p);
            v_axial_tgt = tlm->Range_m * p->AxialKp;
            if (v_axial_tgt < p->MinCloseSpeed) v_axial_tgt = p->MinCloseSpeed;
            if (v_axial_tgt > cap) v_axial_tgt = cap;
            if (v_axial_tgt > guide) v_axial_tgt = guide;
        }
        else if (phase == GNC_PHASE_HOLD)
        {
            /* Position feedback toward HoldRange_m (captured at HOLD entry) so
            ** drift accumulated during the hold is walked back out, not just
            ** velocity-damped. */
            float range_err = tlm->Range_m - GNC_APP_Data.HoldRange_m;
            v_axial_tgt = p->AxialHoldKp * range_err;
            if (v_axial_tgt >  p->MaxHoldSpeed) v_axial_tgt =  p->MaxHoldSpeed;
            if (v_axial_tgt < -p->MaxHoldSpeed) v_axial_tgt = -p->MaxHoldSpeed;
        }
        else /* CORRECT — station-keep axially */
        {
            v_axial_tgt = 0.0f;
        }

        /* ClosingSpeed_ms is the port velocity along D +Z (toward the port) */
        dv_D[2] = v_axial_tgt - tlm->ClosingSpeed_ms;
    }

    /* === Channel 2 — Lateral position + velocity (D X and Y) ========== */
    /* Steers on the docking-PORT-relative LatOffset_X/Y (not the chaser  */
    /* origin), so rotation that sweeps the port off-axis is seen. The    */
    /* velocity term is the port point velocity v_cm + ω×r from NAV       */
    /* (realism phase 4), or the CoM velocity if LatVelAtPort = 0.        */
    dv_D[0] = GNC_APP_LateralAxis(tlm->LatOffset_X, tlm->LatVel_X, phase, p);
    dv_D[1] = GNC_APP_LateralAxis(tlm->LatOffset_Y, tlm->LatVel_Y, phase, p);

    /* === CW feed-forward ============================================== */
    /* ClohessyWiltshire.cs applies the differential gravity to the chaser */
    /* every FixedUpdate; the equal-and-opposite Δv pre-cancels the drift. */
    /* LVLH mapping (same as ClohessyWiltshire.cs, +V-bar approach):       */
    /*   radial = +Y world   along-track = -Z world   cross-track = +X world */
    /* Hill/CW, x radial, y along-track (velocity dir), z cross-track:     */
    /*   a_rad = 3n²·r_rad + 2n·v_along, a_along = -2n·v_rad, a_cross = -n²·r_cross */
    /* The per-cycle Δv (~1e-5 m/s) is far below the minimum impulse bit,  */
    /* so it is accumulated per docking-frame axis and released as one     */
    /* request at FfReleaseDv_ms (realism phase 5: before, it was added    */
    /* inside the deadbanded lateral error and the V-bar Coriolis drift    */
    /* was never cancelled).                                               */
    if (tlm->RelNavValid)
    {
        const float n       = GNC_CW_MEAN_MOTION;
        const float n2      = n * n;
        const float a_rad   = 3.0f * n2 * tlm->RelPos_Y + 2.0f * n * (-tlm->RelVel_Z);
        const float a_along = -2.0f * n * tlm->RelVel_Y;
        const float a_cross = -n2 * tlm->RelPos_X;
        float       ff_L[3], ff_D[3], q_D_L[4];

        ff_L[0] = -a_cross * dt;
        ff_L[1] = -a_rad * dt;
        ff_L[2] = a_along * dt; /* -(a_world_z) = -(-a_along) */
        GNC_QuatConj(tlm->DockQuat_L, q_D_L);
        GNC_QuatRotate(q_D_L, ff_L, ff_D);
        for (i = 0; i < 3; i++)
        {
            GNC_APP_Data.FfAccum_D[i] += ff_D[i];
            if (fabsf(GNC_APP_Data.FfAccum_D[i]) >= p->FfReleaseDv_ms)
            {
                dv_D[i] += GNC_APP_Data.FfAccum_D[i];
                GNC_APP_Data.FfAccum_D[i] = 0.0f;
            }
        }
        GNC_DockToBody(tlm, dv_D, dv);
    }
    else
    {
        /* No usable relative solution (NAV not converged, or LIDAR lost long
        ** enough): coast in translation rather than steer on a guess. Attitude
        ** control below needs only the star tracker and gyro. */
        memset(GNC_APP_Data.FfAccum_D, 0, sizeof(GNC_APP_Data.FfAccum_D));
    }

attitude:
    /* === Channel 3 — Attitude (all active phases) ====================== */
    /* Phase-scaled angle deadband: the deadband breaks the lateral↔attitude */
    /* limit cycle.  CORRECT 2× (wide: lateral burns disturb attitude),      */
    /* APPROACH 0.5× (tight: nose-on-port precision), HOLD / DEPART 1×.      */
    if      (phase == GNC_PHASE_APPROACH) att_db_deg = p->AttDeadband_deg * 0.5f;
    else if (phase == GNC_PHASE_CORRECT)  att_db_deg = p->AttDeadband_deg * 2.0f;
    else                                   att_db_deg = p->AttDeadband_deg;

    if (!tlm->RateValid)
    {
        /* No gyro: neither the rate term nor a safe correction exists */
    }
    else if (!tlm->AttValid)
    {
        /* No attitude: hold rates at zero until the star tracker is back */
        const float w[3] = {tlm->AngVel_X, tlm->AngVel_Y, tlm->AngVel_Z};
        for (i = 0; i < 3; i++)
            dw[i] = (fabsf(w[i]) > p->SpinThreshold_rads) ? -w[i] : 0.0f;
    }
    else
    {
        dw[0] = GNC_APP_PhasePlaneAxis(tlm->PitchError_deg, tlm->AngVel_X, att_db_deg, p);
        dw[1] = GNC_APP_PhasePlaneAxis(tlm->YawError_deg,   tlm->AngVel_Y, att_db_deg, p);
        dw[2] = GNC_APP_PhasePlaneAxis(tlm->RollError_deg,  tlm->AngVel_Z, att_db_deg, p);
    }

to_impulse:
    ctrl.Px = p->VehicleMass * dv[0];
    ctrl.Py = p->VehicleMass * dv[1];
    ctrl.Pz = p->VehicleMass * dv[2];
    ctrl.Lx = p->Inertia_kgm2[0] * dw[0];
    ctrl.Ly = p->Inertia_kgm2[1] * dw[1];
    ctrl.Lz = p->Inertia_kgm2[2] * dw[2];

    GNC_APP_SanitizeControl(&ctrl);
    return ctrl;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_CheckActuatorHealth — axial closed-loop delivery check            */
/*                                                                           */
/* Compares the axial impulse the RCS app actually commanded on the         */
/* *previous* cycle (its THRUSTER_CMD AchievedLinImpulse — after saturation  */
/* and minimum-impulse-bit effects, so a legitimately clipped burn isn't a   */
/* false alarm) against the body +Z Δv the IMU accelerometer measured over  */
/* the cycle that burn covered (realism phase 4; before, the closing-speed   */
/* change, which rotation and gravity also move). A commanded burn with      */
/* almost no corresponding Δv is the signature of a failed/weak thruster or  */
/* a table-vs-vehicle mismatch.                                              */
/*                                                                           */
/* Measurement only (realism phase 3): GNC publishes the consecutive-cycle   */
/* streak in the STATE packet and takes no action itself. The decision —     */
/* how long a streak counts as an anomaly, in which phases, and what to do   */
/* about it — is LC actionpoint 1 → SC RTS 3 (HOLD), all table-driven, so    */
/* the flight rules can change without a GNC code change.                    */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void GNC_APP_CheckActuatorHealth(const GNC_APP_Input_t *tlm, const GNC_ParamTbl_t *p)
{
    /* Nothing to check: no THRUSTER_CMD matched the previous cycle, the Seq
       was discontinuous (sim time passed that no command covered), or the
       IMU gave no Δv this cycle. */
    if (!GNC_APP_Data.PrevBurnValid || !tlm->DvValid)
    {
        GNC_APP_Data.UnderDeliveryStreak = 0;
        return;
    }

    float predicted_dv = GNC_APP_Data.PrevAchievedPz_Ns / p->VehicleMass;

    /* Burn too small to expect a measurable effect above telemetry/physics
       noise — skip rather than risk a false positive. */
    if (fabsf(predicted_dv) < 0.005f)
    {
        GNC_APP_Data.UnderDeliveryStreak = 0;
        return;
    }

    float observed_dv        = tlm->DeltaV_B[2];
    float delivered_fraction = observed_dv / predicted_dv;  /* same-sign & near 1.0 is healthy */

    if (delivered_fraction < 0.5f)
    {
        if (GNC_APP_Data.UnderDeliveryStreak < 0xFFFF)
            GNC_APP_Data.UnderDeliveryStreak++;
    }
    else
    {
        GNC_APP_Data.UnderDeliveryStreak = 0;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_Flags — GNC_APP_FLAG_* bits shared by HK and STATE               */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static uint8 GNC_APP_Flags(void)
{
    uint8 f = 0;

    if (GNC_APP_Data.AbortLatch)     f |= GNC_APP_FLAG_ABORT_LATCH;
    if (GNC_APP_Data.HoldPt1Armed)   f |= GNC_APP_FLAG_HOLDPT1_ARMED;
    if (GNC_APP_Data.HoldPt2Armed)   f |= GNC_APP_FLAG_HOLDPT2_ARMED;
    if (GNC_APP_Data.BadCtrlLatched) f |= GNC_APP_FLAG_BAD_CTRL;
    if (GNC_APP_Data.NavInvalidLatched) f |= GNC_APP_FLAG_NAV_INVALID;
    if (GNC_APP_Data.HaveTlm)
    {
        f |= GNC_APP_FLAG_LINK_UP;
        if (GNC_APP_Data.InCorridor)          f |= GNC_APP_FLAG_IN_CORRIDOR;
        if (GNC_APP_Data.LatestIn.Captured)   f |= GNC_APP_FLAG_CONTACT;
    }
    return f;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_PublishState — per-cycle STATE packet + LC sample request         */
/*                                                                           */
/* STATE is the data channel for this cycle (ground console, DS flight data  */
/* recorder, LC watchpoints). The LC sample request right behind it makes LC */
/* evaluate the cycle-rate actionpoints against the watchpoint results this  */
/* STATE just produced — LC's pipe is FIFO, so the STATE is processed first. */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void GNC_APP_PublishState(const NAV_SolutionMsg_t *Msg, const GNC_APP_Input_t *tlm,
                                 const GNC_Control_t *ctrl)
{
    GNC_APP_StateTlm_t *st = &GNC_APP_Data.StateTlm;

    st->SimTime_s            = Msg->SimTime_s;
    st->Seq                  = Msg->Seq;
    st->Phase                = (uint8)GNC_APP_Data.Phase;
    st->Flags                = GNC_APP_Flags();
    st->UnderDeliveryStreak  = (uint16)GNC_APP_Data.UnderDeliveryStreak;
    st->SettleCounter        = (GNC_APP_Data.SettleCounter > 0xFFFF) ? 0xFFFF : (uint16)GNC_APP_Data.SettleCounter;
    st->Range_m              = tlm->Range_m;
    st->ClosingSpeed_ms      = tlm->ClosingSpeed_ms;
    st->LateralOffset_m      = tlm->LateralOffset_m;
    st->LatOffset_m[0]       = tlm->LatOffset_X;
    st->LatOffset_m[1]       = tlm->LatOffset_Y;
    st->AttErr_deg[0]        = tlm->PitchError_deg;
    st->AttErr_deg[1]        = tlm->YawError_deg;
    st->AttErr_deg[2]        = tlm->RollError_deg;
    st->AngVel_rads[0]       = tlm->AngVel_X;
    st->AngVel_rads[1]       = tlm->AngVel_Y;
    st->AngVel_rads[2]       = tlm->AngVel_Z;
    st->RelPos_m[0]          = tlm->RelPos_X;
    st->RelPos_m[1]          = tlm->RelPos_Y;
    st->RelPos_m[2]          = tlm->RelPos_Z;
    st->RelVel_ms[0]         = tlm->RelVel_X;
    st->RelVel_ms[1]         = tlm->RelVel_Y;
    st->RelVel_ms[2]         = tlm->RelVel_Z;
    st->LinImpulseReq_Ns[0]  = ctrl->Px;
    st->LinImpulseReq_Ns[1]  = ctrl->Py;
    st->LinImpulseReq_Ns[2]  = ctrl->Pz;
    st->AngImpulseReq_Nms[0] = ctrl->Lx;
    st->AngImpulseReq_Nms[1] = ctrl->Ly;
    st->AngImpulseReq_Nms[2] = ctrl->Lz;
    st->OverspeedStreak      = (GNC_APP_Data.OverspeedStreak > 0xFFFF) ? 0xFFFF : (uint16)GNC_APP_Data.OverspeedStreak;
    st->CorridorStreak       = (GNC_APP_Data.CorridorStreak > 0xFFFF) ? 0xFFFF : (uint16)GNC_APP_Data.CorridorStreak;
    st->DriftSafeRange_m     = GNC_APP_Data.DriftSafeRange_m;
    st->CamDv_ms             = GNC_APP_Data.CamDv_ms;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(st->TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(st->TelemetryHeader), true);

    CFE_SB_TransmitMsg(CFE_MSG_PTR(GNC_APP_Data.LcSampleCmd.CommandHeader), true);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_ProcessThrusterCmd — RCS's answer to our last ACT_REQ             */
/*                                                                           */
/* Remember the impulse it really commanded (after saturation and minimum-  */
/* impulse-bit effects) for next cycle's actuator-health check and the CAM  */
/* bookkeeping.                                                              */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void GNC_APP_ProcessThrusterCmd(const SIM_IO_ThrusterCmdMsg_t *Msg)
{
    if (GNC_APP_Data.HavePrevSeq && Msg->Seq == GNC_APP_Data.PrevSeq)
    {
        GNC_APP_Data.PrevAchievedPz_Ns = Msg->AchievedLinImpulse_Ns[2];
        GNC_APP_Data.PrevBurnValid     = true;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_ProcessSimState — sim hardware readings + link watchdog           */
/*                                                                           */
/* SIM_STATE reaches NAV and GNC together. GNC takes from it only what is    */
/* not navigation — the crew hand controllers and the docking-mechanism      */
/* switches — and keeps them until NAV's solution for the same Seq arrives   */
/* and runs the cycle (GNC_APP_ProcessNavSolution). Its arrival is also what */
/* proves the sim link alive for the LC link-loss watchpoint (TlmStaleSec).  */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void GNC_APP_ProcessSimState(const SIM_IO_SimStateMsg_t *Msg)
{
    GNC_APP_Data.HkTlm.SimStateCount++;
    GNC_APP_Data.HaveTlm        = true;
    GNC_APP_Data.TlmSinceLastHk = true;

    /* Informational only — whether the outage was long enough to abort is
    ** LC's call (actionpoint 0), and GO is still the operator's. */
    if (GNC_APP_Data.HkTlm.TlmStaleSec >= GNC_LINK_RESTORED_EVT_SEC)
    {
        CFE_EVS_SendEvent(GNC_APP_LINK_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "GNC_APP: sim link restored at Seq %lu after %lu s",
                          (unsigned long)Msg->Seq, (unsigned long)GNC_APP_Data.HkTlm.TlmStaleSec);
    }
    GNC_APP_Data.HkTlm.TlmStaleSec = 0;

    GNC_APP_Data.HwSeq = Msg->Seq;
    memcpy(GNC_APP_Data.HwThc, Msg->State.Thc, sizeof(GNC_APP_Data.HwThc));
    memcpy(GNC_APP_Data.HwRhc, Msg->State.Rhc, sizeof(GNC_APP_Data.HwRhc));
    GNC_APP_Data.HwMechFlags = Msg->State.MechFlags;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_BuildInput — NAV solution + hardware readings -> control input    */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void GNC_APP_BuildInput(const NAV_SolutionMsg_t *Msg, GNC_APP_Input_t *in)
{
    const GNC_ParamTbl_t *p       = GNC_APP_Data.ParamTblPtr;
    const float           RAD2DEG = 57.29578f;

    memset(in, 0, sizeof(*in));

    in->Range_m         = Msg->Range_m;
    in->ClosingSpeed_ms = Msg->ClosingSpeed_ms;
    in->LateralOffset_m = Msg->LateralOffset_m;
    in->LatOffset_X     = Msg->PortPos_D[0];
    in->LatOffset_Y     = Msg->PortPos_D[1];
    memcpy(in->PortPos_D, Msg->PortPos_D, sizeof(in->PortPos_D));
    memcpy(in->AttQuat_L, Msg->AttQuat_L, sizeof(in->AttQuat_L));
    memcpy(in->DockQuat_L, Msg->DockFrameQuat_L, sizeof(in->DockQuat_L));
    if (p->LatVelAtPort)
    {
        in->LatVel_X = Msg->PortVel_D[0];
        in->LatVel_Y = Msg->PortVel_D[1];
    }
    else
    {
        in->LatVel_X = Msg->CmVel_L[0];
        in->LatVel_Y = Msg->CmVel_L[1];
    }
    in->RelPos_X       = Msg->CmPos_L[0];
    in->RelPos_Y       = Msg->CmPos_L[1];
    in->RelPos_Z       = Msg->CmPos_L[2];
    in->RelVel_X       = Msg->CmVel_L[0];
    in->RelVel_Y       = Msg->CmVel_L[1];
    in->RelVel_Z       = Msg->CmVel_L[2];
    in->AngVel_X       = Msg->AngRate_B[0];
    in->AngVel_Y       = Msg->AngRate_B[1];
    in->AngVel_Z       = Msg->AngRate_B[2];
    in->PitchError_deg = Msg->AttErr_B[0] * RAD2DEG;
    in->YawError_deg   = Msg->AttErr_B[1] * RAD2DEG;
    in->RollError_deg  = Msg->AttErr_B[2] * RAD2DEG;
    memcpy(in->DeltaV_B, Msg->DeltaV_B, sizeof(in->DeltaV_B));
    in->RelNavValid = (Msg->Status & NAV_STATUS_RELNAV_VALID) != 0;
    in->AttValid    = (Msg->Status & NAV_STATUS_ATT_VALID) != 0;
    in->RateValid   = (Msg->Status & NAV_STATUS_RATE_VALID) != 0;
    in->DvValid     = (Msg->Status & NAV_STATUS_DV_VALID) != 0;

    /* Sticks only from the SIM_STATE of this same cycle — never act on a stale
    ** deflection. The capture latch is a latched mechanism state, so the
    ** last reading stands. */
    if (GNC_APP_Data.HaveTlm && GNC_APP_Data.HwSeq == Msg->Seq)
    {
        memcpy(in->Thc, GNC_APP_Data.HwThc, sizeof(in->Thc));
        memcpy(in->Rhc, GNC_APP_Data.HwRhc, sizeof(in->Rhc));
    }
    else
    {
        GNC_APP_Data.HwSeqMismatches++;
    }
    in->Captured = (GNC_APP_Data.HwMechFlags & SIMLINK_MECH_CAPTURE) != 0;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_ProcessNavSolution — one lock-step control cycle                  */
/*                                                                           */
/* Runs once per NAV_SOLUTION (NAV runs once per SIM_STATE). Unity is        */
/* blocked waiting for the answer, so this is the whole control cycle:       */
/* 1. Validate the cycle and assemble the input (single task — no mutex).    */
/* 2. Run the phase state machine.                                           */
/* 3. Run the control law and publish ACT_REQ tagged with this Seq.          */
/* 4. Publish the STATE packet and request the LC cycle-rate AP sample.      */
/*                                                                           */
/* HK publication runs separately on the 1 Hz SEND_HK wakeup                 */
/* (GNC_APP_SendHk) — it must keep going when no SIM_STATE arrives at all,   */
/* since its TlmStaleSec is what LC watches for sim-link loss.               */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void GNC_APP_ProcessNavSolution(const NAV_SolutionMsg_t *Msg)
{
    static const char *const PHASE_NAMES[] = {"IDLE", "LAT_CORR", "APPROACH", "DOCKED", "HOLD", "MANUAL", "DEPART"};

    GNC_APP_Input_t tlm;
    GNC_Control_t   ctrl = {0};
    float           dt   = Msg->CycleDt_s;

    GNC_APP_Data.HkTlm.CycleCount++;

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
    ** Reject a nonsensical cycle length — it feeds the CW feedforward directly.
    ** Still answer with a coast so Unity's lock-step isn't left waiting.
    */
    if (!(dt >= GNC_MIN_CYCLE_DT_S && dt <= GNC_MAX_CYCLE_DT_S))
    {
        CFE_EVS_SendEvent(GNC_APP_BAD_DT_EID, CFE_EVS_EventType_ERROR,
                          "GNC_APP: NAV_SOLUTION Seq %lu CycleDt_s %g out of range — coasting",
                          (unsigned long)Msg->Seq, (double)dt);
        GNC_APP_PublishCommand(&ctrl, Msg->Seq);
        GNC_APP_Data.PrevBurnValid = false;
        return;
    }

    GNC_APP_BuildInput(Msg, &tlm);
    GNC_APP_Data.LatestIn = tlm;

    /* NAV's relative solution becoming unusable (or usable again) changes what
    ** the control law can do — say so once per transition. */
    if (tlm.RelNavValid == GNC_APP_Data.NavInvalidLatched)
    {
        if (tlm.RelNavValid)
            CFE_EVS_SendEvent(GNC_APP_NAV_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC_APP: NAV relative solution usable — translation control enabled");
        else
            CFE_EVS_SendEvent(GNC_APP_NAV_INF_EID, CFE_EVS_EventType_ERROR,
                              "GNC_APP: NAV relative solution NOT usable — translation coasts, attitude held");
    }
    GNC_APP_Data.NavInvalidLatched = !tlm.RelNavValid;

    /* Inside the approach cone? (the same corridor radius SelectPhase reverts on) */
    {
        const GNC_ParamTbl_t *p       = GNC_APP_Data.ParamTblPtr;
        float                 allowed = tlm.Range_m * tanf(p->ConeHalfAngle_deg * GNC_DEG2RAD);
        if (allowed < p->MinAllowedLateral_m)
            allowed = p->MinAllowedLateral_m;
        GNC_APP_Data.InCorridor = tlm.RelNavValid && tlm.LateralOffset_m <= allowed;
    }

    /* A Seq discontinuity (Unity ran free, or restarted) means sim time passed
    ** that the previous command didn't cover — drop the actuator-health
    ** baseline instead of attributing that gap to one burn. */
    if (!GNC_APP_Data.HavePrevSeq || Msg->Seq != GNC_APP_Data.PrevSeq + 1)
        GNC_APP_Data.PrevBurnValid = false;
    GNC_APP_Data.PrevSeq     = Msg->Seq;
    GNC_APP_Data.HavePrevSeq = true;

    /*
    ** Phase state machine — evaluate before running the control law.
    ** Emit an EVS event on every transition for ground visibility.
    */
    GNC_Phase_t prev_phase = GNC_APP_Data.Phase;
    GNC_Phase_t new_phase  = GNC_APP_SelectPhase(&tlm, prev_phase, dt);

    if (new_phase != prev_phase)
    {
        CFE_EVS_SendEvent(GNC_APP_PHASE_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "GNC mode: %s → %s (lat=%.2fm rng=%.2fm pyr=%.1f/%.1f/%.1f)",
                          PHASE_NAMES[prev_phase], PHASE_NAMES[new_phase],
                          (double)tlm.LateralOffset_m, (double)tlm.Range_m,
                          (double)tlm.PitchError_deg, (double)tlm.YawError_deg,
                          (double)tlm.RollError_deg);
        GNC_APP_Data.Phase = new_phase;
        if (new_phase == GNC_PHASE_MANUAL)
            CFE_EVS_SendEvent(GNC_APP_MANUAL_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC_APP: CREW MANUAL TAKEOVER (hand controller) — autopilot off until GO");
    }

    /* Passive safety: where would free drift take the port from here? */
    {
        const GNC_ParamTbl_t *p = GNC_APP_Data.ParamTblPtr;
        GNC_APP_Data.DriftSafeRange_m = GNC_APP_DriftSafeRange(&tlm, p);
        GNC_APP_Data.DriftSafe        = tlm.RelNavValid && GNC_APP_Data.DriftSafeRange_m >= p->PassiveSafeRange_m;
    }

    /* Abort manoeuvre bookkeeping: Δv delivered along the retreat direction
    ** (IMU-measured over the cycle that just ended), and the end condition. */
    if (GNC_APP_Data.Phase == GNC_PHASE_DEPART)
    {
        const GNC_ParamTbl_t *p = GNC_APP_Data.ParamTblPtr;
        float                 dir_B[3];
        GNC_APP_CamDirection(&tlm, dir_B);
        GNC_APP_Data.CamCycles++;
        if (tlm.DvValid)
        {
            float along = tlm.DeltaV_B[0] * dir_B[0] + tlm.DeltaV_B[1] * dir_B[1] + tlm.DeltaV_B[2] * dir_B[2];
            if (along > 0.0f)
                GNC_APP_Data.CamDv_ms += along;
        }
        else
        {
            /* No accelerometer: count what was asked for (open loop) */
            GNC_APP_Data.CamDv_ms += GNC_APP_Data.CamLastCmd_ms;
        }
        if ((GNC_APP_Data.CamDv_ms >= p->CamDeltaV_ms && (GNC_APP_Data.DriftSafe || !tlm.RelNavValid)) ||
            GNC_APP_Data.CamDv_ms >= p->CamMaxDeltaV_ms || GNC_APP_Data.CamCycles >= GNC_CAM_MAX_CYCLES)
        {
            CFE_EVS_SendEvent(GNC_APP_CAM_INF_EID,
                              GNC_APP_Data.DriftSafe ? CFE_EVS_EventType_INFORMATION : CFE_EVS_EventType_ERROR,
                              "GNC_APP: abort manoeuvre complete — %.2f m/s in %lu cycles, free drift %s "
                              "(closest %.1f m); coasting, send GO to resume",
                              (double)GNC_APP_Data.CamDv_ms, (unsigned long)GNC_APP_Data.CamCycles,
                              GNC_APP_Data.DriftSafe ? "passively safe" : tlm.RelNavValid ? "NOT SAFE" : "unknown (no NAV)",
                              (double)GNC_APP_Data.DriftSafeRange_m);
            GNC_APP_Data.Phase = GNC_PHASE_IDLE;
        }
    }

    /* Approach-envelope monitors (LC decides; GNC only counts) */
    {
        const GNC_ParamTbl_t *p     = GNC_APP_Data.ParamTblPtr;
        GNC_Phase_t           ph    = GNC_APP_Data.Phase;
        bool                  auto_ = ph == GNC_PHASE_CORRECT || ph == GNC_PHASE_APPROACH || ph == GNC_PHASE_HOLD;

        if (tlm.RelNavValid && (ph == GNC_PHASE_APPROACH || ph == GNC_PHASE_HOLD) &&
            tlm.ClosingSpeed_ms > GNC_APP_ProfileLimit(tlm.Range_m, p))
            GNC_APP_Data.OverspeedStreak++;
        else
            GNC_APP_Data.OverspeedStreak = 0;

        if (tlm.RelNavValid && auto_ && tlm.Range_m < p->CorridorCheckRange_m && !GNC_APP_Data.InCorridor)
            GNC_APP_Data.CorridorStreak++;
        else
            GNC_APP_Data.CorridorStreak = 0;
    }

    GNC_APP_Data.HkTlm.Phase = (uint8)GNC_APP_Data.Phase;

    /* Check the previous cycle's axial burn against the Δv the IMU measured
       over that cycle, before overwriting Prev* with this cycle's command. */
    GNC_APP_CheckActuatorHealth(&tlm, GNC_APP_Data.ParamTblPtr);

    /* Run the control law for the active phase and request the impulse */
    ctrl = GNC_APP_ComputeControl(&tlm, GNC_APP_Data.Phase, dt);
    GNC_APP_PublishCommand(&ctrl, Msg->Seq);

    /* Health baseline for next cycle: PrevBurnValid is set again only once the
    ** RCS's THRUSTER_CMD answering this Seq arrives with the achieved impulse. */
    GNC_APP_Data.PrevBurnValid = false;

    GNC_APP_PublishState(Msg, &tlm, &ctrl);

    /* Per-cycle one-liner, now a DEBUG event: the STATE packet above is the
    ** data channel (ground console, DS recorder). EVS filters DEBUG out by
    ** default; enable it for a console trace with EVS ENABLE_APP_EVENT_TYPE
    ** (GNC_APP, DEBUG) — see Docs/OPERATIONS.md.
    ** PYR = pitch/yaw/roll error (deg), W = body rates (rad/s), C/D = in
    ** corridor / captured, N = NAV relative solution usable,
    ** P = linear impulse request (N·s), L = angular impulse request (N·m·s). */
    CFE_EVS_SendEvent(GNC_APP_WAKEUP_INF_EID, CFE_EVS_EventType_DEBUG,
                      "GNC #%u [%s] Rng=%.2f Spd=%.3f Lat=%.3f "
                      "PYR=%.1f/%.1f/%.1f W=%.3f/%.3f/%.3f C%dD%dN%d "
                      "P=%.0f/%.0f/%.0f L=%.0f/%.0f/%.0f",
                      (unsigned int)GNC_APP_Data.HkTlm.CycleCount,
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
                      (int)GNC_APP_Data.InCorridor, (int)tlm.Captured, (int)tlm.RelNavValid,
                      (double)ctrl.Px, (double)ctrl.Py, (double)ctrl.Pz,
                      (double)ctrl.Lx, (double)ctrl.Ly, (double)ctrl.Lz);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/* GNC_APP_SendHk — 1 Hz SCH_LAB wakeup: link-staleness count + HK publish   */
/*                                                                           */
/* TlmStaleSec counts whole HK periods with no SIM_STATE, starting only once */
/* the link has been up (a cFS that boots before Unity is not an outage).    */
/* GNC only counts. LC actionpoint 0 turns a long enough count into SC RTS 2 */
/* (ABORT) — the same latched safe state a ground ABORT gives, so guidance   */
/* never silently resumes on whatever state it finds when a free-running     */
/* Unity reconnects.                                                         */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void GNC_APP_SendHk(void)
{
    if (GNC_APP_Data.HaveTlm && !GNC_APP_Data.TlmSinceLastHk && GNC_APP_Data.HkTlm.TlmStaleSec < 0xFFFFFFFFu)
        GNC_APP_Data.HkTlm.TlmStaleSec++;
    GNC_APP_Data.TlmSinceLastHk = false;

    GNC_APP_Data.HkTlm.Phase = (uint8)GNC_APP_Data.Phase;
    GNC_APP_Data.HkTlm.Flags = GNC_APP_Flags();

    /* Publish HK telemetry to the SB (TO_LAB forwards it to the ground, LC watches it) */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(GNC_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(GNC_APP_Data.HkTlm.TelemetryHeader), true);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void GNC_APP_PublishCommand(const GNC_Control_t *ctrl, uint32 Seq)
{
    RCS_ActReqMsg_t *m = &GNC_APP_Data.ActReqMsg;

    m->Seq               = Seq;
    m->GncPhase          = (int32)GNC_APP_Data.Phase;
    m->LinImpulse_Ns[0]  = ctrl->Px;
    m->LinImpulse_Ns[1]  = ctrl->Py;
    m->LinImpulse_Ns[2]  = ctrl->Pz;
    m->AngImpulse_Nms[0] = ctrl->Lx;
    m->AngImpulse_Nms[1] = ctrl->Ly;
    m->AngImpulse_Nms[2] = ctrl->Lz;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(m->TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(m->TelemetryHeader), true);
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
/* ABORT — forces IDLE, sets AbortLatch; the next cycle commands coast.    */
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
            GNC_APP_Data.HkTlm.SimStateCount      = 0;
            CFE_EVS_SendEvent(GNC_APP_RST_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC_APP: counters reset");
            break;

        /* ---------------------------------------------------------------- */
        case GNC_APP_HOLD_CC:
        {
            if (!GNC_APP_VerifyCmdLength(&MsgBuf->Msg, sizeof(GNC_APP_HoldCmd_t))) break;
            if (GNC_APP_Data.Phase == GNC_PHASE_DEPART || GNC_APP_Data.Phase == GNC_PHASE_DOCKED)
            {
                GNC_APP_Data.HkTlm.CmdErrCount++;
                CFE_EVS_SendEvent(GNC_APP_HOLD_INF_EID, CFE_EVS_EventType_ERROR,
                                  "GNC_APP: HOLD rejected — %s",
                                  GNC_APP_Data.Phase == GNC_PHASE_DOCKED ? "docked" : "abort manoeuvre in progress");
                break;
            }
            GNC_APP_Data.HkTlm.CmdCount++;
            GNC_APP_Data.Phase          = GNC_PHASE_HOLD;
            GNC_APP_Data.HkTlm.Phase    = (uint8)GNC_PHASE_HOLD;
            /* LatestIn is owned by this task — no mutex needed */
            float rng = GNC_APP_Data.LatestIn.Range_m;
            GNC_APP_Data.HoldRange_m    = rng;
            CFE_EVS_SendEvent(GNC_APP_HOLD_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "GNC_APP: HOLD — station-keep at %.2f m (await GO)", (double)rng);
            break;
        }

        /* ---------------------------------------------------------------- */
        case GNC_APP_GO_CC:
        {
            if (!GNC_APP_VerifyCmdLength(&MsgBuf->Msg, sizeof(GNC_APP_GoCmd_t))) break;

            bool was_hold   = (GNC_APP_Data.Phase == GNC_PHASE_HOLD);
            bool was_manual = (GNC_APP_Data.Phase == GNC_PHASE_MANUAL);
            bool was_abort  = GNC_APP_Data.AbortLatch;

            if (GNC_APP_Data.Phase == GNC_PHASE_DEPART)
            {
                GNC_APP_Data.HkTlm.CmdErrCount++;
                CFE_EVS_SendEvent(GNC_APP_GO_INF_EID, CFE_EVS_EventType_ERROR,
                                  "GNC_APP: GO rejected — abort manoeuvre in progress");
                break;
            }

            if (!was_hold && !was_manual && !was_abort)
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
            GNC_APP_Data.AbortLatch = true;
            if (GNC_APP_Data.Phase == GNC_PHASE_DOCKED || GNC_APP_Data.LatestIn.Captured)
            {
                /* Captured: the mechanism holds the vehicle; nothing to fly away from */
                CFE_EVS_SendEvent(GNC_APP_ABORT_INF_EID, CFE_EVS_EventType_CRITICAL,
                                  "GNC_APP: *** ABORT *** while captured — thrust inhibited");
            }
            else if (GNC_APP_Data.Phase == GNC_PHASE_IDLE || GNC_APP_Data.Phase == GNC_PHASE_DEPART)
            {
                /* Already stopped, or already leaving: just latch */
                CFE_EVS_SendEvent(GNC_APP_ABORT_INF_EID, CFE_EVS_EventType_CRITICAL,
                                  "GNC_APP: *** ABORT *** (latched; %s)",
                                  GNC_APP_Data.Phase == GNC_PHASE_DEPART ? "manoeuvre continues" : "thrust inhibited");
            }
            else
            {
                /* Collision-avoidance manoeuvre. In lock-step it starts on the
                ** next cycle, at most one GNC period away. */
                GNC_APP_Data.Phase     = GNC_PHASE_DEPART;
                GNC_APP_Data.CamDv_ms      = 0.0f;
                GNC_APP_Data.CamCycles     = 0;
                GNC_APP_Data.CamLastCmd_ms = 0.0f;
                CFE_EVS_SendEvent(GNC_APP_ABORT_INF_EID, CFE_EVS_EventType_CRITICAL,
                                  "GNC_APP: *** ABORT *** collision-avoidance manoeuvre: retreat %.2f m/s, "
                                  "then coast — send GO to resume",
                                  (double)GNC_APP_Data.ParamTblPtr->CamDeltaV_ms);
            }
            GNC_APP_Data.HkTlm.Phase = (uint8)GNC_APP_Data.Phase;
            break;

        /* ---------------------------------------------------------------- */
        default:
            GNC_APP_Data.HkTlm.CmdErrCount++;
            CFE_EVS_SendEvent(GNC_APP_CMD_CODE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "GNC_APP: invalid command code %u", (unsigned int)fc);
            break;
    }
}
