#ifndef GNC_APP_MSG_H
#define GNC_APP_MSG_H

#include <stddef.h>
#include "cfe.h"

/*
** GNC App public message definitions — everything another app, a table or a
** ground tool needs to talk to gnc_app: command codes, the 1 Hz HK packet and
** the per-cycle STATE packet.
**
** LC watchpoint tables (sample_defs/cpu1/tables/lc_def_wdt.c) read fields out of
** these packets with offsetof(), and ground_console.py / tools/fdr_decode.py
** decode them with fixed struct formats — the _Static_asserts at the bottom
** catch a layout change that would silently break either.
*/

/*
** Ground command function codes (GNC_APP_CMD_MID)
**
** NOOP            — heartbeat; no state change; increments CmdCount.
** RESET_COUNTERS  — zero CmdCount, CmdErrCount, SimStateCount.
** HOLD            — immediately enter HOLD phase (station-keep, await GO).
** GO              — release HOLD, MANUAL or the ABORT latch; resume at CORRECT.
** ABORT           — inhibit all thrust immediately; latch IDLE until GO.
**
** HOLD and ABORT are also what the SC fault-response RTSs send
** (sample_defs/cpu1/tables/sc_rts00N.c), so FDIR and ground drive the same paths.
*/
#define GNC_APP_NOOP_CC           0
#define GNC_APP_RESET_COUNTERS_CC 1
#define GNC_APP_HOLD_CC           2
#define GNC_APP_GO_CC             3
#define GNC_APP_ABORT_CC          4

typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
} GNC_APP_NoArgsCmd_t;

/* GNC_Phase_t values as carried in the Phase byte of HK and STATE */
#define GNC_APP_PHASE_IDLE     0
#define GNC_APP_PHASE_CORRECT  1
#define GNC_APP_PHASE_APPROACH 2
#define GNC_APP_PHASE_DOCKED   3
#define GNC_APP_PHASE_HOLD     4
#define GNC_APP_PHASE_MANUAL   5

/* Flags bits in HK and STATE */
#define GNC_APP_FLAG_ABORT_LATCH   0x01u /* ABORT latched — guidance inhibited until GO    */
#define GNC_APP_FLAG_HOLDPT1_ARMED 0x02u /* outer autonomous hold point not yet reached    */
#define GNC_APP_FLAG_HOLDPT2_ARMED 0x04u /* inner autonomous hold point not yet reached    */
#define GNC_APP_FLAG_IN_CORRIDOR   0x08u /* NAV port offset inside the approach cone       */
#define GNC_APP_FLAG_CONTACT       0x10u /* capture latches engaged (mechanism switch)     */
#define GNC_APP_FLAG_BAD_CTRL      0x20u /* last control output was non-finite (coasting)  */
#define GNC_APP_FLAG_LINK_UP       0x40u /* at least one SIM_STATE received since start    */
#define GNC_APP_FLAG_NAV_INVALID   0x80u /* NAV relative solution unusable: translation coasts */

/*
** Housekeeping telemetry (GNC_APP_HK_TLM_MID), 1 Hz on SCH_LAB's SEND_HK.
**
** Keeps publishing when no SIM_STATE arrives, which is what makes it the
** source for the LC sim-link-loss watchpoint (TlmStaleSec).
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint32                    CmdCount;
    uint32                    CmdErrCount;
    uint32                    CycleCount;    /* lock-step control cycles run (SIM_STATEs accepted) */
    uint32                    SimStateCount; /* SIM_STATE messages received                        */
    uint8                     Phase;         /* GNC_APP_PHASE_*                                    */
    uint8                     Flags;         /* GNC_APP_FLAG_*                                     */
    uint16                    Spare;
    uint32                    TlmStaleSec;   /* consecutive 1 Hz HK periods with no SIM_STATE,
                                             ** counted only once the link has been up — LC
                                             ** watchpoint input for the link-loss abort        */
} GNC_APP_HkTlm_t;

/*
** Per-cycle GNC state (GNC_APP_STATE_TLM_MID), one per lock-step cycle.
**
** Replaces the per-cycle "GNC #" EVS event as the data channel: events get
** squelched and truncated, a packet doesn't. TO_LAB forwards it to the ground
** console, DS records it to the flight data recorder file, and LC watches the
** fault-monitor fields (UnderDeliveryStreak, Phase).
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    double                    SimTime_s;           /* sim time of the SIM_STATE this cycle ran on     */
    uint32                    Seq;                 /* SIM_STATE Seq                                   */
    uint8                     Phase;               /* GNC_APP_PHASE_* in effect for this cycle        */
    uint8                     Flags;               /* GNC_APP_FLAG_*                                  */
    uint16                    UnderDeliveryStreak; /* consecutive cycles the axial burn delivered
                                                   ** < 50% of predicted Δv (fault monitor; LC
                                                   ** decides when it counts as an anomaly)          */
    uint16                    SettleCounter;       /* CORRECT->APPROACH settle-gate progress (cycles) */
    uint16                    Spare;
    float                     Range_m;             /* NAV: port to port                              */
    float                     ClosingSpeed_ms;     /* + = closing                                     */
    float                     LateralOffset_m;
    float                     LatOffset_m[2];      /* NAV: port lateral offset, docking frame X/Y    */
    float                     AttErr_deg[3];       /* NAV quaternion error, body X/Y/Z (pitch/yaw/roll) */
    float                     AngVel_rads[3];      /* gyro body rates                                 */
    float                     RelPos_m[3];         /* NAV: chaser CoM - target ref, LVLH (+Y radial)  */
    float                     RelVel_ms[3];
    float                     LinImpulseReq_Ns[3]; /* impulse requested from RCS this cycle (body)    */
    float                     AngImpulseReq_Nms[3];
} GNC_APP_StateTlm_t;

_Static_assert(sizeof(GNC_APP_HkTlm_t) == 40, "GNC HK layout changed - update LC WDT and ground tools");
_Static_assert(sizeof(GNC_APP_StateTlm_t) == 128, "GNC STATE layout changed - update LC WDT and ground tools");
_Static_assert(offsetof(GNC_APP_StateTlm_t, SimTime_s) == 16, "GNC STATE layout changed - update ground tools");

#endif /* GNC_APP_MSG_H */
