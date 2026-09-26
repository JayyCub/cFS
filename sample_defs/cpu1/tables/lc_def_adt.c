/*
** LC Actionpoint Definition Table — docking mission (realism phase 3)
**
** Overrides apps/lc/fsw/tables/lc_def_adt.c. An actionpoint combines
** watchpoints (lc_def_wdt.c) in an RPN equation; when the equation is TRUE on
** MaxFailsBeforeRTS consecutive samples, LC emits EventText and asks SC to
** start RTSId (sample_defs/cpu1/tables/sc_rts00N.c). LC then sets the actionpoint
** PASSIVE so a persisting fault doesn't restart the RTS every sample — RTS 4
** (FDIR re-arm) sets them ACTIVE again once the operator has recovered.
**
** Actionpoints are only evaluated when LC is asked to sample them, in two rate
** groups:
**   AP 0      SCH_LAB, 1 Hz wall clock — must evaluate while the sim is silent.
**   AP 1..1   gnc_app, end of every lock-step cycle (GNC_APP_LC_CYCLE_AP_*) —
**             persistence counts sim cycles, not wall seconds.
**
**   AP  Fails when                                          Response
**   0   link silent >= 2 s AND NOT docked AND NOT aborted  RTS 2: GNC ABORT
**   1   axial under-delivery >= 3 cycles AND APPROACH      RTS 3: GNC HOLD
**
** HOLD, not ABORT, for the actuator anomaly: the monitor can false-alarm
** (contact forces, CW drift on tiny burns), and HOLD stops the closure while
** still flying — the ground decides with GO or ABORT.
*/

#include "cfe_tbl_filedef.h"
#include "cfe_evs_extern_typedefs.h"
#include "lc_platform_cfg.h"
#include "lc_msgdefs.h"
#include "lc_tbl.h"
#include "lc_eventids.h"

/* SC RTS numbers — must match sample_defs/cpu1/tables/sc_rts00N.c */
#define DOCK_RTS_LINK_LOSS_ABORT 2
#define DOCK_RTS_ACTUATOR_HOLD   3

CFE_TBL_FileDef_t CFE_TBL_FileDef = { "LC_ADT",
                                      LC_APP_NAME "." LC_ADT_TABLENAME,
                                      "LC actionpoint definition table",
                                      "lc_def_adt.tbl",
                                      (sizeof(LC_ADTEntry_t) * LC_MAX_ACTIONPOINTS) };

#define LC_ADT_UNUSED                                    \
    {                                                    \
        .DefaultState      = LC_APSTATE_NOT_USED,        \
        .MaxPassiveEvents  = 0,                          \
        .MaxPassFailEvents = 0,                          \
        .MaxFailPassEvents = 0,                          \
        .RTSId             = 0,                          \
        .MaxFailsBeforeRTS = 0,                          \
        .EventType         = CFE_EVS_EventType_INFORMATION, \
        .EventID           = 0,                          \
        .EventText         = {" "},                      \
        .RPNEquation       = {0, LC_RPN_EQUAL},          \
    }

LC_ADTEntry_t LC_ADT[LC_MAX_ACTIONPOINTS] = {
    /* #0: Sim link loss -> RTS 2 (GNC ABORT). Rate group: SCH_LAB 1 Hz. */
    {
        .DefaultState      = LC_APSTATE_ACTIVE,
        .MaxPassiveEvents  = 2,
        .MaxPassFailEvents = 2,
        .MaxFailPassEvents = 2,
        .RTSId             = DOCK_RTS_LINK_LOSS_ABORT,
        .MaxFailsBeforeRTS = 1,
        .EventType         = CFE_EVS_EventType_CRITICAL,
        .EventID           = LC_BASE_AP_EID + 0,
        .EventText         = {"GNC sim link loss: ABORT"},
        /* silent AND NOT docked AND NOT already aborted */
        .RPNEquation       = {0, 1, LC_RPN_NOT, LC_RPN_AND, 2, LC_RPN_NOT, LC_RPN_AND, LC_RPN_EQUAL},
    },

    /* #1: Axial actuator under-delivery on approach -> RTS 3 (GNC HOLD). Rate group: GNC cycle. */
    {
        .DefaultState      = LC_APSTATE_ACTIVE,
        .MaxPassiveEvents  = 2,
        .MaxPassFailEvents = 2,
        .MaxFailPassEvents = 2,
        .RTSId             = DOCK_RTS_ACTUATOR_HOLD,
        .MaxFailsBeforeRTS = 1,
        .EventType         = CFE_EVS_EventType_ERROR,
        .EventID           = LC_BASE_AP_EID + 1,
        .EventText         = {"Axial under-delivery: HOLD"},
        /* streak AND APPROACH */
        .RPNEquation       = {3, 4, LC_RPN_AND, LC_RPN_EQUAL},
    },

    /* #2 .. #175: unused */
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED,
    LC_ADT_UNUSED
};

/* Fails to compile if LC_MAX_ACTIONPOINTS changes and the unused padding above is not regenerated */
typedef char LC_ADT_size_check[(sizeof(LC_ADT) / sizeof(LC_ADT[0]) == 176) ? 1 : -1];
