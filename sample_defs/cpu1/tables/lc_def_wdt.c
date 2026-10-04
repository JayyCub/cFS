/*
** LC Watchpoint Definition Table — docking mission (realism phase 3)
**
** Overrides apps/lc/fsw/tables/lc_def_wdt.c. Watchpoints are evaluated every
** time their packet arrives; they only *describe* conditions. What to do about
** a combination of them is in lc_def_adt.c.
**
** Offsets come from offsetof() on the real packet structs in gnc_app_msg.h, so
** a GNC packet layout change can't silently point LC at the wrong bytes.
** Data types are _LE: the native build is little-endian and cFE copies the
** structs onto the bus as-is.
**
**   WP  Packet     Field                  Condition
**   0   GNC HK     TlmStaleSec            >= 2 s
**   1   GNC HK     Phase                  == DOCKED
**   2   GNC HK     Flags & ABORT_LATCH    set
**   3   GNC STATE  UnderDeliveryStreak    >= 3 cycles
**   4   GNC STATE  Phase                  == APPROACH
*/

#include <stddef.h>
#include "cfe_tbl_filedef.h"
#include "cfe_sb_api_typedefs.h"
#include "lc_platform_cfg.h"
#include "lc_msgdefs.h"
#include "lc_tbl.h"
#include "gnc_app_msgids.h"
#include "gnc_app_msg.h"
#include "nav_msgids.h"
#include "nav_msg.h"

CFE_TBL_FileDef_t CFE_TBL_FileDef = { "LC_WDT",
                                      LC_APP_NAME "." LC_WDT_TABLENAME,
                                      "LC watchpoint definition table",
                                      "lc_def_wdt.tbl",
                                      (sizeof(LC_WDTEntry_t) * LC_MAX_WATCHPOINTS) };

#define LC_WDT_UNUSED                                    \
    {                                                    \
        .DataType                   = LC_DATA_WATCH_NOT_USED, \
        .OperatorID                 = LC_OPER_NONE,      \
        .MessageID                  = CFE_SB_MSGID_RESERVED, \
        .WatchpointOffset           = 0,                 \
        .BitMask                    = LC_BITMASK_NONE,   \
        .CustomFuncArgument         = 0,                 \
        .ResultAgeWhenStale         = 0,                 \
        .ComparisonValue.Unsigned32 = 0,                 \
    }

LC_WDTEntry_t LC_WDT[LC_MAX_WATCHPOINTS] = {
    /* #0: GNC HK: sim link silent >= 2 s (TlmStaleSec) */
    {
        .DataType                   = LC_DATA_WATCH_UDWORD_LE,
        .OperatorID                 = LC_OPER_GE,
        .MessageID                  = CFE_SB_MSGID_WRAP_VALUE(GNC_APP_HK_TLM_MID),
        .WatchpointOffset           = offsetof(GNC_APP_HkTlm_t, TlmStaleSec),
        .BitMask                    = LC_BITMASK_NONE,
        .CustomFuncArgument         = 0,
        .ResultAgeWhenStale         = 0,
        .ComparisonValue.Unsigned32 = 2,
    },

    /* #1: GNC HK: phase is DOCKED */
    {
        .DataType                   = LC_DATA_WATCH_UBYTE,
        .OperatorID                 = LC_OPER_EQ,
        .MessageID                  = CFE_SB_MSGID_WRAP_VALUE(GNC_APP_HK_TLM_MID),
        .WatchpointOffset           = offsetof(GNC_APP_HkTlm_t, Phase),
        .BitMask                    = LC_BITMASK_NONE,
        .CustomFuncArgument         = 0,
        .ResultAgeWhenStale         = 0,
        .ComparisonValue.Unsigned8  = GNC_APP_PHASE_DOCKED,
    },

    /* #2: GNC HK: ABORT already latched (guidance already safe) */
    {
        .DataType                   = LC_DATA_WATCH_UBYTE,
        .OperatorID                 = LC_OPER_EQ,
        .MessageID                  = CFE_SB_MSGID_WRAP_VALUE(GNC_APP_HK_TLM_MID),
        .WatchpointOffset           = offsetof(GNC_APP_HkTlm_t, Flags),
        .BitMask                    = GNC_APP_FLAG_ABORT_LATCH,
        .CustomFuncArgument         = 0,
        .ResultAgeWhenStale         = 0,
        .ComparisonValue.Unsigned8  = GNC_APP_FLAG_ABORT_LATCH,
    },

    /* #3: GNC STATE: axial burns under-delivering for >= 3 consecutive cycles */
    {
        .DataType                   = LC_DATA_WATCH_UWORD_LE,
        .OperatorID                 = LC_OPER_GE,
        .MessageID                  = CFE_SB_MSGID_WRAP_VALUE(GNC_APP_STATE_TLM_MID),
        .WatchpointOffset           = offsetof(GNC_APP_StateTlm_t, UnderDeliveryStreak),
        .BitMask                    = LC_BITMASK_NONE,
        .CustomFuncArgument         = 0,
        .ResultAgeWhenStale         = 0,
        .ComparisonValue.Unsigned16 = 3,
    },

    /* #4: GNC STATE: phase is APPROACH */
    {
        .DataType                   = LC_DATA_WATCH_UBYTE,
        .OperatorID                 = LC_OPER_EQ,
        .MessageID                  = CFE_SB_MSGID_WRAP_VALUE(GNC_APP_STATE_TLM_MID),
        .WatchpointOffset           = offsetof(GNC_APP_StateTlm_t, Phase),
        .BitMask                    = LC_BITMASK_NONE,
        .CustomFuncArgument         = 0,
        .ResultAgeWhenStale         = 0,
        .ComparisonValue.Unsigned8  = GNC_APP_PHASE_APPROACH,
    },

    /* #5: GNC STATE: NAV relative solution unusable (translation coasting) */
    {
        .DataType                   = LC_DATA_WATCH_UBYTE,
        .OperatorID                 = LC_OPER_EQ,
        .MessageID                  = CFE_SB_MSGID_WRAP_VALUE(GNC_APP_STATE_TLM_MID),
        .WatchpointOffset           = offsetof(GNC_APP_StateTlm_t, Flags),
        .BitMask                    = GNC_APP_FLAG_NAV_INVALID,
        .CustomFuncArgument         = 0,
        .ResultAgeWhenStale         = 0,
        .ComparisonValue.Unsigned8  = GNC_APP_FLAG_NAV_INVALID,
    },

    /* #6: NAV SOLUTION: >= 5 LIDAR fixes rejected in a row */
    {
        .DataType                   = LC_DATA_WATCH_UWORD_LE,
        .OperatorID                 = LC_OPER_GE,
        .MessageID                  = CFE_SB_MSGID_WRAP_VALUE(NAV_SOLUTION_MID),
        .WatchpointOffset           = offsetof(NAV_SolutionMsg_t, ConsecRpsRejects),
        .BitMask                    = LC_BITMASK_NONE,
        .CustomFuncArgument         = 0,
        .ResultAgeWhenStale         = 0,
        .ComparisonValue.Unsigned16 = 5,
    },

    /* #7 .. #175: unused */
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED,
    LC_WDT_UNUSED
};

/* Fails to compile if LC_MAX_WATCHPOINTS changes and the unused padding above is not regenerated */
typedef char LC_WDT_size_check[(sizeof(LC_WDT) / sizeof(LC_WDT[0]) == 176) ? 1 : -1];
