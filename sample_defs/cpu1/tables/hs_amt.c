/*
** HS Application Monitor Table — docking mission (realism phase 3)
**
** Overrides apps/hs/fsw/tables/hs_amt.c. HS checks each listed app's ES
** execution counter (bumped by every CFE_ES_RunLoop pass) once per HS cycle
** (1 Hz, SCH_LAB HS_WAKEUP); an app whose counter hasn't moved for CycleCount
** cycles is declared hung and the action is taken.
**
** Every mission app wakes at least once a second on its 1 Hz HK / wakeup
** request, so 10 cycles is a genuine hang, not a paused sim. The action is an
** event only: an app restart or processor reset is a flight-rules decision for
** later — a restarted GNC comes back abort-latched, which is safe, but a
** processor reset mid-approach is not something to automate casually.
**
** Entries past the last one listed are zero-filled, i.e. NOACT (unused).
*/

#include "cfe.h"
#include "hs_mission_cfg.h"
#include "hs_tbl.h"
#include "hs_tbldefs.h"
#include "cfe_tbl_filedef.h"

HS_AMTEntry_t HS_AppMon_Tbl[HS_MAX_MONITORED_APPS] = {
    /*          AppName       NullTerm CycleCount ActionType */
    /* cFE core services: monitored, no action (as the stock table) */
    /*   0 */ {"CFE_ES",   0, 10, HS_AMTActType_NOACT},
    /*   1 */ {"CFE_EVS",  0, 10, HS_AMTActType_NOACT},
    /*   2 */ {"CFE_TIME", 0, 10, HS_AMTActType_NOACT},
    /*   3 */ {"CFE_TBL",  0, 10, HS_AMTActType_NOACT},
    /*   4 */ {"CFE_SB",   0, 10, HS_AMTActType_NOACT},

    /* docking mission flight software */
    /*   5 */ {"GNC_APP",  0, 10, HS_AMTActType_EVENT},
    /*   6 */ {"RCS",      0, 10, HS_AMTActType_EVENT},
    /*   7 */ {"SIM_IO",   0, 10, HS_AMTActType_EVENT},
    /*   8 */ {"LC",       0, 10, HS_AMTActType_EVENT},
    /*   9 */ {"SC",       0, 10, HS_AMTActType_EVENT},
    /*  10 */ {"DS",       0, 10, HS_AMTActType_EVENT},
    /*  11 */ {"NAV",      0, 10, HS_AMTActType_EVENT},
};

CFE_TBL_FILEDEF(HS_AppMon_Tbl, HS.AppMon_Tbl, Dock HS AppMon Table, hs_amt.tbl)
