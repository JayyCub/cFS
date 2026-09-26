/*
** DS Packet Filter Table — docking mission flight data recorder
** (realism phase 3)
**
** Overrides apps/ds/fsw/tables/ds_filter_tbl.c. Routes packets to the
** recorder files in ds_file_tbl.c. Filter {file, type, N, X, O} stores N of
** every X packets starting at O; {.., DS_BY_COUNT, 1, 1, 0} stores all of them.
**
** Unused entries are all-zero (MessageID = CFE_SB_MSGID_RESERVED), which is
** what DS expects, so they are left to C zero-fill.
*/

#include "cfe_sb_api_typedefs.h"
#include "cfe_tbl_filedef.h"
#include "ds_platform_cfg.h"
#include "ds_tbl.h"
#include "ds_msg.h"

#include "cfe_msgids.h"
#include "lc_msgids.h"
#include "sc_msgids.h"
#include "gnc_app_msgids.h"
#include "sim_io_msgids.h"
#include "rcs_msgids.h"

#define FDR_EVS 0 /* ds_file_tbl.c index: event log     */
#define FDR_GNC 1 /* ds_file_tbl.c index: GNC flight data */

#define FDR_ALL(file)                          \
    {                                          \
        {(file), DS_BY_COUNT, 1, 1, 0},        \
        {DS_UNUSED, DS_UNUSED, DS_UNUSED, DS_UNUSED, DS_UNUSED}, \
        {DS_UNUSED, DS_UNUSED, DS_UNUSED, DS_UNUSED, DS_UNUSED}, \
        {DS_UNUSED, DS_UNUSED, DS_UNUSED, DS_UNUSED, DS_UNUSED}  \
    }

DS_FilterTable_t DS_FilterTable = {
    .Descriptor = "Dock flight data recorder",
    .Packet =
        {
            {.MessageID = CFE_SB_MSGID_WRAP_VALUE(CFE_EVS_LONG_EVENT_MSG_MID), .Filter = FDR_ALL(FDR_EVS)},

            /* per lock-step cycle */
            {.MessageID = CFE_SB_MSGID_WRAP_VALUE(GNC_APP_STATE_TLM_MID), .Filter = FDR_ALL(FDR_GNC)},
            {.MessageID = CFE_SB_MSGID_WRAP_VALUE(SIM_IO_THRUSTER_CMD_MID), .Filter = FDR_ALL(FDR_GNC)},

            /* housekeeping, 1 Hz */
            {.MessageID = CFE_SB_MSGID_WRAP_VALUE(GNC_APP_HK_TLM_MID), .Filter = FDR_ALL(FDR_GNC)},
            {.MessageID = CFE_SB_MSGID_WRAP_VALUE(RCS_HK_TLM_MID), .Filter = FDR_ALL(FDR_GNC)},
            {.MessageID = CFE_SB_MSGID_WRAP_VALUE(SIM_IO_HK_TLM_MID), .Filter = FDR_ALL(FDR_GNC)},
            {.MessageID = CFE_SB_MSGID_WRAP_VALUE(LC_HK_TLM_MID), .Filter = FDR_ALL(FDR_GNC)},
            {.MessageID = CFE_SB_MSGID_WRAP_VALUE(SC_HK_TLM_MID), .Filter = FDR_ALL(FDR_GNC)},
        },
};

CFE_TBL_FILEDEF(DS_FilterTable, DS.FILTER_TBL, Dock FDR filter, ds_filter_tbl.tbl)
