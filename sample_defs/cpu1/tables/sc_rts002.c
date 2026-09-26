/*
** SC RTS 002 — sim link loss response: GNC ABORT (realism phase 3)
**
** Overrides apps/sc/fsw/tables/sc_rts002.c. Started by LC actionpoint 0
** (sample_defs/cpu1/tables/lc_def_adt.c) when GNC has heard no SIM_STATE for 2 s
** and is neither docked nor already aborted. ABORT latches GNC in IDLE with
** all thrust inhibited, so guidance never resumes on its own when a
** free-running Unity reconnects — the operator reviews and sends GO.
*/

#include "cfe.h"
#include "cfe_tbl_filedef.h"

#include "sc_tbldefs.h"
#include "sc_platform_cfg.h"
#include "sc_msgdefs.h"
#include "sc_msgids.h"
#include "sc_msg.h"
#include "gnc_app_msgids.h"
#include "gnc_app_msg.h"

/* CCSDS command checksum — see sc_rts001.c */
#define DOCK_CKSUM(mid, size, fc, pxor)                                                          \
    (0xFF ^ (((mid) >> 8) & 0xFF) ^ ((mid)&0xFF) ^ 0xC0 ^ ((((size)-7) >> 8) & 0xFF) ^           \
     (((size)-7) & 0xFF) ^ ((fc)&0xFF) ^ ((pxor)&0xFF))

typedef struct
{
    SC_RtsEntryHeader_t hdr1;
    GNC_APP_NoArgsCmd_t cmd1;
} SC_RtsStruct002_t;

typedef union
{
    SC_RtsStruct002_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable002_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct002_t *)0)->member))

SC_RtsTable002_t SC_Rts002 = {
    /* 1: GNC ABORT */
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {CFE_MSG_CMD_HDR_INIT(GNC_APP_CMD_MID, SC_MEMBER_SIZE(cmd1), GNC_APP_ABORT_CC,
                                      DOCK_CKSUM(GNC_APP_CMD_MID, SC_MEMBER_SIZE(cmd1), GNC_APP_ABORT_CC, 0))},
};

CFE_TBL_FILEDEF(SC_Rts002, SC.RTS_TBL002, Dock FDIR link loss ABORT, sc_rts002.tbl)
