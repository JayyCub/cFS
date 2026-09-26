/*
** SC RTS 003 — axial actuator anomaly response: GNC HOLD (realism phase 3)
**
** Overrides apps/sc/fsw/tables/sc_rts003.c. Started by LC actionpoint 1
** (sample_defs/cpu1/tables/lc_def_adt.c) when commanded axial burns deliver under
** half their predicted Δv for 3 consecutive cycles during APPROACH. HOLD stops
** the closure but keeps station-keeping (unlike ABORT, which coasts); the
** operator decides whether to GO or ABORT.
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
} SC_RtsStruct003_t;

typedef union
{
    SC_RtsStruct003_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable003_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct003_t *)0)->member))

SC_RtsTable003_t SC_Rts003 = {
    /* 1: GNC HOLD */
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {CFE_MSG_CMD_HDR_INIT(GNC_APP_CMD_MID, SC_MEMBER_SIZE(cmd1), GNC_APP_HOLD_CC,
                                      DOCK_CKSUM(GNC_APP_CMD_MID, SC_MEMBER_SIZE(cmd1), GNC_APP_HOLD_CC, 0))},
};

CFE_TBL_FILEDEF(SC_Rts003, SC.RTS_TBL003, Dock FDIR actuator HOLD, sc_rts003.tbl)
