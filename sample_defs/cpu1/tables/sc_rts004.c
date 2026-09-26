/*
** SC RTS 004 — FDIR re-arm (realism phase 3)
**
** Overrides apps/sc/fsw/tables/sc_rts004.c. When an LC actionpoint fires it
** goes PASSIVE, so the same fault can't restart its RTS every sample — and a
** second occurrence would go unanswered. After recovering (link back, GO
** sent), the operator starts this RTS (SC START_RTS 4, "Re-arm FDIR" on the
** ground console) to set every defined actionpoint ACTIVE again. An AP whose
** condition is still true fires again on its next sample, which is the point.
*/

#include "cfe.h"
#include "cfe_tbl_filedef.h"

#include "sc_tbldefs.h"
#include "sc_platform_cfg.h"
#include "sc_msgdefs.h"
#include "sc_msgids.h"
#include "sc_msg.h"
#include "lc_msgids.h"
#include "lc_msg.h"

/* CCSDS command checksum — see sc_rts001.c */
#define DOCK_CKSUM(mid, size, fc, pxor)                                                          \
    (0xFF ^ (((mid) >> 8) & 0xFF) ^ ((mid)&0xFF) ^ 0xC0 ^ ((((size)-7) >> 8) & 0xFF) ^           \
     (((size)-7) & 0xFF) ^ ((fc)&0xFF) ^ ((pxor)&0xFF))
#define DOCK_U16_XOR(v) (((v)&0xFF) ^ (((v) >> 8) & 0xFF))

typedef struct
{
    SC_RtsEntryHeader_t hdr1;
    LC_SetAPStateCmd_t  cmd1;
} SC_RtsStruct004_t;

typedef union
{
    SC_RtsStruct004_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable004_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct004_t *)0)->member))

SC_RtsTable004_t SC_Rts004 = {
    /* 1: every defined actionpoint -> ACTIVE */
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {CFE_MSG_CMD_HDR_INIT(LC_CMD_MID, SC_MEMBER_SIZE(cmd1), LC_SET_AP_STATE_CC,
                                      DOCK_CKSUM(LC_CMD_MID, SC_MEMBER_SIZE(cmd1), LC_SET_AP_STATE_CC,
                                                 DOCK_U16_XOR(LC_ALL_ACTIONPOINTS) ^
                                                     DOCK_U16_XOR(LC_APSTATE_ACTIVE)))},
    .rts.cmd1.Payload.APNumber   = LC_ALL_ACTIONPOINTS,
    .rts.cmd1.Payload.NewAPState = LC_APSTATE_ACTIVE,
};

CFE_TBL_FILEDEF(SC_Rts004, SC.RTS_TBL004, Dock FDIR re-arm LC APs, sc_rts004.tbl)
