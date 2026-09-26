/*
** SC RTS 001 — docking mission power-on sequence (realism phase 3)
**
** Overrides apps/sc/fsw/tables/sc_rts001.c. SC starts RTS 1 automatically on
** a power-on reset (RTS_ID_AUTO_POWER_ON), so this is the "bring FDIR up"
** sequence every boot runs:
**   1. LC -> ACTIVE. LC powers on DISABLED (LC_STATE_POWER_ON_RESET); until
**      this command it evaluates nothing and requests no RTSs.
**   2. Enable the fault-response RTSs 2-4. SC loads every RTS disabled; LC's
**      start request for a disabled RTS is rejected, so without this step the
**      actionpoints would fire into nothing.
**
** Checksums: SC validates each command's checksum before sending it (header
** update is off), so they are computed here from the exact header bytes.
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

/*
** CCSDS command checksum: XOR of every byte of the packet must be 0xFF.
** Primary header: MID (2), sequence {0xC0, 0x00}, length-7 (2); secondary
** header: function code, checksum; PXOR = XOR of all payload bytes.
*/
#define DOCK_CKSUM(mid, size, fc, pxor)                                                          \
    (0xFF ^ (((mid) >> 8) & 0xFF) ^ ((mid)&0xFF) ^ 0xC0 ^ ((((size)-7) >> 8) & 0xFF) ^           \
     (((size)-7) & 0xFF) ^ ((fc)&0xFF) ^ ((pxor)&0xFF))
#define DOCK_U16_XOR(v) (((v)&0xFF) ^ (((v) >> 8) & 0xFF))

typedef struct
{
    SC_RtsEntryHeader_t  hdr1;
    LC_SetLCStateCmd_t   cmd1;
    SC_RtsEntryHeader_t  hdr2;
    SC_EnableRtsGrpCmd_t cmd2;
} SC_RtsStruct001_t;

typedef union
{
    SC_RtsStruct001_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable001_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct001_t *)0)->member))

#define RTS1_LC_STATE   LC_STATE_ACTIVE
#define RTS1_FIRST_RTS  2
#define RTS1_LAST_RTS   4

SC_RtsTable001_t SC_Rts001 = {
    /* 1: LC active */
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {CFE_MSG_CMD_HDR_INIT(LC_CMD_MID, SC_MEMBER_SIZE(cmd1), LC_SET_LC_STATE_CC,
                                      DOCK_CKSUM(LC_CMD_MID, SC_MEMBER_SIZE(cmd1), LC_SET_LC_STATE_CC,
                                                 DOCK_U16_XOR(RTS1_LC_STATE)))},
    .rts.cmd1.Payload.NewLCState = RTS1_LC_STATE,

    /* 2: enable fault-response RTSs 2..4 */
    .rts.hdr2.WakeupCount = 0,
    .rts.cmd2 = {CFE_MSG_CMD_HDR_INIT(SC_CMD_MID, SC_MEMBER_SIZE(cmd2), SC_ENABLE_RTS_GRP_CC,
                                      DOCK_CKSUM(SC_CMD_MID, SC_MEMBER_SIZE(cmd2), SC_ENABLE_RTS_GRP_CC,
                                                 DOCK_U16_XOR(RTS1_FIRST_RTS) ^ DOCK_U16_XOR(RTS1_LAST_RTS)))},
    .rts.cmd2.Payload.FirstRtsNum = SC_RTS_NUM_INITIALIZER(RTS1_FIRST_RTS),
    .rts.cmd2.Payload.LastRtsNum  = SC_RTS_NUM_INITIALIZER(RTS1_LAST_RTS),
};

CFE_TBL_FILEDEF(SC_Rts001, SC.RTS_TBL001, Dock power-on FDIR bring-up, sc_rts001.tbl)
