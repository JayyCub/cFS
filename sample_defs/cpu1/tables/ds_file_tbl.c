/*
** DS Destination File Table — docking mission flight data recorder
** (realism phase 3)
**
** Overrides apps/ds/fsw/tables/ds_file_tbl.c. Two onboard recorder files:
**
**   0  /cf/fdr_evs<seq>.dat  every EVS event (the onboard event log)
**   1  /cf/fdr_gnc<seq>.dat  per-cycle GNC STATE + valve commands, plus the
**                            GNC / RCS / SIM_IO / LC / SC housekeeping — the
**                            data a post-flight analysis needs (what the
**                            "GNC #" console lines used to be abused for)
**
** Which packets go to which file is ds_filter_tbl.c. tools/fdr_decode.py turns
** a recorder file into CSV. /cf is used because it is the only volume the
** native PSP maps; on hardware this would be a RAM disk or mass memory.
** Sequence counts restart each boot, so a new run overwrites the previous
** run's files — copy them off first to keep them.
**
** Unused entries (2..DS_DEST_FILE_CNT-1) are all-zero, which is what DS
** expects for an unused destination, so they are left to C zero-fill.
*/

#include "cfe_tbl_filedef.h"
#include "ds_mission_cfg.h"
#include "ds_platform_cfg.h"
#include "ds_tbl.h"
#include "ds_msg.h"

DS_DestFileTable_t DS_DestFileTable = {
    .Descriptor = "Dock flight data recorder",
    .File =
        {
            /* 0: onboard event log */
            {
                .Movename      = DS_EMPTY_STRING,
                .Pathname      = "/cf/",
                .Basename      = "fdr_evs",
                .Extension     = ".dat",
                .FileNameType  = DS_BY_COUNT,
                .EnableState   = DS_ENABLED,
                .MaxFileSize   = (1024 * 1024 * 4), /* 4 MB */
                .MaxFileAge    = (60 * 60 * 4),     /* 4 hours */
                .SequenceCount = 1,
            },
            /* 1: GNC flight data (~0.7 kB/s at real time) */
            {
                .Movename      = DS_EMPTY_STRING,
                .Pathname      = "/cf/",
                .Basename      = "fdr_gnc",
                .Extension     = ".dat",
                .FileNameType  = DS_BY_COUNT,
                .EnableState   = DS_ENABLED,
                .MaxFileSize   = (1024 * 1024 * 32), /* 32 MB */
                .MaxFileAge    = (60 * 60 * 4),      /* 4 hours */
                .SequenceCount = 1,
            },
        },
};

CFE_TBL_FILEDEF(DS_DestFileTable, DS.FILE_TBL, Dock FDR files, ds_file_tbl.tbl)
