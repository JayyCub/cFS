#include "cfe_tbl_filedef.h"
#include "sim_io_tbl.h"

/*
** Default SIM_IO link configuration — matches cfs-dev.sh port forwarding:
** Unity -> container on 5005, container -> host.docker.internal:5006.
*/
SIM_IO_CfgTbl_t SIM_IO_CfgTbl = {
    .ListenPort = 5005,
    .DestPort   = 5006,
    .DestHost   = "host.docker.internal",
};

CFE_TBL_FILEDEF(SIM_IO_CfgTbl, SIM_IO.CfgTbl, SIM_IO link configuration, sim_io_cfg_tbl.tbl)
