#ifndef SIM_IO_TBL_H
#define SIM_IO_TBL_H

#include "common_types.h"

#define SIM_IO_CFG_TBL_NAME "CfgTbl"
#define SIM_IO_CFG_TBL_FILE "/cf/sim_io_cfg_tbl.tbl"

#define SIM_IO_HOST_NAME_LEN 64

/*
** SIM_IO link configuration. Read once at init (sockets are opened then);
** a reloaded table takes effect on the next cFS start.
*/
typedef struct
{
    uint16 ListenPort;                     /* UDP port Unity sends SIM_STATE to   */
    uint16 DestPort;                       /* UDP port Unity listens for commands */
    char   DestHost[SIM_IO_HOST_NAME_LEN]; /* host name or dotted-quad of Unity   */
} SIM_IO_CfgTbl_t;

#endif /* SIM_IO_TBL_H */
