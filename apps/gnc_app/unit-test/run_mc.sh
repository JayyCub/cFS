#!/bin/bash
# Build and run the docking Monte Carlo (dock_mc.c) inside the cfs-dev container.
#
#   apps/gnc_app/unit-test/run_mc.sh [build dir] [-- dock_mc args]
#     build dir  a configured cFS build tree for its generated headers
#                (default build-native_std; must have been built once)
#   e.g.  run_mc.sh build-native_std -- -n 200 -m abort
#
# Run from the cFS root. Compiles the flight sources unmodified with the same
# warnings as the real build (-std=c99 -pedantic -Wall -Werror).
set -e
cd "$(dirname "$0")/../../.."
BUILD=${1:-build-native_std}
[ "$1" != "" ] && [ "$1" != "--" ] && shift
[ "$1" == "--" ] && shift
INC="-Iapps/gnc_app/unit-test -Iapps/nav/unit-test/stubs
     -I$BUILD/inc -I$BUILD/native/default_cpu1/inc -Isample_defs/inc
     -Iapps/gnc_app/fsw/inc -Iapps/gnc_app/fsw/src -Iapps/nav/fsw/inc -Iapps/nav/fsw/src
     -Iapps/sim_io/fsw/inc -Iapps/rcs/fsw/inc -Iapps/rcs/fsw/src -Iapps/lc/fsw/inc
     -Icfe/modules/core_api/fsw/inc -Icfe/modules/config/fsw/inc -Icfe/modules/es/fsw/inc
     -Icfe/modules/evs/fsw/inc -Icfe/modules/fs/fsw/inc -Icfe/modules/sb/fsw/inc
     -Icfe/modules/tbl/fsw/inc -Icfe/modules/time/fsw/inc -Icfe/modules/msg/fsw/inc
     -Icfe/modules/resourceid/fsw/inc -Iosal/src/os/inc -I$BUILD/native/default_cpu1/osal/inc
     -Ipsp/fsw/inc -I$BUILD/native/default_cpu1/psp/inc"
FLAGS="-std=c99 -pedantic -Wall -Werror -O2 -D_XOPEN_SOURCE=600 -D_LINUX_OS_ -D_POSIX_OS_"
gcc $FLAGS $INC -o /tmp/dock_mc \
    apps/gnc_app/unit-test/dock_mc.c apps/gnc_app/unit-test/cfe_stubs.c \
    apps/gnc_app/fsw/src/gnc_app.c apps/nav/fsw/src/nav_filter.c apps/rcs/fsw/src/rcs_alloc.c -lm
/tmp/dock_mc "$@"
