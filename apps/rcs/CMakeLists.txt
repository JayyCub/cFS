project(RCS C)

add_cfe_app(rcs
    fsw/src/rcs.c
    fsw/src/rcs_alloc.c
)

# sim_io/fsw/inc: THRUSTER_CMD (this app's output) is the valve-driver
# interface message SIM_IO forwards to Unity.
target_include_directories(rcs PUBLIC fsw/inc fsw/src ${sim_io_MISSION_DIR}/fsw/inc)

target_link_libraries(rcs m)

add_cfe_tables(rcs fsw/tables/rcs_thr_tbl.c)
