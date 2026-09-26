project(GNC_APP C)

add_cfe_app(gnc_app
    fsw/src/gnc_app.c
)

# sim_io/fsw/inc provides the SIM_STATE / WRENCH_CMD Software Bus message
# definitions — GNC's only interface to the vehicle (it owns no sockets).
target_include_directories(gnc_app PUBLIC fsw/inc fsw/src ${sim_io_MISSION_DIR}/fsw/inc)

# tanf() (corridor-radius calc in SelectPhase/ComputeControl) is a genuine libm
# call, unlike fabsf() which GCC inlines as a builtin — link libm explicitly so
# the module doesn't fail to load with "undefined symbol: tanf".
target_link_libraries(gnc_app m)

add_cfe_tables(gnc_app fsw/tables/gnc_param_tbl.c)
