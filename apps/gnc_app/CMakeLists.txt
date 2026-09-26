project(GNC_APP C)

add_cfe_app(gnc_app
    fsw/src/gnc_app.c
)

# sim_io/fsw/inc: SIM_STATE (input) and THRUSTER_CMD (actuator-health check);
# rcs/fsw/inc: ACT_REQ (GNC output). GNC owns no sockets and no thruster data.
target_include_directories(gnc_app PUBLIC fsw/inc fsw/src ${sim_io_MISSION_DIR}/fsw/inc ${rcs_MISSION_DIR}/fsw/inc)

# lc: GNC closes each control cycle with an LC_SAMPLE_AP request for its
# cycle-rate actionpoint group (headers only — no link-time dependency).
add_cfe_app_dependency(gnc_app lc)

# tanf() (corridor-radius calc in SelectPhase/ComputeControl) is a genuine libm
# call, unlike fabsf() which GCC inlines as a builtin — link libm explicitly so
# the module doesn't fail to load with "undefined symbol: tanf".
target_link_libraries(gnc_app m)

add_cfe_tables(gnc_app fsw/tables/gnc_param_tbl.c)
