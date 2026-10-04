project(NAV C)

add_cfe_app(nav
    fsw/src/nav.c
    fsw/src/nav_filter.c
)

# sim_io/fsw/inc: SIM_STATE (the raw sensor readings NAV consumes) and the
# SimLink payload layout inside it.
target_include_directories(nav PUBLIC fsw/inc fsw/src ${sim_io_MISSION_DIR}/fsw/inc)

target_link_libraries(nav m)

add_cfe_tables(nav fsw/tables/nav_cfg_tbl.c)
