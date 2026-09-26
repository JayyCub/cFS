project(SIM_IO C)

add_cfe_app(sim_io
    fsw/src/sim_io.c
    fsw/src/sim_io_link.c
)

target_include_directories(sim_io PUBLIC fsw/inc fsw/src)

add_cfe_tables(sim_io fsw/tables/sim_io_cfg_tbl.c)
