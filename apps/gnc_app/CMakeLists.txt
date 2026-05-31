project(GNC_APP C)

add_cfe_app(gnc_app
    fsw/src/gnc_app.c
    fsw/src/gnc_app_udp.c
)

target_include_directories(gnc_app PUBLIC fsw/inc fsw/src)

add_cfe_tables(gnc_app fsw/tables/gnc_param_tbl.c)
