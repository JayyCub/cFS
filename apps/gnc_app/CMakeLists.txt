project(GNC_APP C)

add_cfe_app(gnc_app
    fsw/src/gnc_app.c
    fsw/src/gnc_app_udp.c
)

target_include_directories(gnc_app PUBLIC fsw/inc fsw/src)

# tanf() (corridor-radius calc in SelectPhase/ComputeControl) is a genuine libm
# call, unlike fabsf() which GCC inlines as a builtin — link libm explicitly so
# the module doesn't fail to load with "undefined symbol: tanf".
target_link_libraries(gnc_app m)

add_cfe_tables(gnc_app fsw/tables/gnc_param_tbl.c)
