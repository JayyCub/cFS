
# For the native builds, include a wrapper script to start cFS on the container.
if (${SIMULATION} MATCHES "^native")
    install(PROGRAMS ${CMAKE_CURRENT_LIST_DIR}/container-start DESTINATION cpu1)
endif()

set(APP_DEFINITION_TABLE_LIST)
set(APP_INCLUDE_TABLE_LIST
    $<TARGET_PROPERTY:ci_lab,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:to_lab,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:sample_app,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:hs,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:cf,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:md,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:mm,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:cs,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:fm,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:lc,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:sc,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:ds,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:hk,INCLUDE_DIRECTORIES>
    # docking mission apps: MIDs, command codes and packet layouts used by the
    # scheduler, TO_LAB, LC watchpoint, SC RTS, DS and HS tables
    $<TARGET_PROPERTY:gnc_app,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:sim_io,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:rcs,INCLUDE_DIRECTORIES>
)

if (TARGET sbn)
    list(APPEND APP_DEFINITION_TABLE_LIST
        CFS_SBN_ENABLED
    )
    list(APPEND APP_INCLUDE_TABLE_LIST
        $<TARGET_PROPERTY:sbn,INCLUDE_DIRECTORIES>
    )
endif()

# specify extra include dirs for the table builds that reference other apps'
# messages: to_lab + sch_lab (routing/scheduling), and the FDIR / recorder
# tables (lc watchpoints + actionpoints, sc RTSs, ds recorder, hs app monitor)
foreach(TBL_APP to_lab sch_lab lc sc ds hs)
    target_include_directories(${TBL_APP}.table INTERFACE ${APP_INCLUDE_TABLE_LIST})
    target_compile_definitions(${TBL_APP}.table INTERFACE ${APP_DEFINITION_TABLE_LIST})
endforeach()

# An example of an alternate table load file
add_cfe_tables(sample_app sample_app_alt1.c)
add_cfe_tables(to_lab to_lab_sub_alt.c)
add_cfe_tables(to_lab to_lab_sub_bad.c)
add_cfe_tables(lc lc_def_adt-test.c)

# This executes the startup script generator
install(SCRIPT ${MISSION_DEFS}/generate_startup.cmake)
install(CODE "generate_cfs_startup_script(\"${TGTNAME}/${INSTALL_SUBDIR}\" ${${TGTNAME}_APPLIST})")
