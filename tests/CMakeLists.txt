# ---------------------------------------------------------------------------
# Two test executables, one per top-level test group.
#
# foam-core covers utilities, primitives and core mesh classes.
# foam-mesh covers blockMesh and the full PolyMesh assembly path.
# ---------------------------------------------------------------------------

function(xfoam_add_test_executable group)
    set(group_dir "${CMAKE_CURRENT_SOURCE_DIR}/${group}")
    if(NOT EXISTS "${group_dir}/main.cpp")
        message(WARNING "Test group ${group}: ${group_dir}/main.cpp is missing; skipping.")
        return()
    endif()

    file(GLOB group_sources CONFIGURE_DEPENDS
        "${group_dir}/*.cpp"
        "${group_dir}/*.h"
    )

    set(exe "x-test-${group}")
    add_executable(${exe} ${group_sources})

    target_link_libraries(${exe}
        PRIVATE
            XFoam
            doctest
            Boost::filesystem
            Boost::system
    )

    target_include_directories(${exe} PRIVATE "${group_dir}")

    add_test(NAME ${exe} COMMAND ${exe})
    set_tests_properties(${exe} PROPERTIES
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    )

    source_group(TREE "${group_dir}" FILES ${group_sources})
endfunction()

xfoam_add_test_executable(foam-core)
xfoam_add_test_executable(foam-mesh)
xfoam_add_test_executable(foam-snap)
# foam-topo 是 OCCT 路径的端到端测试；OCCT OFF 时跳过整个 group。
# 里面的测试会用 BRepPrimAPI_MakeBox 自造 shape → 写 STEP → 回读 → 验证；
# 因此运行不依赖任何外部 fixture。
if(XFOAM_WITH_OCCT)
    xfoam_add_test_executable(foam-topo)
endif()
