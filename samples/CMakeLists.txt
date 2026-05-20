# Optional sample executables. The original tree has three sample folders;
# all three currently contain stub or empty main.cpp files. We register them
# only when their main.cpp is non-empty so an empty translation unit doesn't
# turn into a build error.

function(xfoam_add_sample name)
    set(dir "${CMAKE_CURRENT_SOURCE_DIR}/${name}")
    set(main "${dir}/main.cpp")
    if(NOT EXISTS "${main}")
        return()
    endif()

    file(SIZE "${main}" size)
    if(size EQUAL 0)
        message(STATUS "sample ${name}: main.cpp is empty, skipping.")
        return()
    endif()

    file(GLOB sample_sources CONFIGURE_DEPENDS
        "${dir}/*.cpp"
        "${dir}/*.h"
    )

    set(exe "x-sample-${name}")
    add_executable(${exe} ${sample_sources})
    target_link_libraries(${exe} PRIVATE XFoam Boost::filesystem Boost::system)
endfunction()

xfoam_add_sample(foam-core)
xfoam_add_sample(foam-blockmesh)
xfoam_add_sample(foam-snappyhexmesh)
xfoam_add_sample(foam-cmsh)
if(XFOAM_WITH_OCCT)
    xfoam_add_sample(foam-occt)
endif()
