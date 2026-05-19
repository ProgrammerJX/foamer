# Core library: primitives + polyMesh data structures.
add_library(myfoam_core STATIC
    meshes/polyMesh/Face.cpp
)

target_include_directories(myfoam_core PUBLIC ${CMAKE_SOURCE_DIR}/src)
