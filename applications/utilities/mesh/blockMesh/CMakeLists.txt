# Stage 3 will turn this into the blockMesh executable.
add_executable(blockMesh main.cpp)
target_link_libraries(blockMesh PRIVATE myfoam_core myfoam_blockMesh)
