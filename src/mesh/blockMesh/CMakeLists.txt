add_library(myfoam_blockMesh STATIC
    block/Block.cpp
)
target_link_libraries(myfoam_blockMesh PUBLIC myfoam_core)
target_include_directories(myfoam_blockMesh PUBLIC ${CMAKE_SOURCE_DIR}/src)
