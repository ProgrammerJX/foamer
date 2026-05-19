add_executable(test_primitives test_primitives.cpp)
target_link_libraries(test_primitives PRIVATE myfoam_core)
add_test(NAME test_primitives COMMAND test_primitives)

add_executable(test_block test_block.cpp)
target_link_libraries(test_block PRIVATE myfoam_core myfoam_blockMesh)
add_test(NAME test_block COMMAND test_block)
