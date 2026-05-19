cmake_minimum_required(VERSION 3.16)
project(myfoam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
    add_compile_options(/W4 /permissive- /utf-8)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# All headers under src/ are accessible as <module/path/header.hpp>
include_directories(${CMAKE_SOURCE_DIR}/src)

enable_testing()

add_subdirectory(src/OpenFOAM)
add_subdirectory(src/mesh/blockMesh)
add_subdirectory(applications/utilities/mesh/blockMesh)
add_subdirectory(tests)
