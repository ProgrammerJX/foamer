cmake_minimum_required(VERSION 3.20)
project(myfoam VERSION 0.1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Project-wide language settings.
#
# Hard constraint from project ownership: C++11 only. Modern std::filesystem
# (C++17) is therefore unavailable, which is why the dependency on
# boost::filesystem is retained and downloaded below via FetchContent.
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(MSVC)
    add_compile_options(/utf-8 /MP /W3 /permissive-)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(XFOAM_BUILD_TESTS    "Build doctest-based unit tests"           ON)
option(XFOAM_BUILD_SAMPLES  "Build sample executables under samples/"  OFF)

# Scalar precision; mirrors the XFOAM_SCALAR variable of the original tree.
# foam_code.md states XFoam_Scalar is hard-typedef'd to double regardless,
# so this only flips the legacy preprocessor flags for any code that still
# checks them.
set(XFOAM_SCALAR "DOUBLE" CACHE STRING "FLOAT | DOUBLE | LONGDOUBLE")
if(XFOAM_SCALAR STREQUAL "FLOAT")
    add_compile_definitions(XFOAM_SCALAR_FLOAT)
elseif(XFOAM_SCALAR STREQUAL "LONGDOUBLE")
    add_compile_definitions(XFOAM_SCALAR_LONGDOUBLE)
else()
    add_compile_definitions(XFOAM_SCALAR_DOUBLE)
endif()

# ---------------------------------------------------------------------------
# Third-party libraries: download + build via FetchContent.
#
# Boost is fetched via the official cmake-packaged release tarball. Only the
# requested libraries (and their transitive deps) are configured, so the
# build cost is roughly that of boost::filesystem alone.
#
# doctest is a single-header library; we fetch the repo and use its header.
# ---------------------------------------------------------------------------
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

# --- doctest -----------------------------------------------------------------
FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG        v2.4.11
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(doctest)

# --- Boost (filesystem + system) --------------------------------------------
set(BOOST_INCLUDE_LIBRARIES filesystem system)
set(BOOST_ENABLE_CMAKE ON)
# Boost source URL hosts the cmake-packaged release. The hash pins the
# specific tarball so reconfigures are reproducible.
FetchContent_Declare(
    Boost
    URL                  https://github.com/boostorg/boost/releases/download/boost-1.85.0/boost-1.85.0-cmake.tar.xz
    URL_HASH             SHA256=0a9cc56ceae46986f5f4d43fe0311d90cf6d2fa9028258a95cab49ffdacf92ad
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(Boost)

# ---------------------------------------------------------------------------
# XFoam library
# ---------------------------------------------------------------------------
# Source layout mirrors OpenFOAM-style separation between inc/ and src/.
file(GLOB_RECURSE XFOAM_HEADERS CONFIGURE_DEPENDS "inc/XFoam/*.h")
file(GLOB_RECURSE XFOAM_SOURCES CONFIGURE_DEPENDS "src/*.cpp")

add_library(XFoam STATIC ${XFOAM_SOURCES} ${XFOAM_HEADERS})

# Static build of XFoam: the XFoam_API macro should expand to nothing. The
# legacy header xfoam_types.h gates this on X_BUILD_STATIC.
target_compile_definitions(XFoam PUBLIC X_BUILD_STATIC)

target_include_directories(XFoam
    PUBLIC
        ${CMAKE_SOURCE_DIR}/inc
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(XFoam
    PUBLIC
        Boost::filesystem
        Boost::system
)

# Mirror the source tree in IDE explorers (MSVC, Xcode).
source_group(TREE "${CMAKE_SOURCE_DIR}/inc" PREFIX "Header Files" FILES ${XFOAM_HEADERS})
source_group(TREE "${CMAKE_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${XFOAM_SOURCES})

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(XFOAM_BUILD_TESTS)
    enable_testing()
    include(CTest)
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Samples
# ---------------------------------------------------------------------------
if(XFOAM_BUILD_SAMPLES)
    add_subdirectory(samples)
endif()

# ---------------------------------------------------------------------------
# Configuration summary
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "================ myfoam configuration ================")
message(STATUS "  CMake               : ${CMAKE_VERSION}")
message(STATUS "  Generator           : ${CMAKE_GENERATOR}")
message(STATUS "  Compiler            : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  C++ standard        : C++${CMAKE_CXX_STANDARD}")
message(STATUS "  Build type          : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Scalar precision    : ${XFOAM_SCALAR}")
message(STATUS "  Build tests         : ${XFOAM_BUILD_TESTS}")
message(STATUS "  Build samples       : ${XFOAM_BUILD_SAMPLES}")
message(STATUS "======================================================")
message(STATUS "")
