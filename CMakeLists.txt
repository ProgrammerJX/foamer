cmake_minimum_required(VERSION 3.20)
project(myfoam VERSION 0.1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Project-wide language settings.
#
# 历史：项目最初守 C++11；接入 OCCT 7.9+（要求 C++17，部分头里用 std::optional /
# inline variables）时升级到 C++17。Boost.filesystem 仍保留，避免上层代码大改 ──
# C++17 的 std::filesystem 接口与 Boost 几乎一致，将来如要彻底切换，对应文件
# 改三行 include 即可。
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
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

# ---------------------------------------------------------------------------
# OCCT 集成（参数化 B-rep / STEP / IGES）
#
# OFF（默认）：XFoam_MBrep::readFromStep / readFromIges / tessellate 抛
#              "OCCT disabled at build time" ── 整树 ~30s 内构建。
# ON：FetchContent 拉 OCCT 源码（约 200MB）并构建最小 CAD 子集：
#     FoundationClasses + ModelingData + ModelingAlgorithms + DataExchange。
#     不启用 Visualization / Draw / ApplicationFramework / FreeType / TBB /
#     TCL，避免外部依赖与编译时间膨胀。首次配置 + 编译耗时 ~15-30 min（多核）。
#
# 之所以放 OFF 默认：日常迭代不应被 OCCT 编译时间绑架；需要 STEP/IGES 的人
# 显式 `-DXFOAM_WITH_OCCT=ON` 即可。
#
# 版本：OCCT 7.9.x。要求 C++17（本项目已升），需要 CMake 3.20+。
# 仅在 MSVC 2019+ / GCC 9+ / Clang 12+ 上测过；其它编译器可能要打补丁。
# ---------------------------------------------------------------------------
option(XFOAM_WITH_OCCT      "Fetch + build OCCT for STEP/IGES support"  OFF)

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

# --- OCCT (optional, isolated source build via ExternalProject) -------------
# 为什么不用 FetchContent_MakeAvailable：OCCT 的 CMakeLists 把 ${CMAKE_SOURCE_DIR}
# 当成自己的源根目录（include 路径都基于此），作 add_subdirectory 子项目时会去
# myfoam 根目录下找 adm/cmake/*.cmake → 必失败。所以走 ExternalProject_Add 隔离
# 构建：OCCT 在它自己的 binary dir 里全程独立 configure + build + install，
# myfoam 只 import 装好的 .lib + 头。
#
# 安装路径：${CMAKE_BINARY_DIR}/_deps/occt-install
#   inc/   合并后的 OCCT 头（INSTALL_DIR_INCLUDE=inc）
#   lib/   静态库 TK*.lib（INSTALL_DIR_LIB=lib）
#
# 模块开关：只编 STEP/IGES 路径需要的四个 module，关掉 Visualization / Draw /
# ApplicationFramework / 各 USE_* 第三方依赖，单线避免拉系统包。
if(XFOAM_WITH_OCCT)
    message(STATUS "OCCT integration: ENABLED (ExternalProject build of OCCT 7.9.1; ~15-30 min first time)")

    include(ExternalProject)

    set(OCCT_INSTALL_DIR "${CMAKE_BINARY_DIR}/_deps/occt-install")
    set(OCCT_INC_DIR     "${OCCT_INSTALL_DIR}/inc")
    set(OCCT_LIB_DIR     "${OCCT_INSTALL_DIR}/lib")
    # 先把 inc 目录占住，否则 INTERFACE_INCLUDE_DIRECTORIES 指向不存在路径
    # CMake 3.26+ 会硬报错（CMP0145）。
    file(MAKE_DIRECTORY "${OCCT_INC_DIR}")

    # 与 myfoam 主项目一致用 C++17；OCCT 7.9 有 std::optional 等的使用，
    # C++11/14 编译可能挂。
    ExternalProject_Add(occt_external
        GIT_REPOSITORY  https://github.com/Open-Cascade-SAS/OCCT.git
        GIT_TAG         V7_9_1
        GIT_SHALLOW     TRUE
        UPDATE_DISCONNECTED TRUE      # 防止每次 cmake 都 git fetch
        PREFIX          "${CMAKE_BINARY_DIR}/_deps/occt-ep"
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_INSTALL_PREFIX=${OCCT_INSTALL_DIR}
            -DBUILD_CPP_STANDARD=C++17
            -DBUILD_LIBRARY_TYPE=Static
            -DBUILD_DOC_Overview=OFF
            -DBUILD_MODULE_FoundationClasses=ON
            -DBUILD_MODULE_ModelingData=ON
            -DBUILD_MODULE_ModelingAlgorithms=ON
            -DBUILD_MODULE_DataExchange=ON
            -DBUILD_MODULE_ApplicationFramework=OFF
            -DBUILD_MODULE_DETools=OFF
            -DBUILD_MODULE_Draw=OFF
            -DBUILD_MODULE_Visualization=OFF
            -DUSE_FREETYPE=OFF
            -DUSE_TBB=OFF
            -DUSE_VTK=OFF
            -DUSE_TK=OFF
            -DUSE_OPENGL=OFF
            -DUSE_GLES2=OFF
            -DUSE_D3D=OFF
            -DUSE_RAPIDJSON=OFF
            -DUSE_DRACO=OFF
            -DUSE_FFMPEG=OFF
            -DUSE_FREEIMAGE=OFF
            -DUSE_OPENVR=OFF
            -DINSTALL_DIR_LIB=lib
            -DINSTALL_DIR_BIN=bin
            -DINSTALL_DIR_INCLUDE=inc
        BUILD_BYPRODUCTS
            ${OCCT_LIB_DIR}/TKernel.lib
            ${OCCT_LIB_DIR}/TKMath.lib
            ${OCCT_LIB_DIR}/TKG2d.lib
            ${OCCT_LIB_DIR}/TKG3d.lib
            ${OCCT_LIB_DIR}/TKGeomBase.lib
            ${OCCT_LIB_DIR}/TKBRep.lib
            ${OCCT_LIB_DIR}/TKGeomAlgo.lib
            ${OCCT_LIB_DIR}/TKTopAlgo.lib
            ${OCCT_LIB_DIR}/TKPrim.lib
            ${OCCT_LIB_DIR}/TKBO.lib
            ${OCCT_LIB_DIR}/TKBool.lib
            ${OCCT_LIB_DIR}/TKShHealing.lib
            ${OCCT_LIB_DIR}/TKHLR.lib
            ${OCCT_LIB_DIR}/TKMesh.lib
            ${OCCT_LIB_DIR}/TKFeat.lib
            ${OCCT_LIB_DIR}/TKFillet.lib
            ${OCCT_LIB_DIR}/TKOffset.lib
            ${OCCT_LIB_DIR}/TKXSBase.lib
            ${OCCT_LIB_DIR}/TKDE.lib
            ${OCCT_LIB_DIR}/TKDESTEP.lib
            ${OCCT_LIB_DIR}/TKDEIGES.lib
    )

    # 把每个 OCCT 静态库包成 IMPORTED target；用 GLOBAL 让 tests/samples 也能引用。
    # add_dependencies(... occt_external) 保证 ninja 先把 OCCT install 跑完。
    set(_xfoam_occt_libs
        TKernel TKMath
        TKG2d TKG3d TKGeomBase TKBRep
        TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool TKShHealing TKHLR TKMesh
        TKFeat TKFillet TKOffset
        TKXSBase TKDE TKDESTEP TKDEIGES
    )
    foreach(_lib IN LISTS _xfoam_occt_libs)
        add_library(${_lib} STATIC IMPORTED GLOBAL)
        set_target_properties(${_lib} PROPERTIES
            IMPORTED_LOCATION "${OCCT_LIB_DIR}/${_lib}.lib"
            INTERFACE_INCLUDE_DIRECTORIES "${OCCT_INC_DIR}"
        )
        add_dependencies(${_lib} occt_external)
    endforeach()
endif()

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

if(XFOAM_WITH_OCCT)
    # 在 cpp 里通过 `#ifdef XFOAM_WITH_OCCT` 走 OCCT 路径。public 是因为 mbrep.h
    # 暴露 OcctData 的 pImpl 类型大小，源代码外部包 mbrep.h 也需要这个宏一致。
    target_compile_definitions(XFoam PUBLIC XFOAM_WITH_OCCT)

    # 链接顺序（从依赖最重到最轻）：DataExchange → ModelingAlgorithms →
    # ModelingData → FoundationClasses。MSVC 静态链接对顺序不敏感（resolves
    # all in one pass），但 ld 友好的顺序更清晰。
    target_link_libraries(XFoam
        PUBLIC
            # DataExchange (STEP/IGES)
            TKDESTEP TKDEIGES TKDE TKXSBase
            # ModelingAlgorithms
            TKMesh TKBO TKBool TKTopAlgo TKGeomAlgo TKShHealing TKPrim
            TKFeat TKFillet TKOffset TKHLR
            # ModelingData
            TKBRep TKG3d TKG2d TKGeomBase
            # FoundationClasses
            TKMath TKernel
    )
endif()

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
message(STATUS "  OCCT (STEP/IGES)    : ${XFOAM_WITH_OCCT}")
message(STATUS "======================================================")
message(STATUS "")
