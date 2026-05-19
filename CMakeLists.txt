set(MODULE_NAME XFoam)

set(XFOAM_SCALAR "DOUBLE" CACHE STRING "FLOAT/DOUBLE/LONGDOUBLE")

if("FLOAT" STREQUAL ${XFOAM_SCALAR})
    add_definitions(-DXFOAM_SCALAR_FLOAT)
elseif("DOUBLE" STREQUAL ${XFOAM_SCALAR})
    add_definitions(-DXFOAM_SCALAR_DOUBLE)
elseif("LONGDOUBLE" STREQUAL ${XFOAM_SCALAR})
    add_definitions(-DXFOAM_SCALAR_LONGDOUBLE)
else()
    add_definitions(-DXFOAM_SCALAR_DOUBLE)
endif()

x_prepare_add_module()
x_add_module_subdir(foam)
x_add_module_subdir(tools)
x_add_module_subdir(block)
x_add_module_subdir(snappyhexmesh)
x_add_module_subdir(utilities)
x_add_module_subdir(primitive)
x_add_module_subdir(mesh)
x_add_module_subdir(topo)
x_add_module_subdir(cartesian)

add_nlohmann_json()
x_add_occt()
# XFoam library: no XCommon/XData; tests/sample still link XCommon below.
x_add_module()

###########################################

x_prepare_add_module()
set(TEST_MODULE_NAME foam-core)
add_nlohmann_json()
x_add_lib(XCommon)
x_add_lib(XFoam)
x_add_testing_module()

###########################################

x_prepare_add_module()
set(TEST_MODULE_NAME foam-mesh)
add_nlohmann_json()
x_add_lib(XCommon)
x_add_lib(XFoam)
x_add_testing_module()

##################################################

x_prepare_add_module()
set(SAMPLE_NAME foam-core)
x_add_occt()
x_add_lib(XFoam)
x_add_sample()
