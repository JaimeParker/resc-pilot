cmake_minimum_required(VERSION 3.24)
project(resclib VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../rescrl/resclib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../rescrl/resclib
)

#set(CMAKE_BUILD_TYPE Debug)

find_package(pybind11 REQUIRED)
if (NOT pybind11_FOUND)
    message("install pybind11 using pip install pybind11 or from https://github.com/pybind/pybind11/releases")
    message(FATAL_ERROR "pybind11 not found!")
endif ()

if (WIN32)
    set(Eigen3_DIR "C:/Program Files (x86)/Eigen3/share/eigen3/cmake")
endif ()
find_package(Eigen3 REQUIRED)
if (NOT EIGEN3_FOUND)
    message(FATAL_ERROR "Eigen3 not found.")
else ()
    message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
endif ()

# with new dependencies, add it to /cmake/ResclibConfig.cmake.in

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(resclib SHARED
        include/Resclib.h
        src/control/RateControl.cc
        include/RateControl.h
        src/math/RK4.cc
        include/RK4.h
        src/map/SignedDistanceField.cc
        include/SignedDistanceField.h
        src/dynamics/QuadrotorDynamics.cc
        include/QuadrotorDynamics.h
        include/Params.h
        src/math/Rotation.cc
        include/Rotation.h
        src/math/Convertor.cc
        include/Convertor.h
        include/drone/DroneBase.h
        include/drone/Iris.h
        include/drone/IMP250.h
        src/control/ControlAllocator.cc
        include/ControlAllocator.h
        src/map/K_GPEP.cc
        include/K_GPEP.h
)

set_target_properties(resclib PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)

# add pybind11 modules
pybind11_add_module(sdf_cpp src/map/SignedDistanceFieldPyFace.cc)
set_target_properties(sdf_cpp PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../rescrl/resclib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../rescrl/resclib
)
pybind11_add_module(quadrotor_dynamics_cpp src/dynamics/QuadrotorDynamicsPyFace.cc)
set_target_properties(quadrotor_dynamics_cpp PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../rescrl/resclib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../rescrl/resclib
)
pybind11_add_module(params_cpp src/params/ParamsPyFace.cc)
set_target_properties(params_cpp PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../rescrl/resclib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../rescrl/resclib
)
pybind11_add_module(kgpep_cpp src/map/KGPEPPyFace.cc)
set_target_properties(kgpep_cpp PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../rescrl/resclib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../rescrl/resclib
)

# examples
add_executable(test_rk4 example/test_rk4.cc)
add_executable(test_sdf example/test_sdf.cc)
add_executable(test_rate_ctrl example/test_rate_ctrl.cc)
add_executable(test_dynamic example/test_dynamic.cc)
add_executable(test_control_allocator example/test_control_allocator.cc)
add_executable(test_k_gpep example/test_kgpep.cc)

# resclib link
target_link_libraries(resclib ${TORCH_LIBRARIES})

# test link
target_link_libraries(test_rk4 resclib)
target_link_libraries(test_sdf resclib)
target_link_libraries(test_rate_ctrl resclib)
target_link_libraries(test_dynamic resclib)
target_link_libraries(test_control_allocator resclib)
target_link_libraries(test_k_gpep resclib)

# pybind11 interface links
target_link_libraries(sdf_cpp PRIVATE resclib)
target_link_libraries(quadrotor_dynamics_cpp PRIVATE resclib)
target_link_libraries(params_cpp PRIVATE resclib)
target_link_libraries(kgpep_cpp PRIVATE resclib)

include(CMakePackageConfigHelpers)

# install resclib to system (optional)
install(TARGETS resclib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
)
install(DIRECTORY include/
        DESTINATION include/Resclib
        FILES_MATCHING PATTERN "*.h"
)

write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/ResclibConfigVersion.cmake"
        VERSION 1.0.0
        COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ResclibConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/ResclibConfig.cmake"
        INSTALL_DESTINATION lib/cmake/Resclib
)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/ResclibConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/ResclibConfigVersion.cmake"
        DESTINATION lib/cmake/Resclib
)

# Add uninstall target
add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ResclibUninstall.cmake)
