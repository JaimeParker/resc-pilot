cmake_minimum_required(VERSION 3.24.2)
project(backend_optimizer
	VERSION 0.1
	DESCRIPTION "Backend optimization module using LibTorch for mogen"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(Torch_DIR ~/3rdParty/libtorch/share/cmake/Torch)

find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	roslib
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES backend_optimizer
	CATKIN_DEPENDS roscpp rospy std_msgs roslib
)

include_directories(
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${catkin_INCLUDE_DIRS}
    SYSTEM
        ${EIGEN3_INCLUDE_DIR}
)

add_library(backend_optimizer
    src/rl_motion.cc
)
target_compile_options(backend_optimizer
    PRIVATE
        $<$<CONFIG:Release>:-O3 -march=native>
        $<$<CONFIG:Debug>:-g -O0 -Wall -Wextra>
)
# FIXME: 如果在下方加上PRIVATE则会导致一些plan_admin中torch头文件找不到的问题
target_link_libraries(backend_optimizer
	${catkin_LIBRARIES}
	${TORCH_LIBRARIES}
)

# test model prediction
add_executable(test_model_prediction
    example/test_model_prediction.cc
    src/rl_motion.cc
)
target_compile_options(test_model_prediction
    PRIVATE
        -g -O0 -Wall -Wextra -Wpedantic
)
target_link_libraries(test_model_prediction
	${catkin_LIBRARIES}
	${TORCH_LIBRARIES}
)
