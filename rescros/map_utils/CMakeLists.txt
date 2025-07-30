cmake_minimum_required(VERSION 3.20)
project(map_utils
	VERSION 0.1
	DESCRIPTION "Map utilities for mogen"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")  # for PC
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")  # for nuc

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	pcl_conversions
	visualization_msgs
	cv_bridge
	message_filters
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES map_utils
	CATKIN_DEPENDS roscpp rospy std_msgs
	DEPENDS OpenCV PCL
)

include_directories(
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	PRIVATE
		${catkin_INCLUDE_DIRS}
	SYSTEM
		${EIGEN3_INCLUDE_DIR}
		${PCL_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
	src/ObstacleGenerator.cc
	src/MapBridge.cc
	src/RayCaster.cc
	src/SDFMap.cc
)
target_compile_options(${PROJECT_NAME}
    PRIVATE
		# $<$<CONFIG:Release>:-O3 -march=native -funroll-loops>  # for PC
		$<$<CONFIG:Release>:-O3>  # for nuc
        $<$<CONFIG:Debug>:-g -O0 -Wall -Wextra>
)
target_link_libraries(${PROJECT_NAME}
	PUBLIC
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		${OpenCV_LIBRARIES}
)

# examples
set(TEST_TARGETS
    test_random_map_generation
    test_map_bridge
    test_sdf_map
)

foreach(target ${TEST_TARGETS})
    add_executable(${target} 
        example/${target}.cc
        $<$<STREQUAL:${target},test_random_map_generation>:
			src/ObstacleGenerator.cc>
        $<$<STREQUAL:${target},test_map_bridge>:
            src/ObstacleGenerator.cc
            src/MapBridge.cc
            src/RayCaster.cc
            src/SDFMap.cc>
        $<$<STREQUAL:${target},test_sdf_map>:
            src/SDFMap.cc
            src/RayCaster.cc>
    )
    target_compile_options(${target}
        PRIVATE -g -O0 -Wall -Wextra
    )
    target_link_libraries(${target}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )
endforeach()