cmake_minimum_required(VERSION 3.0.2)
project(frontend_searcher
	VERSION 0.1
	DESCRIPTION "Frontend path searching module")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Use -std=c++17 instead of -std=gnu++17
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # For clang-tools

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Compiler options
include(CheckCXXCompilerFlag)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")  # for PC
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")  # for nuc

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	map_utils
	rviz_utils
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES frontend_searcher
	CATKIN_DEPENDS roscpp rospy std_msgs map_utils rviz_utils
)

include_directories(
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	PRIVATE ${catkin_INCLUDE_DIRS}
	SYSTEM ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
	src/fake_search.cc
	src/PathSearch.cc
)
target_compile_options(${PROJECT_NAME} 
    PRIVATE 
		# for nuc
        # $<$<CONFIG:Release>:-O3>  # funroll-loops for searching loops

		# for PC
		$<$<CONFIG:Release>:-O3 -march=native -funroll-loops>  # funroll-loops for searching loops
		
        $<$<CONFIG:Debug>:-g -O0 -Wall -Wextra>
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC ${catkin_LIBRARIES}
    PRIVATE Eigen3::Eigen
)

# Test executables
foreach(target test_fake_search test_path_search)
    add_executable(${target} example/${target}.cc)
	target_compile_options(${target} 
		PRIVATE -g -O0 -Wall -Wextra
	)
    target_link_libraries(${target} 
        PRIVATE 
            ${PROJECT_NAME}
            ${catkin_LIBRARIES}
    )
endforeach()
