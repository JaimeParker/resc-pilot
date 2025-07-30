#cmake_minimum_required(VERSION 3.0.2)
#project(plan_admin
#	VERSION 0.1
#	DESCRIPTION "Planning administration module for mogen"
#)
#
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#
#if(NOT CMAKE_BUILD_TYPE)
#	# 一旦改成release，kgpep就会出问题，而不用kgpep的模型就没问题
#	# 说明release对kgpep的优化有问题，或是kgpep本身有问题
#	# 最新发现改成Debug，kgpep也有问题，只有用老版本的cmakelist（即下面的）才没问题
#    set(CMAKE_BUILD_TYPE "Release")
#endif()
#
#set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
#
#find_package(Eigen3 REQUIRED)
#find_package(catkin REQUIRED COMPONENTS
#	roscpp
#	rospy
#	std_msgs
#	mavros_msgs
#	geometry_msgs
#	map_utils
#	rviz_utils
#	backend_optimizer
#	frontend_searcher
#)
#
#catkin_package(
#	INCLUDE_DIRS include
#	LIBRARIES plan_admin
#	CATKIN_DEPENDS roscpp rospy std_msgs map_utils rviz_utils frontend_searcher
#		backend_optimizer mavros_msgs
#)
#
#include_directories(
#    PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>
#    PRIVATE
#        ${catkin_INCLUDE_DIRS}
#    SYSTEM
#        ${EIGEN3_INCLUDE_DIR}
#)
#
## main node of mogen
#add_executable(mogen_node src/mogen_node.cc
#	src/ctrl_point_gen.cc
#	src/motion_gen.cc
#	src/mogen_replan_fsm.cc
#	src/mogen_plan_manager.cc
#	src/k_gpep.cc
#)
#target_compile_options(mogen_node
#    PRIVATE
#        $<$<CONFIG:Release>:-O3 -march=native>
#        $<$<CONFIG:Debug>:-g -O0 -Wall -Wextra>
#)
#target_link_libraries(mogen_node PRIVATE ${catkin_LIBRARIES})
#
## a simple test example, might be removed later
#add_executable(mogen_node_test
#    example/plan_admin.cc
#    src/ctrl_point_gen.cc
#    src/motion_gen.cc
#    src/k_gpep.cc
#)
#target_compile_options(mogen_node_test PRIVATE PRIVATE -g -O0 -Wall -Wextra)
#target_link_libraries(mogen_node_test PRIVATE ${catkin_LIBRARIES})


# old version
cmake_minimum_required(VERSION 3.0.2)
project(plan_admin)

set(CMAKE_BUILD_TYPE "Debug")  # 如果改成Release，kgpep就会出问题，而不用kgpep的模型就没问题
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	mavros_msgs
	geometry_msgs
	map_utils
	rviz_utils
	backend_optimizer
	frontend_searcher
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES plan_admin
	CATKIN_DEPENDS roscpp rospy std_msgs map_utils rviz_utils frontend_searcher
	backend_optimizer mavros_msgs
)

include_directories(
	include
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

# main node of mogen
add_executable(mogen_node src/mogen_node.cc
	src/ctrl_point_gen.cc
	src/motion_gen.cc
	src/mogen_replan_fsm.cc
	src/mogen_plan_manager.cc
	src/k_gpep.cc
)
target_link_libraries(mogen_node ${catkin_LIBRARIES})

# a simple test example, might be removed later
add_executable(mogen_node_test example/plan_admin.cc src/ctrl_point_gen.cc src/motion_gen.cc src/k_gpep.cc)
target_link_libraries(mogen_node_test ${catkin_LIBRARIES})
