cmake_minimum_required(VERSION 3.0.2)
project(mogen_simulator)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17")

# Add the cmake directory to module path to find FindResclib.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../plan_admin/cmake")

find_package(Eigen3 REQUIRED)
find_package(Resclib REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        mavros_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES mogen_simulator
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs mavros_msgs
)

include_directories(
        include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${RESCLIB_INCLUDE_DIRS}
)

add_executable(mogen_simulator_node src/MogenSimulatorNode.cc
        src/MogenSimulator.cc)
target_link_libraries(mogen_simulator_node ${catkin_LIBRARIES} ${RESCLIB_LIBRARIES})
