cmake_minimum_required(VERSION 3.20.2)
project(rviz_utils)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        mavros_msgs
        visualization_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rviz_utils
        CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs visualization_msgs
)

include_directories(
        include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(rviz_utils
        src/RvizVisualization.cc
        src/RvizParams.cc
        src/VisualizationColor.cc
        src/PoseVisualization.cc
        src/PathVisualization.cc
        src/CtrlPointVisualization.cc
        src/TrajMarkerHandler.cc
)

# python scripts
catkin_install_python(PROGRAMS
        scripts/global_traj_publish.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(${PROJECT_NAME}_node
        src/RvizVisualization.cc
        src/RvizParams.cc
        src/VisualizationColor.cc
        src/PoseVisualization.cc
)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)
