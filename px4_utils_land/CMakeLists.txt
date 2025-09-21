cmake_minimum_required(VERSION 3.0.2)
project(px4_utils_land)

add_compile_options(-std=c++14)

##zhiyuan: build cv_bridge and image_transport
find_package(Eigen3 REQUIRED
        Opencv REQUIRED
)
find_package(catkin REQUIRED COMPONENTS
        mavros_msgs
        roscpp
        rospy
        std_msgs
        quadrotor_msgs
        cv_bridge
        image_transport 
        sensor_msgs
        rosbag
)
find_package(OpenCV REQUIRED COMPONENTS
        core
        imgproc
        features2d
        highgui
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES px4_utils_land
        CATKIN_DEPENDS mavros_msgs roscpp rospy std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
        src/utils/Convertor.cc
        include/px4_utils_land/Convertor.h
        src/motion_capture/MocapProcessor.cc
        include/px4_utils_land/MocapProcessor.h
        src/monitor/AttitudeMonitor.cc
        include/px4_utils_land/AttitudeMonitor.h
        src/utils/Filter.cc
        include/px4_utils_land/Filter.h
        src/utils/PX4CtrlFSM.cc
        include/px4_utils_land/PX4CtrlFSM.h
        src/utils/ImgMatching.cc
        include/px4_utils_land/ImgMatching.h
)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${OpenCV_LIBRARIES}
)


add_executable(cmd_to_px4_node_land src/rl_cmd_to_px4.cc src/utils/Convertor.cc)
target_link_libraries(cmd_to_px4_node_land ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(frame_human_viewer_node_land src/frame_human_viewer.cc src/utils/Convertor.cc)
target_link_libraries(frame_human_viewer_node_land ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(mocap_re_publish_node_land src/MocapRePublisher.cc)
target_link_libraries(mocap_re_publish_node_land ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(run_system_id_land src/run_system_id.cc)
target_link_libraries(run_system_id_land ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(px4_ctrl_fsm_node_land src/px4_ctrl_fsm_node.cc src/utils/PX4CtrlFSM.cc)
target_link_libraries(px4_ctrl_fsm_node_land ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(land_command_publisher_node_land src/land_command_publisher.cc)
target_link_libraries(land_command_publisher_node_land ${catkin_LIBRARIES})

add_executable(px4_fsm_interface_node_land src/px4_fsm_interface.cc)
target_link_libraries(px4_fsm_interface_node_land ${catkin_LIBRARIES})

##zhiyuan: add test executables
# add_executable(vision_node src/utils/vision_orb.cc)
# target_link_libraries(vision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# test
add_executable(test_mocap_process_land example/test_mocap.cc src/motion_capture/MocapProcessor.cc)
target_link_libraries(test_mocap_process_land ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_actuator_ctrl_land example/test_actuator_ctrl.cc)
target_link_libraries(test_actuator_ctrl_land ${catkin_LIBRARIES})

add_executable(test_imgmatching example/test_imgmatching.cc)
target_link_libraries(test_imgmatching ${catkin_LIBRARIES} ${PROJECT_NAME})

# Unified HSV beacon detection test program (supports ROS live, rosbag, and offline)
add_executable(test_hsv_unified example/test_hsv_unified.cc)
target_link_libraries(test_hsv_unified ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
