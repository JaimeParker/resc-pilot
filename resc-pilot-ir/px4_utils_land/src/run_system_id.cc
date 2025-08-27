/**
 * @file offb_node.cpp from PX4-AutoPilot
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 * revised by Zhaohong Liu on 23-7-18.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "px4_utils_land/AttitudeMonitor.h"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped pose_current;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_current = *msg;
}

bool arrive_target(const geometry_msgs::PoseStamped& pose, const geometry_msgs::PoseStamped &pose_target) {
    double x_tol = 0.2;
    double y_tol = 0.2;
    double z_tol = 0.2;

    return std::abs(pose.pose.position.x - pose_target.pose.position.x) < x_tol &&
           std::abs(pose.pose.position.y - pose_target.pose.position.y) < y_tol &&
           std::abs(pose.pose.position.z - pose_target.pose.position.z) < z_tol;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_takeoff_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose_init;
    pose_init.pose.position.x = 0;
    pose_init.pose.position.y = 0;
    pose_init.pose.position.z = 1.0;

    //send a few set points before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose_init);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    bool is_offboard_once = false;
    bool is_armed_once = false;
    bool is_take_off = false;

    AttitudeMonitor monitor;
    monitor.init(nh);
    monitor.initSystemID();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && !is_offboard_once &&
            (ros::Time::now() - last_request > ros::Duration(4.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                is_offboard_once = true;  // allow changing to offboard mode only once
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && !is_armed_once &&
                (ros::Time::now() - last_request > ros::Duration(4.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    is_armed_once = true;  // allow arming only once
                }
                last_request = ros::Time::now();
            }
        }

        if (!is_take_off) {
            if (arrive_target(pose_current, pose_init)) {
                is_take_off = true;
                ROS_INFO("Take off successfully");
            } else {
                local_pos_pub.publish(pose_init);
            }
        } else {
            monitor.runSystemIdentification();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}