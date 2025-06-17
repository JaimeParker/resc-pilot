//
// Created by Zhaohong Liu on 24-5-10.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <Eigen/Eigen>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

bool is_cmd_received = false;
mavros_msgs::AttitudeTarget cmd;
void cmd_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg){
    cmd = *msg;
    is_cmd_received = true;
}

bool isInGeoFence(const geometry_msgs::PoseStamped& pos) {
    double x_range = 10.0;
    double y_range = 10.0;
    double z_range = 3.0;

    return pos.pose.position.x > -x_range && pos.pose.position.x < x_range &&
           pos.pose.position.y > -y_range && pos.pose.position.y < y_range &&
           pos.pose.position.z < z_range;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "cmd_to_px4_node");
    ros::NodeHandle nh;
    Eigen::Quaterniond q;

    std::string rl_cmd_topic;
    nh.param<std::string>("rl_att_cmd_topic", rl_cmd_topic, "/rl_att_cmd");
    int sub_queue_size;
    nh.param<int>("sub_queue_size", sub_queue_size, 1);
    int pub_queue_size;
    nh.param<int>("pub_queue_size", pub_queue_size, 1);
    double dt;
    nh.param<double>("rl/env_dt", dt, 0.02);
    double frequency = 1 / dt;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10,
             [&](const geometry_msgs::PoseStamped::ConstPtr &msg) {
                 q = Eigen::Quaterniond (msg->pose.orientation.w, msg->pose.orientation.x,
                                         msg->pose.orientation.y, msg->pose.orientation.z);
             });
    ros::Subscriber rl_cmd_sub = nh.subscribe<mavros_msgs::AttitudeTarget>
            (rl_cmd_topic, sub_queue_size, cmd_cb);

    ros::Publisher cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", pub_queue_size);
    ros::Publisher cmd_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", pub_queue_size);


    //the setpoint publishing rate MUST be faster than 2 Hz
    ros::Rate rate(frequency);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.0;

    //send a few set points before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
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

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && !is_offboard_once &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                is_offboard_once = true;  // allow changing to offboard mode only once
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && !is_armed_once &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    is_armed_once = true;  // allow arming only once
                }
                last_request = ros::Time::now();
            }
        }

        if (!isInGeoFence(pose)) {
            ROS_WARN("Out of geo fence, stop sending set-points");
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 1.0;
        }

        if (is_cmd_received && !std::isnan(cmd.body_rate.x) && isInGeoFence(pose)) {
            cmd_pub.publish(cmd);
        } else {
            local_pos_pub.publish(pose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}