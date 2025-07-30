//
// Created by Zhaohong Liu on 24-6-18.
//

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "px4_utils_land/Convertor.h"

#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN "\033[36m"

geometry_msgs::PoseStamped pose;
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose = *msg;
}

geometry_msgs::TwistStamped vel_local;
void vel_local_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    vel_local = *msg;
}

geometry_msgs::TwistStamped vel_body;
void vel_body_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    vel_body = *msg;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "frame_human_viewer_node");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
            "mavros/local_position/pose", 5, pose_callback);
    ros::Subscriber vel_local_sub = nh.subscribe<geometry_msgs::TwistStamped>(
            "mavros/local_position/velocity_local", 5, vel_local_callback);
    ros::Subscriber vel_body_sub = nh.subscribe<geometry_msgs::TwistStamped>(
            "mavros/local_position/velocity_body", 5, vel_body_callback);

    Eigen::Vector3d pos;
    Eigen::Quaterniond q;
    double roll, pitch, yaw;
    Eigen::Vector3d linear_vel_local;
    Eigen::Vector3d linear_vel_body;
    Eigen::Vector3d angular_vel_body;
    Eigen::Vector3d angular_vel_local;

    ros::Rate rate(10.0);
    while (ros::ok()) {
        // receive msg
        pos << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
        q.w() = pose.pose.orientation.w;
        q.x() = pose.pose.orientation.x;
        q.y() = pose.pose.orientation.y;
        q.z() = pose.pose.orientation.z;
        Convertor::q2EulerAngle(q, roll, pitch, yaw);
        linear_vel_local << vel_local.twist.linear.x, vel_local.twist.linear.y, vel_local.twist.linear.z;
        linear_vel_body << vel_body.twist.linear.x, vel_body.twist.linear.y, vel_body.twist.linear.z;
        angular_vel_body << vel_body.twist.angular.x, vel_body.twist.angular.y, vel_body.twist.angular.z;
        angular_vel_local << vel_local.twist.angular.x, vel_local.twist.angular.y, vel_local.twist.angular.z;

        roll = roll * 180 / M_PI;
        pitch = pitch * 180 / M_PI;
        yaw = yaw * 180 / M_PI;

        angular_vel_body = angular_vel_body * 180 / M_PI;
        angular_vel_local = angular_vel_local * 180 / M_PI;

        // show msg
        std::cout << RED << "pos: " << pos.transpose() << RESET << std::endl;
        std::cout << GREEN << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << RESET << std::endl;
        std::cout << YELLOW << "linear vel local: " << linear_vel_local.transpose() << RESET << std::endl;
        std::cout << BLUE << "linear vel body: " << linear_vel_body.transpose() << RESET << std::endl;
        std::cout << MAGENTA << "angular vel local: " << angular_vel_local.transpose() << RESET << std::endl;
        std::cout << CYAN << "angular vel body: " << angular_vel_body.transpose() << RESET << std::endl;
        std::cout << std::endl;

        // spin
        ros::spinOnce();
        rate.sleep();
    }


}