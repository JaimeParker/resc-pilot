//
// Created by Zhaohong Liu on 24-9-21.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "px4_utils_land/MocapProcessor.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_mocap_node");
    ros::NodeHandle nh;

    ros::Publisher fake_mocap_pose_pub =
            nh.advertise<geometry_msgs::PoseStamped>("/vrpn_client_node/uav/pose", 1);
    geometry_msgs::PoseStamped fake_mocap_pose;

    MocapProcessor mocap_processor;
    mocap_processor.init(nh);

    auto rate = ros::Rate(1);

    Eigen::Vector3d euler = Eigen::Vector3d::Zero();
    int i = 0;

    while (ros::ok()) {
        euler[0] += 0.1 * (1 + i);
        euler[1] = 0.2 * (1 + i);
        euler[2] = -0.3 * (1 + i);
        Eigen::Quaterniond q;
        Convertor::euler2Quaternion(q, euler);

        fake_mocap_pose.header.stamp = ros::Time::now();
        fake_mocap_pose.pose.orientation.x = q.x();
        fake_mocap_pose.pose.orientation.y = q.y();
        fake_mocap_pose.pose.orientation.z = q.z();
        fake_mocap_pose.pose.orientation.w = q.w();
        fake_mocap_pose_pub.publish(fake_mocap_pose);

        mocap_processor.publishRate();
        ros::spinOnce();
        rate.sleep();
        i += 1;
    }

    return 0;
}