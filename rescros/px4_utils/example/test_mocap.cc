/**
 * This file is part of Resc-Pilot.
 *
 * Copyright 2025 Zhaohong Liu, IRMV Lab, Shanghai Jiao Tong University, <https://www.sjtu.edu.cn/>
 * Developed by Zhaohong Liu <zhliu25 at outlook dot com>, <jaimefriedhelmzhao at gmail dot com>
 * for more information see <https://github.com/JaimeParker/resc-pilot>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Resc-Pilot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Resc-Pilot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Resc-Pilot. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Created by Zhaohong Liu on 24-9-21.
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "px4_utils/MocapProcessor.h"

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