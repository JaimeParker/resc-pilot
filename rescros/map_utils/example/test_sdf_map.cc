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
 * Created by Zhaohong Liu on 24-12-22.
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "map_utils/SDFMap.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_sdf_map_node");
    ros::NodeHandle nh;

    ros::Publisher fake_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.5;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    SDFMap sdf_map;
    sdf_map.initMap(nh);

    auto rate = ros::Rate(10);
    while (ros::ok()) {
        pose.header.stamp = ros::Time::now();
        fake_pose_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}