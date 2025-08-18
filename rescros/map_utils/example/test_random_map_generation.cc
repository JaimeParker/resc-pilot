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
 * Created by Zhaohong Liu on 24-9-20.
*/

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "map_utils/ObstacleGenerator.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_random_map_generation_node");
    ros::NodeHandle nh("~");
    ros::Publisher global_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_utils/global_cloud", 1);

    ObstacleGenerator map_gen;
    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    sensor_msgs::PointCloud2 global_map_pcd;

    map_gen.setRectPcl(1.345, 2.897, 1.1, 3.2, 1.0, global_cloud);

    auto rate = ros::Rate(1);
    while (ros::ok()) {
        global_cloud.width = global_cloud.points.size();
        global_cloud.height = 1;
        global_cloud.is_dense = true;
        pcl::toROSMsg(global_cloud, global_map_pcd);
        global_map_pcd.header.frame_id = "world";
        global_map_pcd.header.stamp = ros::Time::now();
        global_pcl_pub.publish(global_map_pcd);
    }

    return 0;
}