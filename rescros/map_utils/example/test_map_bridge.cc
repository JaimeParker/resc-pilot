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
 * Created by Zhaohong Liu on 24-10-19.
 * 
 * Notes of using the MapBridge class:
 * 1. The MapBridge class is designed to be used in a ROS node.
 * 2. The MapBridge class is used to bridge the gap between the map generation and the map usage (simulation).
 * 3. The MapBridge class is used to generate a global ESDF map and a local ESDF map.
 * 4. The MapBridge class is used to publish the point cloud of the map.
 * 5. The MapBridge class is used to get the distance value of the ESDF map at a given position.
 * 6. The MapBridge class is used to show the global ESDF map.
 * Usage:
 * 1. Init the MapBridge object, run the init() function.
 * 2. Set random obstacles, run the setRandomObstacles() function.
 * 3. Get local ESDF map, run the getLocalSDFMap() function.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>

#include "map_utils/MapBridge.h"

//void posePublishTimerCallback(const ros::TimerEvent&, ros::Publisher& pose_pub,
//                              geometry_msgs::PoseStamped& pose_msg) {
//    pose_msg.header.stamp = ros::Time::now();
//    pose_pub.publish(pose_msg);
//}

int main (int argc, char** argv) {
    // cv test
    cv::Mat occupancy_grid = (cv::Mat_<uchar>(5, 5) <<
            1, 1, 1, 1, 1,
            1, 1, 1, 1, 1,
            1, 1, 0, 1, 1,
            1, 1, 1, 1, 1,
            1, 1, 1, 1, 1);
    cv::Mat esdf_map;
    cv::distanceTransform(occupancy_grid, esdf_map, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    std::cout << "Occupancy Grid:\n" << occupancy_grid << std::endl;
    std::cout << "ESDF Map (Distance Transform):\n" << esdf_map << std::endl;

    // ros map bridge test
    ros::init(argc, argv, "test_map_bridge_node");
    ros::NodeHandle nh("~");

    auto map_bridge_ptr = MapBridge::Ptr(new MapBridge(nh));

    MapBridge map_bridge(nh);
    // MapBridge usage step1: init and set random obstacles
    map_bridge.init();
    map_bridge.initSDFMap();
    map_bridge.initObsGenerator();
    map_bridge.setRandomObstacles();
    map_bridge.showGlobalESDFMap(5);
    map_bridge.updateInflatedObstacle();

    double resolution = map_bridge.getResolution();
    double ground_height = map_bridge.getGroundHeight();
    int ground_index = map_bridge.getGroundIndex();
    std::cout << "Resolution: " << resolution << std::endl;
    std::cout << "Ground Height: " << ground_height << std::endl;
    std::cout << "Ground Index: " << ground_index << std::endl;

    bool is_valid = map_bridge.isVoxelValid(Eigen::Vector3i(0, 0, 0));
    std::cout << "Is Voxel Valid: " << is_valid << std::endl;

    // fake topic
    geometry_msgs::PoseStamped fake_pose;
    fake_pose.pose.position.x = 0.0;
    fake_pose.pose.position.y = 0.0;
    fake_pose.pose.position.z = 0.5;
    fake_pose.header.frame_id = "world";
    ros::Publisher fake_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 1);

    Eigen::Vector3d drone_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    double sdf_value = map_bridge.getDistance(drone_pos);
    std::cout << "SDF Value at Drone Position: " << sdf_value << std::endl;

    bool is_occupied = map_bridge.isOccupied(drone_pos);
    std::cout << "Is Drone Position Occupied: " << is_occupied << std::endl;
    auto pos_voxel = map_bridge.pos2Voxel(drone_pos);
    is_occupied = map_bridge.isInflateOccupied(pos_voxel);
    std::cout << "Is Drone Position Inflated Occupied: " << is_occupied << std::endl;

    cv::namedWindow("SDF Map", cv::WINDOW_AUTOSIZE);

    auto rate = ros::Rate(0.5);
    while (ros::ok()) {
        fake_pose.header.stamp = ros::Time::now();
        fake_pose_pub.publish(fake_pose);

        // MapBridge usage step2: publish pcl and get a local SDF map (if needed)
        map_bridge.publishGlobalPCL();
        map_bridge.publishInflatedObstaclePCL();
        Eigen::MatrixXd sdf_mat = map_bridge.getLocalSDFMap(1);
        std::cout << "Local SDF Map:\n" << sdf_mat << std::endl;

        auto sdf_cv_mat = map_bridge.getGlobalSDFCVMat();

        cv::Mat sdf_visual;
        cv::normalize(sdf_cv_mat, sdf_visual, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::imshow("SDF Map", sdf_visual);
        cv::waitKey(1);
        cv::destroyWindow("SDF Map");

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}