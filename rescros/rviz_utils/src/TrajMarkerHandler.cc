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
 * Created by Zhaohong Liu on 24-10-22.
*/

#include "rviz_utils/TrajMarkerHandler.h"

void TrajMarkerHandler::init() {
    setCommonMarkerProperties(marker_);
    marker_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.ns = "drone_trajectory";
    marker_.id = 0;
    marker_.color = traj_color_;
    marker_.scale.x = marker_size_;
    marker_.scale.y = marker_size_;
    marker_.scale.z = marker_size_;
    marker_.pose.orientation.w = 1.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.x = 0.0;

    path_.header.frame_id = frame_id_;

    pose_sub_ = nh_.subscribe(pose_topic_, 1, &TrajMarkerHandler::poseCallback, this);
    traj_rviz_pub_ = nh_.advertise<visualization_msgs::Marker>(traj_rviz_topic_, 5);
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 5);

    is_initialized_ = true;

    color_start_ = {68.0 / 255.0, 1.0 / 255.0, 84.0 / 255.0};
    color_mid1_ = {58.0 / 255.0, 82.0 / 255.0, 139.0 / 255.0};
    color_mid2_ = {32.0 / 255.0, 144.0 / 255.0, 140.0 / 255.0};
    color_mid3_ = {94.0 / 255.0, 201.0 / 255.0, 97.0 / 255.0};
    color_end_ = {253.0 / 255.0, 231.0 / 255.0, 37.0 / 255.0};

    last_time_ = ros::Time::now().toSec();
}

void TrajMarkerHandler::updateMarker() {
    marker_.header.stamp = ros::Time::now();
}

void TrajMarkerHandler::publish() {
    if (is_initialized_) {
        updateMarker();
        traj_rviz_pub_.publish(marker_);
        path_pub_.publish(path_);
    } else {
        ROS_WARN("Call init() before publishing.");
    }
}

void TrajMarkerHandler::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    geometry_msgs::Point p;
    p.x = msg->pose.position.x;
    p.y = msg->pose.position.y;
    p.z = msg->pose.position.z;

    marker_.points.push_back(p);

    if (marker_.points.size() > max_points_) {
        marker_.points.erase(marker_.points.begin());
    }

    const auto& pose_msg = *msg;
    if (ros::Time::now().toSec() - last_time_ > duration_pose_) {
        path_.poses.push_back(pose_msg);
        last_time_ = ros::Time::now().toSec();
    }
    if (path_.poses.size() > max_points_) {
        path_.poses.erase(path_.poses.begin());
    }
}

//void TrajMarkerHandler::velCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
//    vel_norm_ = std::sqrt(std::pow(msg->twist.linear.x, 2) +
//                          std::pow(msg->twist.linear.y, 2) +
//                          std::pow(msg->twist.linear.z, 2));
//}
//
//Eigen::Vector3f TrajMarkerHandler::getColor(const double &val) {
//    if (val < 0.25) {
//        return color_start_ * (1.0 - val / 0.25) + color_mid1_ * (val / 0.25);
//    } else if (val < 0.5) {
//        return color_mid1_ * (1.0 - (val - 0.25) / 0.25) + color_mid2_ * ((val - 0.25) / 0.25);
//    } else if (val < 0.75) {
//        return color_mid2_ * (1.0 - (val - 0.5) / 0.25) + color_mid3_ * ((val - 0.5) / 0.25);
//    } else {
//        return color_mid3_ * (1.0 - (val - 0.75) / 0.25) + color_end_ * ((val - 0.75) / 0.25);
//    }
//}
