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
 * Created by Zhaohong Liu on 24-9-18.
*/

#include "rviz_utils/PoseVisualization.h"

void PoseMarkerHandler::init() {
    initialized_ = true;
    setCommonMarkerProperties(marker_);
    setCommonMarkerProperties(heading_marker_);
    setCommonMarkerProperties(x_axis_marker_);
    setCommonMarkerProperties(y_axis_marker_);
    setCommonMarkerProperties(z_axis_marker_);

    // for meshing a drone
    marker_.mesh_resource = RP::mesh_resource_path_;
    marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.ns = "robot";
    marker_.scale.x = RP::robot_scale_x_;
    marker_.scale.y = RP::robot_scale_y_;
    marker_.scale.z = RP::robot_scale_z_;
    marker_.color = drone_color_;

    heading_marker_.type = visualization_msgs::Marker::ARROW;
    heading_marker_.color.r = VC::RED.r;
    heading_marker_.color.g = VC::RED.g;
    heading_marker_.color.b = VC::RED.b;
    heading_marker_.scale.x = 0.8;
    heading_marker_.scale.y = 0.05;
    heading_marker_.scale.z = 0.05;

    x_axis_marker_.type = visualization_msgs::Marker::ARROW;
    x_axis_marker_.action = visualization_msgs::Marker::ADD;
    x_axis_marker_.ns = "body_x_axis";
    x_axis_marker_.id = 0;
    x_axis_marker_.color.r = VC::RED.r;
    x_axis_marker_.color.g = VC::RED.g;
    x_axis_marker_.color.b = VC::RED.b;
    x_axis_marker_.scale.x = 0.5;
    x_axis_marker_.scale.y = 0.05;
    x_axis_marker_.scale.z = 0.05;

    y_axis_marker_.type = visualization_msgs::Marker::ARROW;
    y_axis_marker_.action = visualization_msgs::Marker::ADD;
    y_axis_marker_.ns = "body_y_axis";
    y_axis_marker_.id = 1;
    y_axis_marker_.color.r = VC::GREEN.r;
    y_axis_marker_.color.g = VC::GREEN.g;
    y_axis_marker_.color.b = VC::GREEN.b;
    y_axis_marker_.scale.x = 0.5;
    y_axis_marker_.scale.y = 0.05;
    y_axis_marker_.scale.z = 0.05;

    z_axis_marker_.type = visualization_msgs::Marker::ARROW;
    z_axis_marker_.action = visualization_msgs::Marker::ADD;
    z_axis_marker_.ns = "body_z_axis";
    z_axis_marker_.id = 2;
    z_axis_marker_.color.r = VC::BLUE.r;
    z_axis_marker_.color.g = VC::BLUE.g;
    z_axis_marker_.color.b = VC::BLUE.b;
    z_axis_marker_.scale.x = 0.5;
    z_axis_marker_.scale.y = 0.05;
    z_axis_marker_.scale.z = 0.05;

    // ros subscriber and publisher
    pose_sub_ = nh_.subscribe(pose_sub_topic_, RP::sub_queue_size_,
                              &PoseMarkerHandler::poseCallback, this);
    pose_marker_pub_ = nh_.advertise<visualization_msgs::Marker>
            (RP::pose_marker_pub_topic_, RP::pub_queue_size_);
    heading_marker_pub_ = nh_.advertise<visualization_msgs::Marker>
            (heading_marker_pub_topic_, RP::pub_queue_size_);
    axis_marker_pub_ = nh_.advertise<visualization_msgs::Marker>
            (drone_att_in_axis_pub_topic_, RP::pub_queue_size_);
}

void PoseMarkerHandler::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose_msg_ = *msg;
}

void PoseMarkerHandler::updateMarker() {
    marker_.header.stamp = pose_msg_.header.stamp;
    marker_.pose = pose_msg_.pose;

    heading_marker_.header.stamp = pose_msg_.header.stamp;
    heading_marker_.pose.position = pose_msg_.pose.position;
    heading_marker_.pose.orientation = pose_msg_.pose.orientation;
}

void PoseMarkerHandler::publish() {
    if (initialized_) {
        updateMarker();
        pose_marker_pub_.publish(marker_);
        heading_marker_pub_.publish(heading_marker_);
    } else {
        ROS_WARN("PoseMarkerHandler is not initialized!");
    }
}





