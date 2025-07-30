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
 * Created by Zhaohong Liu on 24-10-21.
*/

#include "rviz_utils/CtrlPointVisualization.h"

void CtrlPtMarkerHandler::init() {
    setCommonMarkerProperties(marker_);
    setCommonMarkerProperties(ctrl_pt_second_marker_);

    marker_.type = visualization_msgs::Marker::SPHERE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.header.frame_id = frame_id_;
    marker_.ns = "ctrl_pt_1st";
    marker_.id = 0;
    marker_.color.r = VC::RED.r;
    marker_.color.g = VC::RED.g;
    marker_.color.b = VC::RED.b;
    marker_.scale.x = 0.2;
    marker_.scale.y = 0.2;
    marker_.scale.z = 0.2;
    marker_.pose.orientation.w = 1.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.x = 0.0;

    ctrl_pt_second_marker_.type = visualization_msgs::Marker::SPHERE;
    ctrl_pt_second_marker_.action = visualization_msgs::Marker::ADD;
    ctrl_pt_second_marker_.header.frame_id = frame_id_;
    ctrl_pt_second_marker_.ns = "ctrl_pt_2nd";
    ctrl_pt_second_marker_.id = 1;
    ctrl_pt_second_marker_.color.r = VC::BLUE.r;
    ctrl_pt_second_marker_.color.g = VC::BLUE.g;
    ctrl_pt_second_marker_.color.b = VC::BLUE.b;
    ctrl_pt_second_marker_.scale.x = 0.2;
    ctrl_pt_second_marker_.scale.y = 0.2;
    ctrl_pt_second_marker_.scale.z = 0.2;
    ctrl_pt_second_marker_.pose.orientation.w = 1.0;
    ctrl_pt_second_marker_.pose.orientation.z = 0.0;
    ctrl_pt_second_marker_.pose.orientation.y = 0.0;
    ctrl_pt_second_marker_.pose.orientation.x = 0.0;

    ctrl_pt_rviz_pub_ = nh_.advertise<visualization_msgs::Marker>(ctrl_pt_rviz_topic_, 5);
    ctrl_pt_1st_sub_ = nh_.subscribe(ctrl_pt_1st_topic_, 1, &CtrlPtMarkerHandler::ctrlPt1Callback, this);
    ctrl_pt_2nd_sub_ = nh_.subscribe(ctrl_pt_2nd_topic_, 1, &CtrlPtMarkerHandler::ctrlPt2Callback, this);

    is_initialized_ = true;
}

void CtrlPtMarkerHandler::updateMarker() {

}

void CtrlPtMarkerHandler::publish() {
    if (!is_initialized_) {
        ROS_WARN("CtrlPtMarkerHandler is not initialized! Call init() first.");
        return;
    }
    ctrl_pt_rviz_pub_.publish(marker_);
    ctrl_pt_rviz_pub_.publish(ctrl_pt_second_marker_);
}

void CtrlPtMarkerHandler::ctrlPt1Callback(const geometry_msgs::Point::ConstPtr &msg) {
    marker_.pose.position.x = msg->x;
    marker_.pose.position.y = msg->y;
    marker_.pose.position.z = msg->z;
}

void CtrlPtMarkerHandler::ctrlPt2Callback(const geometry_msgs::Point::ConstPtr &msg) {
    ctrl_pt_second_marker_.pose.position.x = msg->x;
    ctrl_pt_second_marker_.pose.position.y = msg->y;
    ctrl_pt_second_marker_.pose.position.z = msg->z;
}
