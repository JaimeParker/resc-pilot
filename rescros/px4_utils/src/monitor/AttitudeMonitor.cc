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
 * Created by Zhaohong Liu on 24-9-25.
*/

#include "px4_utils/AttitudeMonitor.h"

void AttitudeMonitor::init(ros::NodeHandle &nh) {
    nh_ = nh;
    pose_sub_ = nh.subscribe(pose_sub_topic_, 1, &AttitudeMonitor::poseCallback, this);
    rate_sub_ = nh.subscribe(rate_sub_topic_, 1, &AttitudeMonitor::rateCallback, this);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
}

void AttitudeMonitor::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    Eigen::Quaterniond q;
    q.x() = msg->pose.orientation.x;
    q.y() = msg->pose.orientation.y;
    q.z() = msg->pose.orientation.z;
    q.w() = msg->pose.orientation.w;

    pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    Convertor::q2EulerAngle(q, att_);
}

void AttitudeMonitor::rateCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    rate_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
}

bool AttitudeMonitor::isAggressive() {
    return std::abs(att_[0]) > roll_tol_ || std::abs(att_[1]) > pitch_tol_;
}

bool AttitudeMonitor::isOutOfGeoFence() {
    if (std::abs(pos_[0]) > geo_fence_x_
        || std::abs(pos_[1]) > geo_fence_y_
        || pos_[2] > geo_fence_z_ || pos_[2] <= 0.3) {
        return true;
    }
    return false;
}

void AttitudeMonitor::initSystemID() {
    rate_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
    ROS_INFO("[Monitor] Ready to perform system identification");
}

void AttitudeMonitor::runSystemIdentification() {
    switch (status_) {
        case SYSTEM_ID:
            if (isAggressive() || isOutOfGeoFence()) {
                status_ = HOLD;
                frozen_time_ = ros::Time::now();
                rate_execution_count_++;
                ROS_WARN("[Monitor] Aggressive motion or out of geo fence detected, PID takes control");
            } else {
                if (rate_execution_count_ < each_rate_exe_times) {
                    setRate(rate_type_);
                    rate_pub_.publish(rate_pub_msg_);
                } else {
                    rate_execution_count_ = 0;
                    rate_type_ = (rate_type_ + 1) % 3;
                }
            }
            break;

        case HOLD:
            frozen_pos_ = pos_;
            frozen_att_ = {0.0, 0.0, att_[2]};
            if (!holdPosition(4.0)) {
                status_ = RTB;
                ROS_INFO("[Monitor] RTB now");
                frozen_time_ = ros::Time::now();
            }
            break;

        case RTB:
            if (!backToOrigin()) {
                status_ = SYSTEM_ID;
                ROS_INFO("[Monitor] Proceed with system identification");
            }
            break;
    }
}

bool AttitudeMonitor::holdPosition(const double duration) {
    auto time_now = ros::Time::now();
    if ((time_now - frozen_time_).toSec() > duration) {
        ROS_INFO("[Monitor] Hold position, checked");
        return false;
    }
    setPoseMsg(frozen_pos_, frozen_att_);
    pose_pub_.publish(pose_pub_msg_);

    return true;
}

bool AttitudeMonitor::backToOrigin() {
    if ((origin_pos_ - pos_).norm() < 0.2 && (origin_att_ - att_).norm() < M_PI / 6 &&
        (ros::Time::now() - frozen_time_).toSec() > 5.0) {
        ROS_INFO("[Monitor] RTB, checked");
        return false;
    }
    setPoseMsg(origin_pos_, origin_att_);
    pose_pub_.publish(pose_pub_msg_);
    return true;
}

void AttitudeMonitor::setPoseMsg(const Eigen::Vector3d &pos, const Eigen::Vector3d &att) {
    pose_pub_msg_.header.stamp = ros::Time::now();
    pose_pub_msg_.header.frame_id = pose_pub_frame_id_;
    pose_pub_msg_.pose.position.x = pos[0];
    pose_pub_msg_.pose.position.y = pos[1];
    pose_pub_msg_.pose.position.z = pos[2];
    auto q = Convertor::euler2Quaternion(att);
    pose_pub_msg_.pose.orientation.x = q.x();
    pose_pub_msg_.pose.orientation.y = q.y();
    pose_pub_msg_.pose.orientation.z = q.z();
    pose_pub_msg_.pose.orientation.w = q.w();
}

void AttitudeMonitor::setRate(int angle_type) {
    if (angle_type == 0) {
        // set roll rate only
        rate_pub_msg_.body_rate.x = rate_[0] + delta_rate_;
        rate_pub_msg_.body_rate.y = rate_[1];
        rate_pub_msg_.body_rate.z = rate_[2];
    } else if (angle_type == 1) {
        // set pitch rate only
        rate_pub_msg_.body_rate.x = rate_[0];
        rate_pub_msg_.body_rate.y = rate_[1] + delta_rate_;
        rate_pub_msg_.body_rate.z = rate_[2];
    } else if (angle_type == 2) {
        // set yaw rate only
        rate_pub_msg_.body_rate.x = rate_[0];
        rate_pub_msg_.body_rate.y = rate_[1];
        rate_pub_msg_.body_rate.z = rate_[2] + delta_rate_;
    }

    rate_pub_msg_.header.stamp = ros::Time::now();
    rate_pub_msg_.header.frame_id = "base_link";
    rate_pub_msg_.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    // 0.38 for iris when mass is 0.5 kg, whole mass 0.535 kg
    // 0.42 for imp250 drone with 1.64 kg
    rate_pub_msg_.thrust = 0.38;
}
