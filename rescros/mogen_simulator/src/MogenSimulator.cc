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

#include "mogen_simulator/MogenSimulator.h"

void MogenSimulator::init(ros::NodeHandle &nh) {
    getParamWithWarning(nh, "sim/drone", drone_name);

    quadrotor_dynamics_ = QuadrotorDynamics();
    quadrotor_dynamics_.initDrone(drone_name);

    if (drone_name == "imp250") {
        quadrotor_dynamics_.setSysCtrlAlloc(true);
        float ca_min = 0.3;
        float ca_max = 0.8;
        quadrotor_dynamics_.resetCtrlAllocRange(ca_min, ca_max);
    }

    getParamWithWarning(nh, "sim/rl_att_cmd_topic", rl_cmd_topic_);
    getParamWithWarning(nh, "sim/pose_sub_topic", pose_topic_);
    getParamWithWarning(nh, "sim/vel_sub_topic", vel_topic_);
    getParamWithWarning(nh, "sim/body_rate_sub_topic", body_rate_topic_);
    getParamWithWarning(nh, "sim/ground_height", ground_height_);
    getParamWithWarning(nh, "sim/dt", dt_);
    getParamWithWarning(nh, "sim/start_x", start_pt_.x());
    getParamWithWarning(nh, "sim/start_y", start_pt_.y());
    getParamWithWarning(nh, "sim/start_z", start_pt_.z());

    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
    vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>(vel_topic_, 1);
    body_rate_pub_ = nh.advertise<geometry_msgs::TwistStamped>(body_rate_topic_, 1);
    rl_cmd_sub_ = nh.subscribe(rl_cmd_topic_, 1, &MogenSimulator::rlCmdCallback, this);
    use_rl_sub_ = nh.subscribe(use_rl_topic_, 1, &MogenSimulator::useRLCallback, this);
    crash_timer_ = nh.createTimer(ros::Duration(crash_timer_period_), &MogenSimulator::crashCallback, this);
    use_pid_sub_ = nh.subscribe(use_pid_topic_, 1, &MogenSimulator::usePIDCallback, this);
    pid_cmd_sub_ = nh.subscribe(pid_cmd_topic_, 1, &MogenSimulator::pidCmdCallback, this);

    pos_ = start_pt_;
    pos_.z() = 1.0;
    ori_ = Eigen::Quaterniond::Identity();
    att_euler_ = Eigen::Vector3d::Zero();
    vel_ = Eigen::Vector3d::Zero();
    body_rate_ = Eigen::Vector3d::Zero();
    body_rate_req_ = Eigen::Vector3d::Zero();
    vel_req_ = Eigen::Vector3d::Zero();
    yaw_req_ = 0.0;

    is_initialized_ = true;
    ROS_INFO("\033[1;32mMogen simulator initialized.\033[0m");
}

void MogenSimulator::euler2Quaternion(const Eigen::Vector3d &euler, Eigen::Quaterniond &q) {
    double roll = euler[0];
    double pitch = euler[1];
    double yaw = euler[2];

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;
}

void MogenSimulator::updateDroneState() {
    if (!crashed_) {
        if (use_rl_  && !use_pid_) {
            state_.segment(0, 3) = pos_;
            state_.segment(3, 3) = vel_;
            state_.segment(6, 3) = att_euler_;
            state_.segment(9, 3) = body_rate_;

            double thrust_req = quadrotor_dynamics_.rescaledThrust2Thrust(rescaled_thrust_req_) * 4;
            state_ = quadrotor_dynamics_.run(state_, body_rate_req_,
                                             motor_thrusts_, thrust_req);
            motor_thrusts_ = quadrotor_dynamics_.getFinalMotorThrusts();

            pos_ = state_.segment(0, 3);
            vel_ = state_.segment(3, 3);
            att_euler_ = state_.segment(6, 3);
            body_rate_ = state_.segment(9, 3);
        } else if (use_pid_) {
            // pid priority higher than rl
            // use fake pid control for simple simulation
            pos_ += vel_req_ * dt_;
            vel_ = vel_req_;
            body_rate_ = Eigen::Vector3d::Zero();
            att_euler_ = {0, 0, yaw_req_};
        } else {
            // No RL or PID commands - continue with current velocity (natural physics)
            pos_ += vel_ * dt_;
            
            double drag_coefficient = 0.6;
            vel_ *= drag_coefficient;
            
            
            body_rate_ *= 0.5;
            
            att_euler_.x() *= 0.9;
            att_euler_.y() *= 0.9;
        }
    } else {
        // for simple simulation only, crashed
        vel_ = Eigen::Vector3d::Zero();
        body_rate_ = Eigen::Vector3d::Zero();
        att_euler_ = {0, 0, att_euler_.z()};
    }
    euler2Quaternion(att_euler_, ori_);

    // set ros messages
    auto cur_time = ros::Time::now();
    pose_msg_.header.stamp = cur_time;
    pose_msg_.pose.position.x = pos_[0];
    pose_msg_.pose.position.y = pos_[1];
    pose_msg_.pose.position.z = pos_[2];
    pose_msg_.pose.orientation.w = ori_.w();
    pose_msg_.pose.orientation.x = ori_.x();
    pose_msg_.pose.orientation.y = ori_.y();
    pose_msg_.pose.orientation.z = ori_.z();

    vel_msg_.header.stamp = cur_time;
    vel_msg_.twist.linear.x = vel_[0];
    vel_msg_.twist.linear.y = vel_[1];
    vel_msg_.twist.linear.z = vel_[2];

    body_rate_msg_.header.stamp = cur_time;
    body_rate_msg_.twist.angular.x = body_rate_[0];
    body_rate_msg_.twist.angular.y = body_rate_[1];
    body_rate_msg_.twist.angular.z = body_rate_[2];
}

void MogenSimulator::publishDroneState() {
    if (is_initialized_) {
        pose_pub_.publish(pose_msg_);
        vel_pub_.publish(vel_msg_);
        body_rate_pub_.publish(body_rate_msg_);
    } else {
        ROS_WARN("MogenSimulator not initialized.");
    }
}

void MogenSimulator::rlCmdCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg) {
    body_rate_req_ << msg->body_rate.x, msg->body_rate.y, msg->body_rate.z;
    rescaled_thrust_req_ = msg->thrust;
}

void MogenSimulator::useRLCallback(const std_msgs::Bool::ConstPtr &msg) {
    use_rl_ = msg->data;
}

void MogenSimulator::crashCallback(const ros::TimerEvent &) {
    if (pos_.z() < ground_height_ || crashed_) {
        pos_ = {pos_.x(), pos_.y(), ground_height_};
        vel_ = Eigen::Vector3d::Zero();
        body_rate_ = Eigen::Vector3d::Zero();
        att_euler_ = {0, 0, att_euler_.z()};

        std::cout << "\033[1;31m[simulator]: Mayday! Mayday! drone crashed!\033[0m" << std::endl;
        use_rl_ = false;
        crashed_ = true;
    }
}

void MogenSimulator::usePIDCallback(const std_msgs::Bool::ConstPtr &msg) {
    use_pid_ = msg->data;
}

void MogenSimulator::pidCmdCallback(const mavros_msgs::PositionTarget::ConstPtr &msg) {
    vel_req_ = {msg->velocity.x, msg->velocity.y, msg->velocity.z};
    yaw_req_ = msg->yaw;
}
