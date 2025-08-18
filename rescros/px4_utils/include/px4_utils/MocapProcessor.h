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

#ifndef PX4_UTILS_MOCAPPROCESSOR_H
#define PX4_UTILS_MOCAPPROCESSOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <mavros_msgs/Thrust.h>
#include <deque>

#include "px4_utils/Convertor.h"
#include "px4_utils/Filter.h"

class MocapProcessor {
public:
    /* filter */
    double cutoff_freq_ = 5;
    ButterworthFilter pos_filter_ = ButterworthFilter(cutoff_freq_, 3);
    ButterworthFilter att_filter_ = ButterworthFilter(cutoff_freq_, 3);
public:
    void init(ros::NodeHandle &nh);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void accelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void ProcessGeoMsgsPose();
    void calculateRate();
    void publishThrust();
    void publishRate();
private:
    /* ros utils */
    ros::NodeHandle nh_;
    ros::Subscriber mocap_pose_sub_;
    ros::Subscriber mocap_accel_sub_;
    ros::Publisher mocap_rate_pub_;
    ros::Publisher mocap_rate_dot_pub_;
    ros::Publisher mocap_thrust_pub_;
    ros::Publisher mocap_pose_pub;
    geometry_msgs::PoseStamped mocap_pose_;
    geometry_msgs::TwistStamped body_rate_msg_;
    geometry_msgs::AccelStamped rate_dot_msg_;
    mavros_msgs::Thrust thrust_msg_;

    /* params */
    std::string mocap_pose_topic_ = "/vrpn_client_node/uav/pose";
    std::string mocap_accel_topic_ = "/vrpn_client_node/uav/accel";
    std::string body_rate_topic_ = "/mocap/body_rate";
    std::string rate_dot_topic_ = "/mocap/body_rate_dot";
    std::string thrust_topic_ = "/mocap/thrust";
    std::string body_frame_id_ = "base_link";
    std::string mocap_pose_filtered_topic_ = "/mocap/pose_filtered";

    /* derivative deque setting */
    const size_t pose_queue_size_ = 5;
    const size_t att_dot_queue_size_ = pose_queue_size_ - 2;
    const size_t rate_queue_size_ = att_dot_queue_size_;

    /* drone property */
    double mass_ = 1.64;

    /* queue */
    std::deque<geometry_msgs::PoseStamped> pose_queue_;
    std::deque<double> time_queue_;
    std::deque<Eigen::Vector3d> att_queue_;
    std::deque<Eigen::Vector3d> att_dot_queue_;
    std::deque<Eigen::Vector3d> rate_queue_;
    Eigen::Vector3d rate_;
    Eigen::Vector3d rate_dot_;
    float thrust_ = 0.0;

    /* data */
    Eigen::Vector3d pos_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d att_;
    double last_time_ = 0.0;
};


#endif //PX4_UTILS_MOCAPPROCESSOR_H
