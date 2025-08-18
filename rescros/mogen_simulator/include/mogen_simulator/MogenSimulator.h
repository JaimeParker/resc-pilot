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

#ifndef MOGEN_SIMULATOR_MOGENSIMULATOR_H
#define MOGEN_SIMULATOR_MOGENSIMULATOR_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>

#include <Resclib/Resclib.h>
#include <Resclib/QuadrotorDynamics.h>
#include <Resclib/Params.h>
#include <Resclib/RateControl.h>

class MogenSimulator {
public:
    void init(ros::NodeHandle &nh);
    static void euler2Quaternion(const Eigen::Vector3d& euler, Eigen::Quaterniond& q);
    void updateDroneState();
    void publishDroneState();
    void rlCmdCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
    void useRLCallback(const std_msgs::Bool::ConstPtr& msg);
    void crashCallback(const ros::TimerEvent& /* event */);
    void usePIDCallback(const std_msgs::Bool::ConstPtr& msg);
    void pidCmdCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
    template<typename T>
    void getParamWithWarning(ros::NodeHandle& nh, const std::string& param_name, T& param) {
        if (!nh.getParam(param_name, param)) {
            ROS_WARN_STREAM("Failed to get param: " << param_name);
        }
    }

private:
    /* ros utils */
    ros::Publisher pose_pub_;
    geometry_msgs::PoseStamped pose_msg_;
    ros::Publisher vel_pub_;
    geometry_msgs::TwistStamped vel_msg_;
    ros::Publisher body_rate_pub_;
    geometry_msgs::TwistStamped body_rate_msg_;
    ros::Subscriber rl_cmd_sub_;
    ros::Subscriber use_rl_sub_;
    bool use_rl_ = false;
    ros::Subscriber use_pid_sub_;
    bool use_pid_ = false;
    ros::Subscriber pid_cmd_sub_;
    ros::Timer crash_timer_;

    /* topic names */
    std::string rl_cmd_topic_ = "/rl_att_cmd";
    std::string pose_topic_ = "/mavros/local_position/pose";
    std::string vel_topic_ = "/mavros/local_position/velocity_local";
    std::string body_rate_topic_ = "/mavros/local_position/velocity_body";
    std::string use_rl_topic_ = "/use_rl";
    std::string use_pid_topic_ = "/use_pid";
//    std::string pid_cmd_topic_ = "/mavros/setpoint_raw/local";
    std::string pid_cmd_topic_ = "/fake_position_target";

    /* drone state */
    Eigen::Matrix<double, 12, 1> state_;
    Eigen::Vector3d pos_;
    Eigen::Quaterniond ori_;
    Eigen::Vector3d att_euler_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d body_rate_;

    /* params */
    std::string drone_name = "iris";
    double ground_height_ = 0.0;
    double crash_timer_period_ = 0.5;
    double dt_ = 0.02;
    Eigen::Vector3d start_pt_ = Eigen::Vector3d::Zero();

    /* flight control */
    Eigen::Vector3d body_rate_req_;
    double rescaled_thrust_req_;
    Eigen::Vector4d motor_thrusts_ = Eigen::Vector4d::Zero();
    Eigen::Vector3d vel_req_;
    double yaw_req_;

    /* flags */
    bool is_initialized_ = false;
    bool crashed_ = false;

    /* resclib utils */
    QuadrotorDynamics quadrotor_dynamics_;
};


#endif //MOGEN_SIMULATOR_MOGENSIMULATOR_H
