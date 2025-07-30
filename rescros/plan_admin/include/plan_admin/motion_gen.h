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
 * Created by Zhaohong Liu on 24-5-9.
*/

#ifndef PLAN_ADMIN_MOTION_GEN_H
#define PLAN_ADMIN_MOTION_GEN_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>

#include <backend_optimizer/rl_motion.h>

#include "k_gpep.h"

using WptPair = std::pair<Eigen::Vector3d, Eigen::Vector3d>;

class MotionGen {
private:
    /* modules */
    RLMotion::Ptr rl_inference_ptr_;
    KGPEP kgpep_;

    /* RL motion parameters */
    Eigen::Vector3d att_req_;
    Eigen::Quaterniond att_req_quat_;
    Eigen::Vector3d body_rate_req_;
    double collective_thrust_req_;  // newton
    float rescaled_thrust_req_;  // rescaled to 0-1
    Eigen::Vector3d wpt0_;
    Eigen::Vector3d wpt1_;
    WptPair wpt_pair_;

    /* RL env config */
    std::string policy_name_;
    int ctrl_type_ = -1;
    int num_obs_ = 0;
    const int obstacle_free_obs_ = 25;
    double rl_dt_ = 0.02;
    double rl_frequency_ = 1 / rl_dt_;
    double thrust_min_single_motor_ = 0.0;  // newton
    double thrust_max_single_motor_ = 0.0;  // newton
    double collective_thrust_dg_min_ = 0.2;
    double collective_thrust_dg_max_ = 1.8;
    int rl_use_monitor_ = 0;

    /* Drone state */
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d acc_;
    Eigen::Vector3d att_received_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d body_rate_received_;
    Eigen::Vector3d ang_vel_;

    /* Drone parameters */
    std::string drone_ = "iris";
    const float g_ = 9.7946;
    double max_body_rate_ = 200 * M_PI / 180;
    double max_yaw_rate_ = 125 * M_PI / 180;
    double rescaled_thrust_min_ = 0.15;
    double rescaled_thrust_max_ = 0.60;
    // mass, size, inertia and torque constant
    float mass_ = 0.535;
    double ixx_ = 0.029125;
    double iyy_ = 0.029125;
    double izz_ = 0.055225;
    Eigen::Matrix3d inertia_;
    Eigen::Matrix4d allocation_mat_;
    Eigen::Matrix4d allocation_mat_inv_;
    double arm_x_ = 0.13;
    double arm_y_long_ = 0.22;
    double arm_y_short_ = 0.2;
    double torqueConstant_ = 0.12;  // for yaw control
    double thrust_intercept_gram_ = -1744.1436;

    /* PX4 and QGC parameters */
    float min_pwm_ = 1000;
    float max_pwm_ = 2000;
    double p_roll_rate_ = 10.0;
    double p_pitch_rate_ = 10.0;
    double p_yaw_rate_ = 5.0;
    Eigen::Matrix3d rate_p_gain_;

    /* ROS utils */
    ros::Subscriber pose_sub_;
    ros::Subscriber wpt0_sub_;
    ros::Subscriber wpt1_sub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber accel_sub_;
    ros::Subscriber body_rate_sub_;
    ros::Publisher cmd_att_pub_;  // for setpoint_raw/attitude, ideal one
    ros::Publisher monitor_pos_target_pub_;
    ros::Publisher monitor_att_target_pub_;
    mavros_msgs::AttitudeTarget cmd_att_;
    mavros_msgs::PositionTarget monitor_pos_target_;
    mavros_msgs::AttitudeTarget monitor_att_target_;
    int pub_queue_size_ = 1;
    int sub_queue_size_ = 1;
    std::string wpt0_topic_ = "/search/wpt0";
    std::string wpt1_topic_ = "/search/wpt1";
    std::string rl_att_cmd_topic_ = "/rl_att_cmd";
    std::string rl_monitor_pos_target_topic_ = "/rl_monitor_pos_target";
    std::string rl_monitor_att_target_topic_ = "/rl_monitor_att_target";
    std::string pose_topic_ = "/mavros/local_position/pose";
    std::string vel_topic_ = "/mavros/local_position/velocity_local";
    std::string accel_topic_ = "/mavros/imu/data";
    std::string body_rate_topic_ = "/mavros/imu/body_rate";
public:
    MotionGen();
    ~MotionGen();
    /**
     * @brief load ros params, init allocation mat, init rl motion ptr and init subscribers and publishers.
     * will also run a parma check in terminal.
     * @param nh ros::NodeHandle
     */
    void init(ros::NodeHandle& nh);

    void initAllocationMat();
    void readParams(ros::NodeHandle& nh);

    /**
     * @brief pose callback, update pos and att.
     * @param msg geometry_msgs::PoseStamped
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief local velocity callback, update vel.
     * @param msg geometry_msgs::TwistStamped
     */
    void velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    /**
     * @brief main loop of updating state and get RL action, will also publish rl cmd.
     */
    void rlMotion();

    /**
     * @brief publish rl cmd, recommended to use in a ros timer.
     */
    void publishRLCmd();

    /**
     * @brief provide some monitor topics about drone state, including pos, att, vel and body rate.
     * This is useful for checking the difference between real world drone state and RL expected ones.
     * Publishers are utilized in this function.
     */
    void rlMonitorRateThrust();

    /**
     * @brief set attitude and yaw rate, source from body_rate_req_, att_req_quat_ and collective_thrust_req_.
     * make sure the input is updated before calling this function.     
     */
    void setCmdAtt();

    /**
     * @brief set body rate and collective thrust, source from body_rate_req_ and collective_thrust_req_.
     */
    void setCmdRate();

    /**
     * @brief collective thrust to rescaled thrust, based on the thrust mapping curve.
     * @param collective_thrust thrust in newton, double
     */
    [[nodiscard]] float rescaleCollectiveThrust(double collective_thrust) const;

    /**
     * @brief quaternion to euler angles.
     * only apply on private member q_ and will update att_received_.
     * @param q Eigen::Quaterniond
     */
    void q2euler(const Eigen::Quaterniond& q);

    /**
     * @brief Translating box2d action from rl policy to 4 motor thrusts in newton.
     * To be noticed, this transferring process need to follow RL env setting.
     * Only for study, do not use in real flight.
     * @param actions_box2d
     * @return 4 motor thrusts in newton
     */
    [[maybe_unused]] Eigen::Vector4d action2Thrust(Eigen::Vector4d& actions_box2d) const;

    /**
     * @brief convert the box2d action from rl policy to rate required and collective thrust required.
     * @param[in] actions_box2d The input action from rl policy.
     * @note This function modifies the following class member variables:
     *      - body_rate_req_
     *      - collective_thrust_req_
     *      - rescaled_thrust_req_
     */
    void action2RateThrust(Eigen::Vector4d& actions_box2d);

    /**
     * @brief convert the box2d action (double) to collective thrust required.
     * @param[in] action_box2d The last element (usually) of the input action from rl policy.
     * @note This function modifies the following class member variables:
     *     - collective_thrust_req_
     *     - rescaled_thrust_req_
     */
    void action2CollectiveThrust(double action_box2d);

    /**
     * @brief convert the box2d actions to delta body rate and collective thrust required.
     *  Then calculating the body rate required and collective thrust required.
     * @param[in] actions_box2d The input action from rl policy.
     * @note This function modifies the following class member variables:
     *     - body_rate_req_
     *     - collective_thrust_req_
     *     - rescaled_thrust_req_
     */
    void action2DeltaRateThrust(Eigen::Vector4d& actions_box2d);

    /**
     * @brief clamp the body rate required to range.
     * @param[in] body_rate_req Eigen::Vector3d
     */
    void clampRateRequired(Eigen::Vector3d& body_rate_req) const;

    /**
     * @brief run the flight control loop, will apply a P ctrl on body rate,
     * calculating control unit (4) and update real motor thrusts required.
     * @return 4 motor thrusts clamped in newton
     */
    Eigen::Vector4d runFlightCtrl();

    /**
     * @brief thrust to torque, based on the allocation matrix.
     * @param motor_thrusts
     * @return Eigen::Vector3d torques
     */
    Eigen::Vector3d thrust2Torque(Eigen::Vector4d& motor_thrusts) const;

    /**
     * @brief torque to body rate required, based on the inertia matrix.
     * @param torques Eigen::Vector3d
     * @param old_body_rate Eigen::Vector3d
     * @return Eigen::Vector3d body rate dot based on current state
     */
    Eigen::Vector3d torque2BodyRateDot(Eigen::Vector3d& torques,
                                       Eigen::Vector3d& old_body_rate) const;

    /**
     * @brief body rate to attitude required, based on the euler angles.
     * @param body_rate Eigen::Vector3d
     * @param old_att Eigen::Vector3d
     * @param dt double
     * @return Eigen::Vector3d attitude required
     */
    static Eigen::Vector3d bodyRate2AttitudeReq(Eigen::Vector3d& body_rate,
                                                Eigen::Vector3d& old_att,
                                                double dt);

    /**
     * @brief update body rate req and attitude req.
     * @param torques Eigen::Vector3d
     */
    void updateAttitude(Eigen::Vector3d& torques,
                        Eigen::Vector3d& body_rate_cur,
                        Eigen::Vector3d& attitude_cur) const;

    /**
     * @brief applying the attitude and body rate required to the drone.
     * @param euler
     * @return Eigen::Matrix3d that will convert body rate to euler rate.
     */
    static Eigen::Matrix3d getRotMatB2A(Eigen::Vector3d& euler);

    /**
     * @brief providing the rotation mat that will convert a vector in body frame to world frame.
     * @param euler
     * @return Eigen::Matrix3d rotation mat
     */
    static Eigen::Matrix3d getRotMatB2W(Eigen::Vector3d& euler);

    /**
     * @brief convert a rescaled thrust in [0, 1] to real thrust in newton based on thrust mapping.
     * @param rescaled_thrust double
     */
    double irisThrustMapping(double& rescaled_thrust) const;

    /**
     * @brief clamp the thrusts to the range of [thrust_min_single_motor_, thrust_max_single_motor_].
     * will revise the value of input reference motor_thrusts.
     * @param motor_thrusts Eigen::Vector4d
     */
    void clampThrust(Eigen::Vector4d& motor_thrusts);

    /**
     * @brief set some private members based on ros params.
     * @tparam T ros param type, might be int, double or string
     * @param nh ros::NodeHandle
     * @param param_name corresponding private member by its ros param
     */
    template<typename T>
    void getParamWithWarning(ros::NodeHandle& nh, const std::string& param_name, T& param) {
        if (!nh.getParam(param_name, param)) {
            ROS_WARN_STREAM("Failed to get param: " << param_name);
        }
    }

    /**
     * @brief check all ros params, will print out all params in terminal.
     */
    void paramCheck();

    void setMapBridge(const MapBridge::Ptr& ptr);
public:
    using Ptr = std::unique_ptr<MotionGen>;
};


#endif //PLAN_ADMIN_MOTION_GEN_H
