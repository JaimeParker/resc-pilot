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

#include <plan_admin/motion_gen.h>

MotionGen::MotionGen() {
    ROS_INFO_STREAM("\033[1;34m" << "RL Optimizer initializing..." << "\033[0m");
    pos_ = Eigen::Vector3d::Zero();
    vel_ = Eigen::Vector3d::Zero();
    acc_ = Eigen::Vector3d::Zero();
    att_received_ = Eigen::Vector3d::Zero();
    att_req_ = Eigen::Vector3d::Zero();
    body_rate_req_ = Eigen::Vector3d::Zero();
    q_ = Eigen::Quaterniond::Identity();
    ang_vel_ = Eigen::Vector3d::Zero();
    rescaled_thrust_req_ = 0.0f;
    collective_thrust_req_ = 0.0;

    inertia_ = Eigen::Matrix3d::Identity();
    rate_p_gain_ = Eigen::Matrix3d::Identity();
    allocation_mat_ = Eigen::Matrix4d::Identity();
    allocation_mat_inv_ = Eigen::Matrix4d::Identity();
}

MotionGen::~MotionGen() {
    std::cout<< "Plan Admin: MotionGen is destructed" << std::endl;
}

void MotionGen::init(ros::NodeHandle& nh) {
    // read params from server
    readParams(nh);

    // update some params
    initAllocationMat();
    inertia_(0, 0) = ixx_;
    inertia_(1, 1) = iyy_;
    inertia_(2, 2) = izz_;
    rate_p_gain_(0, 0) = p_roll_rate_ / ixx_;
    rate_p_gain_(1, 1) = p_pitch_rate_ / iyy_;
    rate_p_gain_(2, 2) = p_yaw_rate_ / izz_;

    // init motion ptr
    rl_inference_ptr_ = std::make_unique<RLMotion>();
    // set model name before init
    rl_inference_ptr_->setModelName(policy_name_);
    rl_inference_ptr_->setNumObs(num_obs_);
    rl_inference_ptr_->init();

    // init sub
    pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
            pose_topic_,
            sub_queue_size_,
            &MotionGen::poseCallback, this);
    vel_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(
            vel_topic_,
            sub_queue_size_,
            &MotionGen::velCallback, this);
    // note: NOKOV got a TwistStamped type of accel message for using vrpn directly
    accel_sub_ = nh.subscribe<sensor_msgs::Imu>(
            accel_topic_,
            sub_queue_size_,
            [&](const sensor_msgs::Imu::ConstPtr &msg) {
                acc_ << msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z;
            });
    body_rate_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(
            body_rate_topic_,
            sub_queue_size_,
            [&](const geometry_msgs::TwistStamped::ConstPtr &msg) {
                body_rate_received_ << msg->twist.angular.x,
                                       msg->twist.angular.y,
                                       msg->twist.angular.z;
            });
    wpt0_sub_ = nh.subscribe<geometry_msgs::Point>(
            wpt0_topic_, sub_queue_size_,
            [&](const geometry_msgs::Point::ConstPtr &msg) {
                wpt0_ << msg->x, msg->y, msg->z;
            });
    wpt1_sub_ = nh.subscribe<geometry_msgs::Point>(
            wpt1_topic_, sub_queue_size_,
            [&](const geometry_msgs::Point::ConstPtr &msg) {
                wpt1_ << msg->x, msg->y, msg->z;
            });

    // init pub
    cmd_att_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(
            rl_att_cmd_topic_, pub_queue_size_);
    monitor_pos_target_pub_ = nh.advertise<mavros_msgs::PositionTarget>(
            rl_monitor_pos_target_topic_, pub_queue_size_);
    monitor_att_target_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(
            rl_monitor_att_target_topic_, pub_queue_size_);

    // run the check
    paramCheck();
}

void MotionGen::initAllocationMat() {
    allocation_mat_ << 1, 1, 1, 1,
            -arm_y_long_, -arm_y_short_, arm_y_short_, arm_y_long_,
            -arm_x_, arm_x_, arm_x_, -arm_x_,
            -torqueConstant_, torqueConstant_, -torqueConstant_, torqueConstant_;
    allocation_mat_inv_ = allocation_mat_.inverse();
}

void MotionGen::poseCallback(const geometry_msgs::PoseStamped_<std::allocator<void>>::ConstPtr &msg) {
    pos_ << msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z;
    q_ = Eigen::Quaterniond (msg->pose.orientation.w, msg->pose.orientation.x,
                             msg->pose.orientation.y, msg->pose.orientation.z);
    q2euler(q_);
}

void MotionGen::velCallback(const geometry_msgs::TwistStamped_<std::allocator<void>>::ConstPtr &msg) {
    // get velocity from world frame, using "/mavros/local_position/velocity_local"
    vel_ << msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z;
}

void MotionGen::rlMotion() {
    ros::Rate rate(rl_frequency_);
    while (ros::ok()) {
        publishRLCmd();
        ros::spinOnce();
        rate.sleep();
    }
}

void MotionGen::publishRLCmd() {
    wpt_pair_ = std::make_pair(wpt0_, wpt1_);
    if (num_obs_ > obstacle_free_obs_) {
        Eigen::Vector2d rela_pos = kgpep_.getRelativePos(pos_);
        std::vector<double> sdf_values = kgpep_.getSurroundingSDF(pos_);
        std::vector<double> raycast = kgpep_.getKinematicPseudoRaycast(pos_, vel_, wpt0_);

//        std::cout << "ray info ";
//        for (const auto & val : raycast) {
//            std::cout << val << " ";
//        }
//        std::cout << std::endl;

        rl_inference_ptr_->setAllStates(pos_, vel_, att_received_, body_rate_received_, wpt_pair_,
                                        rela_pos, sdf_values, raycast);
    } else {
        rl_inference_ptr_->setAllStates(pos_, vel_, att_received_, body_rate_received_, wpt_pair_);
    }
    Eigen::Vector4d action = rl_inference_ptr_->forwardModel();

    if (ctrl_type_ == 0) {
        // action: [p, q, r, c_thrust]
        action2RateThrust(action);
        setCmdRate();
        cmd_att_pub_.publish(cmd_att_);

        // call the monitor for real flight debug
        if (rl_use_monitor_ == 1) {
            rlMonitorRateThrust();
        }
    } else if (ctrl_type_ == 1) {
        // action: [f0, f1, f2, f3]
        // difficult for real flight
        Eigen::Vector4d motor_thrusts = action2Thrust(action);
        collective_thrust_req_ = motor_thrusts.sum();
        ROS_WARN("This method has not been implemented yet.");
    } else if (ctrl_type_ == 2) {
        // action: [delta_p, delta_q, delta_r, c_thrust]
        action2DeltaRateThrust(action);
        setCmdRate();
        cmd_att_pub_.publish(cmd_att_);

        // call the monitor for real flight debug
        if (rl_use_monitor_ == 1) {
            rlMonitorRateThrust();
        }
    } else if (ctrl_type_ == 3) {
        // use RL predicted attitude as the control command
        action2RateThrust(action);
        rlMonitorRateThrust();  // necessary

        body_rate_req_ = Eigen::Vector3d(monitor_att_target_.body_rate.x,
                                         monitor_att_target_.body_rate.y,
                                         monitor_att_target_.body_rate.z);
        att_req_quat_ = Eigen::Quaterniond(monitor_att_target_.orientation.w,
                                           monitor_att_target_.orientation.x,
                                           monitor_att_target_.orientation.y,
                                           monitor_att_target_.orientation.z);
        setCmdAtt();
        cmd_att_pub_.publish(cmd_att_);
    } else {
        ROS_ERROR("Unknown control type, will set motor thrusts as 0.0f.");
        ROS_INFO("told u to use 0 or 1 in launch file, what the hell did u do?");
        exit(1);
    }
}

void MotionGen::rlMonitorRateThrust() {
    // run ctrl to get motor thrusts based on that req control commands
    Eigen::Vector4d motor_thrusts = runFlightCtrl();

    // update rl expected state, including body rate, att, acc, vel and pos in the next dt
    // we expect the motor thrust can change immediately
    Eigen::Vector3d torques = thrust2Torque(motor_thrusts);
    Eigen::Vector3d body_rate_next = body_rate_received_;
    Eigen::Vector3d att_next = att_received_;
    updateAttitude(torques, body_rate_next, att_next);
    Eigen::Quaterniond att_next_quat =
            Eigen::AngleAxisd(att_next[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(att_next[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(att_next[0], Eigen::Vector3d::UnitX());

    Eigen::Vector3d thrust_force = motor_thrusts.sum() * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d g_accel = -Eigen::Vector3d::UnitZ() * g_;
    Eigen::Vector3d acc_next = getRotMatB2W(att_next) * thrust_force / mass_ + g_accel;
    Eigen::Vector3d vel_next = vel_ + (acc_next + acc_) / 2 * rl_dt_;
    Eigen::Vector3d pos_next = pos_ + (vel_next + vel_) / 2 * rl_dt_;

    
    auto ros_time_now = ros::Time::now();

    // mavros_msgs::PositionTarget
    monitor_pos_target_.header.stamp = ros_time_now;
    monitor_pos_target_.header.frame_id = "map";
    monitor_pos_target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    monitor_pos_target_.type_mask = 0;
    monitor_pos_target_.position.x = pos_next[0];
    monitor_pos_target_.position.y = pos_next[1];
    monitor_pos_target_.position.z = pos_next[2];
    monitor_pos_target_.velocity.x = vel_next[0];
    monitor_pos_target_.velocity.y = vel_next[1];
    monitor_pos_target_.velocity.z = vel_next[2];
    monitor_pos_target_.acceleration_or_force.x = acc_next[0];
    monitor_pos_target_.acceleration_or_force.y = acc_next[1];
    monitor_pos_target_.acceleration_or_force.z = acc_next[2];
    monitor_pos_target_.yaw = static_cast<float>(att_next[2]);
    monitor_pos_target_.yaw_rate = static_cast<float>(body_rate_next[2]);

    // mavros_msgs::AttitudeTarget
    monitor_att_target_.header.stamp = ros_time_now;
    monitor_att_target_.header.frame_id = "map";
    monitor_att_target_.type_mask = 0;
    monitor_att_target_.body_rate.x = body_rate_next[0];
    monitor_att_target_.body_rate.y = body_rate_next[1];
    monitor_att_target_.body_rate.z = body_rate_next[2];
    monitor_att_target_.thrust = static_cast<float>(collective_thrust_req_ / mass_);
    monitor_att_target_.orientation.w = att_next_quat.w();
    monitor_att_target_.orientation.x = att_next_quat.x();
    monitor_att_target_.orientation.y = att_next_quat.y();
    monitor_att_target_.orientation.z = att_next_quat.z();

    monitor_pos_target_pub_.publish(monitor_pos_target_);
    monitor_att_target_pub_.publish(monitor_att_target_);
}

void MotionGen::setCmdAtt() {
    // yaw rate is necessary for attitude control
    cmd_att_.header.stamp = ros::Time::now();
    cmd_att_.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                         mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
    cmd_att_.header.frame_id = "map";
    cmd_att_.body_rate.z = body_rate_req_[2];
    cmd_att_.thrust = rescaled_thrust_req_;
    cmd_att_.orientation.w = att_req_quat_.w();
    cmd_att_.orientation.x = att_req_quat_.x();
    cmd_att_.orientation.y = att_req_quat_.y();
    cmd_att_.orientation.z = att_req_quat_.z();
}

void MotionGen::setCmdRate() {
    cmd_att_.header.stamp = ros::Time::now();
    cmd_att_.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;  // blacklist attitude
    cmd_att_.header.frame_id = "map";
    cmd_att_.body_rate.x = body_rate_req_[0];
    cmd_att_.body_rate.y = body_rate_req_[1];
    cmd_att_.body_rate.z = body_rate_req_[2];
    cmd_att_.thrust = rescaled_thrust_req_;
}

float MotionGen::rescaleCollectiveThrust(double collective_thrust) const{
    float all_thrust = static_cast<float>(collective_thrust) / g_ * 1000;  // thrust in g

    float thrust_rescaled = 0.0f;

    if (drone_ == "imp250") {
        /**
         * PWM to drag mapping for T-Motor F60 PRO KV2550 with 5477 propeller,
         * tested by Zhaohong Liu, Yinshuai Sun and Tao Cui on 24-5-15.
         * Valid PWM range (that we tested): 1200-1800.
         * Ref on: https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter.html#thrust-curve,
         * and https://docs.px4.io/main/en/advanced_config/parameter_reference.html#THR_MDL_FAC,
         * which tells that rel_thrust = factor * rel_signal^2 + (1-factor) * rel_signal.
         * PWM to drag is usually modeled as a quadratic polynomial, and that's what we did here.
         * However, the PWM to drag mapping for this motor-propeller pair is almost linear.
         * After fitting with quadratic poly: standard error: 19.0058g, max error: 43.3346g.
         * drag_one_rotor (in gram) = 1.4987 * PWM (in mus, 1e-6 s) - 1744.1436, yes, a linear one
         * Special thanks to Wenxuan Gao and ZeShuai Chen.
         */
        auto pwm = static_cast<float>((all_thrust / 4.0 - thrust_intercept_gram_) / 1.4987);
        // the slope might be precise, but we highly recommend that you check the hover throttle,
        // then revise the intercept (this can be inaccurate).
        if (pwm < min_pwm_) {
            pwm = min_pwm_;
            ROS_WARN("PWM is lower than min_pwm, check this bug");
        } else if (pwm > max_pwm_) {
            pwm = max_pwm_;
            ROS_WARN("PWM is higher than max_pwm, check this bug");
        }
        thrust_rescaled = (pwm - min_pwm_) / (max_pwm_ - min_pwm_);
    } else if (drone_ == "iris") {
        /**
         * For a iris model in gazebo, the formula is quadratic, for default iris.sdf:
         * drag (gram) = (10 * (1000t + 100))^2 * 5.84e-6, for each rotor
         * motor rot vel: 1000t + 100, t is the rescaled thrust in [0, 1],
         *   1000 is input_scaling in iris.sdf of mavlink_interface, 100 is zero_position_armed
         * real motor rot vel: 10 * motor_tor_vel_, 10 is rotorVelocitySlowdownSim in motor model
         * 5.84e-6 is the motor constant in iris.sdf
         * Great thanks to Wenxuan Gao.
         */
         // no need to calculate pwm for iris model
        thrust_rescaled = static_cast<float>((std::sqrt(all_thrust / 4.0 * 1e6 / 5.84) - 1e3) / 1e4);
    } else {
        ROS_ERROR("Unknown drone model, will set rescaled thrust as 0.0f.");
    }

    return thrust_rescaled;
}

void MotionGen::q2euler(const Eigen::Quaterniond &q) {
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    // roll (x-axis rotation)
    double sr_cp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cr_cp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    att_received_[0] = std::atan2(sr_cp, cr_cp);

    // pitch (y-axis rotation)
    double sp = std::sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z()));
    double cp = std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()));
    att_received_[1] = 2 * std::atan2(sp, cp) - M_PI / 2;

    // yaw (z-axis rotation)
    double sy_cp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cy_cp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    att_received_[2] = std::atan2(sy_cp, cy_cp);
}

[[maybe_unused]] Eigen::Vector4d MotionGen::action2Thrust(Eigen::Vector4d &actions_box2d) const {
    Eigen::Vector4d actions_rescaled = (actions_box2d.array() + 1.0) / 2.0;
    Eigen::Vector4d motor_thrust = actions_rescaled.array()
            * (thrust_max_single_motor_ - thrust_min_single_motor_) + thrust_min_single_motor_;

    return motor_thrust;
}

void MotionGen::action2RateThrust(Eigen::Vector4d &actions_box2d) {
    body_rate_req_[0] = actions_box2d[0] * max_body_rate_;
    body_rate_req_[1] = actions_box2d[1] * max_body_rate_;
    body_rate_req_[2] = actions_box2d[2] * max_yaw_rate_;

    action2CollectiveThrust(actions_box2d[3]);
}

void MotionGen::action2CollectiveThrust(double action_box2d) {
    double ac1d = (action_box2d + 1.0) / 2.0;  // [0, 1]
    collective_thrust_req_ = ac1d * (collective_thrust_dg_max_ - collective_thrust_dg_min_)
            + collective_thrust_dg_min_;
    collective_thrust_req_ *= mass_ * g_;
    rescaled_thrust_req_ = rescaleCollectiveThrust(collective_thrust_req_);
}

Eigen::Vector4d MotionGen::runFlightCtrl() {
    auto body_rate_err = body_rate_req_ - body_rate_received_;
    auto torque_req = inertia_ * rate_p_gain_ * body_rate_err +
            body_rate_received_.cross(inertia_ * body_rate_received_);
    Eigen::Vector4d ctrl_unit;
    ctrl_unit << collective_thrust_req_, torque_req[0], torque_req[1], torque_req[2];

    Eigen::Vector4d motor_thrusts = allocation_mat_inv_ * ctrl_unit;
    clampThrust(motor_thrusts);

    return motor_thrusts;
}

void MotionGen::action2DeltaRateThrust(Eigen::Vector4d &actions_box2d) {
    Eigen::Vector3d delta_body_rate(
            actions_box2d[0] * 4500 / 180 * M_PI,
            actions_box2d[1] * 4500 / 180 * M_PI,
            actions_box2d[2] * 1000 / 180 * M_PI
    );
    delta_body_rate = delta_body_rate.array() * rl_dt_;
    Eigen::Vector3d body_rate_req_raw = delta_body_rate + body_rate_received_;
    clampRateRequired(body_rate_req_raw);

    body_rate_req_ = body_rate_req_raw;
    action2CollectiveThrust(actions_box2d[3]);
}

void MotionGen::clampRateRequired(Eigen::Vector3d &body_rate_req) const {
    body_rate_req = body_rate_req.array()
            .min(Eigen::Array3d(max_body_rate_, max_body_rate_, max_yaw_rate_))
            .max(Eigen::Array3d(-max_body_rate_, -max_body_rate_, -max_yaw_rate_));
}

Eigen::Vector3d MotionGen::thrust2Torque(Eigen::Vector4d &motor_thrusts) const {
    // forward, left and up frame (similar to ENU)
    Eigen::Vector3d torque = Eigen::Vector3d::Zero();
    double force0 = motor_thrusts[0];
    double force1 = motor_thrusts[1];
    double force2 = motor_thrusts[2];
    double force3 = motor_thrusts[3];

    torque[0] = (force2 - force1) * arm_y_short_ + (force3 - force0) * arm_y_long_;  // roll
    torque[1] = (force1 + force2 - force0 - force3) * arm_x_;  // pitch
    torque[2] = (force1 + force3 - force0 - force2) * torqueConstant_;  // yaw

    return torque;
}

Eigen::Vector3d MotionGen::torque2BodyRateDot(Eigen::Vector3d& torques,
                                              Eigen::Vector3d& old_body_rate) const {
    Eigen::Vector3d body_rate_dot;

    body_rate_dot[0] = torques[0] + (iyy_ - izz_) * old_body_rate[1] * old_body_rate[2];
    body_rate_dot[1] = torques[1] + (izz_ - ixx_) * old_body_rate[0] * old_body_rate[2];
    body_rate_dot[2] = torques[2] + (ixx_ - iyy_) * old_body_rate[0] * old_body_rate[1];

    body_rate_dot[0] /= ixx_;
    body_rate_dot[1] /= iyy_;
    body_rate_dot[2] /= izz_;

    return body_rate_dot;
}

Eigen::Vector3d MotionGen::bodyRate2AttitudeReq(Eigen::Vector3d& body_rate,
                                                Eigen::Vector3d& old_att,
                                                double dt) {
    Eigen::Vector3d att_dot = getRotMatB2A(old_att) * body_rate;
    Eigen::Vector3d new_att = old_att + att_dot * dt;

    return new_att;
}

void MotionGen::updateAttitude(Eigen::Vector3d& torques,
                               Eigen::Vector3d& body_rate_cur,
                               Eigen::Vector3d& attitude_cur) const {
    // use the same ddt in rl training
    double ddt = rl_dt_ / 4;

    for (int i = 0; i < 4; i++) {
        auto body_rate_dot = torque2BodyRateDot(torques, body_rate_cur);
        body_rate_cur += body_rate_dot * ddt;
        attitude_cur = bodyRate2AttitudeReq(body_rate_cur, attitude_cur, ddt);
    }
}

Eigen::Matrix3d MotionGen::getRotMatB2A(Eigen::Vector3d &euler) {
    double phi = euler[0];
    double theta = euler[1];

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 1, std::tan(theta) * std::sin(phi), std::tan(theta) * std::cos(phi),
            0, std::cos(phi), -std::sin(phi),
            0, std::sin(phi) / (std::cos(theta) + 1e-8), std::cos(phi) / (std::cos(theta) + 1e-8);

    return rotation_matrix;
}

Eigen::Matrix3d MotionGen::getRotMatB2W(Eigen::Vector3d &euler) {
    double phi = euler[0];
    double theta = euler[1];
    double psi = euler[2];

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << cos(theta) * cos(psi), cos(theta) * sin(psi), -sin(theta),
            sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi),
            sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi),
            sin(phi) * cos(theta),
            cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi),
            cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi),
            cos(phi) * cos(theta);

    return rotation_matrix;
}

void MotionGen::readParams(ros::NodeHandle &nh) {
    // ROS utils
    getParamWithWarning(nh, "sub_queue_size", sub_queue_size_);
    getParamWithWarning(nh, "pub_queue_size", pub_queue_size_);
    getParamWithWarning(nh, "wpt0_topic", wpt0_topic_);
    getParamWithWarning(nh, "wpt1_topic", wpt1_topic_);
    getParamWithWarning(nh, "rl_att_cmd_topic", rl_att_cmd_topic_);
    getParamWithWarning(nh, "rl_monitor_pos_topic", rl_monitor_pos_target_topic_);
    getParamWithWarning(nh, "rl_monitor_att_topic", rl_monitor_att_target_topic_);
    getParamWithWarning(nh, "pose_sub_topic", pose_topic_);
    getParamWithWarning(nh, "vel_sub_topic", vel_topic_);
    getParamWithWarning(nh, "accel_sub_topic", accel_topic_);
    getParamWithWarning(nh, "body_rate_sub_topic", body_rate_topic_);

    // RL env params
    getParamWithWarning(nh, "rl/policy", policy_name_);
    getParamWithWarning(nh, "rl/env_num_obs", num_obs_);
    getParamWithWarning(nh, "rl/env_dt", rl_dt_);
    getParamWithWarning(nh, "rl/ctrl_type", ctrl_type_);
    rl_frequency_ = 1 / rl_dt_;

    // Sim to real check
    getParamWithWarning(nh, "rl/monitor", rl_use_monitor_);

    // Drone params
    getParamWithWarning(nh, "drone/drone_id", drone_);
    getParamWithWarning(nh, "drone/rescaled_thrust_min", rescaled_thrust_min_);
    getParamWithWarning(nh, "drone/rescaled_thrust_max", rescaled_thrust_max_);
    getParamWithWarning(nh, "drone/mass", mass_);
    getParamWithWarning(nh, "drone/ixx", ixx_);
    getParamWithWarning(nh, "drone/iyy", iyy_);
    getParamWithWarning(nh, "drone/izz", izz_);
    getParamWithWarning(nh, "drone/arm_x", arm_x_);
    getParamWithWarning(nh, "drone/arm_y_long", arm_y_long_);
    getParamWithWarning(nh, "drone/arm_y_short", arm_y_short_);
    getParamWithWarning(nh, "drone/torque_constant", torqueConstant_);
    getParamWithWarning(nh, "drone/body_rate_max", max_body_rate_);
    getParamWithWarning(nh, "drone/yaw_rate_max", max_yaw_rate_);
    max_body_rate_ = max_body_rate_ * M_PI / 180;  // default in degree in launch file
    max_yaw_rate_ = max_yaw_rate_ * M_PI / 180;  // default in degree in launch file
    getParamWithWarning(nh, "drone/collective_thrust_dg_min", collective_thrust_dg_min_);
    getParamWithWarning(nh, "drone/collective_thrust_dg_max", collective_thrust_dg_max_);
    getParamWithWarning(nh, "drone/thrust_intersect_gram", thrust_intercept_gram_);

    // QGC and PX4 params
    getParamWithWarning(nh, "qgc/pwm_min", min_pwm_);
    getParamWithWarning(nh, "qgc/pwm_max", max_pwm_);
    getParamWithWarning(nh, "qgc/p_roll_rate", p_roll_rate_);
    getParamWithWarning(nh, "qgc/p_pitch_rate", p_pitch_rate_);
    getParamWithWarning(nh, "qgc/p_yaw_rate", p_yaw_rate_);

    // init thrust range
    if (drone_ == "iris") {
        thrust_min_single_motor_ = irisThrustMapping(rescaled_thrust_min_);
        thrust_max_single_motor_ = irisThrustMapping(rescaled_thrust_max_);
    } else if (drone_ == "imp250") {
        // TODO: for real flight
        thrust_min_single_motor_ = 0.0;
        thrust_max_single_motor_ = 0.0;
    }
}

double MotionGen::irisThrustMapping(double &rescaled_thrust) const {
    return std::pow((1e4 * rescaled_thrust + 1e3), 2) * 5.84 * 1e-6 / 1e3 * g_;  // newton
}

void MotionGen::paramCheck() {
    ROS_INFO_STREAM("sub_queue_size: " << sub_queue_size_);
    ROS_INFO_STREAM("pub_queue_size: " << pub_queue_size_);
    ROS_INFO_STREAM("wpt0_topic: " << wpt0_topic_);
    ROS_INFO_STREAM("wpt1_topic: " << wpt1_topic_);
    ROS_INFO_STREAM("rl_att_cmd_topic: " << rl_att_cmd_topic_);
    ROS_INFO_STREAM("rl monitor for position target: " << rl_monitor_pos_target_topic_);
    ROS_INFO_STREAM("rl monitor for attitude target: " << rl_monitor_att_target_topic_);
    ROS_INFO_STREAM("policy_name_: " << policy_name_);
    ROS_INFO_STREAM("num_obs_: " << num_obs_);
    ROS_INFO_STREAM("rl_dt_: " << rl_dt_);
    ROS_INFO_STREAM("rl_frequency_: " << rl_frequency_);
    ROS_INFO_STREAM("drone_: " << drone_);
    ROS_INFO_STREAM("rescaled_thrust_min_: " << rescaled_thrust_min_);
    ROS_INFO_STREAM("rescaled_thrust_max_: " << rescaled_thrust_max_);
    ROS_INFO_STREAM("max body rate(p, q): " << max_body_rate_);
    ROS_INFO_STREAM("max yaw rate(r): " << max_yaw_rate_);
    ROS_INFO_STREAM("mass_: " << mass_);
    ROS_INFO_STREAM("ixx_: " << ixx_);
    ROS_INFO_STREAM("iyy_: " << iyy_);
    ROS_INFO_STREAM("izz_: " << izz_);
    ROS_INFO_STREAM("arm_x_: " << arm_x_);
    ROS_INFO_STREAM("arm_y_long_: " << arm_y_long_);
    ROS_INFO_STREAM("arm_y_short_: " << arm_y_short_);
    ROS_INFO_STREAM("torqueConstant_: " << torqueConstant_);
    ROS_INFO_STREAM("min_pwm_: " << min_pwm_);
    ROS_INFO_STREAM("max_pwm_: " << max_pwm_);
    ROS_INFO_STREAM("p_roll_rate_: " << p_roll_rate_);
    ROS_INFO_STREAM("p_pitch_rate_: " << p_pitch_rate_);
    ROS_INFO_STREAM("p_yaw_rate_: " << p_yaw_rate_);
    ROS_INFO_STREAM("thrust_min_single_motor_ (newton): " << thrust_min_single_motor_);
    ROS_INFO_STREAM("thrust_max_single_motor_ (newton): " << thrust_max_single_motor_);
}

void MotionGen::clampThrust(Eigen::Vector4d &motor_thrusts) {
    std::for_each(motor_thrusts.data(), motor_thrusts.data() + motor_thrusts.size(),
                  [this](double &thrust) {
                        thrust = std::clamp(thrust, thrust_min_single_motor_, thrust_max_single_motor_);
                    });
}

void MotionGen::setMapBridge(const MapBridge::Ptr &ptr) {
    kgpep_.setMapBridge(ptr);
    std::cout << "running a test for kgpep method." << std::endl;
    Eigen::Vector3d temp_pos = Eigen::Vector3d(1.0, 2.0, 1.0);
    Eigen::Vector3d temp_vel = Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Vector3d temp_wpt0 = Eigen::Vector3d(2.0, 2.0, 1.0);
    auto ray_info = kgpep_.getKinematicPseudoRaycast(temp_pos, temp_vel, temp_wpt0);
    std::cout << "ray info ";
    for (const auto & val : ray_info) {
        std::cout << val << " ";
    }
}
