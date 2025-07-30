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
 * Created by Zhaohong Liu on 24-11-7.
*/

#include "px4_utils/PX4CtrlFSM.h"

void PX4CtrlFSM::init(ros::NodeHandle &nh) {
    getParamWithWarning(nh, "px4fsm/target_thresh", target_thresh_);
    getParamWithWarning(nh, "px4fsm/exec_period", exec_period_);
    getParamWithWarning(nh, "px4fsm/cruise_height", cruise_height_);
    getParamWithWarning(nh, "px4fsm/fence_x", fence_x_);
    getParamWithWarning(nh, "px4fsm/fence_y", fence_y_);
    getParamWithWarning(nh, "px4fsm/fence_z", fence_z_);
    getParamWithWarning(nh, "px4fsm/ground_height", ground_height_);
    getParamWithWarning(nh, "px4fsm/fence_offset", fence_offset_);
    getParamWithWarning(nh, "px4fsm/max_attitude_degree", max_attitude_);
    max_attitude_ *= deg2rad_;

    getParamWithWarning(nh, "px4fsm/use_rl_topic", use_rl_topic_);
    getParamWithWarning(nh, "px4fsm/rl_cmd_topic", rl_cmd_topic_);
    getParamWithWarning(nh, "px4fsm/land_cmd_topic", land_cmd_topic_);

    offb_mode_setter_.request.custom_mode = "OFFBOARD";
    arm_cmd_.request.value = true;

    exec_timer_ = nh.createTimer(ros::Duration(exec_period_), &PX4CtrlFSM::execCallback, this);
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>(arming_topic_);
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(set_mode_topic_);
    state_sub_ = nh.subscribe<mavros_msgs::State>(state_topic_, 10, &PX4CtrlFSM::stateCallback, this);
    pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic_, 1, &PX4CtrlFSM::poseCallback, this);
    traj_cmd_sub_ = nh.subscribe<quadrotor_msgs::PositionCommand>(traj_cmd_topic_, 1, &PX4CtrlFSM::trajCmdCallback, this);
    use_rl_sub_ = nh.subscribe(use_rl_topic_, 1, &PX4CtrlFSM::useRLCallback, this);
    rl_cmd_sub_ = nh.subscribe(rl_cmd_topic_, 1, &PX4CtrlFSM::rlCmdCallback, this);
    land_cmd_sub_ = nh.subscribe(land_cmd_topic_, 1, &PX4CtrlFSM::landCmdCallback, this);
    extended_state_sub_ = nh.subscribe("/mavros/extended_state", 10, &PX4CtrlFSM::extendedStateCallback, this);

    // pose setpoint is high level, while traj target is mid level
    pose_setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>(pose_setpoint_topic_, 1);
    traj_target_pub_ = nh.advertise<mavros_msgs::PositionTarget>(traj_target_topic_, 1);
    att_target_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(att_target_topic_, 1);

    // set a default thrust for att_target in case of dropping due to delay
    att_target_.thrust = throttle_default_;
}

void PX4CtrlFSM::execCallback(const ros::TimerEvent &) {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100) {
        printFSMExecState();
        fsm_num = 0;
    }

    switch (exec_state_) {
        case INIT:
            if (state_.connected && init_pos_set_) {
                //send a few set points before starting
                for (int i = 0; i < 100; i++) {
                    publishPoseSetpoint(takeoff_pos_);
                    ros::spinOnce();
                    rate_.sleep();
                }
                last_request_time_ = ros::Time::now();
                changeFSMState(OFFBOARD);
            } else if (!state_.connected && fsm_num == 99) {
                std::cout << "\033[1;33m[PX4 FSM]: FCU disconnected.\033[0m" << std::endl;
            } else if (!init_pos_set_ && fsm_num == 99) {
                std::cout << "\033[1;33m[PX4 FSM]: Waiting for init position.\033[0m" << std::endl;
            }
            break;

        case ARM:
            if (!state_.armed && ros::Time::now() - last_request_time_ > ros::Duration(waiting_time_)) {
                if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
                    if (fsm_num == 99) std::cout << "[PX4 FSM]: Arming..." << std::endl;
                    last_request_time_ = ros::Time::now();
                }
            } else if (state_.armed) {
                std::cout << "[PX4 FSM]: Vehicle armed." << std::endl;
                changeFSMState(TAKEOFF);
            }
            break;

        case OFFBOARD:
            publishPoseSetpoint(takeoff_pos_);  // send a few set points before starting, vital!
            if (state_.mode != "OFFBOARD" && ros::Time::now() - last_request_time_ > ros::Duration(waiting_time_)) {
                offb_mode_setter_.request.custom_mode = "OFFBOARD";
                if (set_mode_client_.call(offb_mode_setter_) && offb_mode_setter_.response.mode_sent) {
                    std::cout << "[PX4 FSM]: Offboard mode requested..." << std::endl;
                }
                last_request_time_ = ros::Time::now();
            } else if (state_.mode == "OFFBOARD") {
                std::cout << "[PX4 FSM]: Offboard confirmed." << std::endl;
                changeFSMState(ARM);
            }
            break;

        case TAKEOFF:
            if (state_.mode != "OFFBOARD" && fsm_num == 99) {
                std::cout << "\033[1;33m[PX4 FSM]: Attempt to take off in non-offboard mode.\033[0m" << std::endl;
                break;
            }

            if (isReachedTarget(takeoff_pos_)) {
                std::cout << "\033[1;32m[PX4 FSM]: Takeoff done, holding.\033[0m" << std::endl;
                hold_pos_ = takeoff_pos_;
                changeFSMState(HOLD);
            } else {
                publishPoseSetpoint(takeoff_pos_);
            }
            break;

        case HOLD:
            // any state that wants to change to hold must redefine hold_pos_
            if (use_rl_ && motion_smooth_ && in_geo_fence_) {
                changeFSMState(RL_MOTION);
            } else if (traj_cmd_received_ && in_geo_fence_) {
                last_traj_cmd_time_ = ros::Time::now();
                changeFSMState(TRAJ_CMD);
            } else {
                publishPoseSetpoint(hold_pos_);
            }
            break;

        case RL_MOTION:
            // TODO: if check doesn't need this frequency, changing to use a ros timer
            checkAggressiveMotion();
            if (!motion_smooth_) {
                std::cout << "\033[1;33m[PX4 FSM]: Aggressive motion! Hold now.\033[0m" << std::endl;
                hold_pos_ = pos_;
                changeFSMState(HOLD);
                break;
            }

            if (!in_geo_fence_) {
                std::cout << "\033[1;33m[PX4 FSM]: Out of geo fence! Returning.\033[0m" << std::endl;
                geoFenceClamp(pos_);
                changeFSMState(HOLD);
                break;
            }

            if (use_rl_ && rl_cmd_received_) {
                att_target_pub_.publish(att_target_);
            } else {
                if (!use_rl_) {
                    std::cout << "[PX4 FSM]: RL command is not allowed to use, will hold." << std::endl;
                }
                hold_pos_ = pos_;
                changeFSMState(HOLD);
            }
            break;

        case TRAJ_CMD:
            if (!in_geo_fence_) {
                std::cout << "\033[1;33m[PX4 FSM]: Out of geo fence! Returning.\033[0m" << std::endl;
                geoFenceClamp(pos_);
                changeFSMState(HOLD);
                break;
            }

            if (traj_cmd_received_ && ros::Time::now() - last_traj_cmd_time_ < ros::Duration(traj_cmd_timeout_)) {
                publishTrajSetpoint();
            } else {
                traj_cmd_received_ = false;
                if (ros::Time::now() - last_traj_cmd_time_ > ros::Duration(traj_cmd_timeout_)) {
                    std::cout << "\033[1;33m[PX4 FSM]: Trajectory command timeout.\033[0m" << std::endl;
                    changeFSMState(HOLD);
                }
            }

            break;

        case SOFT_LAND:
            fsmSoftLand();
            break;

        case AUTO_LAND:
            static bool auto_land_triggered = false;
            static ros::Time land_start_time;
        
            if (!auto_land_triggered) {
                if (triggerPX4AutoLand()) {
                    land_start_time = ros::Time::now();
                    auto_land_triggered = true;
                }
                break;
            }
        
            // Wait until PX4 declares landed
            if (extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
                std::cout << "\033[1;32m[PX4 FSM]: AUTO.LAND complete.\033[0m" << std::endl;
                auto_land_triggered = false;
                changeFSMState(LANDED);
                break;
            }
        
            if ((ros::Time::now() - land_start_time).toSec() > 10.0) {
                std::cout << "\033[1;33m[PX4 FSM]: AUTO.LAND timeout.\033[0m" << std::endl;
                auto_land_triggered = false;
            }

            break;

        case LANDED:
            std::cout << "\033[1;32m[PX4 FSM]: Vehicle landed.\033[0m" << std::endl;
            changeFSMState(DISARM);
            break;

        case DISARM:
            static bool disarm_attempted = false;
            static ros::Time disarm_start_time;

            if (!state_.armed) break;
        
            if (!disarm_attempted) {
                disarm_attempted = true;
                disarm_start_time = ros::Time::now();
                std::cout << "[PX4 FSM]: Attempting to disarm..." << std::endl;
            }
            
            // Retry disarming periodically
            if (ros::Time::now() - disarm_start_time > ros::Duration(1.0) && !state_.armed) {
                std::cout << "\033[1;32m[PX4 FSM]: Vehicle disarmed.\033[0m" << std::endl;
                disarm_attempted = false;
            } else if (ros::Time::now() - disarm_start_time > ros::Duration(1.0) && state_.armed) {
                std::cout << "[PX4 FSM]: Disarming..." << std::endl;
                triggerPX4Disarm();
                disarm_start_time = ros::Time::now();
            }
            
            // Give up after timeout
            if (ros::Time::now() - disarm_start_time > ros::Duration(10.0)) {
                ROS_WARN("[PX4 FSM]: Disarm timeout. Giving up and returning to HOLD.");
                disarm_attempted = false;
                changeFSMState(HOLD);
            }

            break;
            
    }
}

void PX4CtrlFSM::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    state_ = *msg;
}

void PX4CtrlFSM::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    att_quat_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                   msg->pose.orientation.y, msg->pose.orientation.z);
    Convertor::q2EulerAngle(att_quat_, att_);

    if (!init_pos_set_) {
        init_pos_buffer_.emplace_back(pos_);
        if (init_pos_buffer_.size() > init_pos_buffer_max_size_) {
            init_pos_set_ = true;
            init_pos_ = Eigen::Vector3d::Zero();
            for (const auto &pos : init_pos_buffer_) {
                init_pos_ += pos;
            }
            auto buffer_size = static_cast<int>(init_pos_buffer_.size());
            init_pos_ /= buffer_size;

            std::cout << "[PX4 FSM]: Init position set to [" << init_pos_.x() << ", "
                      << init_pos_.y() << ", " << init_pos_.z() << "]" << std::endl;
            takeoff_pos_.head(2) = init_pos_.head(2);
            takeoff_pos_.z() = cruise_height_;
        }
    }

    if (pos_.x() < -fence_x_ / 2.0 || pos_.x() > fence_x_ / 2.0 ||
        pos_.y() < -fence_y_ / 2.0 || pos_.y() > fence_y_ / 2.0 ||
        pos_.z() > fence_z_ || pos_.z() < ground_height_) {
        in_geo_fence_ = false;
//        use_rl_ = false;
    } else {
        in_geo_fence_ = true;
    }
}

void PX4CtrlFSM::useRLCallback(const std_msgs::Bool::ConstPtr &msg) {
    use_rl_ = msg->data;
}

void PX4CtrlFSM::rlCmdCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg) {
    att_target_ = *msg;
    if (!rl_cmd_received_) {
        rl_cmd_received_ = true;
    }
}

void PX4CtrlFSM::trajCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg) {
    traj_cmd_received_ = true;
    last_traj_cmd_time_ = ros::Time::now();
    quad_pos_cmd_ = *msg;
}

void PX4CtrlFSM::landCmdCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data && exec_state_ == HOLD || exec_state_ == RL_MOTION || exec_state_ == TRAJ_CMD) {
        std::cout << "[PX4 FSM]: Landing command received. Switching to SOFT_LAND." << std::endl;
        hold_pos_ = pos_;
        changeFSMState(SOFT_LAND);
    }
}

void PX4CtrlFSM::extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg) {
    extended_state_ = *msg;
}

void PX4CtrlFSM::changeFSMState(PX4CtrlFSM::FSM_EXEC_STATE new_state) {
    int pre_state_id = exec_state_;
    exec_state_ = new_state;
    std::cout << "\033[1;34m[PX4 FSM]: " << state_str_[pre_state_id] << " -> "
              << state_str_[exec_state_] << "\033[0m" << std::endl;
}

void PX4CtrlFSM::printFSMExecState() {
    std::cout << "\033[1;35m[PX4 FSM]: " << state_str_[exec_state_] << "\033[0m" << std::endl;
}

bool PX4CtrlFSM::isReachedTarget(const Eigen::Vector3d &target) const {
    return (pos_ - target).norm() <= target_thresh_;
}

void PX4CtrlFSM::publishPoseSetpoint(const Eigen::Vector3d &pos, const double & yaw) {
    pose_setpoint_.pose.position.x = pos.x();
    pose_setpoint_.pose.position.y = pos.y();
    pose_setpoint_.pose.position.z = pos.z();

    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    pose_setpoint_.pose.orientation.w = q.w();
    pose_setpoint_.pose.orientation.x = q.x();
    pose_setpoint_.pose.orientation.y = q.y();
    pose_setpoint_.pose.orientation.z = q.z();

    pose_setpoint_.header.stamp = ros::Time::now();

    pose_setpoint_pub_.publish(pose_setpoint_);
}

void PX4CtrlFSM::publishTrajSetpoint() {
    traj_target_.header.stamp = ros::Time::now();
    traj_target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    traj_target_.position.x = quad_pos_cmd_.position.x;
    traj_target_.position.y = quad_pos_cmd_.position.y;
    traj_target_.position.z = quad_pos_cmd_.position.z;

    traj_target_.velocity.x = quad_pos_cmd_.velocity.x;
    traj_target_.velocity.y = quad_pos_cmd_.velocity.y;
    traj_target_.velocity.z = quad_pos_cmd_.velocity.z;

    traj_target_.acceleration_or_force.x = quad_pos_cmd_.acceleration.x;
    traj_target_.acceleration_or_force.y = quad_pos_cmd_.acceleration.y;
    traj_target_.acceleration_or_force.z = quad_pos_cmd_.acceleration.z;

    traj_target_.yaw = static_cast<float>(quad_pos_cmd_.yaw);
    traj_target_.yaw_rate = static_cast<float>(quad_pos_cmd_.yaw_dot);

    traj_target_.type_mask = 0;

    traj_target_pub_.publish(traj_target_);
}

void PX4CtrlFSM::fsmSoftLand() {
    // Initialize target only once
    static bool land_initialized = false;
    static ros::Time land_start_time;
    static bool near_ground = false;
    static ros::Time ground_detect_time;

    if (!land_initialized) {
        hold_pos_ = pos_;  // Lock x, y
        land_start_time = ros::Time::now();
        land_initialized = true;
        near_ground = false;
        std::cout << "[PX4 FSM]: Soft landing initialized at [" << hold_pos_.x() << ", "
                  << hold_pos_.y() << ", " << hold_pos_.z() << "]" << std::endl;
    }

    // Gradually descend
    hold_pos_.z() -= 0.005;

    publishPoseSetpoint(hold_pos_);

    // Check PX4's internal land detection
    if (extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
        std::cout << "[PX4 FSM]: Landed detected by PX4. Switching to LANDED." << std::endl;
        land_initialized = false;
        changeFSMState(LANDED);
        return;
    }

    // Safety timeout
    if ((ros::Time::now() - land_start_time).toSec() > soft_landing_timeout_) {
        std::cout << "[PX4 FSM]: Soft landing timeout. Switching to AUTO.LAND." << std::endl;
        land_initialized = false;
        changeFSMState(AUTO_LAND);
    }
}


bool PX4CtrlFSM::triggerPX4AutoLand() {
    mavros_msgs::SetMode land_mode;
    land_mode.request.custom_mode = "AUTO.LAND";

    if (set_mode_client_.call(land_mode) && land_mode.response.mode_sent) {
        std::cout << "[PX4 FSM]: AUTO.LAND mode sent to PX4." << std::endl;
        return true;
    } else {
        std::cout << "\033[1;33m[PX4 FSM]: Failed to send AUTO.LAND to PX4.\033[0m" << std::endl;
        return false;
    }
}

bool PX4CtrlFSM::triggerPX4Disarm() {
    if (state_.mode != "OFFBOARD") {
        mavros_msgs::SetMode mode_cmd;
        mode_cmd.request.custom_mode = "OFFBOARD";
        if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
            std::cout << "[PX4 FSM]: OFFBOARD mode sent to PX4." << std::endl;
            ros::Duration(0.5).sleep();  // Give time for mode change
        }
    }

    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;

    for (int i = 0; i < 3; i++) {
        if (arming_client_.call(disarm_cmd) && disarm_cmd.response.success) {
            std::cout << "\033[1;32m[PX4 FSM]: PX4 disarmed successfully.\033[0m" << std::endl;
            return true;
        }
        std::cout << "Retrying disarm..." << std::endl;
        ros::Duration(0.5).sleep();
    }
    
    ROS_ERROR("[PX4 FSM]: All disarm attempts failed.");
    return false;
}

void PX4CtrlFSM::checkAggressiveMotion() {
    if (std::abs(att_.x()) >= max_attitude_ || std::abs(att_.y()) >= max_attitude_) {
        motion_smooth_ = false;
    } else {
        motion_smooth_ = true;
    }
    //TODO: more safety checks
}

void PX4CtrlFSM::geoFenceClamp(const Eigen::Vector3d &pos) {
    if (pos.x() < -fence_x_ / 2.0) {
        hold_pos_.x() = -fence_x_ / 2.0 + fence_offset_;
    } else if (pos.x() > fence_x_ / 2.0) {
        hold_pos_.x() = fence_x_ / 2.0 - fence_offset_;
    }

    if (pos.y() < -fence_y_ / 2.0) {
        hold_pos_.y() = -fence_y_ / 2.0 + fence_offset_;
    } else if (pos.y() > fence_y_ / 2.0) {
        hold_pos_.y() = fence_y_ / 2.0 - fence_offset_;
    }

    if (pos.z() < ground_height_) {
        hold_pos_.z() = ground_height_ + fence_offset_;
    } else if (pos.z() > fence_z_) {
        hold_pos_.z() = fence_z_ - fence_offset_;
    }
}
