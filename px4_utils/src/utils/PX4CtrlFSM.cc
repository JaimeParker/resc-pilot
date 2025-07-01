//
// Created by Zhaohong Liu on 24-11-7.
//

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

    // for return-landing-takeoff-traj-landing process
    nav_goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &PX4CtrlFSM::navGoalCallback, this);
    edit_mode_sub_ = nh.subscribe("/editable_mode", 1, &PX4CtrlFSM::editModeCallback, this);
    nav_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    rtb_sub_ = nh.subscribe("/return_to_base", 1, &PX4CtrlFSM::rtbCallback, this);
    arm_sub_ = nh.subscribe("/trigger_arming", 1, &PX4CtrlFSM::armCallback, this);

    // Publisher for origin position as geometry_msgs::Point
    origin_pos_pub_ = nh.advertise<geometry_msgs::Point>("/origin_pos", 1);
    height_change_sub_ = nh.subscribe("/height_change", 10, &PX4CtrlFSM::heightChangeCallback, this);
    abs_height_pub_ = nh.advertise<std_msgs::Float32>("/abs_height", 1);
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

            // uncomment this if you want a bang-bang takeoff
            // applyble for robust localization like motion capture)

            if (isReachedTarget(takeoff_pos_)) {
                std::cout << "\033[1;32m[PX4 FSM]: Takeoff done, holding.\033[0m" << std::endl;
                hold_pos_ = takeoff_pos_;
                changeFSMState(HOLD);
            } else {
                publishPoseSetpoint(takeoff_pos_);
            }

            // fsmGradualTakeoff();

            break;

        case HOLD:
            // any state that wants to change to hold must redefine hold_pos_
            if (use_rl_ && motion_smooth_ && in_geo_fence_) {
                changeFSMState(RL_MOTION);
            } else if (traj_cmd_received_ && in_geo_fence_) {
                last_traj_cmd_time_ = ros::Time::now();
                changeFSMState(TRAJ_CMD);
            } else {
                publishPoseSetpoint(hold_pos_, hold_yaw_);
            }
            break;

        case RL_MOTION:
            // TODO: if check doesn't need this frequency, changing to use a ros timer
            checkAggressiveMotion();
            if (!motion_smooth_) {
                std::cout << "\033[1;33m[PX4 FSM]: Aggressive motion! Hold now.\033[0m" << std::endl;
                hold_pos_ = pos_;
                hold_yaw_ = att_.z();
                changeFSMState(HOLD);
                break;
            }

            if (!in_geo_fence_) {
                std::cout << "\033[1;33m[PX4 FSM]: Out of geo fence! Returning.\033[0m" << std::endl;
                geoFenceClamp(pos_);
                hold_pos_ = pos_;
                hold_yaw_ = att_.z();
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
                hold_yaw_ = att_.z();
                changeFSMState(HOLD);
            }
            break;

        case TRAJ_CMD:
            if (!in_geo_fence_) {
                std::cout << "\033[1;33m[PX4 FSM]: Out of geo fence! Returning.\033[0m" << std::endl;
                geoFenceClamp(pos_);
                hold_pos_ = pos_;
                hold_yaw_ = att_.z();
                changeFSMState(HOLD);
                break;
            }

            if (traj_cmd_received_ && ros::Time::now() - last_traj_cmd_time_ < ros::Duration(traj_cmd_timeout_)) {
                publishTrajSetpoint();
            } 
            else {
                traj_cmd_received_ = false;
                // if (ros::Time::now() - last_traj_cmd_time_ > ros::Duration(traj_cmd_timeout_)) {
                //     std::cout << "\033[1;33m[PX4 FSM]: Trajectory command timeout.\033[0m" << std::endl;
                //     hold_pos_ = pos_;
                //     changeFSMState(HOLD);
                // }
                hold_pos_ = pos_;
                hold_yaw_ = att_.z();
                changeFSMState(HOLD);
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

        case EDIT:
            handleEditMode();
            break;
            
    }

    if (landing_sequence_active_) {
        executeAutoLandingSequence();
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
            takeoff_pos_.z() = cruise_height_ + init_pos_.z();
            
            init_pos_buffer_.clear();
        }
    }

    if (!origin_pos_initialized_ && init_pos_set_) {
        // fix origin position when the first init position is set
        origin_point_.x = init_pos_.x();
        origin_point_.y = init_pos_.y();
        origin_point_.z = init_pos_.z();
        origin_pos_initialized_ = true;
        std::cout << "[PX4 FSM]: Origin position initialized to [" << origin_point_.x << ", "
                  << origin_point_.y << ", " << origin_point_.z << "]" << std::endl;
    } else {
        origin_pos_pub_.publish(origin_point_);
    }

    if (pos_.x() < -fence_x_ / 2.0 || pos_.x() > fence_x_ / 2.0 ||
        pos_.y() < -fence_y_ / 2.0 || pos_.y() > fence_y_ / 2.0 ||
        pos_.z() > fence_z_ ) {
        // remove || pos_.z() < ground_height_ since z of rtk may be negative
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

    traj_target_vel_ = std::sqrt(
        quad_pos_cmd_.velocity.x * quad_pos_cmd_.velocity.x +
        quad_pos_cmd_.velocity.y * quad_pos_cmd_.velocity.y +
        quad_pos_cmd_.velocity.z * quad_pos_cmd_.velocity.z);

    if (traj_target_vel_ < 0.01) {
        traj_cmd_received_ = false;
    }
}

void PX4CtrlFSM::landCmdCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data && exec_state_ == HOLD || exec_state_ == RL_MOTION || exec_state_ == TRAJ_CMD) {
        std::cout << "[PX4 FSM]: Landing command received. Switching to SOFT_LAND." << std::endl;
        hold_pos_ = pos_;
        hold_yaw_ = att_.z();
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
    // planner normal z(near 0 initially) -> rtk z(may be -10 to 10 m although on the ground)
    traj_target_.position.z = quad_pos_cmd_.position.z + origin_point_.z;

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

void PX4CtrlFSM::fsmGradualTakeoff() {
    // Initialize takeoff variables only once
    static ros::Time takeoff_start_time;

    if (!takeoff_initialized_) {
        hold_pos_ = takeoff_pos_;
        // TODO(zhaohong): if u set pos_ to hold_pos_ here, the x and y element set of takeoff_pos_ will be useless
        //  However, it seems more smart to set x and y element of pos_ to hold_pos_ here since its newer than takeoff_pos_
        hold_pos_.z() = pos_.z();
        takeoff_start_time = ros::Time::now();
        takeoff_initialized_ = true;
        std::cout << "[PX4 FSM]: Takeoff position [" << takeoff_pos_.transpose() << "] initialized." << std::endl;
    }

    hold_pos_.z() += 0.005; // Gradually increase altitude

    if (hold_pos_.z() > takeoff_pos_.z()) {
        hold_pos_.z() = takeoff_pos_.z();
    }

    publishPoseSetpoint(hold_pos_);

    if (isReachedTarget(takeoff_pos_)) {
        std::cout << "\033[1;32m[PX4 FSM]: Takeoff done, holding.\033[0m" << std::endl;
        hold_pos_ = takeoff_pos_;
        takeoff_initialized_ = false; // Reset for next takeoff
        changeFSMState(HOLD);
    }

    // Safety timeout
    if ((ros::Time::now() - takeoff_start_time).toSec() > 30.0) { // 30 seconds timeout
        std::cout << "\033[1;33m[PX4 FSM]: Takeoff timeout. Switching to HOLD at current position.\033[0m" << std::endl;
        hold_pos_ = pos_;
        takeoff_initialized_ = false;
        changeFSMState(HOLD);
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
            init_pos_set_ = false;  // Reset init position for next takeoff
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
    //TODO(zhaohong): more safety checks
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

void PX4CtrlFSM::editModeCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data && !in_edit_mode_) {
        enterEditMode();
    } else if (!msg->data && in_edit_mode_) {
        exitEditMode();
    }
}

void PX4CtrlFSM::enterEditMode() {
    in_edit_mode_ = true;
    hold_pos_ = pos_;
    hold_yaw_ = att_.z();
    changeFSMState(EDIT);
    std::cout << "[PX4 FSM]: Entered EDIT mode. Click on 2D Nav Goal to set landing position." << std::endl;
}

void PX4CtrlFSM::exitEditMode() {
    in_edit_mode_ = false;
    landing_sequence_active_ = true;
    return_traj_sent_ = false;
    landing_sequence_state_ = RETURN_TO_TAKEOFF;
    std::cout << "[PX4 FSM]: Exited EDIT mode. Starting landing sequence." << std::endl;
    changeFSMState(HOLD);
}

void PX4CtrlFSM::navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (in_edit_mode_) {
        landing_target_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        std::cout << "[PX4 FSM]: Landing target set to [" << landing_target_pos_.transpose() << "]" << std::endl;
    }
}

void PX4CtrlFSM::handleEditMode() {
    // TODO(zhaohong): yaw will be set to 0.0, need to change it later
    publishPoseSetpoint(hold_pos_, hold_yaw_);
}

void PX4CtrlFSM::executeAutoLandingSequence() {
    static ros::Time sequence_start_time;

    static int landing_sequence_state_num = 0;
    landing_sequence_state_num++;
    if (landing_sequence_state_num == 300) {  // lower print frequency than FSM state
        landing_sequence_state_num = 0;
        printLandingSequenceState();
    }

    switch (landing_sequence_state_) {
        case RETURN_TO_TAKEOFF:
            if (!return_traj_sent_) {
                geometry_msgs::PoseStamped goal_msg;
                goal_msg.header.stamp = ros::Time::now();
                goal_msg.header.frame_id = "map";
                goal_msg.pose.position.x = takeoff_pos_.x();
                goal_msg.pose.position.y = takeoff_pos_.y();
                goal_msg.pose.position.z = takeoff_pos_.z();
                nav_goal_pub_.publish(goal_msg);
                return_traj_sent_ = true;
                std::cout << "[PX4 FSM]: Published takeoff pose to planner." << std::endl;
            }

            if (isReachedTargetHorizontal(takeoff_pos_) && !final_align_started_) {
                std::cout << "[PX4 FSM]: Close to horizontal takeoff pos, start precision alignment.\033[0m" << std::endl;
                re_takeoff_pos_.z() = hold_pos_.z();  // let the re-takeoff position be the current height
                final_align_started_ = true;
            }

            if (final_align_started_) {
                publishPoseSetpoint(takeoff_pos_);  // Precision alignment
                if (isReachedTarget(takeoff_pos_)) {
                    landing_sequence_state_ = LAND_AT_TAKEOFF;
                    sequence_start_time = ros::Time::now();
                    final_align_started_ = false;
                    std::cout << "[PX4 FSM]: Reached takeoff position, preparing to land." << std::endl;
                }
            }
            break;

        case LAND_AT_TAKEOFF:
            fsmSoftLand();
            if (extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
                landing_sequence_state_ = TAKEOFF_AGAIN;
                sequence_start_time = ros::Time::now();
            }
            break;

        case TAKEOFF_AGAIN:
            re_takeoff_pos_.head(2) = takeoff_pos_.head(2);  // Keep x, y same as takeoff position
            publishPoseSetpoint(re_takeoff_pos_);
            if (isReachedTarget(re_takeoff_pos_)) {
                landing_sequence_state_ = MOVE_TO_TARGET;

                geometry_msgs::PoseStamped goal_msg;
                goal_msg.header.stamp = ros::Time::now();
                goal_msg.header.frame_id = "map";
                goal_msg.pose.position.x = landing_target_pos_.x();
                goal_msg.pose.position.y = landing_target_pos_.y();
                goal_msg.pose.position.z = re_takeoff_pos_.z();
                goal_msg.pose.orientation.w = 1.0;

                nav_goal_pub_.publish(goal_msg);
            }
            break;

        case MOVE_TO_TARGET:
            if (isReachedTargetHorizontal(landing_target_pos_)) {
                landing_sequence_state_ = LAND_AT_TARGET;
            }
            break;

        case LAND_AT_TARGET:
            hold_pos_ = pos_;
            changeFSMState(SOFT_LAND);
            landing_sequence_active_ = false;
            break;
    }
}

void PX4CtrlFSM::rtbCallback(const std_msgs::Bool::ConstPtr &msg) {
    // TODO(zhaohong): undeveloped, do not use it yet
    if (msg->data) {
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = takeoff_pos_.x();
        goal_msg.pose.position.y = takeoff_pos_.y();
        goal_msg.pose.position.z = takeoff_pos_.z();
        nav_goal_pub_.publish(goal_msg);
    }
}

void PX4CtrlFSM::armCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data && exec_state_ == DISARM) {
        // TODO(zhaohong): expand this function, no need to be only in DISARM state
        std::cout << "[PX4 FSM]: Arming command received. Attempting to arm." << std::endl;
        changeFSMState(INIT);
    }
}

void PX4CtrlFSM::heightChangeCallback(const std_msgs::Float32::ConstPtr &msg) {
    if (exec_state_ == HOLD) {
        hold_pos_.z() += msg->data;
        std::cout << "[PX4 FSM] Height changed to: " << hold_pos_.z() << std::endl;
        // TODO: publish the new z height to the planner
        abs_height_msg_.data = hold_pos_.z() - origin_point_.z;
        abs_height_pub_.publish(abs_height_msg_);
    } else {
        ROS_WARN("[PX4 FSM] Height can only be changed in HOLD mode. Current mode: %s", state_str_[exec_state_].c_str());
    }
}

void PX4CtrlFSM::printLandingSequenceState() {
    std::cout << "\033[1;36m[PX4 Landing Sequence]: "
              << landing_state_str_[landing_sequence_state_] << "\033[0m" << std::endl;
}

bool PX4CtrlFSM::isReachedTargetHorizontal(const Eigen::Vector3d &target) const {
    return (pos_.head(2) - target.head(2)).norm() <= target_thresh_;
}