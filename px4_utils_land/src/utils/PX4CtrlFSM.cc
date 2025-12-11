//
// Created by Zhaohong Liu on 24-11-7.
//

#include "px4_utils_land/PX4CtrlFSM.h"
#include <cmath>
#include <limits>



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

    getParamWithWarning(nh, "px4fsm/kp", kp_);
    getParamWithWarning(nh, "px4fsm/ki", ki_);

    getParamWithWarning(nh, "px4fsm/use_rl_topic", use_rl_topic_);
    getParamWithWarning(nh, "px4fsm/rl_cmd_topic", rl_cmd_topic_);
    getParamWithWarning(nh, "px4fsm/land_cmd_topic", land_cmd_topic_);

    getParamWithWarning(nh, "px4fsm/enable_auto_mission", enable_auto_mission_);
    
    // If auto mission is enabled, wait for preprocessing node to complete
    if (enable_auto_mission_) {
        ros::NodeHandle gnh; // global namespace
        ros::Rate wait_rate(2);
        int waited = 0;
        const int max_wait_sec = 60; // wait up to 60 seconds
        bool ready = false;
        
        while (ros::ok() && waited < max_wait_sec) {
            if (gnh.hasParam("/global_gps/ready")) {
                gnh.getParam("/global_gps/ready", ready);
                if (ready) {
                    break;
                }
            }
            wait_rate.sleep();
            waited += 0.5;
            
            if (waited % 10 == 0) { // Print every 10 seconds
                ROS_INFO("[PX4 FSM]: Still waiting... (%d/%d seconds)", (int)waited, max_wait_sec);
            }
        }
        
        if (!ready) {
            ROS_WARN("[PX4 FSM]: GPS preprocessing timeout after %d seconds. Using fallback values.", max_wait_sec);
        }
    }
    
    double yaml_lat, yaml_lon, yaml_alt;
    double yaml_target_x, yaml_target_y, yaml_target_z;
    
    bool has_yaml_gps = nh.getParam("/global_gps/target/latitude", yaml_lat) &&
                        nh.getParam("/global_gps/target/longitude", yaml_lon) &&
                        nh.getParam("/global_gps/target/altitude", yaml_alt);
    
    bool has_yaml_enu = nh.getParam("/global_gps/enu_relative/east", yaml_target_x) &&
                        nh.getParam("/global_gps/enu_relative/north", yaml_target_y) &&
                        nh.getParam("/global_gps/enu_relative/up", yaml_target_z);
    
    if (has_yaml_gps) {
        global_setpoint_lat_ = yaml_lat;
        global_setpoint_lon_ = yaml_lon;
        global_setpoint_alt_ = yaml_alt;
        publish_global_setpoint_ = true;
    } else {
        // Fall back to px4fsm namespace parameters
        getParamWithWarning(nh, "px4fsm/global_setpoint_lat", global_setpoint_lat_);
        getParamWithWarning(nh, "px4fsm/global_setpoint_lon", global_setpoint_lon_);
        getParamWithWarning(nh, "px4fsm/global_setpoint_alt", global_setpoint_alt_);
    }
    
    if (has_yaml_enu) {
        auto_mission_target_x_ = yaml_target_x;
        auto_mission_target_y_ = yaml_target_y;
        auto_mission_target_z_ = yaml_target_z;
        ROS_INFO("[PX4 FSM]: Using ENU target: x=%.3f, y=%.3f, z=%.3f", 
                 yaml_target_x, yaml_target_y, yaml_target_z);
    } else {
        // Fall back to px4fsm namespace parameters
        getParamWithWarning(nh, "px4fsm/auto_mission_target_x", auto_mission_target_x_);
        getParamWithWarning(nh, "px4fsm/auto_mission_target_y", auto_mission_target_y_);
        getParamWithWarning(nh, "px4fsm/auto_mission_target_z", auto_mission_target_z_);
        ROS_INFO("[PX4 FSM]: Using ENU fallback: x=%.3f, y=%.3f, z=%.3f", 
                 auto_mission_target_x_, auto_mission_target_y_, auto_mission_target_z_);
    }
    
    getParamWithWarning(nh, "px4fsm/publish_global_setpoint", publish_global_setpoint_);
    
    // Set auto mission target
    auto_mission_target_ << auto_mission_target_x_, auto_mission_target_y_, auto_mission_target_z_;

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
    global_position_sub_ = nh.subscribe("/mavros/global_position/global", 10, &PX4CtrlFSM::globalPositionCallback, this);
    global_setpoint_sub_ = nh.subscribe("/mavros/setpoint_raw/global", 10, &PX4CtrlFSM::globalSetpointCallback, this);
    lidar_sub_ = nh.subscribe("/scan", 10, &PX4CtrlFSM::lidarCallback, this);  // Subscribe to YDLidar

    // pose setpoint is high level, while traj target is mid level
    pose_setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>(pose_setpoint_topic_, 1);
    traj_target_pub_ = nh.advertise<mavros_msgs::PositionTarget>(traj_target_topic_, 1);
    att_target_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(att_target_topic_, 1);
    global_setpoint_pub_ = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 1);

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

    // for convenient operation on drone pos
    pos_change_sub_ = nh.subscribe("/position_change", 10, &PX4CtrlFSM::positionChangeCallback, this);
    yaw_change_sub_ = nh.subscribe("/yaw_change", 10, &PX4CtrlFSM::yawChangeCallback, this);

    refined_goal_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/refined_goal_marker", 1);

    //zhiyuan:get Imgmatching
    nh_ = nh;
    initGoalMarker();


    if (publish_global_setpoint_) {
        mavros_msgs::GlobalPositionTarget gpt;
        gpt.header.stamp = ros::Time::now();
        gpt.latitude = global_setpoint_lat_;
        gpt.longitude = global_setpoint_lon_;
        gpt.altitude = global_setpoint_alt_;
        // set type_mask to indicate only position is used (no velocity/acceleration)
        gpt.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX | mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                        mavros_msgs::GlobalPositionTarget::IGNORE_VZ | mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                        mavros_msgs::GlobalPositionTarget::IGNORE_AFY | mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                        mavros_msgs::GlobalPositionTarget::IGNORE_YAW | mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;

        global_setpoint_pub_.publish(gpt);
        // ROS_INFO_STREAM("[PX4 FSM]: Published initial global setpoint: lat=" << global_setpoint_lat_
        //                 << " lon=" << global_setpoint_lon_ << " alt=" << global_setpoint_alt_);
    }
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

        case HOLD:{
            Eigen::Vector3d target_pos(quad_pos_cmd_.position.x, quad_pos_cmd_.position.y, quad_pos_cmd_.position.z);
            double distance_to_target = (target_pos - pos_).norm();
            // any state that wants to change to hold must redefine hold_pos_
            if (!auto_mission_started_) {
                geometry_msgs::PoseStamped goal_msg;
                goal_msg.header.stamp = ros::Time::now();
                goal_msg.header.frame_id = "map";
                goal_msg.pose.position.x = auto_mission_target_.x();
                goal_msg.pose.position.y = auto_mission_target_.y();
                goal_msg.pose.position.z = auto_mission_target_.z();
                goal_msg.pose.orientation.w = 1.0;
                nav_goal_pub_.publish(goal_msg);
                auto_mission_started_ = true;
                std::cout << "[PX4 FSM]: Published auto-mission target to ego-planner: [" 
                            << auto_mission_target_.x() << ", " << auto_mission_target_.y() 
                            << ", " << auto_mission_target_.z() << "]" << std::endl;
            }
            
            if (traj_cmd_received_ && in_geo_fence_ && (distance_to_target > target_thresh_) && auto_mission_started_) {
                last_traj_cmd_time_ = ros::Time::now();
                changeFSMState(TRAJ_CMD);
            } else if (!auto_mission_started_ && enable_auto_mission_) {
                std::cout << "\033[1;32m[PX4 FSM]: Starting auto mission to target [" 
                          << auto_mission_target_.transpose() << "]\033[0m" << std::endl;
                auto_mission_started_ = true;
                changeFSMState(AUTO_MISSION);
            } else {
                publishPoseSetpoint(hold_pos_, hold_yaw_);
            }
            break;
        }
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
            
        case TRAJ_CMD:{
            if (!in_geo_fence_) {
                std::cout << "\033[1;33m[PX4 FSM]: Out of geo fence! Returning.\033[0m" << std::endl;
                geoFenceClamp(pos_);
                hold_pos_ = pos_;
                hold_yaw_ = att_.z();
                changeFSMState(HOLD);
                break;
            }
            // get the distance to the target
            Eigen::Vector3d mission_target_pos = auto_mission_target_;
            double mission_distance = (mission_target_pos - pos_).norm();
            
            Eigen::Vector3d target_pos(quad_pos_cmd_.position.x, quad_pos_cmd_.position.y, quad_pos_cmd_.position.z);
            double distance_to_target = (target_pos - pos_).norm();
            if (fsm_num % 200 == 0)
                printf("[PX4 FSM]: Distance to mission target: %.2f m, traj distance: %.2f m\n", mission_distance, distance_to_target);
            
            if ((mission_distance > target_thresh_) && traj_cmd_received_ && ros::Time::now() - last_traj_cmd_time_ < ros::Duration(traj_cmd_timeout_)) {
                publishTrajSetpoint();
            } else {
                handlePrecisionPositioning(mission_distance, fsm_num);
            }

            break;
        }

        case AUTO_MISSION:
            if (!in_geo_fence_) {
                std::cout << "\033[1;33m[PX4 FSM]: Out of geo fence! Returning to HOLD.\033[0m" << std::endl;
                geoFenceClamp(pos_);
                hold_pos_ = pos_;
                hold_yaw_ = att_.z();
                changeFSMState(HOLD);
                break;
            }

            // Check if reached target
            if (isReachedTarget(auto_mission_target_)) {
                std::cout << "\033[1;32m[PX4 FSM]: Reached auto mission target, starting landing.\033[0m" << std::endl;
                hold_pos_ = pos_;
                hold_yaw_ = att_.z();
                changeFSMState(SOFT_LAND);
            } else {
                publishPoseSetpoint(auto_mission_target_);
                Eigen::Vector3d distance_vec = auto_mission_target_ - pos_;
                std::cout << "[PX4 FSM]: Flying to auto mission target, distance: " 
                          << std::fixed << std::setprecision(2) << distance_vec.norm() << " m" << std::endl;
            }
            break;

        case SOFT_LAND:
            //TODO(zhiyuan 7_16):create a state & write the alignment logic
            if (!image_matcher_) {
                image_matcher_ = std::make_unique<px4_utils_land::Imgmatching>();
                image_matcher_->init(nh_);

                // Use latest YDLidar range data to set landing position
                if (init_pos_set_) {
                    // Use current position minus lidar detected ground distance
                    double land_height = pos_.z() - ranges_from_ydlidar;
                    image_matcher_->setLandPos(land_height);
                    ROS_INFO("[PX4 FSM]: Set landing position from YDLidar: %.2f m (current: %.2f m, range: %.2f m)", 
                             land_height, pos_.z(), ranges_from_ydlidar);
                } else {
                    image_matcher_->setLandPos(ground_height_);
                }
            }

            fsmVisionLand();
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

// std::mt19937 PX4CtrlFSM::random_engine_(std::random_device{}());
// std::uniform_real_distribution<double> PX4CtrlFSM::rtk_uniform_dist_(-0.5, 0.5);

void PX4CtrlFSM::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    // pos_.x() += rtk_uniform_dist_(random_engine_);
    // pos_.y() += rtk_uniform_dist_(random_engine_);
    // pos_.z() += rtk_uniform_dist_(random_engine_);

    att_quat_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                   msg->pose.orientation.y, msg->pose.orientation.z);
    Convertor::q2EulerAngle(att_quat_, att_);
    // Update Z history for landing detection based on Z stability
    updateZHistory(pos_.z(), msg->header.stamp);

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
            
            // Set auto mission target in world frame
            auto_mission_target_ = auto_mission_target_ + init_pos_;
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

    // traj_target_vel_ = std::sqrt(
    //     quad_pos_cmd_.velocity.x * quad_pos_cmd_.velocity.x +
    //     quad_pos_cmd_.velocity.y * quad_pos_cmd_.velocity.y +
    //     quad_pos_cmd_.velocity.z * quad_pos_cmd_.velocity.z);

    // if (traj_target_vel_ < 0.01) {
    //     traj_cmd_received_ = false;
    //     ROS_INFO_STREAM("[PX4 FSM] traj_cmd_received_ set to false2");
    // }
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

void PX4CtrlFSM::globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_global_position_ = *msg;
    global_position_received_ = true;
}

void PX4CtrlFSM::globalSetpointCallback(const mavros_msgs::GlobalPositionTarget::ConstPtr &msg) {
    target_global_position_ = *msg;
    global_setpoint_received_ = true;
}

void PX4CtrlFSM::lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // Store the latest lidar data
    latest_lidar_scan_ = msg;
    last_lidar_time_ = ros::Time::now();
    
    // Get the latest range value directly from ranges array
    ranges_from_ydlidar = msg->ranges.back();  // Get the last element as latest value
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

void PX4CtrlFSM::fsmVisionLand() {
    static bool land_initialized = false;
    static ros::Time land_start_time;
    static int land_num = 0;
    land_num++;
    if (!land_initialized) {
        hold_pos_ = pos_;  
        land_start_time = ros::Time::now();
        land_initialized = true;
        std::cout << "[PX4 FSM]: HSV beacon landing initialized at [" 
                  << hold_pos_.x() << ", " << hold_pos_.y() << ", " << hold_pos_.z() << "]" << std::endl;
    }

    image_matcher_->setHoldPos(hold_pos_.z()); 

    if (image_matcher_ && image_matcher_->isTargetMatched()) {
        hold_pos_ = adjustPositionWithPIControl(image_matcher_->getOffset());
        
        hold_pos_.z() -= 0.001;
        
        cv::Point2f offset = image_matcher_->getOffset();
        if (land_num % 99 == 0)
            std::cout << "[PX4 FSM]: Beacon detected, offset: (" 
                    << std::fixed << std::setprecision(3) << offset.x << ", " << offset.y 
                    << ") m, height: " << hold_pos_.z() << " m" << std::endl;
    } else {
        hold_pos_.z() -= 0.001;
        
        static int no_beacon_count = 0;
        if (++no_beacon_count % 100 == 0) {
            std::cout << "[PX4 FSM]: No beacon detected, height: " 
                      << std::fixed << std::setprecision(3) << hold_pos_.z() << " m" << std::endl;
        }
    }

 
    publishPoseSetpoint(hold_pos_);

    if (extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
        std::cout << "[PX4 FSM]: Landed detected by PX4. Switching to LANDED." << std::endl;
        land_initialized = false;
        if (image_matcher_) {
            image_matcher_->disableMatching();
            image_matcher_.reset();  
        }
        changeFSMState(LANDED);
        return;
    }

    if (hasLandedFromZHistory(z_stationary_window_sec_, z_stationary_epsilon_)) {
        std::cout << "[PX4 FSM]: Z stable for " << z_stationary_window_sec_ 
                  << "s (|dz|<" << z_stationary_epsilon_ 
                  << ") -> Switching to AUTO.LAND." << std::endl;

        land_initialized = false;
        if (image_matcher_) {
            image_matcher_->disableMatching();
            image_matcher_.reset();  
        }
        changeFSMState(AUTO_LAND);
        return;
    }

    // Switch to AUTO.LAND only when Z is steady for a period (touchdown),
    // avoids yaw jump while still in the air due to estimator noise.

    if ((hold_pos_.z() - image_matcher_->land_pos_z_) < 1e-6) {
        std::cout << "[PX4 FSM]: Vision becomes blurry. Switching to AUTO.LAND." << std::endl;
        std::cout << "[PX4 FSM]: Z stable for " << z_stationary_window_sec_ 
                  << "s (|dz|<" << z_stationary_epsilon_ 
                  << ") -> Switching to AUTO.LAND." << std::endl;

        land_initialized = false;
        if (image_matcher_) {
            image_matcher_->disableMatching();
            image_matcher_.reset();  // 安全地销毁对象
        }
        changeFSMState(AUTO_LAND);
        return;
    }
}

// Z stability helpers
void PX4CtrlFSM::updateZHistory(double z, const ros::Time &now) {
    z_history_.emplace_back(now, z);

    // Keep history covering a bit more than the window and cap size
    const double keep_sec = std::max(2.5 * z_stationary_window_sec_, 2.0);
    while (!z_history_.empty() && (now - z_history_.front().first).toSec() > keep_sec) {
        z_history_.pop_front();
    }
    while (z_history_.size() > z_history_max_len_) {
        z_history_.pop_front();
    }
}

bool PX4CtrlFSM::hasLandedFromZHistory(double window_sec, double epsilon) const {
    if (z_history_.empty()) return false;
    const ros::Time now = z_history_.back().first;
    const double z_latest = z_history_.back().second;

    ros::Time oldest_in_window = now;
    for (auto it = z_history_.rbegin(); it != z_history_.rend(); ++it) {
        const double dt = (now - it->first).toSec();
        if (dt > window_sec) break;
        if (std::fabs(it->second - z_latest) > epsilon) {
            return false; // not stable
        }
        oldest_in_window = it->first;
    }
    return (now - oldest_in_window).toSec() >= window_sec;
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

        refined_goal_pos_.pose.position.x = landing_target_pos_.x();
        refined_goal_pos_.pose.position.y = landing_target_pos_.y();
        refined_goal_pos_.pose.position.z = pos_.z();

        publishRefinedGoalMarker();
    }
}

void PX4CtrlFSM::handleEditMode() {
    // TODO(zhaohong): yaw will be set to 0.0, need to change it later
    publishPoseSetpoint(hold_pos_, hold_yaw_);

    publishRefinedGoalMarker();
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
                // do not set z in case large height diff (cause a star search failure)
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
                    landing_sequence_state_ = LAND_AT_TAKEOFF;  // change to TAKEOFF_AGAIN if compass fails
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
            hold_pos_ = landing_target_pos_;
            hold_pos_.z() = pos_.z();  // Lock z to current height
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
    } else if (exec_state_ == EDIT) {
        // In EDIT mode, we can fine-tune the refined_goal_pos_ height
        //  However, this height is only for visualization, and the landin
        refined_goal_pos_.pose.position.z += msg->data;
        // nav_goal_pub_.publish(refined_goal_pos_);

        publishRefinedGoalMarker();
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

void PX4CtrlFSM::positionChangeCallback(const geometry_msgs::Point::ConstPtr &msg) {
    if (exec_state_ == HOLD) {
        double current_yaw = att_.z();
        
        // Transform body frame commands to world frame
        // Body frame: x = forward, y = left, z = up
        // World frame: x = east, y = north, z = up
        double cos_yaw = cos(current_yaw);
        double sin_yaw = sin(current_yaw);
        
        Eigen::Vector3d body_cmd(msg->x, msg->y, msg->z);
        Eigen::Vector3d world_cmd;
        
        world_cmd.x() = cos_yaw * body_cmd.x() - sin_yaw * body_cmd.y();
        world_cmd.y() = sin_yaw * body_cmd.x() + cos_yaw * body_cmd.y();
        world_cmd.z() = body_cmd.z(); // Z (up) remains the same
        
        hold_pos_.x() += world_cmd.x();
        hold_pos_.y() += world_cmd.y();
        hold_pos_.z() += world_cmd.z();
        
        geoFenceClamp(hold_pos_);
        
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "[PX4 FSM] Body frame command: [" << msg->x << ", " << msg->y << ", " << msg->z << "]" << std::endl;
        std::cout << "[PX4 FSM] New position: [" << hold_pos_.x() << ", " << hold_pos_.y() << ", " << hold_pos_.z() << "]" << std::endl;
        
        // TODO: uncomment and revise this if height change is also realized in this callback
        //  Currently, height change is only realized in heightChangeCallback

        // Publish updated absolute height
        // abs_height_msg_.data = hold_pos_.z() - origin_point_.z;
        // abs_height_pub_.publish(abs_height_msg_);
    } else if (exec_state_ == EDIT) {
        // In EDIT mode, we can fine-tune the landing position, sharing the same x and y as refined_goal_pos_
        refined_goal_pos_.pose.position.x += msg->x;
        refined_goal_pos_.pose.position.y += msg->y;
        // nav_goal_pub_.publish(refined_goal_pos_);

        landing_target_pos_.x() = refined_goal_pos_.pose.position.x;
        landing_target_pos_.y() = refined_goal_pos_.pose.position.y;

        publishRefinedGoalMarker();
    } else {
        ROS_WARN("[PX4 FSM] Position can only be changed in HOLD mode. Current mode: %s", state_str_[exec_state_].c_str());
    }
}

void PX4CtrlFSM::yawChangeCallback(const std_msgs::Float32::ConstPtr &msg) {
    if (exec_state_ == HOLD) {
        float yaw_change_rad = msg->data * deg2rad_;

        hold_yaw_ = att_.z() + yaw_change_rad;
        
        // Normalize yaw to [-pi, pi] range (might not be necessary if using quaternion)
        while (hold_yaw_ > M_PI) hold_yaw_ -= 2.0 * M_PI;
        while (hold_yaw_ < -M_PI) hold_yaw_ += 2.0 * M_PI;
        
        std::cout << "[PX4 FSM] Yaw changed by " << msg->data << " degrees to " 
                  << hold_yaw_ * 180.0 / M_PI << " degrees" << std::endl;
    } else {
        ROS_WARN("[PX4 FSM] Yaw can only be changed in HOLD mode. Current mode: %s", state_str_[exec_state_].c_str());
    }
}

void PX4CtrlFSM::initGoalMarker() {
    // Initialize refined goal marker
    refined_goal_marker_.header.frame_id = "world";
    refined_goal_marker_.ns = "landing_target";
    refined_goal_marker_.id = 0;
    refined_goal_marker_.type = visualization_msgs::Marker::CYLINDER;
    refined_goal_marker_.action = visualization_msgs::Marker::ADD;
    
    // Set marker scale (cylinder dimensions)
    refined_goal_marker_.scale.x = 0.5;  // diameter
    refined_goal_marker_.scale.y = 0.5;  // diameter
    refined_goal_marker_.scale.z = 0.1;  // height
    
    // Set marker color (bright red for visibility)
    refined_goal_marker_.color.r = 1.0;
    refined_goal_marker_.color.g = 0.0;
    refined_goal_marker_.color.b = 0.0;
    refined_goal_marker_.color.a = 0.8;  // semi-transparent
    
    // Set marker orientation (upright cylinder)
    refined_goal_marker_.pose.orientation.w = 1.0;
    refined_goal_marker_.pose.orientation.x = 0.0;
    refined_goal_marker_.pose.orientation.y = 0.0;
    refined_goal_marker_.pose.orientation.z = 0.0;
}

void PX4CtrlFSM::publishRefinedGoalMarker() {
    if (in_edit_mode_ || exec_state_ == SOFT_LAND) {
        refined_goal_marker_.header.stamp = ros::Time::now();
        refined_goal_marker_.pose.position = refined_goal_pos_.pose.position;
        
        refined_goal_marker_pub_.publish(refined_goal_marker_);
    }
}

Eigen::Vector3d PX4CtrlFSM::adjustPositionWithPIControl(const cv::Point2f& offset) {
    static bool pi_initialized = false;
    static Eigen::Vector2d integral_error(0.0, 0.0);
    static ros::Time last_update_time;
    
    const double max_integral = 1.0; // Anti-windup limit
    const double max_correction = 0.5; // Maximum position correction per cycle
    

    if (!pi_initialized) {
        last_update_time = ros::Time::now();
        integral_error.setZero();
        pi_initialized = true;
        return hold_pos_;
    }
    
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_update_time).toSec();
    last_update_time = current_time;
    
    // Convert pixel offset to normalized error (assuming offset is in pixels)
    // Positive x offset means target is to the right, drone should move right (positive y in body frame)
    // Positive y offset means target is below center, drone should move forward (positive x in body frame)
    Eigen::Vector2d error;
    error.x() = -offset.y; // Forward/backward error (body frame x)
    error.y() = -offset.x;  // Left/right error (body frame y)
    
    
    integral_error += error * dt;
    integral_error.x() = std::max(-max_integral, std::min(max_integral, integral_error.x()));
    integral_error.y() = std::max(-max_integral, std::min(max_integral, integral_error.y()));

    Eigen::Vector2d body_correction = kp_ * error + ki_ * integral_error;

    // Limit correction magnitude
    double correction_magnitude = body_correction.norm();
    if (correction_magnitude > max_correction) {
        body_correction = body_correction * (max_correction / correction_magnitude);
    }
    
    // Transform body frame correction to world frame
    double current_yaw = att_.z();
    double cos_yaw = cos(current_yaw);
    double sin_yaw = sin(current_yaw);
    
    Eigen::Vector2d world_correction;
    world_correction.x() = cos_yaw * body_correction.x() - sin_yaw * body_correction.y();
    world_correction.y() = sin_yaw * body_correction.x() + cos_yaw * body_correction.y();
    
    if (world_correction.x() > 0.1) {
        world_correction.x() = 0.1;
        std::cout << "\033[1;33m[PX4 FSM]: Warning.\033[0m" << std::endl;
    } else if (world_correction.x() < -0.1) {
        world_correction.x() = -0.1;
        std::cout << "\033[1;33m[PX4 FSM]: Warning.\033[0m" << std::endl;
    }

    if (world_correction.y() > 0.1) {
        world_correction.y() = 0.1;
        std::cout << "\033[1;33m[PX4 FSM]: Warning.\033[0m" << std::endl;
    } else if (world_correction.y() < -0.1) {
        world_correction.y() = -0.1;
        std::cout << "\033[1;33m[PX4 FSM]: Warning.\033[0m" << std::endl;
    }

    // TODO(zhaohong): or using pos_ here due to time delay?
    Eigen::Vector3d corrected_pos = hold_pos_ ;
    corrected_pos.x() += world_correction.x();
    corrected_pos.y() += world_correction.y();

    geoFenceClamp(corrected_pos);
    
    return corrected_pos;
}

double PX4CtrlFSM::calculateGPSDistance(const sensor_msgs::NavSatFix& pos1, const mavros_msgs::GlobalPositionTarget& pos2) {
    const double R = 6371000.0; // Earth radius in meters
    
    double lat1_rad = pos1.latitude * M_PI / 180.0;
    double lat2_rad = pos2.latitude * M_PI / 180.0;
    double dlat_rad = (pos2.latitude - pos1.latitude) * M_PI / 180.0;
    double dlon_rad = (pos2.longitude - pos1.longitude) * M_PI / 180.0;
    
    double a = sin(dlat_rad/2) * sin(dlat_rad/2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon_rad/2) * sin(dlon_rad/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return R * c; // Distance in meters
}

Eigen::Vector2d PX4CtrlFSM::calculateGPSVector(const sensor_msgs::NavSatFix& current, const mavros_msgs::GlobalPositionTarget& target) {
    const double R = 6371000.0; // Earth radius in meters
    
    double lat1_rad = current.latitude * M_PI / 180.0;
    double lon1_rad = current.longitude * M_PI / 180.0;
    double lat2_rad = target.latitude * M_PI / 180.0;
    double lon2_rad = target.longitude * M_PI / 180.0;
    
    // Calculate local cartesian coordinates (East-North-Up frame)
    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;
    
    // Convert to meters (approximation for small distances)
    double dx = dlon * R * cos(lat1_rad); // East direction (positive = east)
    double dy = dlat * R;                 // North direction (positive = north)
    
    return Eigen::Vector2d(dx, dy);
}

void PX4CtrlFSM::handlePrecisionPositioning(double mission_distance, int fsm_num) {          // Enter precision mode at target_thresh_
    if (mission_distance < target_thresh_) {
        target_thresh_ = target_thresh_ * 2;
        // Check GPS accuracy for precise positioning
        if (global_position_received_ && global_setpoint_received_) {
            double gps_distance = calculateGPSDistance(current_global_position_, target_global_position_);
            if (fsm_num % 200 == 0)
                std::cout << "[PX4 FSM]: Local distance: " << std::fixed << std::setprecision(2) 
                          << mission_distance << " m, GPS distance: " << gps_distance << " m" << std::endl;
            
            if (gps_distance < 0.1) { // 10cm threshold
                traj_cmd_received_ = false;
                std::cout << "\033[1;32m[PX4 FSM]: GPS position accurate (<10cm), switching to SOFT_LAND.\033[0m" << std::endl;
                changeFSMState(SOFT_LAND);
            } else {
                // Calculate GPS correction vector in local coordinates
                Eigen::Vector2d gps_correction = calculateGPSVector(current_global_position_, target_global_position_);
                
                // Limit correction magnitude for safety
                double max_correction = 0.1; // 10cm max movement per cycle
                if (gps_correction.norm() > max_correction) {
                    gps_correction = gps_correction.normalized() * max_correction;
                }

                if (fsm_num % 100 == 0)
                    std::cout << "[PX4 FSM]: Published GPS setpoint for precision positioning" << std::endl;
                    
                // Apply GPS correction to current position using local coordinates
                Eigen::Vector3d corrected_pos = pos_;
                corrected_pos.x() += gps_correction.x(); // East correction
                corrected_pos.y() += gps_correction.y(); // North correction
                pos_.z() -= 0.01;

                publishPoseSetpoint(corrected_pos, hold_yaw_);

                // Stay in TRAJ_CMD to continue GPS-based positioning
                traj_cmd_received_ = false;
            }
        } else {
            // No GPS data available, fall back to local positioning
            traj_cmd_received_ = false;
            changeFSMState(SOFT_LAND);
        }
    } else {
        // Not in precision mode, wait for planner
        traj_cmd_received_ = false;
        if (fsm_num % 200 == 0) // Reduce log frequency
            ROS_INFO_STREAM("[PX4 FSM] traj_cmd_received_ set to false (not arrived, wait planner)");
    }
}