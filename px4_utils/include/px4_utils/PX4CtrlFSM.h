//
// Created by Zhaohong Liu on 24-11-7.
//

#ifndef PX4_UTILS_PX4CTRLFSM_H
#define PX4_UTILS_PX4CTRLFSM_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ExtendedState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>

#include "px4_utils/Convertor.h"
#include "px4_utils/ImgMatching.h"

class PX4CtrlFSM {
private:
    /* flags */
    enum FSM_EXEC_STATE { INIT, ARM, OFFBOARD, TAKEOFF, HOLD, RL_MOTION, TRAJ_CMD, SOFT_LAND, AUTO_LAND, DISARM, LANDED, EDIT};
    std::string state_str_[12] = {"INIT", "ARM", "OFFBOARD", "TAKEOFF", "HOLD", "RL_MOTION", "TRAJ_CMD", "SOFT_LAND", "AUTO_LAND", "DISARM", "LANDED", "EDIT"};

    /* ros utils */
    ros::Timer exec_timer_;
    ros::Timer safety_timer_;  // undefined
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber use_rl_sub_;
    ros::Subscriber rl_cmd_sub_;
    ros::Subscriber traj_cmd_sub_;
    ros::Subscriber land_cmd_sub_;
    ros::Subscriber auto_land_cmd_sub_;
    ros::Subscriber extended_state_sub_;
    ros::Publisher pose_setpoint_pub_;
    ros::Publisher att_target_pub_;
    ros::Publisher traj_target_pub_;
    ros::Rate rate_ = ros::Rate(20);
    ros::Time last_traj_cmd_time_;

    /* px4 mavros messages */
    mavros_msgs::State state_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::SetMode offb_mode_setter_;
    mavros_msgs::AttitudeTarget att_target_;
    geometry_msgs::PoseStamped pose_setpoint_;
    quadrotor_msgs::PositionCommand quad_pos_cmd_;
    mavros_msgs::PositionTarget traj_target_;
    mavros_msgs::ExtendedState extended_state_;

    /* state */
    FSM_EXEC_STATE exec_state_ = INIT;
    Eigen::Vector3d init_pos_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d att_;
    Eigen::Quaterniond att_quat_;
    Eigen::Vector3d takeoff_pos_;
    Eigen::Vector3d hold_pos_;
    float hold_yaw_ = 0.0;
    bool use_rl_ = false;
    bool rl_cmd_received_ = false;
    bool traj_cmd_received_ = false;
    bool motion_smooth_ = true;
    bool in_geo_fence_ = false;
    bool init_pos_set_ = false;
    ros::Time last_request_time_;

    /* params */
    double target_thresh_ = 0.25;
    double exec_period_ = 0.01;
    double cruise_height_ = 1.0;
    double fence_x_ = 10.0;
    double fence_y_ = 10.0;
    double fence_z_ = 2.0;
    double ground_height_ = 0.0;
    double fence_offset_ = 0.5;
    double max_attitude_ = 37.5 * deg2rad_;
    const double waiting_time_ = 5.0;
    const float throttle_default_ = 0.5;
    const double traj_cmd_timeout_ = 0.5;
    const double deg2rad_ = M_PI / 180;
    const double soft_landing_timeout_ = 5.0;
    std::string arming_topic_ = "/mavros/cmd/arming";
    std::string set_mode_topic_ = "/mavros/set_mode";
    std::string state_topic_ = "/mavros/state";
    std::string pose_topic_ = "/mavros/local_position/pose";
    std::string use_rl_topic_ = "/use_rl";
    std::string rl_cmd_topic_ = "/rl_att_cmd";
    std::string traj_cmd_topic_ = "/planning/pos_cmd";
    std::string traj_target_topic_ = "/mavros/setpoint_raw/local";
    std::string pose_setpoint_topic_ = "/mavros/setpoint_position/local";
    std::string att_target_topic_ = "/mavros/setpoint_raw/attitude";
    std::string land_cmd_topic_ = "/trigger_landing";
    std::string extended_state_topic_ = "/mavros/extended_state";

    /* utils */
    std::vector<Eigen::Vector3d> init_pos_buffer_;
    int init_pos_buffer_max_size_ = 100;

    /* for return-landing-takeoff-traj-landing process */
    Eigen::Vector3d landing_target_pos_;
    Eigen::Vector3d re_takeoff_pos_;
    geometry_msgs::Point origin_point_;
    std_msgs::Float32 abs_height_msg_;
    geometry_msgs::PoseStamped refined_goal_pos_;
    visualization_msgs::Marker refined_goal_marker_; 

    bool in_edit_mode_ = false;
    bool landing_sequence_active_ = false;
    bool return_traj_sent_ = false;
    bool final_align_started_ = false;
    bool takeoff_initialized_ = false;
    bool origin_pos_initialized_ = false;
    ros::Subscriber nav_goal_sub_, edit_mode_sub_;
    ros::Publisher nav_goal_pub_;
    ros::Subscriber rtb_sub_;
    ros::Subscriber arm_sub_;
    ros::Subscriber hold_sub_;  // TODO: not used yet, for holding position
    ros::Subscriber height_change_sub_;
    ros::Publisher origin_pos_pub_;
    ros::Publisher abs_height_pub_;
    ros::Subscriber pos_change_sub_;
    ros::Subscriber yaw_change_sub_;
    ros::Publisher refined_goal_marker_pub_;
    
    double traj_target_vel_ = 0.0;

    enum LandingSequenceState {
        RETURN_TO_TAKEOFF,
        LAND_AT_TAKEOFF,
        TAKEOFF_AGAIN,
        MOVE_TO_TARGET,
        LAND_AT_TARGET
    };
    std::string landing_state_str_[5] = {"RETURN_TO_TAKEOFF", "LAND_AT_TAKEOFF", "TAKEOFF_AGAIN", "MOVE_TO_TARGET", "LAND_AT_TARGET"};
    LandingSequenceState landing_sequence_state_ = RETURN_TO_TAKEOFF;

    /* to init Imgmatching */
    std::string downward_camera_topic_ = "/camera/rgb/image_raw";
    ros::NodeHandle nh_;  
    std::unique_ptr<px4_utils::Imgmatching> image_matcher_; 

public:
    void init(ros::NodeHandle &nh);
    void execCallback(const ros::TimerEvent& /* event */);
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void useRLCallback(const std_msgs::Bool::ConstPtr &msg);
    void rlCmdCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg);
    void trajCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
    void changeFSMState(FSM_EXEC_STATE new_state);
    void printFSMExecState();
    [[nodiscard]] bool isReachedTarget(const Eigen::Vector3d &target) const;
    [[nodiscard]] bool isReachedTargetHorizontal(const Eigen::Vector3d &target) const;
    void publishPoseSetpoint(const Eigen::Vector3d &pos, const double & yaw = 0);
    void publishTrajSetpoint();
    void checkAggressiveMotion();
    void geoFenceClamp(const Eigen::Vector3d & pos);
    void fsmSoftLand();
    void fsmGradualTakeoff();
    bool triggerPX4AutoLand();
    bool triggerPX4Disarm();
    void landCmdCallback(const std_msgs::Bool::ConstPtr &msg);
    void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg);

    void enterEditMode();
    void exitEditMode();
    void handleEditMode();
    void executeAutoLandingSequence();
    void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void editModeCallback(const std_msgs::Bool::ConstPtr &msg);
    void rtbCallback(const std_msgs::Bool::ConstPtr &msg);
    void armCallback(const std_msgs::Bool::ConstPtr &msg);
    void printLandingSequenceState();
    void heightChangeCallback(const std_msgs::Float32::ConstPtr &msg);
    void positionChangeCallback(const geometry_msgs::Point::ConstPtr &msg);
    void yawChangeCallback(const std_msgs::Float32::ConstPtr &msg);
    void initGoalMarker();
    void publishRefinedGoalMarker();
    Eigen::Vector3d adjustPositionWithPIControl(const cv::Point2f& offset);
    
    //zhiyuan:z (actual height) and f (focal length)
    //TODO: go to PX4CtrlFSM.cc and set these values
    static float z_;
    static float fx_;
    static float fy_;

    template<typename T>
    void getParamWithWarning(ros::NodeHandle& nh, const std::string& param_name, T& param) {
        if (!nh.getParam(param_name, param)) {
            ROS_WARN_STREAM("Failed to get param: " << param_name);
        }
    }
};


#endif //PX4_UTILS_PX4CTRLFSM_H
