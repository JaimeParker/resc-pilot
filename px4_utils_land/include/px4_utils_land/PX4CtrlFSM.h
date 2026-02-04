//
// Created by Zhaohong Liu on 24-11-7.
//

#ifndef PX4_UTILS_LAND_PX4CTRLFSM_H
#define PX4_UTILS_LAND_PX4CTRLFSM_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <random>
#include <deque>

#include "px4_utils_land/Convertor.h"
#include "px4_utils_land/ImgMatching.h"
#include "px4_utils_land/ImgYoloDetect.h"

#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointClear.h>

class PX4CtrlFSM {
private:

    // static std::mt19937 random_engine_;
    // static std::uniform_real_distribution<double> rtk_uniform_dist_; 

    /* flags */
    enum FSM_EXEC_STATE { INIT, ARM, OFFBOARD, TAKEOFF, HOLD, RL_MOTION, TRAJ_CMD, AUTO_MISSION, SOFT_LAND, AUTO_LAND, DISARM, LANDED, EDIT, RTL};
    std::string state_str_[14] = {"INIT", "ARM", "OFFBOARD", "TAKEOFF", "HOLD", "RL_MOTION", "TRAJ_CMD", "AUTO_MISSION", "SOFT_LAND", "AUTO_LAND", "DISARM", "LANDED", "EDIT", "RTL"};

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
    ros::Subscriber global_position_sub_;
    ros::Subscriber global_setpoint_sub_;
    ros::Subscriber lidar_sub_;  // YDLidar subscriber
    ros::Publisher pose_setpoint_pub_;
    ros::Publisher att_target_pub_;
    ros::Publisher traj_target_pub_;
    ros::Publisher global_setpoint_pub_;
    ros::Rate rate_ = ros::Rate(20);
    ros::Time last_traj_cmd_time_;
    ros::ServiceClient wp_client_;
    ros::ServiceClient wp_clear_client_;

    /* px4 mavros messages */
    mavros_msgs::State state_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::SetMode offb_mode_setter_;
    mavros_msgs::AttitudeTarget att_target_;
    geometry_msgs::PoseStamped pose_setpoint_;
    quadrotor_msgs::PositionCommand quad_pos_cmd_;
    mavros_msgs::PositionTarget traj_target_;
    mavros_msgs::ExtendedState extended_state_;
    sensor_msgs::NavSatFix current_global_position_;
    mavros_msgs::GlobalPositionTarget target_global_position_;
    bool global_position_received_ = false;
    bool global_setpoint_received_ = false;
    // Optional initial global setpoint configuration (from launch/param)
    bool publish_global_setpoint_ = false;
    double global_setpoint_lat_ = 0.0;
    double global_setpoint_lon_ = 0.0;
    double global_setpoint_alt_ = 0.0;

    double global_initial_lat_ = 0.0;
    double global_initial_lon_ = 0.0;
    double global_initial_alt_ = 0.0;

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

    /* auto mission */
    bool enable_auto_mission_ = false;
    Eigen::Vector3d auto_mission_target_;
    bool auto_mission_started_ = false;

    /* auto rtl */
    double auto_takeoff_alt_ = 2.2;
    bool enable_auto_rtl_ = false;
    int landed_wait_time_ = 0;
    bool auto_rtl_ = false;

    /* params */
    double target_thresh_ = 0.50;
    double exec_period_ = 0.01;
    double cruise_height_ = 1.0;
    double fence_x_ = 10.0;
    double fence_y_ = 10.0;
    double fence_z_ = 2.0;
    double kp_ = 0.002;  
    double ki_ = 0.0001; 
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
    std::string rtl_topic_ = "/mavros/mission/push";
    std::string clear_mission_topic_ = "/mavros/mission/clear";
    double auto_mission_target_x_ = 0.0;
    double auto_mission_target_y_ = 0.0;
    double auto_mission_target_z_ = 1.0;

    /* lidar data */
    sensor_msgs::LaserScan::ConstPtr latest_lidar_scan_;
    ros::Time last_lidar_time_;
    double ranges_from_ydlidar = 0.0;  // Latest minimum range from lidar
    double range_threshold_ = 2.0;  // New parameter for range threshold

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

    // Z stability tracking for reliable landing detection
    std::deque<std::pair<ros::Time, double>> z_history_;
    double z_stationary_window_sec_ = 1.0;     // duration to consider z as unchanged
    double z_stationary_epsilon_ = 0.01;       // 1 cm tolerance
    size_t z_history_max_len_ = 500;           // cap history to ~5s at 100Hz
    void updateZHistory(double z, const ros::Time &now);
    bool hasLandedFromZHistory(double window_sec, double epsilon) const;

    enum LandingSequenceState {
        RETURN_TO_TAKEOFF,
        LAND_AT_TAKEOFF,
        TAKEOFF_AGAIN,
        MOVE_TO_TARGET,
        LAND_AT_TARGET
    };
    std::string landing_state_str_[5] = {"RETURN_TO_TAKEOFF", "LAND_AT_TAKEOFF", "TAKEOFF_AGAIN", "MOVE_TO_TARGET", "LAND_AT_TARGET"};
    LandingSequenceState landing_sequence_state_ = RETURN_TO_TAKEOFF;

    /* to init Imgyolodetect */
    ros::NodeHandle nh_;  
    std::unique_ptr<px4_utils_land::Imgyolodetect> image_matcher_; 

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
    void fsmVisionLand();
    void fsmGradualTakeoff();
    bool triggerPX4AutoLand();
    bool triggerPX4Disarm();
    void landCmdCallback(const std_msgs::Bool::ConstPtr &msg);
    void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg);
    void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void globalSetpointCallback(const mavros_msgs::GlobalPositionTarget::ConstPtr &msg);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg);  // YDLidar callback

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
    double calculateGPSDistance(const sensor_msgs::NavSatFix& pos1, const mavros_msgs::GlobalPositionTarget& pos2);
    Eigen::Vector2d calculateGPSVector(const sensor_msgs::NavSatFix& current, const mavros_msgs::GlobalPositionTarget& target);
    void handlePrecisionPositioning(double mission_distance, int fsm_num);
    

    template<typename T>
    void getParamWithWarning(ros::NodeHandle& nh, const std::string& param_name, T& param) {
        if (!nh.getParam(param_name, param)) {
            ROS_WARN_STREAM("Failed to get param: " << param_name);
        }
    }

    bool triggerPX4AutoRTL();
    bool triggerPX4AutoTAKEOFF();
    bool RTLSetLandingPoint();
    bool checkHeightForTargetReached();
};


#endif //PX4_UTILS_LAND_PX4CTRLFSM_H
