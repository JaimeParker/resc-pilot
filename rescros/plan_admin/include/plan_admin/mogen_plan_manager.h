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
 * Created by Zhaohong Liu on 24-10-15.
 * Ref: https://github.com/HKUST-Aerial-Robotics/Fast-Planner/tree/master/fast_planner/plan_manage
*/

#ifndef PLAN_ADMIN_MOGEN_PLAN_MANAGER_H
#define PLAN_ADMIN_MOGEN_PLAN_MANAGER_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>

#include "ctrl_point_gen.h"
#include "motion_gen.h"
#include "k_gpep.h"
#include "frontend_searcher/PathSearch.h"
#include "map_utils/MapBridge.h"
#include "rviz_utils/MarkerHandler.h"
#include "rviz_utils/PoseVisualization.h"
#include "rviz_utils/CtrlPointVisualization.h"
#include "rviz_utils/TrajMarkerHandler.h"

class MogenPlanManager {
private:
    /* mogen utils */
    CtrlPointGen::Ptr ctrl_point_gen_ptr_;
    MotionGen::Ptr rl_motion_gen_ptr_;
    MapBridge::Ptr map_bridge_ptr_;
    PathSearch::Ptr path_search_ptr_;

    /* rviz utils */
    std::unique_ptr<MarkerHandler> pose_marker_handler_;
    std::unique_ptr<MarkerHandler> ctrl_pt_marker_handler_;
    std::unique_ptr<MarkerHandler> traj_marker_handler_;

    /* planning data */
    bool allow_rl_cmd_ = false;
    bool path_valid_ = false;
    std::string frame_id_ = "world";
    bool need_pid_ctrl_ = false;
    Eigen::Vector3d goal_;

    /* ros utils */
    ros::Timer rl_cmd_pub_timer_;
    ros::Timer search_timer_;
    ros::Timer use_rl_timer_;
    ros::Publisher use_rl_pub_;
    std_msgs::Bool use_rl_msg_;
    ros::Timer global_pcl_timer_;
    ros::Timer local_pcl_timer_;
    ros::Timer rviz_pose_timer_;
    ros::Timer pid_ctrl_timer_;
    mavros_msgs::PositionTarget pid_ctrl_msg_;
    ros::Publisher pid_ctrl_pub_;
    std_msgs::Bool use_pid_msg_;
    ros::Publisher use_pid_pub_;

    /* params */
    double search_period_ = 0.1;
    double rl_cmd_pub_period_ = 0.02;
    double use_rl_period_ = 0.01;
    std::string use_rl_topic_ = "/use_rl";
    double global_pcl_period_ = 0.5;
    double local_pcl_period_ = 0.1;
    double reach_goal_thresh_ = 0.5;
    double rviz_pose_period_ = 0.02;
    double pid_ctrl_period_ = 0.02;
    std::string pid_pos_ctrl_topic_ = "/fake_position_target";
    std::string use_pid_topic_ = "/use_pid";
    double resolution_ = 0.1;
    double search_radius_ = 10.0;
    double collide_warn_dist_ = 0.29;

    /* RL env params */
    double rl_yaw_init_thresh_ = M_PI / 4;
public:
    using Ptr = std::unique_ptr<MogenPlanManager>;
public:
    void initPlannerModules(ros::NodeHandle &nh);
    void searchCallback(const ros::TimerEvent& /* event */);
    void rlCmdPubCallback(const ros::TimerEvent& /* event */);
    void useRLCallback(const ros::TimerEvent& /* event */);
    void changeRLCmdState(bool allow_rl_cmd);
    void changePathValidState(bool path_valid);
    [[nodiscard]] bool isCollide(const Eigen::Vector3d & pos) const;
    void globalPclCallback(const ros::TimerEvent& /* event */);
    void localPclCallback(const ros::TimerEvent& /* event */);
    bool pathGenerate(const Eigen::Vector3d & start_pos, const Eigen::Vector3d & end_pos, bool is_replan = false);
    void rvizCallback(const ros::TimerEvent& /* event */);
    bool yawPlanning(const double &yaw, const Eigen::Vector3d & pos, const Eigen::Vector3d & vel);
    static void calcNextYaw(const double& last_yaw, double& yaw);
    void pidCtrlCallback(const ros::TimerEvent& /* event */);
    bool callCtrlPtCollisionCheck(const Eigen::Vector3d & pos_curr);
    void setGoal(Eigen::Vector3d & target_pos) { goal_ = target_pos; }
    [[nodiscard]] Eigen::Vector3d getReplanEnd(Eigen::Vector3d & drone_pos) const;
    Eigen::Vector3d findSafeReplanTarget(const Eigen::Vector3d &start, 
                                         const Eigen::Vector3d &final_goal, 
                                         double max_dist) const;
    Eigen::Vector3d findAnySafePointInRadius(const Eigen::Vector3d &center, 
                                             double radius) const;
    Eigen::Vector3d findSafeTargetAtDistance(const Eigen::Vector3d &start, 
                                             const Eigen::Vector3d &goal, 
                                             double target_dist) const;

    template<typename T>
    void getParamWithWarning(ros::NodeHandle& nh, const std::string& param_name, T& param) {
        if (!nh.getParam(param_name, param)) {
            ROS_WARN_STREAM("Failed to get param: " << param_name);
        }
    }
};


#endif //PLAN_ADMIN_MOGEN_PLAN_MANAGER_H
