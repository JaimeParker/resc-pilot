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

#ifndef PLAN_ADMIN_MOGEN_REPLAN_FSM_H
#define PLAN_ADMIN_MOGEN_REPLAN_FSM_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Eigen>

#include "plan_admin/mogen_plan_manager.h"

class MogenReplanFSM {
private:
    /* flags */
    enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_PATH, EXEC_MOTION, PID_ASSIST, REPLAN};
    std::string state_str_[6] = {"INIT", "WAIT_TARGET", "GEN_PATH", "EXEC_MOTION", "PID_ASSIST", "REPLAN"};
    enum MSG_COLOR {RED, GREEN, YELLOW, BLUE, WHITE};

    /* planning utils */
    MogenPlanManager::Ptr planner_manager_;

    /* planning data */
    FSM_EXEC_STATE exec_state_;
    bool has_target_, has_pose_;
    Eigen::Vector3d drone_pos_, drone_att_, drone_vel_;
    Eigen::Vector3d start_pos_, end_pos_;
    double cruising_altitude_ = 1.0;

    /* ros utils */
    ros::Timer exec_timer_;
    ros::Timer safety_timer_;
    ros::Subscriber drone_pose_sub_;
    ros::Subscriber drone_vel_sub_;
    ros::Subscriber goal_sub_;

    /* parameters */
    double exec_period_ = 0.01;
    double safety_period_ = 0.05;
    std::string drone_pose_topic_ = "/mavros/local_position/pose";
    std::string drone_vel_topic_ = "/mavros/local_position/velocity_local";
    std::string nav_goal_topic_ = "/move_base_simple/goal";
    double finish_thresh_ = 0.5;
    double terrain_height_ = 0.2;
    double max_down_vel_ = -1.0;

    /* hint */
    std::string fsm_init_hint_ = "[FSM]: FSM ready.";
    std::string goal_received_hint_ = "[FSM]: Target position received.";
    std::string goal_reached_hint_ = "[FSM]: Mission completed, standby for next target.";
    std::string path_found_hint_ = "[FSM]: Path found! Executing motion.";
    std::string no_path_hint_ = "[FSM]: No path.";
    std::string terrain_hint_ = "[FSM]: Terrain, PULL UP!";
    std::string vel_down_hint_ = "[FSM]: Warning! Max descend velocity! PULL UP!";

public:
    void init(ros::NodeHandle &nh);
    void changeFSMState(FSM_EXEC_STATE new_state);
    void printFSMExecState();
    void execCallback(const ros::TimerEvent& /* event */);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void checkSafetyCallback(const ros::TimerEvent& /* event */);
    bool callPathGenerate(const Eigen::Vector3d & start, const Eigen::Vector3d & end);
    static void printHintMsg(std::string &msg, MSG_COLOR color);
    bool isGoalReached();
    static void q2EulerAngle(const Eigen::Quaterniond& q, Eigen::Vector3d & euler);
};


#endif //PLAN_ADMIN_MOGEN_REPLAN_FSM_H
