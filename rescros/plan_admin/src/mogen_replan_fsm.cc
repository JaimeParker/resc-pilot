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

#include "plan_admin/mogen_replan_fsm.h"

void MogenReplanFSM::init(ros::NodeHandle &nh) {
    exec_state_  = FSM_EXEC_STATE::INIT;
    has_target_ = false;
    has_pose_ = false;
    drone_pos_.z() = 1.0;
    drone_vel_ = Eigen::Vector3d::Zero();

    exec_timer_ = nh.createTimer(ros::Duration(exec_period_), &MogenReplanFSM::execCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(safety_period_), &MogenReplanFSM::checkSafetyCallback, this);
    drone_pose_sub_ = nh.subscribe(drone_pose_topic_, 1, &MogenReplanFSM::poseCallback, this);
    drone_vel_sub_ = nh.subscribe(drone_vel_topic_, 1, &MogenReplanFSM::velCallback, this);
    goal_sub_ = nh.subscribe(nav_goal_topic_, 10, &MogenReplanFSM::goalCallback, this);

    planner_manager_ = std::make_unique<MogenPlanManager>();
    planner_manager_->initPlannerModules(nh);

    printHintMsg(fsm_init_hint_, GREEN);
}

void MogenReplanFSM::changeFSMState(MogenReplanFSM::FSM_EXEC_STATE new_state) {
    int pre_state_id = exec_state_;
    exec_state_ = new_state;
    std::cout << "\033[1;34m[FSM]: " << state_str_[pre_state_id] << " -> "
        << state_str_[exec_state_] << "\033[0m" << std::endl;
}

void MogenReplanFSM::printFSMExecState() {
    std::cout << "\033[1;35m[FSM]: " << state_str_[exec_state_] << "\033[0m" << std::endl;
}

void MogenReplanFSM::execCallback(const ros::TimerEvent &) {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100) {
        printFSMExecState();
        if (!has_pose_) std::cout << "no pose." << std::endl;
        if (!has_target_) std::cout << "no target." << std::endl;
        fsm_num = 0;
    }

    switch (exec_state_) {
        case INIT:
            if (!has_pose_) {
                return;
            }
            changeFSMState(WAIT_TARGET);
            break;

        case WAIT_TARGET:
            if (!has_target_) {
                return;
            }
            changeFSMState(GEN_PATH);
            break;

        case GEN_PATH:
            start_pos_ = drone_pos_;
            if (callPathGenerate(start_pos_, end_pos_)) {
                // 其实以模型的适应能力，可能都不需要对yaw进行规划，直接接RL cmd就行
//                if (!planner_manager_->yawPlanning(drone_att_.z(), drone_pos_, drone_vel_)) {
//                    changeFSMState(EXEC_MOTION);
//                } else {
//                    changeFSMState(PID_ASSIST);
//                }
                changeFSMState(EXEC_MOTION);
            }
            break;

        case EXEC_MOTION:
            if (isGoalReached()) {
                changeFSMState(WAIT_TARGET);
                printHintMsg(goal_reached_hint_, GREEN);
                has_target_ = false;
                planner_manager_->changeRLCmdState(false);
                planner_manager_->changePathValidState(false);
            } else {
                // clear to use RL cmd
                planner_manager_->changeRLCmdState(true);
            }
            break;

        case PID_ASSIST:
            if (!planner_manager_->yawPlanning(drone_att_.z(), drone_pos_, drone_vel_)) {
                changeFSMState(EXEC_MOTION);
            }
            break;

        case REPLAN:
            // TODO: 当前的重规划策略在复杂（远距离）场景行不通，但是当start-goal距离小于search_radius_时，可以使用
            //  因此暂时已经满足室内飞行的需要了
            if (planner_manager_->pathGenerate(drone_pos_, planner_manager_->getReplanEnd(drone_pos_), true)) {
                changeFSMState(EXEC_MOTION);
            }
            break;

        default:
            break;
    }
}

void MogenReplanFSM::checkSafetyCallback(const ros::TimerEvent &) {
    if (drone_pos_.z() <= terrain_height_ && drone_vel_.norm() > 0.3) {
        printHintMsg(terrain_hint_, RED);
    }

    if (drone_vel_.z() <= max_down_vel_) {
        printHintMsg(vel_down_hint_, YELLOW);
    }

    if (planner_manager_->isCollide(drone_pos_)) {
        std::cout << "\033[1;31m[FSM]: Mayday! Collision! \033[0m" << std::endl;
    }

    double speed = drone_vel_.norm();
    if (speed > 3.0) {
        std::cout << "\033[1;33m[FSM]: Warning! Max velocity! \033[0m" << speed << " - [" << drone_vel_.x()
            << ", " << drone_vel_.y() << ", " << drone_vel_.z() << "]" << std::endl;
    }

    // future ctrl pts collision check
    bool path_is_clean = planner_manager_->callCtrlPtCollisionCheck(drone_pos_);
    if (!path_is_clean && exec_state_ == EXEC_MOTION) {
        changeFSMState(REPLAN);
    }
}

bool MogenReplanFSM::callPathGenerate(const Eigen::Vector3d & start, const Eigen::Vector3d & end) {
    bool success = planner_manager_->pathGenerate(start, end, false);  // false = initial planning, not replan
    if (success) {
        planner_manager_->changePathValidState(true);
        printHintMsg(path_found_hint_, GREEN);
    } else {
        printHintMsg(no_path_hint_, RED);
    }
    return success;
}

void MogenReplanFSM::printHintMsg(std::string &msg, MogenReplanFSM::MSG_COLOR color) {
    switch (color) {
        case RED:
            std::cout << "\033[1;31m" << msg << "\033[0m" << std::endl;
            break;
        case GREEN:
            std::cout << "\033[1;32m" << msg << "\033[0m" << std::endl;
            break;
        case YELLOW:
            std::cout << "\033[1;33m" << msg << "\033[0m" << std::endl;
            break;
        case BLUE:
            std::cout << "\033[1;34m" << msg << "\033[0m" << std::endl;
            break;
        case WHITE:
            std::cout << "\033[1;37m" << msg << "\033[0m" << std::endl;
            break;
        default:
            std::cout << msg << std::endl;
    }
}

bool MogenReplanFSM::isGoalReached() {
    return (drone_pos_ - end_pos_).norm() < finish_thresh_;
}

void MogenReplanFSM::q2EulerAngle(const Eigen::Quaterniond &q, Eigen::Vector3d &euler) {
    double sr_cp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cr_cp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    euler.x() = atan2(sr_cp, cr_cp);

    double sin_p = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sin_p) >= 1)
        euler.y() = copysign(M_PI / 2, sin_p);  // pi/2
    else
        euler.y() = asin(sin_p);

    double sy_cp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cy_cp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    euler.z() = atan2(sy_cp, cy_cp);
}

void MogenReplanFSM::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    drone_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
    q2EulerAngle(q, drone_att_);
    has_pose_ = true;
}

void MogenReplanFSM::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (msg->pose.position.z < -0.1) return;

    printHintMsg(goal_received_hint_, GREEN);
    has_target_ = true;
    end_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    end_pos_[2] = cruising_altitude_;

    std::cout << "[FSM]: Target position: " << end_pos_.transpose() << std::endl;
    planner_manager_->setGoal(end_pos_);

    if (exec_state_ == WAIT_TARGET) {
        changeFSMState(GEN_PATH);
    }
}

void MogenReplanFSM::velCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    drone_vel_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
}
