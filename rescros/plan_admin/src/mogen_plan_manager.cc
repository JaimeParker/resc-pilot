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

#include "plan_admin/mogen_plan_manager.h"

void MogenPlanManager::initPlannerModules(ros::NodeHandle &nh) {
    int use_randomized_map = 2;
    getParamWithWarning(nh, "sim/use_sim_map", use_randomized_map);
    getParamWithWarning(nh, "use_rl_topic", use_rl_topic_);
    getParamWithWarning(nh, "use_pid_topic", use_pid_topic_);
    double fence_x = 0.0;
    double fence_y = 0.0;
    getParamWithWarning(nh, "map/geo_fence_x", fence_x);
    getParamWithWarning(nh, "map/geo_fence_y", fence_y);

    // construct mogen modules
    ctrl_point_gen_ptr_ = std::make_unique<CtrlPointGen>();
    rl_motion_gen_ptr_ = std::make_unique<MotionGen>();
    map_bridge_ptr_ = std::make_shared<MapBridge>(nh);
    path_search_ptr_ = std::make_shared<PathSearch>();

    // init mogen modules
    ctrl_point_gen_ptr_->init(nh);
    rl_motion_gen_ptr_->init(nh);

    // init map for simulation
    map_bridge_ptr_->init();
    if (use_randomized_map == 1) {
        // use randomized obstacles
        map_bridge_ptr_->setRandomObstacles();
        map_bridge_ptr_->setGodView(false);
    } else if (use_randomized_map == 2) {
        // use a default map
        map_bridge_ptr_->setDefaultObstacles();
        map_bridge_ptr_->setGeoFencePcl(fence_x, fence_y);
        map_bridge_ptr_->initSDFMap();
        map_bridge_ptr_->overrideRealSDF(false);
        map_bridge_ptr_->setGodView(false);  // close god view to allow replan
    } else {
        // use a real map
        map_bridge_ptr_->setGeoFencePcl(fence_x, fence_y);
        map_bridge_ptr_->initSDFMap();
        map_bridge_ptr_->setGodView(false);  // close god view to allow replan
    }
    resolution_ = map_bridge_ptr_->getResolution();

    // ros utils
    search_timer_ = nh.createTimer(ros::Duration(search_period_), &MogenPlanManager::searchCallback, this);
    rl_cmd_pub_timer_ = nh.createTimer(ros::Duration(rl_cmd_pub_period_), &MogenPlanManager::rlCmdPubCallback, this);
    use_rl_timer_ = nh.createTimer(ros::Duration(use_rl_period_), &MogenPlanManager::useRLCallback, this);
    use_rl_pub_ = nh.advertise<std_msgs::Bool>(use_rl_topic_, 1);
    global_pcl_timer_ = nh.createTimer(ros::Duration(global_pcl_period_), &MogenPlanManager::globalPclCallback, this);
    local_pcl_timer_ = nh.createTimer(ros::Duration(local_pcl_period_), &MogenPlanManager::localPclCallback, this);
    pid_ctrl_timer_ = nh.createTimer(ros::Duration(pid_ctrl_period_), &MogenPlanManager::pidCtrlCallback, this);
    pid_ctrl_pub_ = nh.advertise<mavros_msgs::PositionTarget>(pid_pos_ctrl_topic_, 1);
    use_pid_pub_ = nh.advertise<std_msgs::Bool>(use_pid_topic_, 1);

    // set map data to other modules
    path_search_ptr_->setMapBridge(map_bridge_ptr_);
    rl_motion_gen_ptr_->setMapBridge(map_bridge_ptr_);

    // rviz utils
    pose_marker_handler_ = std::make_unique<PoseMarkerHandler>(nh, frame_id_);
    pose_marker_handler_->init();
    ctrl_pt_marker_handler_ = std::make_unique<CtrlPtMarkerHandler>(nh, frame_id_);
    ctrl_pt_marker_handler_->init();
    traj_marker_handler_ = std::make_unique<TrajMarkerHandler>(nh, frame_id_);
    traj_marker_handler_->init();
    rviz_pose_timer_ = nh.createTimer(ros::Duration(rviz_pose_period_), &MogenPlanManager::rvizCallback, this);

    std::cout << "\033[1;32m[MogenPlanManager]: init done.\033[0m" << std::endl;
}

void MogenPlanManager::searchCallback(const ros::TimerEvent &) {
    if (path_valid_) {
        ctrl_point_gen_ptr_->publishCtrlPointPair();
    }
}

void MogenPlanManager::rlCmdPubCallback(const ros::TimerEvent &) {
    if (allow_rl_cmd_ && path_valid_) {
        rl_motion_gen_ptr_->publishRLCmd();
    }
}

void MogenPlanManager::useRLCallback(const ros::TimerEvent &) {
    // for actual controller to receive, then to decide whether to use RL cmd or not
    use_rl_msg_.data = allow_rl_cmd_;
    use_rl_pub_.publish(use_rl_msg_);
}

void MogenPlanManager::changeRLCmdState(bool allow_rl_cmd) {
    allow_rl_cmd_ = allow_rl_cmd;
}

void MogenPlanManager::globalPclCallback(const ros::TimerEvent &) {
    map_bridge_ptr_->publishGlobalPCL();
}

void MogenPlanManager::localPclCallback(const ros::TimerEvent &) {
    map_bridge_ptr_->updateLocalMap();
    map_bridge_ptr_->updateInflatedObstacle();

    map_bridge_ptr_->publishLocalPCL();
    map_bridge_ptr_->publishInflatedObstaclePCL();
}

bool MogenPlanManager::pathGenerate(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &end_pos, bool is_replan) {
    std::cout << "[path planning]: -------------------------" << std::endl;
    std::cout << "start: " << start_pos.transpose() << std::endl;
    std::cout << "end: " << end_pos.transpose() << std::endl;

    if ((start_pos - end_pos).norm() < reach_goal_thresh_) {
        std::cout << "[path planning]: danger close! re-choose a goal." << std::endl;
        return false;
    }

    if (map_bridge_ptr_->isInflateOccupied(start_pos) ||
        map_bridge_ptr_->isInflateOccupied(end_pos)) {
        std::cout << "[path planning]: start or end is in occupied area." << std::endl;
        return false;
    }

    auto start_time = ros::Time::now();
    path_search_ptr_->reset();
    bool success = path_search_ptr_->visibilitySearch(start_pos, end_pos);
    if (success) {
        std::cout << "[path planning]: path found." << std::endl;
        auto path = path_search_ptr_->getDiscretePath();
        ctrl_point_gen_ptr_->setPath(path);
        
        // Only set the final goal during initial planning, not during replanning
        if (!is_replan) {
            ctrl_point_gen_ptr_->setFinalGoal(end_pos);  // Set the final goal only for initial planning
            std::cout << "[path planning]: Final goal set to: " << end_pos.transpose() << std::endl;
        } else {
            std::cout << "[path planning]: Replanning - final goal preserved." << std::endl;
        }
    } else {
        std::cout << "[path planning]: no path found." << std::endl;
    }
    auto end_time = ros::Time::now();
    std::cout << "[path planning]: time cost: " << "\033[42m\033[97m" << (end_time - start_time).toSec()
        << "s." << "\033[0m" << std::endl;

    return success;
}

void MogenPlanManager::rvizCallback(const ros::TimerEvent &) {
    pose_marker_handler_->publish();
    ctrl_pt_marker_handler_->publish();
    traj_marker_handler_->publish();
}

void MogenPlanManager::changePathValidState(bool path_valid) {
    path_valid_ = path_valid;
}

bool MogenPlanManager::yawPlanning(const double &yaw, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel) {
    auto target = ctrl_point_gen_ptr_->getCtrlPointPair().first;
    Eigen::Vector2d vec_target_2d = target.head(2) - pos.head(2);
    Eigen::Vector2d approx_yaw_vec = Eigen::Vector2d(cos(yaw), sin(yaw));
    double cos_theta = vec_target_2d.dot(approx_yaw_vec) / (vec_target_2d.norm() * approx_yaw_vec.norm() + 1e-8);

    if (std::abs(cos_theta) < std::cos(rl_yaw_init_thresh_)) {
        Eigen::Vector3d vec_target = (target - pos).normalized();
        Eigen::Vector3d new_vel = vel.norm() * vec_target;
        double next_yaw = atan2(vec_target_2d.y(), vec_target_2d.x());
        calcNextYaw(yaw, next_yaw);

        if (!need_pid_ctrl_) {
            // print only once
            std::cout << "[Plan Manager]: need PID control --------------" << std::endl;
            std::cout << "desired pos: " << target.transpose() << std::endl;
            std::cout << "desired vel: " << new_vel.transpose() << std::endl;
            std::cout << "desired yaw: " << next_yaw << std::endl;
        }

        // send to PID controller
        pid_ctrl_msg_.position.x = target.x();
        pid_ctrl_msg_.position.y = target.y();
        pid_ctrl_msg_.position.z = target.z();
        pid_ctrl_msg_.velocity.x = new_vel.x();
        pid_ctrl_msg_.velocity.y = new_vel.y();
        pid_ctrl_msg_.velocity.z = new_vel.z();
        pid_ctrl_msg_.yaw = static_cast<float>(next_yaw);

        need_pid_ctrl_ = true;
    } else {
        need_pid_ctrl_ = false;
    }

    return need_pid_ctrl_;
}

void MogenPlanManager::calcNextYaw(const double &last_yaw, double &yaw) {
    // author: Boyu Zhou

    // make round_last in [-PI, PI]
    double round_last = last_yaw;

    while (round_last < -M_PI) {
        round_last += 2 * M_PI;
    }
    while (round_last > M_PI) {
        round_last -= 2 * M_PI;
    }

    double diff = yaw - round_last;

    // make abs(yaw - last_yaw) <= PI
    if (fabs(diff) <= M_PI) {
        yaw = last_yaw + diff;
    } else if (diff > M_PI) {
        yaw = last_yaw + diff - 2 * M_PI;
    } else if (diff < -M_PI) {
        yaw = last_yaw + diff + 2 * M_PI;
    }
}

void MogenPlanManager::pidCtrlCallback(const ros::TimerEvent &) {
    if (need_pid_ctrl_) {
        pid_ctrl_msg_.header.stamp = ros::Time::now();
        pid_ctrl_pub_.publish(pid_ctrl_msg_);
        use_pid_msg_.data = true;
    } else {
        use_pid_msg_.data = false;
    }

    use_pid_pub_.publish(use_pid_msg_);
}

bool MogenPlanManager::isCollide(const Eigen::Vector3d &pos) const {
    // print warning msg if collide
    return map_bridge_ptr_->getDistance(pos) < collide_warn_dist_;
}

bool MogenPlanManager::callCtrlPtCollisionCheck(const Eigen::Vector3d &pos_curr) {
    if (!path_valid_ || map_bridge_ptr_->getGodViewStatus()) { return true; }

    if (map_bridge_ptr_->isInflateOccupied(pos_curr)) {
        return true;  // already in an obstacle, no need to struggle
    }

    // if control points are in obstacle
    auto ctrl_pt_pair = ctrl_point_gen_ptr_->getCtrlPointPair();
    if (map_bridge_ptr_->isInflateOccupied(ctrl_pt_pair.first)
        || map_bridge_ptr_->isInflateOccupied(ctrl_pt_pair.second)) {
        return false;
    }

    auto dir_p2c = (ctrl_pt_pair.first - pos_curr).normalized();
    double dist = (ctrl_pt_pair.first - pos_curr).norm();
    double dist_travel = 0.0;
    int possible_collision_times = 0;
    while (dist_travel < dist) {
        Eigen::Vector3d temp_pt = pos_curr + dist_travel * dir_p2c;
        if (map_bridge_ptr_->isInflateOccupied(temp_pt)) {
            possible_collision_times += 1;
        }
        dist_travel += resolution_;
    }
    if (possible_collision_times > 2) {
        return false;
    }

    // dir_p2c = (ctrl_pt_pair.second - pos_curr).normalized();
    // dist = (ctrl_pt_pair.second - pos_curr).norm();
    // dist_travel = 0.0;
    // possible_collision_times = 0;
    // while (dist_travel < dist) {
    //     Eigen::Vector3d temp_pt = pos_curr + dist_travel * dir_p2c;
    //     if (map_bridge_ptr_->isInflateOccupied(temp_pt)) {
    //         possible_collision_times += 1;
    //     }
    //     dist_travel += resolution_;
    // }
    // if (possible_collision_times > 1) {
    //     return false;
    // }

    // if all check passed
    return true;
}

Eigen::Vector3d MogenPlanManager::getReplanEnd(Eigen::Vector3d &drone_pos) const {
    double dist = (drone_pos - goal_).norm();
    if (dist < search_radius_) {
        return goal_;
    }
    
    Eigen::Vector3d dir = (goal_ - drone_pos).normalized();
    Eigen::Vector3d candidate_end = drone_pos + search_radius_ * dir;
    
    // Check if candidate is in obstacle
    if (!map_bridge_ptr_->isInflateOccupied(candidate_end)) {
        return candidate_end;
    }
    
    // If direct path is blocked, try to find a safe intermediate point
    return findSafeReplanTarget(drone_pos, goal_, search_radius_);
}

Eigen::Vector3d MogenPlanManager::findSafeReplanTarget(const Eigen::Vector3d &start, 
                                                       const Eigen::Vector3d &final_goal, 
                                                       double max_dist) const {
    // Strategy 1: Try points around the direct path
    Eigen::Vector3d direct_dir = (final_goal - start).normalized();
    
    // Try the direct path first at various distances
    for (double dist = max_dist; dist > max_dist * 0.3; dist -= resolution_ * 5) {
        Eigen::Vector3d candidate = start + dist * direct_dir;
        if (!map_bridge_ptr_->isInflateOccupied(candidate)) {
            return candidate;
        }
    }
    
    // Strategy 2: Try lateral offsets from the direct path
    Eigen::Vector3d up_vec(0, 0, 1);
    Eigen::Vector3d lateral1 = direct_dir.cross(up_vec).normalized();
    Eigen::Vector3d lateral2 = direct_dir.cross(lateral1).normalized();
    
    std::vector<Eigen::Vector3d> offset_dirs = {lateral1, -lateral1, lateral2, -lateral2};
    
    for (double dist = max_dist * 0.8; dist > max_dist * 0.3; dist -= resolution_ * 5) {
        Eigen::Vector3d base_point = start + dist * direct_dir;
        
        for (const auto &offset_dir : offset_dirs) {
            for (double offset = resolution_; offset < max_dist * 0.3; offset += resolution_ * 2) {
                Eigen::Vector3d candidate = base_point + offset * offset_dir;
                if (!map_bridge_ptr_->isInflateOccupied(candidate)) {
                    return candidate;
                }
            }
        }
    }
    
    // Strategy 3: Fallback - find any safe point within radius
    return findAnySafePointInRadius(start, max_dist * 0.5);
}

Eigen::Vector3d MogenPlanManager::findAnySafePointInRadius(const Eigen::Vector3d &center, 
                                                           double radius) const {
    // Grid search for any safe point
    double step = resolution_ * 3;
    for (double x = -radius; x <= radius; x += step) {
        for (double y = -radius; y <= radius; y += step) {
            for (double z = -radius * 0.5; z <= radius * 0.5; z += step) {
                Eigen::Vector3d candidate = center + Eigen::Vector3d(x, y, z);
                if ((candidate - center).norm() <= radius && 
                    !map_bridge_ptr_->isInflateOccupied(candidate)) {
                    return candidate;
                }
            }
        }
    }
    
    // Last resort - return current position (will trigger a different replanning strategy)
    return center;
}
