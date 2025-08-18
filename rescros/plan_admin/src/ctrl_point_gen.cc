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

#include "plan_admin/ctrl_point_gen.h"

void CtrlPointGen::init(ros::NodeHandle &nh) {
    ROS_INFO_STREAM("\033[1;32m" << "CtrlPointGen init." << "\033[0m");
    current_pos_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic_, 1, &CtrlPointGen::poseCallback, this);

    wpt0_pub_ = nh.advertise<geometry_msgs::Point>(wpt0_topic_, 1);
    wpt1_pub_ = nh.advertise<geometry_msgs::Point>(wpt1_topic_, 1);

    // Initialize final goal to current position (will be updated when path is set)
    final_goal_ = Eigen::Vector3d::Zero();

    fake_search_ptr_ = std::make_unique<FakeSearch>();
    fake_search_ptr_->init(nh);
}

void CtrlPointGen::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

void CtrlPointGen::runPublishingLoop() {
    ROS_INFO_STREAM("\033[1;32m" << "CtrlPointGen start search..." << "\033[0m");
    ros::Rate rate(50.0);

    while (ros::ok()) {
        // rect loop
        ctrl_point_pair_ = fake_search_ptr_->getRectLoopWpt(pos_);
        publishCtrlPointPair();
        ros::spinOnce();
        rate.sleep();
    }
}

geometry_msgs::Point CtrlPointGen::vec2Point(const Eigen::Vector3d &vec) {
    geometry_msgs::Point point;
    point.x = vec.x();
    point.y = vec.y();
    point.z = vec.z();
    return point;
}

void CtrlPointGen::publishCtrlPointPair() {
    updateCtrlPointPair();  // comment this when using fake search
    wpt0_pub_.publish(vec2Point(ctrl_point_pair_.first));
    wpt1_pub_.publish(vec2Point(ctrl_point_pair_.second));
}

void CtrlPointGen::setPath(std::vector<Eigen::Vector3d> &path) {
    ctrl_points_buffer_.clear();
    discrete_path_ = path;
    ctrl_points_need_update_ = true;
    updateCtrlPointsBuffer();
}

void CtrlPointGen::updateCtrlPointsBuffer() {
    if (!ctrl_points_need_update_) {
        return;
    }
    std ::cout << "[CtrlPointGen]: update ctrl points buffer for RL." << std::endl;

    // Store current progress to avoid going backward after replanning
    Eigen::Vector3d current_target_pos = Eigen::Vector3d::Zero();
    bool has_current_target = false;
    if (!ctrl_points_buffer_.empty() && buffer_id_ < ctrl_points_buffer_.size()) {
        current_target_pos = ctrl_points_buffer_[buffer_id_];
        has_current_target = true;
    }

    // Clear and rebuild buffer
    ctrl_points_buffer_.clear();
    allocateCtrlPoints(discrete_path_, ctrl_points_buffer_);

    // Try to maintain progress by finding the closest point ahead of current position
    if (has_current_target && !ctrl_points_buffer_.empty()) {
        int new_buffer_id = 0;
        double min_distance_ahead = std::numeric_limits<double>::max();
        
        for (int i = 0; i < ctrl_points_buffer_.size(); i++) {
            double dist_to_current = (ctrl_points_buffer_[i] - pos_).norm();
            double dist_to_old_target = (ctrl_points_buffer_[i] - current_target_pos).norm();
            
            // Prefer points that are:
            // 1. Ahead of current position (dist_to_current > reach_thresh_)
            // 2. Close to the previous target (to maintain continuity)
            if (dist_to_current > reach_thresh_ * 0.8 && dist_to_old_target < min_distance_ahead) {
                min_distance_ahead = dist_to_old_target;
                new_buffer_id = i;
            }
        }
        
        buffer_id_ = new_buffer_id;
        std::cout << "[CtrlPointGen]: Preserved progress - buffer_id set to " << buffer_id_ 
                  << " (distance to old target: " << min_distance_ahead << ")" << std::endl;
    } else {
        buffer_id_ = 0;  // reset id for constructing ctrl point pair
    }

    // Ensure buffer_id_ is within bounds
    if (buffer_id_ >= ctrl_points_buffer_.size()) {
        buffer_id_ = ctrl_points_buffer_.size() - 1;
    }

    ctrl_points_need_update_ = false;
}

void CtrlPointGen::allocateCtrlPoints(const std::vector<Eigen::Vector3d> &discrete_path,
                                      std::vector<Eigen::Vector3d> &ctrl_pts_buffer) const {
    if (discrete_path.empty()) {
        std::cout << "\033[1;33m[CtrlPointGen]: path is empty, cannot update ctrl points buffer.\033[0m" << std::endl;
        return;
    }

    if (discrete_path.size() == 1) {
        ctrl_pts_buffer.emplace_back(discrete_path[0]);
        ctrl_pts_buffer.emplace_back(discrete_path[0]);
        std::cout << "\033[1;33m[CtrlPointGen]: Warning! Path size is 1, shall use directly.\033[0m" << std::endl;
        std::cout << "\033[1;33m[CtrlPointGen]: ctrl points: [" << discrete_path[0].x() << ", "
            << discrete_path[0].y() << ", " << discrete_path[0].z() << "].\033[0m" << std::endl;
        return;
    }

    int idx = 0;
    bool is_done = false;
    while (!is_done) {
        double distance = (discrete_path[idx] - discrete_path[idx + 1]).norm();
        if (isDistanceInRange(distance)) {
            ctrl_pts_buffer.emplace_back(discrete_path[idx]);
        } else {
            auto diff = discrete_path[idx + 1] - discrete_path[idx];
            auto dir = diff.normalized();

            int num_int = static_cast<int>(distance / point_interval_mid_);
            double left = distance - num_int * point_interval_mid_;
            for (int i = 0; i < num_int + 1; i++) {
                if (i == num_int && left < point_interval_max_ - point_interval_mid_) {
                    break;
                }
                ctrl_pts_buffer.emplace_back(discrete_path[idx] + i * point_interval_mid_ * dir);
            }
        }

        if (idx + 1 == discrete_path.size() - 1) {
            ctrl_pts_buffer.emplace_back(discrete_path[idx + 1]);
            is_done = true;
        }
        idx++;
    }
}

bool CtrlPointGen::isDistanceInRange(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) const {
    double distance = (p1 - p2).norm();
    return (point_interval_min_ <= distance) && (distance <= point_interval_max_);
}

bool CtrlPointGen::isDistanceInRange(const double &distance) const {
    return (point_interval_min_ <= distance) && (distance <= point_interval_max_);
}

bool CtrlPointGen::isPosNearTarget(const Eigen::Vector3d &target) const {
    return (pos_ - target).norm() <= reach_thresh_;
}

bool CtrlPointGen::isAtFinalGoal() const {
    return (pos_ - final_goal_).norm() <= reach_thresh_;
}

void CtrlPointGen::extendPathTowardGoal() {
    if (ctrl_points_buffer_.empty()) {
        return;
    }
    
    // Get the current last waypoint
    Eigen::Vector3d last_waypoint = ctrl_points_buffer_.back();
    
    // Calculate direction and distance to final goal
    Eigen::Vector3d direction = (final_goal_ - last_waypoint).normalized();
    double distance_to_goal = (final_goal_ - last_waypoint).norm();
    
    // Add intermediate waypoints toward the goal
    std::vector<Eigen::Vector3d> extension_points;
    double step_size = point_interval_mid_;
    double current_dist = step_size;
    
    while (current_dist < distance_to_goal) {
        Eigen::Vector3d new_point = last_waypoint + current_dist * direction;
        extension_points.push_back(new_point);
        current_dist += step_size;
    }
    
    // Always add the final goal
    extension_points.push_back(final_goal_);
    
    // Add the extension points to the buffer
    ctrl_points_buffer_.insert(ctrl_points_buffer_.end(), extension_points.begin(), extension_points.end());
    
    std::cout << "[CtrlPtGen]: Extended path with " << extension_points.size() 
              << " waypoints toward final goal." << std::endl;
}

void CtrlPointGen::updateCtrlPointPair() {
    if (ctrl_points_buffer_.empty()) {
        std::cout << "\033[1;33m[CtrlPointGen]: ctrl points buffer is empty, "
                     "cannot update ctrl point pair.\033[0m" << std::endl;
        ROS_WARN("out of index might be caused by empty buffer.");
        return;
    }

    // Ensure buffer_id_ is within valid range
    if (buffer_id_ >= ctrl_points_buffer_.size()) {
        std::cout << "\033[1;31m[CtrlPointGen]: buffer_id_ (" << buffer_id_ 
                  << ") exceeds buffer size (" << ctrl_points_buffer_.size() 
                  << "), clamping to last waypoint.\033[0m" << std::endl;
        buffer_id_ = ctrl_points_buffer_.size() - 1;
    }

    auto target = ctrl_points_buffer_[buffer_id_];
    target.z() = ctrl_point_height_;
    if (isPosNearTarget(target)) {
        std::cout << "[CtrlPtGen]: I pass [" << std::fixed << std::setprecision(2)
            << target.x() << ", " << target.y() << ", " << target.z() << "]." << std::endl;
        buffer_id_++;
        
        // Check bounds after increment
        if (buffer_id_ >= ctrl_points_buffer_.size()) {
            buffer_id_ = ctrl_points_buffer_.size() - 1;
            
            // Check if we're actually at the final goal
            if (isAtFinalGoal()) {
                std::cout << "\033[1;32m[CtrlPtGen]: Reached FINAL GOAL at [" << final_goal_.x() 
                          << ", " << final_goal_.y() << ", " << final_goal_.z() << "]!\033[0m" << std::endl;
            } else {
                std::cout << "\033[1;33m[CtrlPtGen]: Reached end of current path segment but NOT at final goal!" << std::endl;
                std::cout << "[CtrlPtGen]: Current pos: [" << pos_.x() << ", " << pos_.y() << ", " << pos_.z() 
                          << "], Final goal: [" << final_goal_.x() << ", " << final_goal_.y() << ", " << final_goal_.z() 
                          << "], Distance: " << (pos_ - final_goal_).norm() << "\033[0m" << std::endl;
                
                // TODO: This should trigger replanning to continue toward the actual goal
                // For now, we'll extend the current waypoint toward the goal
                extendPathTowardGoal();
            }
        }
    }

    // set control point pair with bounds checking
    if (buffer_id_ < ctrl_points_buffer_.size()) {
        ctrl_point_pair_.first = ctrl_points_buffer_[buffer_id_];
        if (buffer_id_ == ctrl_points_buffer_.size() - 1) {
            ctrl_point_pair_.second = ctrl_points_buffer_[buffer_id_];
        } else {
            ctrl_point_pair_.second = ctrl_points_buffer_[buffer_id_ + 1];
        }
    } else {
        // Fallback - this should not happen with the bounds checking above
        std::cout << "\033[1;31m[CtrlPointGen]: Critical error - buffer_id_ out of bounds!\033[0m" << std::endl;
        buffer_id_ = ctrl_points_buffer_.size() - 1;
        ctrl_point_pair_.first = ctrl_points_buffer_[buffer_id_];
        ctrl_point_pair_.second = ctrl_points_buffer_[buffer_id_];
    }

    ctrl_point_pair_.first.z() = ctrl_point_height_;
    ctrl_point_pair_.second.z() = ctrl_point_height_;
}

WptPair CtrlPointGen::getCtrlPointPair() const {
    return ctrl_point_pair_;
}

Eigen::Vector3d CtrlPointGen::getCurrentCtrlPt() const {
    return ctrl_point_pair_.first;
}

std::vector<Eigen::Vector3d> CtrlPointGen::getUncheckedCtrlPts(int num) const {
    // get num ctrl pts start from current buffer_id_ until the end
    std::vector<Eigen::Vector3d> unchecked_ctrl_pts;
    auto buffer_size = ctrl_points_buffer_.size();
    int buffer_id_temp = buffer_id_;
    while (buffer_id_temp < buffer_size && buffer_id_temp - buffer_id_ < num) {
        unchecked_ctrl_pts.emplace_back(ctrl_points_buffer_[buffer_id_temp]);
        buffer_id_temp += 1;
    }

    return unchecked_ctrl_pts;
}

void CtrlPointGen::replaceCtrlPtBuffer(int start_id, int end_id, const std::vector<Eigen::Vector3d> &new_ctrl_pts) {
    if (start_id < 0 || end_id >= ctrl_points_buffer_.size() || start_id > end_id) {
        std::cout << "\033[1;33m[CtrlPointGen]: invalid start or end id for replacing ctrl points buffer.\033[0m" << std::endl;
        return;
    }

    ctrl_points_buffer_.erase(ctrl_points_buffer_.begin() + start_id, ctrl_points_buffer_.begin() + end_id + 1);
    ctrl_points_buffer_.insert(ctrl_points_buffer_.begin() + start_id, new_ctrl_pts.begin(), new_ctrl_pts.end());

    // Debug: print the updated control points
//    std::cout << "[CtrlPointGen]: Updated ctrl points buffer:\n";
//    for (const auto &point : ctrl_points_buffer_) {
//        std::cout << "[" << point.x() << ", " << point.y() << ", " << point.z() << "]\n";
//    }
}
