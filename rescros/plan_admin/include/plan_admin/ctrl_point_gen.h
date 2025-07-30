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

#ifndef PLAN_ADMIN_CTRL_POINT_GEN_H
#define PLAN_ADMIN_CTRL_POINT_GEN_H

#include <memory>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include "frontend_searcher/fake_search.h"
#include "frontend_searcher/PathSearch.h"
#include "map_utils/MapBridge.h"

using WptPair = std::pair<Eigen::Vector3d, Eigen::Vector3d>;

class CtrlPointGen {
private:
    /* planning data */
    Eigen::Vector3d pos_;
    Eigen::Vector3d final_goal_;  // Track the actual final goal
    WptPair ctrl_point_pair_;
    std::vector<Eigen::Vector3d> discrete_path_;
    std::vector<Eigen::Vector3d> ctrl_points_buffer_;

    /* mogen utils */
    FakeSearch::Ptr fake_search_ptr_;

    /* RL env params */
    double point_interval_min_ = 0.5;
    double point_interval_max_ = 2.0;
    double point_interval_mid_ = 1.5;
    double reach_thresh_ = 0.65;
    double ctrl_point_height_ = 1.0;

    /* ROS utils */
    ros::Subscriber current_pos_sub_;
    ros::Publisher wpt0_pub_;
    ros::Publisher wpt1_pub_;

    /* params */
    std::string pose_topic_ = "/mavros/local_position/pose";
    std::string wpt0_topic_ = "/search/wpt0";
    std::string wpt1_topic_ = "/search/wpt1";
    int buffer_id_ = 0;

    /* flags */
    bool ctrl_points_need_update_ = false;
public:
    void init(ros::NodeHandle& nh);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void runPublishingLoop();
    void publishCtrlPointPair();
    static geometry_msgs::Point vec2Point(const Eigen::Vector3d& vec);
    void setPath(std::vector<Eigen::Vector3d> & path);
    void setFinalGoal(const Eigen::Vector3d & goal) { final_goal_ = goal; }  // New method to set final goal
    [[nodiscard]] bool isAtFinalGoal() const;  // Check if we're actually at the final goal
    void extendPathTowardGoal();  // Extend control points toward the final goal
    void updateCtrlPointsBuffer();
    [[nodiscard]] bool isDistanceInRange(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) const;
    [[nodiscard]] bool isDistanceInRange(const double & distance) const;
    [[nodiscard]] bool isPosNearTarget(const Eigen::Vector3d & target) const;
    void updateCtrlPointPair();
    [[nodiscard]] WptPair getCtrlPointPair() const;
    [[nodiscard]] Eigen::Vector3d getCurrentCtrlPt() const;
    [[nodiscard]] Eigen::Vector3d getFinalGoal() const { return final_goal_; }  // Getter for final goal
    [[nodiscard]] int getCtrlPtBufferId() const { return buffer_id_; }
    [[nodiscard]] std::vector<Eigen::Vector3d> getUncheckedCtrlPts(int num) const;
    void allocateCtrlPoints(const std::vector<Eigen::Vector3d> & discrete_path,
                            std::vector<Eigen::Vector3d> & ctrl_pts_buffer) const;
    void replaceCtrlPtBuffer(int start_id, int end_id, const std::vector<Eigen::Vector3d> & new_ctrl_pts);

public:
    using Ptr = std::unique_ptr<CtrlPointGen>;
};


#endif //PLAN_ADMIN_CTRL_POINT_GEN_H
