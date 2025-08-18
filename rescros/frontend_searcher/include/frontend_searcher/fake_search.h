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
 * Created by Zhaohong Liu on 24-5-10.
 * This file is for generating fake search waypoints for the UAV.
 * Make sure you know what you are doing before using this file.
*/

#ifndef FRONTEND_SEARCHER_FAKE_SEARCH_H
#define FRONTEND_SEARCHER_FAKE_SEARCH_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <utility>

using WptPair = std::pair<Eigen::Vector3d, Eigen::Vector3d>;

struct WptNodeList{
    Eigen::Vector3d wpt;
    WptNodeList* next_node;
};

class FakeSearch {
private:
    // rl env params
    double wpt_min_interval_ = 0.5;
    double wpt_max_interval_ = 2.5;
    double wpt_tolerance_ = 0.5;

    // circle loop params
    double circle_loop_radius_ = 1.5;
    Eigen::Vector3d circle_loop_center_ = Eigen::Vector3d(0, 0, 1);

    // rectangle loop params
    std::vector<Eigen::Vector3d> rect_loop_wpt_;
    WptNodeList* rect_loop_head_ = nullptr;
    Eigen::Vector3d rect_loop_tar_;


public:
    FakeSearch();
    ~FakeSearch();
    void init(ros::NodeHandle& nh);
    [[nodiscard]] WptPair doCircleLoop(int option) const;
    void initRectLoopWpt();
    WptPair getRectLoopWpt(const Eigen::Vector3d& pos);
    void checkRectLoopWpt();
public:
    using Ptr = std::unique_ptr<FakeSearch>;
};


#endif //FRONTEND_SEARCHER_FAKE_SEARCH_H
