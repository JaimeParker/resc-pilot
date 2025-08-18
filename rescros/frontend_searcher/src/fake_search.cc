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

#include "frontend_searcher/fake_search.h"

WptNodeList* createWptLoop(std::vector<Eigen::Vector3d>& wpt_list) {
    if (wpt_list.empty()) {
        ROS_ERROR("Waypoint list is empty.");
        return nullptr;
    }

    auto* head = new WptNodeList;
    head->wpt = wpt_list[0];
    head->next_node = head;

    WptNodeList* cur_wpt = head;

    for (int i = 1; i < wpt_list.size(); ++i) {
        auto* new_wpt = new WptNodeList;
        new_wpt->wpt = wpt_list[i];
        new_wpt->next_node = head;

        cur_wpt->next_node = new_wpt;
        cur_wpt = new_wpt;
    }

    return head;
}

void deleteWptNodeList(WptNodeList* head) {
    if (head == nullptr) return;

    WptNodeList* current = head;
    WptNodeList* nextNode;

    do {
        nextNode = current->next_node;
        delete current;
        current = nextNode;
    } while (current != head);
}

void printWptNodeList(WptNodeList* head) {
    if (head == nullptr) return;

    WptNodeList* current = head;

    do {
        std::cout << current->wpt.transpose() << std::endl;
        current = current->next_node;
    } while (current != head);
    std::cout << current->wpt.transpose() << std::endl;
}

FakeSearch::FakeSearch() {
    initRectLoopWpt();
    if (!rect_loop_wpt_.empty()) {
        rect_loop_head_ = createWptLoop(rect_loop_wpt_);
        rect_loop_tar_ = rect_loop_wpt_[0];
    } else {
        ROS_ERROR("Rectangle loop waypoints are empty.");
        rect_loop_head_ = nullptr;
    }
}

FakeSearch::~FakeSearch() {
    deleteWptNodeList(rect_loop_head_);
    std::cout << "FakeSearch is destructed." << std::endl;
}

WptPair FakeSearch::doCircleLoop(int option) const{
    Eigen::Matrix<double, 4, 3> waypoints;
    if (wpt_min_interval_ <= circle_loop_radius_ &&
    1.414 * circle_loop_radius_ <= wpt_max_interval_) {
        waypoints << circle_loop_radius_, 0.0, 1.0,
                0.0, -circle_loop_radius_, 1.0,
                -circle_loop_radius_, 0.0, 1.0,
                0.0, circle_loop_radius_, 1.0;
    } else {
        ROS_ERROR("Circle loop radius is out of range.");
    }

    WptPair wptPair;
    switch(option) {
        case 0:
            wptPair = std::make_pair(waypoints.row(0), waypoints.row(1));
            break;
        case 1:
            wptPair = std::make_pair(waypoints.row(1), waypoints.row(2));
            break;
        case 2:
            wptPair = std::make_pair(waypoints.row(2), waypoints.row(3));
            break;
        case 3:
            wptPair = std::make_pair(waypoints.row(3), waypoints.row(0));
            break;
        default:
            ROS_ERROR("Invalid option for circle loop.");
            break;
    }

    return wptPair;
}

void FakeSearch::init(ros::NodeHandle& nh) {
    if (nh.hasParam("rl/env_wpt_interval_min")) {
        nh.param("rl/env_wpt_interval_min", wpt_min_interval_, 0.5);
    } else {
        ROS_WARN("No param named 'rl/env_wpt_interval_min', will use default value.");
    }

    if (nh.hasParam("rl/env_wpt_interval_max")) {
        nh.param("rl/env_wpt_interval_max", wpt_max_interval_, 2.5);
    } else {
        ROS_WARN("No param named 'rl/env_wpt_interval_max', will use default value.");
    }

    if (nh.hasParam("rl/env_wpt_tolerance")) {
        nh.param("rl/env_wpt_tolerance", wpt_tolerance_, 0.5);
    } else {
        ROS_WARN("No param named 'rl/env_wpt_tolerance', will use default value.");
    }
}

void FakeSearch::initRectLoopWpt() {
    rect_loop_wpt_.emplace_back(0, 0, 1);
    rect_loop_wpt_.emplace_back(2, 1, 1);
    rect_loop_wpt_.emplace_back(2, 2, 1);
    rect_loop_wpt_.emplace_back(4, 2, 1);
    rect_loop_wpt_.emplace_back(4, 4, 1);
    rect_loop_wpt_.emplace_back(7, 4, 1);
    rect_loop_wpt_.emplace_back(6, 2, 1);
    rect_loop_wpt_.emplace_back(5, 0, 1);
    rect_loop_wpt_.emplace_back(3, 0, 1);
    rect_loop_wpt_.emplace_back(1, -1, 1);
}

WptPair FakeSearch::getRectLoopWpt(const Eigen::Vector3d &pos) {
    Eigen::Vector2d pos_2d(pos.x(), pos.y());
    Eigen::Vector2d rect_loop_tar_2d(rect_loop_tar_.x(), rect_loop_tar_.y());

    if (((pos_2d - rect_loop_tar_2d).norm() < wpt_tolerance_ && abs(pos.z() - rect_loop_tar_.z()) < 0.5)
    or (pos - rect_loop_tar_).norm() < wpt_tolerance_) {
        rect_loop_tar_ = rect_loop_head_->next_node->wpt;
        rect_loop_head_ = rect_loop_head_->next_node;
    }

    return std::make_pair(rect_loop_tar_, rect_loop_head_->next_node->wpt);
}

void FakeSearch::checkRectLoopWpt() {
    printWptNodeList(rect_loop_head_);
}
