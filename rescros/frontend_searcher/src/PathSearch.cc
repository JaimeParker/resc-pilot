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
 * Created by Zhaohong Liu on 24-10-17.
*/

#include "frontend_searcher/PathSearch.h"

PathSearch::PathSearch() {
    voxel_directions_ = {
            Eigen::Vector3i(-1, 1, 0),
            Eigen::Vector3i(0, 1, 0),
            Eigen::Vector3i(1, 1, 0),
            Eigen::Vector3i(1, 0, 0),
            Eigen::Vector3i(1, -1, 0),
            Eigen::Vector3i(0, -1, 0),
            Eigen::Vector3i(-1, -1, 0),
            Eigen::Vector3i(-1, 0, 0),
    };
    search_radius_ = static_cast<int>(search_range_ / resolution_);
}

void PathSearch::setMapBridge(const MapBridge::Ptr &ptr) {
    map_bridge_ptr_ = ptr;
    ground_index_ = map_bridge_ptr_->getGroundIndex();
    resolution_ = map_bridge_ptr_->getResolution();
//    collision_threshold_ = map_bridge_ptr_->getCollisionThreshold();
    edge_threshold_ = collision_threshold_ + resolution_ * sqrt(2);
}

[[maybe_unused]]
bool PathSearch::aStarSearch(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &end_pos) {
    // init planning params
    start_voxel_ = map_bridge_ptr_->pos2Voxel(start_pos);
    end_voxel_ = map_bridge_ptr_->pos2Voxel(end_pos);
    start_voxel_[2] = ground_index_;
    end_voxel_[2] = ground_index_;

    // create the start node
    auto start_node = createNode(start_voxel_, nullptr, 0.0);
    open_set_.push(start_node);
    expanded_nodes_.insert(start_node->voxel, start_node);

    if (map_bridge_ptr_->isInflateOccupied(start_pos)) {
        ROS_WARN("Start position is occupied.");
        return false;
    }

    while (!open_set_.empty()) {
        auto cur_node = open_set_.top();
        open_set_.pop();
        closed_set_.insert(cur_node->voxel);

        if (isGoal(cur_node)) {
            retrievePath(cur_node);
            return true;
        }

        // expanding nodes
        for (const auto& dir : voxel_directions_) {
            Eigen::Vector3i neighbor_voxel = cur_node->voxel + dir;
            if (!map_bridge_ptr_->isVoxelValid(neighbor_voxel) ||
                map_bridge_ptr_->isInflateOccupied(neighbor_voxel) ||
                closed_set_.find(neighbor_voxel) != closed_set_.end()) {
                continue;
            }

            double g_score = cur_node->g_score_ + dir.norm();
            auto node_expanded = expanded_nodes_.find(neighbor_voxel);
            if (node_expanded) {
                // prune
                if (g_score < node_expanded->g_score_) {
                    node_expanded->g_score_ = g_score;
                    node_expanded->f_score_ = g_score + getHeuristic(node_expanded);
                    node_expanded->parent_ = cur_node;
                    open_set_.push(node_expanded);
                }
            } else {
                auto node = createNode(neighbor_voxel, cur_node, g_score);
                open_set_.push(node);
                expanded_nodes_.insert(node->voxel, node);
            }
        }
    }

    return false;
}

void PathSearch::retrievePath(const PathNodePtr &end_node_ptr) {
    auto cur_node_ptr = end_node_ptr;
    while (cur_node_ptr) {
        auto node_pos = map_bridge_ptr_->voxel2Pos(cur_node_ptr->voxel);
        discrete_path_.push_back(node_pos);
        path_nodes_.push_back(cur_node_ptr);
        cur_node_ptr = cur_node_ptr->parent_;
    }

    std::reverse(path_nodes_.begin(), path_nodes_.end());
    std::reverse(discrete_path_.begin(), discrete_path_.end());
}

void PathSearch::reset() {
    open_set_ = std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>();
    closed_set_.clear();
    expanded_nodes_.clear();
    path_nodes_.clear();
    discrete_path_.clear();
}

Eigen::Vector3i PathSearch::findClosestFreeVoxel(const Eigen::Vector3i &start_voxel,
                                                 const Eigen::Vector3i &end_voxel) const {
    // we still 2d case for 3d voxel currently
    int x0 = start_voxel.x(), y0 = start_voxel.y();
    int x1 = end_voxel.x(), y1 = end_voxel.y();

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    auto last_free_voxel = Eigen::Vector3i(x0, y0, ground_index_);
    Eigen::Vector3i start_ground_voxel(x0, y0, ground_index_);  // for 2d case

    while (x0 != x1 || y0 != y1) {
        Eigen::Vector3i cur_voxel(x0, y0, ground_index_);

        if (!map_bridge_ptr_->isVoxelValid(cur_voxel) || (cur_voxel - start_ground_voxel).norm() > search_radius_) {
            return last_free_voxel;
        }

        if (isInflateOccupied(cur_voxel)) {
            return start_voxel;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
        last_free_voxel = cur_voxel;  // update last free voxel
    }

    if (isInflateOccupied(Eigen::Vector3i(end_voxel.x(), end_voxel.y(), ground_index_))) {
        return start_voxel;
    } else {
        return end_voxel;
    }
}

bool PathSearch::isVisible(const Eigen::Vector3i &start_voxel, const Eigen::Vector3i &end_voxel) const {
    auto farthest_free_voxel = findClosestFreeVoxel(start_voxel, end_voxel);
    return farthest_free_voxel == end_voxel;
}

std::vector<Eigen::Vector3i> PathSearch::getSquareEdgeVoxels(const Eigen::Vector3i &center_voxel,
                                                             int radius) {
    std::vector<Eigen::Vector3i> edge_voxels;
    int x = center_voxel.x();
    int y = center_voxel.y();
    int z = center_voxel.z();  // for 2d case

    // Top and bottom edges
    for (int i = x - radius; i <= x + radius; ++i) {
        edge_voxels.emplace_back(i, y - radius, z);
        edge_voxels.emplace_back(i, y + radius, z);
    }

    // Left and right edges (excluding corners)
    for (int j = y - radius + 1; j < y + radius; ++j) {
        edge_voxels.emplace_back(x - radius, j, z);
        edge_voxels.emplace_back(x + radius, j, z);
    }

    return edge_voxels;
}

bool PathSearch::visibilitySearch(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &end_pos) {
    // init planning params
    start_voxel_ = map_bridge_ptr_->pos2Voxel(start_pos);
    end_voxel_ = map_bridge_ptr_->pos2Voxel(end_pos);
    start_voxel_[2] = ground_index_;
    end_voxel_[2] = ground_index_;

    // Performance tracking
    auto search_start_time = std::chrono::high_resolution_clock::now();
    int expanded_nodes_count = 0;

    // create the start node
    auto start_node = createNode(start_voxel_, nullptr, 0.0);
    open_set_.push(start_node);
    expanded_nodes_.insert(start_node->voxel, start_node);

    // Reduce search radius for performance in complex scenarios
    int effective_search_radius = std::min(search_radius_, 30);

    while (!open_set_.empty()) {
        // Performance check - timeout
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double>(current_time - search_start_time).count();
        if (elapsed > max_search_time_) {
            std::cout << "[PathSearch]: Search timeout after " << elapsed << "s, expanded " 
                      << expanded_nodes_count << " nodes" << std::endl;
            return false;
        }

        // Performance check - node limit
        if (expanded_nodes_count > max_expanded_nodes_) {
            std::cout << "[PathSearch]: Node limit reached (" << max_expanded_nodes_ 
                      << "), stopping search" << std::endl;
            return false;
        }

        auto cur_node = open_set_.top();
        open_set_.pop();
        closed_set_.insert(cur_node->voxel);
        expanded_nodes_count++;

        if (isGoal(cur_node)) {
            retrievePath(cur_node);
            return true;
        }

        bool is_edge = false;
        for (int i = 1; i <= effective_search_radius; i++) {
            if (i == effective_search_radius) {
                is_edge = true;
            }

            // check if there is no need for search undercurrent radius
            auto far_free_voxel = findClosestFreeVoxel(cur_node->voxel, end_voxel_);
            if (far_free_voxel != cur_node->voxel) {
                double temp_g = cur_node->g_score_ + (far_free_voxel - cur_node->voxel).norm();
                auto far_node = createNode(far_free_voxel, cur_node, temp_g);
                if (isGoal(far_free_voxel)) {
                    retrievePath(far_node);
                    return true;
                }
                open_set_.push(far_node);
                expanded_nodes_.insert(far_free_voxel, far_node);
                break;
            }

            auto square_voxels = getSquareEdgeVoxels(cur_node->voxel, i);
            for (const auto & neighbor_voxel : square_voxels) {
                if (!map_bridge_ptr_->isVoxelValid(neighbor_voxel) ||
                    isInflateOccupied(neighbor_voxel) ||
                    closed_set_.find(neighbor_voxel) != closed_set_.end()) {
                    continue;
                }

                // goal check
                if (isGoal(neighbor_voxel) && isVisible(cur_node->voxel, neighbor_voxel)) {
                    double temp_g = cur_node->g_score_ + (neighbor_voxel - cur_node->voxel).norm();
                    auto node = createNode(neighbor_voxel, cur_node, temp_g);
                    retrievePath(node);
                    return true;
                }

                if (!isCorner(const_cast<Eigen::Vector3i &>(neighbor_voxel), is_edge) &&
                    !isGoal(neighbor_voxel)) {
                    closed_set_.insert(neighbor_voxel);
                    continue;
                }

                if (isVisible(cur_node->voxel, neighbor_voxel)) {
                    double temp_g = cur_node->g_score_ + (neighbor_voxel - cur_node->voxel).norm();
                    // prune
                    auto node_expanded = expanded_nodes_.find(neighbor_voxel);
                    if (node_expanded) {
                        if (temp_g < node_expanded->g_score_) {
                            node_expanded->g_score_ = temp_g;
                            node_expanded->f_score_ = temp_g + getHeuristic(node_expanded);
                            node_expanded->parent_ = cur_node;
                            open_set_.push(node_expanded);
                        }
                    } else {
                        auto node = createNode(neighbor_voxel, cur_node, temp_g);
                        open_set_.push(node);
                        expanded_nodes_.insert(node->voxel, node);
                    }
                }
            }
        }
    }

    return false;
}

bool PathSearch::detailedCornerCheck(Eigen::Vector3i &voxel, bool is_edge) const {
    if (is_edge && map_bridge_ptr_->getDistance(voxel) <= edge_threshold_) {
        return true;
    }

    int occupied_nums = 0;
    bool off_edge = true;
    for (const auto & dir : voxel_directions_) {
        Eigen::Vector3i neighbor_voxel = voxel + dir;
        if (!map_bridge_ptr_->isVoxelValid(neighbor_voxel)) {
            off_edge = false;
        }
        if (isInflateOccupied(neighbor_voxel)) {
            occupied_nums++;
        }
    }

    if (occupied_nums == 1 && off_edge) {
        return true;
    }
    return false;
}

bool PathSearch::isCorner(Eigen::Vector3i &voxel, bool is_edge) const {
    if (map_bridge_ptr_->getDistance(voxel) > edge_threshold_) {
        return false;
    }

    if (detailedCornerCheck(voxel, is_edge)) {
        return true;
    }
    return false;
}

bool PathSearch::isGoal(const Eigen::Vector3i &voxel) {
    return voxel == end_voxel_;
}

bool PathSearch::isGoal(const PathNodePtr& node_ptr) {
    return node_ptr->voxel == end_voxel_;
}

double PathSearch::getHeuristic(const PathNodePtr& node_ptr) const {
    return (node_ptr->voxel - end_voxel_).norm();
}

std::vector<Eigen::Vector3d> PathSearch::getDiscretePath() const {
    return discrete_path_;
}

bool PathSearch::isInflateOccupied(const Eigen::Vector3i &voxel) const {
//    return map_bridge_ptr_->isInflateOccupied(voxel, collision_threshold_);

    // this one is faster than the above one
    return map_bridge_ptr_->getDistance(voxel) <= collision_threshold_;
}

std::shared_ptr<PathNode> PathSearch::createNode(const Eigen::Vector3i &voxel,
                                                 const std::shared_ptr<PathNode> &parent, double g_cost) const {
    auto node = std::make_shared<PathNode>();
    node->voxel = voxel;
    node->g_score_ = g_cost;
    node->h_score_ = getHeuristic(node);
    node->f_score_ = node->g_score_ + node->h_score_;
    node->parent_ = parent;
    return node;
}
