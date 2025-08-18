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

#ifndef FRONTEND_SEARCHER_PATHSEARCH_H
#define FRONTEND_SEARCHER_PATHSEARCH_H

#include <Eigen/Eigen>
#include <vector>
#include <queue>
#include <memory>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <chrono>

#include "map_utils/MapBridge.h"

#define EPSILON 1e-6

class PathNode {
public:
    Eigen::Vector3i voxel;
    double g_score_, h_score_, f_score_;
    std::shared_ptr<PathNode> parent_;

    PathNode() {
        parent_ = nullptr;
        g_score_ = std::numeric_limits<double>::infinity();
        h_score_ = std::numeric_limits<double>::infinity();
        f_score_ = std::numeric_limits<double>::infinity();
    }
};

using PathNodePtr = std::shared_ptr<PathNode>;

class NodeComparator {
public:
    bool operator()(const PathNodePtr& node1, const PathNodePtr& node2) {
        return node1->f_score_ > node2->f_score_;
    }
};

template <typename T>
struct MatHash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
                    (seed >> 2);
        }
        return seed;
    }
};

class NodeHaseTable {
private:
    std::unordered_map<Eigen::Vector3i, PathNodePtr, MatHash<Eigen::Vector3i>> data_3d_;
public:
    NodeHaseTable() = default;
    ~NodeHaseTable() = default;
    void insert(const Eigen::Vector3i & voxel, const PathNodePtr & node) {
        data_3d_.insert(std::make_pair(voxel, node));
    }
    PathNodePtr find(const Eigen::Vector3i & voxel) {
        auto iter = data_3d_.find(voxel);
        if (iter != data_3d_.end()) {
            return iter->second;
        }
        return nullptr;
    }
    void clear() {
        data_3d_.clear();
    }
};

class PathSearch {
private:
    /* planning structure */
    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;
    std::unordered_set<Eigen::Vector3i, MatHash<Eigen::Vector3i>> closed_set_;
    NodeHaseTable expanded_nodes_;
    std::vector<PathNodePtr> path_nodes_;
    std::vector<Eigen::Vector3d> discrete_path_;

    /* record data */
    Eigen::Vector3i start_voxel_, end_voxel_;
    MapBridge::Ptr map_bridge_ptr_;

    /* map */
    double resolution_ = 0.1;
    int ground_index_ = 5;
    double search_range_ = 6.0;
    int search_radius_ = -1;
    double collision_threshold_ = 0.4 + EPSILON;
    double edge_threshold_ = 0.5;
    
    /* performance limits */
    double max_search_time_ = 1.0;  // Maximum search time in seconds
    int max_expanded_nodes_ = 1000; // Maximum nodes to expand

    /* planning data */
    std::vector<Eigen::Vector3i> voxel_directions_;

public:
    PathSearch();
    [[nodiscard]] std::shared_ptr<PathNode> createNode(const Eigen::Vector3i& voxel,
                                                       const std::shared_ptr<PathNode>& parent,
                                                       double g_cost) const;
    void setMapBridge(const MapBridge::Ptr& ptr);
    [[maybe_unused]] bool aStarSearch(const Eigen::Vector3d &start_pos,
                                      const Eigen::Vector3d &end_pos);
    void retrievePath(const PathNodePtr & end_node_ptr);
    void reset();

    bool isGoal(const Eigen::Vector3i & voxel);
    bool isGoal(const PathNodePtr& node_ptr);
    [[nodiscard]] double getHeuristic(const PathNodePtr& node_ptr) const;
    std::vector<Eigen::Vector3d> getDiscretePath() const;

    bool isInflateOccupied(const Eigen::Vector3i & voxel) const;
    Eigen::Vector3i findClosestFreeVoxel(const Eigen::Vector3i & start_voxel,
                                         const Eigen::Vector3i & end_voxel) const;
    bool isVisible(const Eigen::Vector3i & start_voxel, const Eigen::Vector3i & end_voxel) const;
    static std::vector<Eigen::Vector3i> getSquareEdgeVoxels(const Eigen::Vector3i & center_voxel,
                                                            int radius);
    bool detailedCornerCheck(Eigen::Vector3i & voxel, bool is_edge) const;
    bool isCorner(Eigen::Vector3i & voxel, bool is_edge) const;
    [[maybe_unused]] bool visibilitySearch(const Eigen::Vector3d & start_pos, const Eigen::Vector3d & end_pos);

public:
    using Ptr = std::shared_ptr<PathSearch>;
};


#endif //FRONTEND_SEARCHER_PATHSEARCH_H
