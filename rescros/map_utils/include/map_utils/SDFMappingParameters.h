/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SDFMAPPINGPARAMETERS_H
#define SDFMAPPINGPARAMETERS_H

#include <Eigen/Eigen>

class SDFMappingParameters {
public:
    /* map properties */
    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
    Eigen::Vector3i map_voxel_num_;                        // map range in index
    Eigen::Vector3i map_min_idx_, map_max_idx_;
    Eigen::Vector3d local_update_range_;
    double resolution_, resolution_inv_;
    double obstacles_inflation_;
    std::string frame_id_;
    int pose_type_;
    std::string map_input_;  // 1: pose+depth; 2: odom + cloud

    /* camera parameters */
    double cx_, cy_, fx_, fy_;

    /* depth image projection filtering */
    double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
    int depth_filter_margin_;
    bool use_depth_filter_;
    double k_depth_scaling_factor_;
    int skip_pixel_;

    /* raycasting */
    double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
            min_occupancy_log_;                   // logit of occupancy probability
    double min_ray_length_, max_ray_length_;  // range of doing raycasting

    /* local map update and clear */
    double local_bound_inflate_;
    int local_map_margin_;

    /* visualization and computation time display */
    double esdf_slice_height_, visualization_truncate_height_, virtual_ceil_height_, ground_height_;
    bool show_esdf_time_, show_occ_time_;

    /* active mapping */
    double unknown_flag_;
};

#endif //SDFMAPPINGPARAMETERS_H
