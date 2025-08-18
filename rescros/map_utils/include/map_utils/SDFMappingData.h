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

#ifndef SDFMAPPINGDATA_H
#define SDFMAPPINGDATA_H

#include <Eigen/Eigen>
#include <vector>
#include <queue>
#include <cv_bridge/cv_bridge.h>

class SDFMappingData {
public:
    // main map data, occupancy of each voxel and Euclidean distance

    std::vector<double> occupancy_buffer_;
    std::vector<char> occupancy_buffer_neg;
    std::vector<char> occupancy_buffer_inflate_;
    std::vector<double> distance_buffer_;
    std::vector<double> distance_buffer_neg_;
    std::vector<double> distance_buffer_all_;
    std::vector<double> tmp_buffer1_;
    std::vector<double> tmp_buffer2_;

    // camera position and pose data

    Eigen::Vector3d camera_pos_, last_camera_pos_;
    Eigen::Quaterniond camera_q_, last_camera_q_;

    // depth image data

    cv::Mat depth_image_, last_depth_image_;
    int image_cnt_;

    // flags of map state

    bool occ_need_update_, local_updated_, esdf_need_update_;
    bool has_first_depth_;
    bool has_odom_, has_cloud_;

    // depth image projected point cloud

    std::vector<Eigen::Vector3d> proj_points_;
    int proj_points_cnt;

    // flag buffers for speeding up raycasting

    std::vector<short> count_hit_, count_hit_and_miss_;
    std::vector<char> flag_traverse_, flag_rayend_;
    char raycast_num_;
    std::queue<Eigen::Vector3i> cache_voxel_;

    // range of updating ESDF

    Eigen::Vector3i local_bound_min_, local_bound_max_;

    // computation time

    double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
    int update_num_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif //SDFMAPPINGDATA_H
