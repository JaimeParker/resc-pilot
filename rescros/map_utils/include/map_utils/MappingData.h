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
 * Created by Zhaohong Liu on 24-12-22.
*/

#ifndef MAPPINGDATA_H
#define MAPPINGDATA_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <unordered_set>

struct PointHash {
  std::size_t operator()(const std::pair<int, int>& key) const {
    return std::hash<int>()(key.first) ^ (std::hash<int>()(key.second) << 1);
  }
};

struct PointEqual {
  bool operator()(const std::pair<int, int>& a, const std::pair<int, int>& b) const {
    return a.first == b.first && a.second == b.second;
  }
};

class MappingData {
public:
  /*
   * global sdf: 必须是预先加载的已知地图，或是在仿真环境下生成的随机地图生成的esdf地图，否则对于sdf的查询将不会涉及此地图
   * local sdf: 在仿真环境下，根据不断地raycast来观测global中的sdf值，更新到local中；在真实环境下，local map的更新需要如fiesta的方法
   */
  std::vector<int> global_occupancy_buffer_;
  std::vector<int> local_occupancy_buffer_;
  std::vector<double> global_sdf_buffer_;
  std::vector<double> local_sdf_buffer_;  // update by raycast

  cv::Mat cvm_global_occupancy_map_;
  cv::Mat cvm_global_sdf_map_;

  pcl::PointCloud<pcl::PointXYZ> global_cloud_;
  pcl::PointCloud<pcl::PointXYZ> inflated_obstacle_cloud_;
  pcl::PointCloud<pcl::PointXYZ> local_cloud_;

  sensor_msgs::PointCloud2 global_map_pcd_;
  sensor_msgs::PointCloud2 inflated_obstacle_pcd_;
  sensor_msgs::PointCloud2 local_map_pcd_;

  std::unordered_set<std::pair<int, int>, PointHash, PointEqual> local_occupied_set_;
};

#endif //MAPPINGDATA_H
