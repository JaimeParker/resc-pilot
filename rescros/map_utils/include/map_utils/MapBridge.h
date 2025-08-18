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
 * Created by Zhaohong Liu on 24-9-23.
 * Ref: https://github.com/HKUST-Aerial-Robotics/Fast-Planner/tree/master/fast_planner/plan_env/src
*/

#ifndef MAP_UTILS_MAPBRIDGE_H
#define MAP_UTILS_MAPBRIDGE_H

#include <optional>
#include <cassert>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <random>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "map_utils/ObstacleGenerator.h"
#include "map_utils/MappingParams.h"
#include "map_utils/MappingData.h"
#include "map_utils/SDFMap.h"

class MapBridge {
public:
    explicit MapBridge(const ros::NodeHandle &nh) : nh_(nh) {
        drone_pos_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    }

    void init();
    void initObsGenerator();
    void initSDFMap();

    [[nodiscard]] bool isObsValid(double x, double y, double length, double width) const;
    static double genRandomNumber(double range);
    static double genRandomNumber(double low, double high);
    void setRandomObstacles();
    [[maybe_unused]] void setDefaultObstacles();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    [[nodiscard]] bool isInMap(const Eigen::Vector3d & pos) const;
    [[nodiscard]] bool isVoxelValid(const Eigen::Vector3i & voxel) const;
    void initMapping();
    void markOccupied(const Eigen::Vector3i & voxel);
    void setOccupiedRect(double x, double y, double length, double width);
    void updateGlobalSDFMap();
    Eigen::MatrixXd getLocalSDFMap(const int& local_half_size);
    void cvEDTransform(const cv::Mat & occ_map, cv::Mat & sdf_map) const;
    [[maybe_unused]] [[nodiscard]] inline double getDistance(const Eigen::Vector3i & voxel) const;
    [[nodiscard]] inline double getDistance(const Eigen::Vector3d & pos) const;
    [[nodiscard]] cv::Mat getGlobalSDFCVMat() const;
    void showGlobalESDFMap(int zoom_factor) const;

    [[maybe_unused]] [[nodiscard]] bool isOccupied(const Eigen::Vector3i & voxel) const;
    [[nodiscard]] bool isOccupied(const Eigen::Vector3d & pos) const;
    [[nodiscard]] bool isInflateOccupied(const Eigen::Vector3i & voxel,
                                         const std::optional<double>& thresh = std::nullopt) const;
    [[maybe_unused]] [[nodiscard]] bool isInflateOccupied(const Eigen::Vector3d & pos,
                                                          const std::optional<double>& thresh = std::nullopt) const;
    [[nodiscard]] bool isInflatedArea(const Eigen::Vector3d & pos) const;

    [[maybe_unused]] [[nodiscard]] double getCollisionThreshold() const {return mp_.collision_threshold_;};
    void mapBoundTimerCallback(const ros::TimerEvent & /* event */);

    /* local sensing */
    void updateInflatedObstacle();
    [[maybe_unused]] void updateLocalMap();
    [[nodiscard]] inline bool isInSensingRange(const Eigen::Vector3d & querying_pos) const;

    /* publish pcl */
    void publishGlobalPCL();
    void publishInflatedObstaclePCL();
    [[maybe_unused]] void publishLocalPCL();

    [[nodiscard]] inline int voxel2BufferIndex(const Eigen::Vector3i & voxel) const;
    inline int voxel2BufferIndex(const int & x, const int & y, const int & z) const;
    [[nodiscard]] Eigen::Vector3i pos2Voxel(const Eigen::Vector3d & pos) const;
    [[nodiscard]] Eigen::Vector3d voxel2Pos(const Eigen::Vector3i & voxel) const;
    void clampVoxel(Eigen::Vector3i & voxel) const;

    [[nodiscard]] double getResolution() const;
    [[nodiscard]] double getGroundHeight() const;
    [[nodiscard]] int getGroundIndex() const;

    void resetGlobalCloud();
    void resetLocalCloud();
    void resetInflatedCloud();

    [[maybe_unused]] void setGeoFencePcl(const double & fence_size_x, const double & fence_size_y);
    [[maybe_unused]] void resetObstacleRectSize(const double obs_length_max, const double obs_length_min) {
        obs_length_max_ = obs_length_max;
        obs_length_min_ = obs_length_min;
    }

    [[maybe_unused]] void setNumObstaclesManually(const int num_obs) { num_obs_ = num_obs; }

    // set god view to have a global map
    [[maybe_unused]] void setGodView(const bool has_god_view) { mp_.has_god_view_ = has_god_view; }
    [[maybe_unused]] void overrideRealSDF(const bool use_sdf) { use_real_sdf_ = use_sdf; }

    template<typename T>
    static void getParamWithWarning(ros::NodeHandle& nh, const std::string& param_name, T& param) {
        if (!nh.getParam(param_name, param)) {
            ROS_WARN_STREAM("Failed to get param: " << param_name);
        }
    }

    void markObstacle(const Eigen::Vector3d & start, const Eigen::Vector3d & end);
    [[maybe_unused]] void raycast(const Eigen::Vector3d & pos, const Eigen::Vector3d& att);
    static Eigen::Matrix3d getRotB2W(const Eigen::Vector3d & att);
    [[maybe_unused]] [[nodiscard]] bool getGodViewStatus() const { return mp_.has_god_view_; }

private:
    /* ros utils */
    ros::NodeHandle nh_;
    nav_msgs::OccupancyGrid global_occupancy_grid_;
    ros::Subscriber drone_pose_sub_;
    std::string drone_pose_sub_topic_ = "/mavros/local_position/pose";
    ros::Timer map_bound_timer_;
    double map_bound_timer_period_ = 0.5;
    ros::Publisher global_pcl_pub_;
    ros::Publisher inflated_map_pub_;
    ros::Publisher local_pcl_pub_;
    std::string global_pcd_frame_id_ = "world";
    std::string global_pcl_pub_topic_ = "/map_utils/global_cloud";
    std::string inflated_obstacle_pcl_pub_topic_ = "/map_utils/inflated_obstacle_cloud";
    std::string local_pcl_pub_topic_ = "/map_utils/local_cloud";

    /* drone state */
    bool has_pose_ = false;
    Eigen::Vector3d drone_pos_;
    Eigen::Vector3d last_pos_;
    Eigen::Vector3d drone_att_;

    /* obstacle generator */
    ObstacleGenerator obs_gen_;

    /* map properties */
    MappingParams mp_;
    MappingData md_;
    bool has_global_map_ = false;
    bool has_global_sdf_ = false;
    bool sdf_need_update_ = false;
    bool local_sense_update_ = true;
    bool use_real_sdf_ = false;
    int num_obs_ = 0;
    double obs_length_max_ = 0.8;
    double obs_length_min_ = 0.2;
    double obs_height_ = 1.0;
    const double epsilon_ = 1e-6;

    /* camera params */
    double fov_h_ = 120 * M_PI / 180;
    double raycast_range_ = 5.0;

    /* esdf computation */
    SDFMap::Ptr sdf_map_ptr_;
public:
    using Ptr = std::shared_ptr<MapBridge>;
};

/* inline functions */
inline int MapBridge::voxel2BufferIndex(const Eigen::Vector3i &voxel) const{
    return voxel(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
           voxel(1) * mp_.map_voxel_num_(2) +
           voxel(2);
}

inline int MapBridge::voxel2BufferIndex(const int & x, const int & y, const int & z) const{
    return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
           y * mp_.map_voxel_num_(2) +
           z;
}

[[maybe_unused]]
inline double MapBridge::getDistance(const Eigen::Vector3i &voxel) const {
    if (!use_real_sdf_)
        return md_.global_sdf_buffer_[voxel2BufferIndex(voxel)];
    return std::min(sdf_map_ptr_->getDistance(voxel2Pos(voxel)), mp_.camera_range_max_);
}

inline double MapBridge::getDistance(const Eigen::Vector3d &pos) const {
    if (!use_real_sdf_)
        return md_.global_sdf_buffer_[voxel2BufferIndex(pos2Voxel(pos))];
    return std::min(sdf_map_ptr_->getDistance(pos), mp_.camera_range_max_);
}

inline bool MapBridge::isInSensingRange(const Eigen::Vector3d &querying_pos) const {
    return (querying_pos.head(2) - drone_pos_.head(2)).norm() <= mp_.sensing_range_;
}

#endif //MAP_UTILS_MAPBRIDGE_H
