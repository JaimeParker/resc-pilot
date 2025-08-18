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

/**
* Modifications made to Fast-Planner:
*
* - Reformat the code to follow the Google C++ Style Guide.
*
* Modified by Zhaohong Liu <jaimefriedhelmzhao@gmail.com>
* Organization: Shanghai Jiao Tong University
* Date: 2024-12-22
*/

#ifndef MAP_UTILS_SDF_MAP_H
#define MAP_UTILS_SDF_MAP_H


#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include "map_utils/RayCaster.h"
#include "map_utils/SDFMappingParameters.h"
#include "map_utils/SDFMappingData.h"

#define m_logit(x) (log((x) / (1 - (x))))

// voxel hashing
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class SDFMap {
    /* map */
    SDFMappingParameters mp_;
    SDFMappingData md_;

    /* ros utils */
    using SyncPolicyImageOdom = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>;
    using SyncPolicyImagePose = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>;
    using SynchronizerImagePose = std::shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>>;
    using SynchronizerImageOdom = std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>>;

    ros::NodeHandle node_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    SynchronizerImagePose sync_image_pose_;
    SynchronizerImageOdom sync_image_odom_;

    ros::Subscriber indep_depth_sub_, indep_odom_sub_, indep_pose_sub_, indep_cloud_sub_;
    ros::Publisher map_pub_, esdf_pub_, map_inf_pub_, update_range_pub_;
    ros::Publisher unknown_pub_, depth_pub_;
    ros::Timer occ_timer_, esdf_timer_, vis_timer_;

    std::string depth_topic_ = "/camera/aligned_depth_to_color/image_raw";
    std::string pose_topic_ = "/mavros/local_position/pose";

    /* random */
    std::uniform_real_distribution<> rand_noise_;
    std::normal_distribution<> rand_noise2_;
    std::default_random_engine eng_;
public:
    SDFMap() = default;
    enum { POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000 };

    // occupancy map management
    void resetBuffer();
    void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

    inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
    inline int toAddress(const Eigen::Vector3i& id);
    inline int toAddress(const int & x, const int & y, const int & z);
    inline bool isInMap(const Eigen::Vector3d& pos);
    inline bool isInMap(const Eigen::Vector3i& idx);

    inline void setOccupancy(const Eigen::Vector3d &pos, double occ = 1);
    inline void setOccupied(const Eigen::Vector3d &pos);
    inline int getOccupancy(const Eigen::Vector3d &pos);
    inline int getOccupancy(Eigen::Vector3i id);
    inline int getInflateOccupancy(const Eigen::Vector3d &pos);

    inline void boundIndex(Eigen::Vector3i& id);
    inline bool isUnknown(const Eigen::Vector3i& id);
    inline bool isUnknown(const Eigen::Vector3d& pos);
    inline bool isKnownFree(const Eigen::Vector3i& id);
    inline bool isKnownOccupied(const Eigen::Vector3i& id);

    // distance field management
    inline double getDistance(const Eigen::Vector3d& pos);
    inline double getDistance(const Eigen::Vector3i& id);
    inline double getDistWithGradTrilinear(const Eigen::Vector3d &pos, Eigen::Vector3d& grad);
    void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff);

    void updateESDF3d();
    void getSliceESDF(double height, double res, const Eigen::Vector4d& range,
                      std::vector<Eigen::Vector3d>& slice, std::vector<Eigen::Vector3d>& grad,
                      int sign = 1);  // 1 pos, 2 neg, 3 combined
    void initMap(const ros::NodeHandle& nh);

    void publishMap();
    void publishMapInflate(bool all_info = false);
    void publishESDF();
    void publishUpdateRange();

    void publishUnknown();
    void publishDepth();

    void checkDist();
    bool hasDepthObservation();
    bool odomValid();
    void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
    double getResolution();
    Eigen::Vector3d getOrigin();
    int getVoxelNum();

    using Ptr = std::shared_ptr<SDFMap>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

    // get depth image and camera pose
    void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                           const geometry_msgs::PoseStampedConstPtr& pose);
    void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
    void depthCallback(const sensor_msgs::ImageConstPtr& img);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);
    void odomCallback(const nav_msgs::OdometryConstPtr& odom);

    // update occupancy by raycasting, and update ESDF
    void updateOccupancyCallback(const ros::TimerEvent& /*event*/);
    void updateESDFCallback(const ros::TimerEvent& /*event*/);
    void visCallback(const ros::TimerEvent& /*event*/);

    // main update process
    void projectDepthImage();
    void raycastProcess();
    void clearAndInflateLocalMap();

    inline void inflatePoint(const Eigen::Vector3i& pt, int step, std::vector<Eigen::Vector3i>& pts);
    int setCacheOccupancy(Eigen::Vector3d pos, int occ);
    Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);
};

inline int SDFMap::toAddress(const Eigen::Vector3i& id) {
    return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int SDFMap::toAddress(const int & x, const int & y, const int & z) {
    return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void SDFMap::boundIndex(Eigen::Vector3i& id) {
    Eigen::Vector3i id1;
    id1(0) = std::max(std::min(id(0), mp_.map_voxel_num_(0) - 1), 0);
    id1(1) = std::max(std::min(id(1), mp_.map_voxel_num_(1) - 1), 0);
    id1(2) = std::max(std::min(id(2), mp_.map_voxel_num_(2) - 1), 0);
    id = id1;
}

inline double SDFMap::getDistance(const Eigen::Vector3d& pos) {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    boundIndex(id);

    return md_.distance_buffer_all_[toAddress(id)];
}

inline double SDFMap::getDistance(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    return md_.distance_buffer_all_[toAddress(id1)];
}

inline bool SDFMap::isUnknown(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool SDFMap::isUnknown(const Eigen::Vector3d& pos) {
    Eigen::Vector3i idc;
    posToIndex(pos, idc);
    return isUnknown(idc);
}

inline bool SDFMap::isKnownFree(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    const int adr = toAddress(id1);

    // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
    //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
    return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool SDFMap::isKnownOccupied(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    const int adr = toAddress(id1);

    return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline double SDFMap::getDistWithGradTrilinear(const Eigen::Vector3d &pos, Eigen::Vector3d& grad) {
    if (!isInMap(pos)) {
        grad.setZero();
        return 0;
    }

    /* use trilinear interpolation */
    const Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();

    Eigen::Vector3i idx;
    posToIndex(pos_m, idx);

    Eigen::Vector3d idx_pos;
    indexToPos(idx, idx_pos);

    Eigen::Vector3d diff = (pos - idx_pos) * mp_.resolution_inv_;

    double values[2][2][2];
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            for (int z = 0; z < 2; z++) {
                Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                values[x][y][z] = getDistance(current_idx);
            }
        }
    }

    const double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
    const double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
    const double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
    const double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
    const double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
    const double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
    const double dist = (1 - diff[2]) * v0 + diff[2] * v1;

    grad[2] = (v1 - v0) * mp_.resolution_inv_;
    grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
    grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
    grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
    grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
    grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

    grad[0] *= mp_.resolution_inv_;

    return dist;
}

inline void SDFMap::setOccupied(const Eigen::Vector3d &pos) {
    if (!isInMap(pos)) return;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                  id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
}

inline void SDFMap::setOccupancy(const Eigen::Vector3d &pos, const double occ) {
    if (occ != 1 && occ != 0) {
        std::cout << "occ value error!" << std::endl;
        return;
    }

    if (!isInMap(pos)) return;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    md_.occupancy_buffer_[toAddress(id)] = occ;
}

inline int SDFMap::getOccupancy(const Eigen::Vector3d &pos) {
    if (!isInMap(pos)) return -1;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3d &pos) {
    if (!isInMap(pos)) return -1;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    return md_.occupancy_buffer_inflate_[toAddress(id)];
}

inline int SDFMap::getOccupancy(Eigen::Vector3i id) {
    if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
        id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
        return -1;

    return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool SDFMap::isInMap(const Eigen::Vector3d& pos) {
    if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
        pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
        // cout << "less than min range!" << endl;
        return false;
    }
    if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
        pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
        return false;
    }
    return true;
}

inline bool SDFMap::isInMap(const Eigen::Vector3i& idx) {
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
        return false;
    }
    if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
        idx(2) > mp_.map_voxel_num_(2) - 1) {
        return false;
    }
    return true;
}

inline void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void SDFMap::inflatePoint(const Eigen::Vector3i& pt, const int step, std::vector<Eigen::Vector3i>& pts) {
    int num = 0;

    /* ---------- + shape inflate ---------- */
    // for (int x = -step; x <= step; ++x)
    // {
    //   if (x == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
    // }
    // for (int y = -step; y <= step; ++y)
    // {
    //   if (y == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
    // }
    // for (int z = -1; z <= 1; ++z)
    // {
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
    // }

    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
        for (int y = -step; y <= step; ++y)
            for (int z = -step; z <= step; ++z) {
                pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
            }
}


#endif //MAP_UTILS_SDF_MAP_H
