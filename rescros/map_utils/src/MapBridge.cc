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
 * https://github.com/HKUST-Aerial-Robotics/Fast-Planner/tree/master/fast_planner/plan_env/src
*/

#include "map_utils/MapBridge.h"

void MapBridge::init() {
    getParamWithWarning(nh_, "map/map_size_x", mp_.map_size_x_);
    getParamWithWarning(nh_, "map/map_size_y", mp_.map_size_y_);
    getParamWithWarning(nh_, "map/map_size_z", mp_.map_size_z_);
    getParamWithWarning(nh_, "map/resolution", mp_.resolution_);
    getParamWithWarning(nh_, "map/num_obstacles", num_obs_);

    mp_.map_origin_ << -mp_.map_size_x_ / 2, -mp_.map_size_y_ / 2, mp_.ground_height_;

    // set ros utils
    drone_pose_sub_ = nh_.subscribe(drone_pose_sub_topic_, 5, &MapBridge::poseCallback, this);
    map_bound_timer_ = nh_.createTimer(ros::Duration(map_bound_timer_period_), &MapBridge::mapBoundTimerCallback, this);
    global_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(global_pcl_pub_topic_, 5);
    inflated_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(inflated_obstacle_pcl_pub_topic_, 2);
    local_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(local_pcl_pub_topic_, 5);

    // init map settings
    initMapping();

    std::cout << "[Map Bridge]: Init done." << std::endl;
}

void MapBridge::initObsGenerator() {
    obs_gen_ = ObstacleGenerator();
    obs_gen_.setResolution(mp_.resolution_);

    std::cout << "[Map Bridge]: Init obstacle generator." << std::endl;
}

void MapBridge::initSDFMap() {
    sdf_map_ptr_ = std::make_shared<SDFMap>();
    sdf_map_ptr_->initMap(nh_);

    use_real_sdf_ = true;
    std::cout << "[Map Bridge]: Init SDF map." << std::endl;
}

bool MapBridge::isObsValid(double x, double y, double length, double width) const {
    if (x < -mp_.map_size_x_ / 2.0 || x + length > mp_.map_size_x_ / 2.0 ||
        y < -mp_.map_size_y_ / 2.0 || y + width > mp_.map_size_y_ / 2.0) {
        return false;
    }
    return true;
}

double MapBridge::genRandomNumber(double range) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-range, range);
    return dis(gen);
}

double MapBridge::genRandomNumber(double low, double high) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(low, high);
    return dis(gen);
}

void MapBridge::setRandomObstacles() {
    std::cout << "[Map Bridge]: Set random obstacles." << std::endl;
    std::cout << "[Map Bridge]: Number of obstacles: " << num_obs_ << std::endl;
    for (int i = 0; i < num_obs_; i++) {
        const double obs_x = genRandomNumber(mp_.map_size_x_ / 2.0 - mp_.safe_margin_);
        const double obs_y = genRandomNumber(mp_.map_size_y_ / 2.0 - mp_.safe_margin_);
        const double obs_length = genRandomNumber(obs_length_min_, obs_length_max_);
        const double obs_width = genRandomNumber(obs_length_min_, obs_length_max_);

        if (isObsValid(obs_x, obs_y, obs_length, obs_width)) {
            // publish pcl to ros
            obs_gen_.setRectPcl(obs_x, obs_y, obs_length, obs_width, obs_height_,
                                md_.global_cloud_);
            // mark occupied in occupancy grid map
            setOccupiedRect(obs_x, obs_y, obs_length, obs_width);
        }
    }

    has_global_map_ = true;
    std::cout << "\033[1;32mRandom obstacles and global occupancy map are all set.\033[0m" << std::endl;
    updateGlobalSDFMap();
}

[[maybe_unused]]
void MapBridge::setDefaultObstacles() {

    std::vector<std::array<double, 4>> obstacles = {
        {-0.5, 1.0, 0.5, 1.5},
        {0.0, 0.5, 4.0, 0.5},
        {0.0, 2.5, 2.0, 0.5},
        {1.0, 2.5, 1.0, 4.0},
        {4.0, 1.0, 1.0, 3.0},
        {2.0, 5.5, 4.0, 1.0},
        {5.0, 3.0, 4.0, 1.0},
        {8.0, 4.0, 1.0, 4.0},
        {5.0, 6.5, 1.0, 3.5},
        {6.0, 9.5, 3.5, 0.5}
    };

    for (const auto& obs_cfg : obstacles) {
        if (isObsValid(obs_cfg[0], obs_cfg[1], obs_cfg[2], obs_cfg[3])) {
            obs_gen_.setRectPcl(obs_cfg[0], obs_cfg[1], obs_cfg[2], obs_cfg[3], 1.0,
                                md_.global_cloud_);
            setOccupiedRect(obs_cfg[0], obs_cfg[1], obs_cfg[2], obs_cfg[3]);
        }
    }

    has_global_map_ = true;
    std::cout << "\033[1;32mDefault obstacles and global occupancy map are all set.\033[0m" << std::endl;
    updateGlobalSDFMap();
}

bool MapBridge::isInMap(const Eigen::Vector3d &pos) const {
     if (pos(0) < -mp_.map_size_x_ / 2.0 || pos(0) > mp_.map_size_x_ / 2.0 ||
         pos(1) < -mp_.map_size_y_ / 2.0 || pos(1) > mp_.map_size_y_ / 2.0 ||
         pos(2) < mp_.ground_height_ || pos(2) > mp_.map_size_z_) {
         return false;
     }
    return true;
}

bool MapBridge::isVoxelValid(const Eigen::Vector3i &voxel) const {
    if (voxel(0) < 0 || voxel(0) >= mp_.map_voxel_num_(0) ||
        voxel(1) < 0 || voxel(1) >= mp_.map_voxel_num_(1) ||
        voxel(2) < 0 || voxel(2) >= mp_.map_voxel_num_(2)) {
        return false;
    }
    return true;
}

void MapBridge::initMapping() {
    // mapping params
    mp_.map_voxel_num_(0) = static_cast<int>(mp_.map_size_x_ / mp_.resolution_);
    mp_.map_voxel_num_(1) = static_cast<int>(mp_.map_size_y_ / mp_.resolution_);
    mp_.map_voxel_num_(2) = static_cast<int>(mp_.map_size_z_ / mp_.resolution_);
    int global_occ_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
    mp_.half_resolution_ = mp_.resolution_ / 2.0;

    // mapping data
    md_.global_occupancy_buffer_.resize(global_occ_size, mp_.free_threshold_);
    md_.local_occupancy_buffer_.resize(global_occ_size, mp_.free_threshold_);
    md_.global_sdf_buffer_.resize(global_occ_size, mp_.camera_range_max_);
    md_.local_sdf_buffer_.resize(global_occ_size, mp_.camera_range_max_);
}

Eigen::Vector3i MapBridge::pos2Voxel(const Eigen::Vector3d &pos) const {
    Eigen::Vector3i voxel;
    voxel(0) = static_cast<int>((pos(0) - mp_.map_origin_(0)) / mp_.resolution_);
    voxel(1) = static_cast<int>((pos(1) - mp_.map_origin_(1)) / mp_.resolution_);
    voxel(2) = static_cast<int>((pos(2) - mp_.map_origin_(2)) / mp_.resolution_);
    clampVoxel(voxel);
    return voxel;
}

Eigen::Vector3d MapBridge::voxel2Pos(const Eigen::Vector3i &voxel) const {
    Eigen::Vector3d pos;
    pos(0) = voxel(0) * mp_.resolution_ + mp_.map_origin_(0) + mp_.half_resolution_;
    pos(1) = voxel(1) * mp_.resolution_ + mp_.map_origin_(1) + mp_.half_resolution_;
    pos(2) = voxel(2) * mp_.resolution_ + mp_.map_origin_(2);
    return pos;
}

void MapBridge::clampVoxel(Eigen::Vector3i &voxel) const {
    auto clamp = [](int value, int min_val, int max_val) {
        return std::max(min_val, std::min(max_val, value));
    };

    voxel(0) = clamp(voxel(0), 0, mp_.map_voxel_num_(0) - 1);
    voxel(1) = clamp(voxel(1), 0, mp_.map_voxel_num_(1) - 1);
    voxel(2) = clamp(voxel(2), 0, mp_.map_voxel_num_(2) - 1);
}

void MapBridge::markOccupied(const Eigen::Vector3i &voxel) {
    int idx = voxel2BufferIndex(voxel);
    md_.global_occupancy_buffer_[idx] = mp_.occupied_threshold_;
}

void MapBridge::setOccupiedRect(double x, double y, double length, double width) {
    // set z to 0.0 for 2D case
    Eigen::Vector3i start_voxel = pos2Voxel(Eigen::Vector3d(x, y, 0.0));
    Eigen::Vector3i end_voxel = pos2Voxel(Eigen::Vector3d(x + length, y + width, 0.0));

    for (int i = start_voxel(0); i <= end_voxel(0); i++) {
        for (int j = start_voxel(1); j <= end_voxel(1); j++) {
            markOccupied(Eigen::Vector3i(i, j, mp_.ground_index_));
        }
    }
}

void MapBridge::cvEDTransform(const cv::Mat &occ_map, cv::Mat &sdf_map) const {
    cv::distanceTransform(occ_map, sdf_map, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    sdf_map *= mp_.resolution_;
    cv::min(sdf_map, mp_.camera_range_max_, sdf_map);
}

void MapBridge::updateGlobalSDFMap() {
    // TODO: this method will be deprecated after introducing camera simulation,
    //  and replaced by a third-party library for direct ESDF mapping
    if (!has_global_sdf_) {
        int map_width = mp_.map_voxel_num_(0);
        int map_height = mp_.map_voxel_num_(1);
        int map_depth = mp_.map_voxel_num_(2);

        if (md_.cvm_global_occupancy_map_.empty()) {
            md_.cvm_global_occupancy_map_ = cv::Mat(map_height, map_width, CV_8UC1, cv::Scalar(mp_.free_threshold_));
            md_.cvm_global_sdf_map_ = cv::Mat(map_height, map_width, CV_32FC1, cv::Scalar(mp_.camera_range_max_));
        }
        for (int y = 0; y < map_height; y++) {
            for (int x = 0; x < map_width; x++) {
                int idx = voxel2BufferIndex(x, y, mp_.ground_index_);
                md_.cvm_global_occupancy_map_.at<uchar>(map_height -1 - y, x) = md_.global_occupancy_buffer_[idx];
            }
        }

        cvEDTransform(md_.cvm_global_occupancy_map_, md_.cvm_global_sdf_map_);

        for (int y = 0; y < map_height; y++) {
            for (int x = 0; x < map_width; x++) {
                // we are using columns as obstacles, they are identical in z direction
                for (int z = mp_.ground_index_; z < map_depth; z++) {
                    int idx = voxel2BufferIndex(x, y, z);
                    md_.global_sdf_buffer_[idx] = md_.cvm_global_sdf_map_.at<float>(map_height - 1 - y, x);
                }
            }
        }

        ROS_INFO("\033[1;32mGlobal SDF map updated.\033[0m");
        has_global_sdf_ = true;
    }
}

Eigen::MatrixXd MapBridge::getLocalSDFMap(const int& local_half_size) {
    if (!has_global_map_ || !has_pose_ || !sdf_need_update_) {
        ROS_WARN("Not ready for updating local SDF map.");
        return {};
    }
    if (local_half_size <= 0) {
        ROS_ERROR("Local half size should be a positive integer.");
        return {};
    }
    int local_size = 2 * local_half_size + 1;  // make it odd
    Eigen::MatrixXd local_sdf_mat(local_size, local_size);
    for (int i = 0; i < local_size; i++) {
        for (int j = 0; j < local_size; j++) {
            auto temp_pos = Eigen::Vector3d(drone_pos_(0) + i * mp_.resolution_,
                                            drone_pos_(1) + j * mp_.resolution_,
                                            mp_.ground_height_);
            auto voxel = pos2Voxel(temp_pos);
            int idx = voxel2BufferIndex(voxel);

            local_sdf_mat(local_size - 1 - j, i) = md_.global_sdf_buffer_[idx];
        }
    }
    has_pose_ = false;
    sdf_need_update_ = false;

    return local_sdf_mat;
}

cv::Mat MapBridge::getGlobalSDFCVMat() const {
    return md_.cvm_global_sdf_map_;
}

void MapBridge::publishGlobalPCL() {
    if (use_real_sdf_)
        return;

    if (has_global_map_) {
        resetGlobalCloud();
        pcl::toROSMsg(md_.global_cloud_, md_.global_map_pcd_);
        md_.global_map_pcd_.header.frame_id = global_pcd_frame_id_;
        md_.global_map_pcd_.header.stamp = ros::Time::now();
        global_pcl_pub_.publish(md_.global_map_pcd_);
    } else {
        ROS_WARN("No global map PCL to publish, do you set obstacles?");
    }
}

void MapBridge::publishInflatedObstaclePCL() {
    if (use_real_sdf_)
        return;

    resetInflatedCloud();
    pcl::toROSMsg(md_.inflated_obstacle_cloud_, md_.inflated_obstacle_pcd_);
    md_.inflated_obstacle_pcd_.header.frame_id = global_pcd_frame_id_;
    md_.inflated_obstacle_pcd_.header.stamp = ros::Time::now();
    inflated_map_pub_.publish(md_.inflated_obstacle_pcd_);
//    md_.inflated_obstacle_cloud_.clear();
}

[[maybe_unused]]
void MapBridge::publishLocalPCL() {
    if (use_real_sdf_)
        return;

    resetLocalCloud();
    pcl::toROSMsg(md_.local_cloud_, md_.local_map_pcd_);
    md_.local_map_pcd_.header.frame_id = global_pcd_frame_id_;
    md_.local_map_pcd_.header.stamp = ros::Time::now();
    local_pcl_pub_.publish(md_.local_map_pcd_);
}

void MapBridge::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // we only consider 2D case currently
    drone_pos_(0) = msg->pose.position.x;
    drone_pos_(1) = msg->pose.position.y;
    drone_pos_(2) = msg->pose.position.z;

    has_pose_ = true;
    if (isInMap(drone_pos_)) {
        sdf_need_update_ = true;
        if ((drone_pos_ - last_pos_).norm() > 0.1) {
            local_sense_update_ = true;
            last_pos_ = drone_pos_;
        }
    }
}

void MapBridge::showGlobalESDFMap(int zoom_factor) const{
    if (md_.cvm_global_sdf_map_.empty()) {
        std::cout << "Global SDF map is empty." << std::endl;
        return;
    }

    cv::Mat normalized_sdf_map;
    cv::normalize(md_.cvm_global_sdf_map_, normalized_sdf_map, 0, 255, cv::NORM_MINMAX);
    normalized_sdf_map.convertTo(normalized_sdf_map, CV_8UC1);
    cv::Mat zoomed_sdf_map;
    cv::resize(normalized_sdf_map, zoomed_sdf_map, cv::Size(), zoom_factor, zoom_factor, cv::INTER_NEAREST);

    cv::Mat color_mapped_sdf_map;
    cv::applyColorMap(zoomed_sdf_map, color_mapped_sdf_map, cv::COLORMAP_VIRIDIS);

    cv::imshow("Euclidean Signed Distance Map", color_mapped_sdf_map);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

bool MapBridge::isOccupied(const Eigen::Vector3d &pos) const {
    const auto voxel = pos2Voxel(pos);
    const int idx = voxel2BufferIndex(voxel);
    return md_.global_occupancy_buffer_[idx] == mp_.occupied_threshold_;
}

[[maybe_unused]]
bool MapBridge::isOccupied(const Eigen::Vector3i &voxel) const {
    const int idx = voxel2BufferIndex(voxel);
    return md_.global_occupancy_buffer_[idx] == mp_.occupied_threshold_;
}

double MapBridge::getResolution() const{
    return mp_.resolution_;
}

double MapBridge::getGroundHeight() const{
    return mp_.ground_height_;
}

int MapBridge::getGroundIndex() const {
    return mp_.ground_index_;
}

bool MapBridge::isInflateOccupied(const Eigen::Vector3i &voxel, const std::optional<double>& thresh) const {
    if (!isVoxelValid(voxel)) {
        return true;
    }

    const auto pos = voxel2Pos(voxel);
    double temp_thresh = thresh.has_value() ? thresh.value() : mp_.collision_threshold_;

    if (use_real_sdf_) {
        return getDistance(pos) <= temp_thresh + epsilon_;
    }

    if (!mp_.has_god_view_ && !isInSensingRange(pos)) {
        return false;  // 注意，这可能会导致local minima
    }
    // TODO: 具体处理思路见isInflateOccupied(const Eigen::Vector3d & pos)函数

    const int idx = voxel2BufferIndex(voxel);
    assert(idx >= 0 && idx < static_cast<int>(md_.global_sdf_buffer_.size()));
    const auto dist = mp_.has_god_view_ ? md_.global_sdf_buffer_[idx] : md_.local_sdf_buffer_[idx];
    return dist <= temp_thresh + epsilon_;
}

[[maybe_unused]]
bool MapBridge::isInflateOccupied(const Eigen::Vector3d &pos, const std::optional<double>& thresh) const {
    if (!isInMap(pos)) {
        return true;  // occupied while out of the map
    }

    if (!mp_.has_god_view_ && !isInSensingRange(pos)) {
        return false;  // 注意，这可能会导致local minima
    }

    if (!use_real_sdf_) {
        const auto idx = voxel2BufferIndex(pos2Voxel(pos));
        assert(idx >= 0 && idx < static_cast<int>(md_.global_sdf_buffer_.size()));
        const auto dist = mp_.has_god_view_ ? md_.global_sdf_buffer_[idx] : md_.local_sdf_buffer_[idx];
        return dist <= (thresh.value_or(mp_.collision_threshold_) + epsilon_);
    }
    return getDistance(pos) <= (thresh.value_or(mp_.collision_threshold_) + epsilon_);
}

bool MapBridge::isInflatedArea(const Eigen::Vector3d &pos) const {
    // check if the pos is in the inflated area, but not the original obstacle
    const auto voxel = pos2Voxel(pos);
    if (!isVoxelValid(voxel)) {
        return false;
    }

    const int idx = voxel2BufferIndex(voxel);

    if (mp_.has_god_view_) {
        return (0 <= md_.global_sdf_buffer_[idx]) && (md_.global_sdf_buffer_[idx] < mp_.collision_threshold_ - 0.01);
    }
    return (0 <= md_.local_sdf_buffer_[idx]) && (md_.local_sdf_buffer_[idx] < mp_.collision_threshold_ - 0.01);
}

void MapBridge::mapBoundTimerCallback(const ros::TimerEvent &) {
    if (!isInMap(drone_pos_)) {
        std::cout << "\033[1;33m[Map]: Pos out of map: [" << std::fixed << std::setprecision(2) <<
            drone_pos_(0) << ", " << drone_pos_(1) << ", " << drone_pos_(2) << "]\033[0m" << std::endl;
    }
}

void MapBridge::updateInflatedObstacle() {
    if (!local_sense_update_ || use_real_sdf_) {
        return;
    }
    if (!has_global_map_ || !has_pose_) {
        ROS_WARN("Not ready for inflated obstacle update.");
        return;
    }

    // although the local map is used for local planning, it still has the same size of the global map,
    // making it a heavy work load to update it frequently.
    // so we only update the PCL that in sensing range of the drone
    const auto res = static_cast<float>(mp_.resolution_);
    // FIXME: inflated_obstacle_range_ 一旦大于 3.0，会导致 global map 都无法在 rviz 中显示
    const auto half_range = static_cast<float>(mp_.inflated_obstacle_range_ / 2.0);

    const int grid_min_x = static_cast<int>(std::floor((drone_pos_.x() - half_range) / res));
    const int grid_max_x = static_cast<int>(std::floor((drone_pos_.x() + half_range) / res));
    const int grid_min_y = static_cast<int>(std::floor((drone_pos_.y() - half_range) / res));
    const int grid_max_y = static_cast<int>(std::floor((drone_pos_.y() + half_range) / res));

    // create a random number from 0 to 1
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 0.8);

    pcl::PointXYZ pt;

    for (int i = grid_min_x; i < grid_max_x; i++) {
        for (int j = grid_min_y; j < grid_max_y; j++) {

            const float x = static_cast<float>(i) * res + res / 2.0f;
            const float y = static_cast<float>(j) * res + res / 2.0f;

            auto grid_pair = std::make_pair(i, j);

            Eigen::Vector3d temp_pos(x, y, mp_.ground_height_);
            if (!isInMap(temp_pos)) {
                continue;
            }

            if (isInflatedArea(temp_pos)) {
                pt.x = x;
                pt.y = y;
                for (double z = mp_.ground_height_; z < 1.2;) {
                    pt.z = static_cast<float>(z);
                    z += res;
                    md_.inflated_obstacle_cloud_.points.push_back(pt);
                }
            }

            if (getDistance(temp_pos) < 1e-3) {
                if (md_.local_occupied_set_.find(grid_pair) == md_.local_occupied_set_.end()) {
                    pt.x = x;
                    pt.y = y;
                    auto rd_height = dis(gen);
                    for (double z = mp_.ground_height_; z < 1.0 + rd_height;) {
                        pt.z = static_cast<float>(z);
                        z += res;
                        md_.local_cloud_.points.push_back(pt);
                    }
                    md_.local_occupied_set_.insert(grid_pair);
                }

            }

        }
    }

    local_sense_update_ = false;
}

[[maybe_unused]]
void MapBridge::updateLocalMap() {
    if (use_real_sdf_) {
        return;
    }
    // TODO: update local sdf map based on the updated grid map, not values in global sdf map

    const double half_sensing_range = mp_.sensing_range_ / 2.0;
    const double min_x = drone_pos_(0) - half_sensing_range;
    const double max_x = drone_pos_(0) + half_sensing_range;
    const double min_y = drone_pos_(1) - half_sensing_range;
    const double max_y = drone_pos_(1) + half_sensing_range;

    for (double x = min_x; x < max_x; x += mp_.resolution_) {
        for (double y = min_y; y < max_y; y += mp_.resolution_) {
            Eigen::Vector3d temp_pos(x, y, mp_.ground_height_);
            if (!isInMap(temp_pos)) {
                continue;
            }
            auto voxel = pos2Voxel(temp_pos);
            const int idx = voxel2BufferIndex(voxel);
            for (int i = 0; i < mp_.map_voxel_num_(2); i++) {
//                if (md_.global_occupancy_buffer_[idx + i] == mp_.occupied_threshold_) {
//                    md_.local_occupancy_buffer_[idx + i] = mp_.occupied_threshold_;
//                }

                md_.local_sdf_buffer_[idx + i] = md_.global_sdf_buffer_[idx + i];
            }
            // FIXME: 把xy的更新放在for之外，sim250.launch 就卡住了
            // y += mp_.resolution_;
        }
        // x += mp_.resolution_;
    }
}

void MapBridge::resetGlobalCloud() {
    md_.global_cloud_.width = md_.global_cloud_.points.size();
    md_.global_cloud_.height = mp_.cloud_height_;
    md_.global_cloud_.is_dense = true;
}

void MapBridge::resetInflatedCloud() {
    md_.inflated_obstacle_cloud_.width = md_.inflated_obstacle_cloud_.points.size();
    md_.inflated_obstacle_cloud_.height = mp_.cloud_height_;
    md_.inflated_obstacle_cloud_.is_dense = true;
}

void MapBridge::resetLocalCloud() {
    md_.local_cloud_.width = md_.local_cloud_.points.size();
    md_.local_cloud_.height = mp_.cloud_height_;
    md_.local_cloud_.is_dense = true;
}

[[maybe_unused]]
void MapBridge::setGeoFencePcl(const double &fence_size_x, const double &fence_size_y) {
    const double x_min = -fence_size_x / 2.0;
    const double x_max = fence_size_x / 2.0;
    const double y_min = -fence_size_y / 2.0;
    const double y_max = fence_size_y / 2.0;

    obs_gen_.setRectPcl(x_min, y_min, mp_.resolution_, fence_size_y,
                        mp_.ground_height_, md_.global_cloud_, false);
    obs_gen_.setRectPcl(x_max - mp_.resolution_, y_min, mp_.resolution_, fence_size_y,
                        mp_.ground_height_, md_.global_cloud_, false);
    obs_gen_.setRectPcl(x_min, y_min, fence_size_x, mp_.resolution_,
                        mp_.ground_height_, md_.global_cloud_, false);
    obs_gen_.setRectPcl(x_min, y_max - mp_.resolution_, fence_size_x, mp_.resolution_,
                        mp_.ground_height_, md_.global_cloud_, false);

    has_global_map_ = true;
    ROS_INFO("\033[1;32mGeo fence for real flight is set.\033[0m");
}

[[maybe_unused]]
void MapBridge::raycast(const Eigen::Vector3d &pos, const Eigen::Vector3d& att) {
    // TODO: realize real raycast
    if (!isInMap(pos)) {
        return;
    }

     Eigen::Matrix3d rot_mat = getRotB2W(att);
     auto xb_w = getRotB2W(att) * Eigen::Vector3d(1.0, 0.0, 0.0);
     double theta_init = atan2(xb_w(1), xb_w(0));
     double theta_start = theta_init - fov_h_;
     double d_theta = atan2(mp_.resolution_, raycast_range_);
     double theta_bias = 0.0;

     while (theta_bias < fov_h_) {
         double temp_theta = theta_bias + theta_start;
         Eigen::Vector3d end = pos + raycast_range_ * Eigen::Vector3d(cos(temp_theta), sin(temp_theta), 0.0);

         markObstacle(pos, end);

         theta_bias += d_theta;
     }
}

void MapBridge::markObstacle(const Eigen::Vector3d &start, const Eigen::Vector3d &end) {
    // note: make sure the start point is inside the map, end point may be outside
    const double dist_s2e = (end - start).norm();
    const auto dir = (end - start).normalized();

    double travel_dist = 0.0;
    while (travel_dist < dist_s2e) {
        Eigen::Vector3d temp_pos = start + travel_dist * dir;
        if (!isInMap(temp_pos)) {
            return;
        }

        auto voxel = pos2Voxel(temp_pos);
        voxel.z() = mp_.ground_index_;

        if (!isVoxelValid(voxel)) {
            std::cout << "Warning! Voxel is invalid after pos isInMap check." << std::endl;
            std::cout << "in MapBridge::markObstacle" << std::endl;
            return;
        }
        const auto idx = voxel2BufferIndex(voxel);
        md_.local_occupancy_buffer_[idx] = md_.global_occupancy_buffer_[idx];

        // TODO: update local sdf map based on the updated grid map, not values in global sdf map
        md_.local_sdf_buffer_[idx] = md_.global_sdf_buffer_[idx];

        for (int i = 0; i < mp_.map_voxel_num_(2); i++) {
            const int temp_idx = idx + i;
            md_.local_occupancy_buffer_[temp_idx] = md_.global_occupancy_buffer_[temp_idx];
            md_.local_sdf_buffer_[temp_idx] = md_.global_sdf_buffer_[temp_idx];
        }

        if (isOccupied(temp_pos)) {
            break;
        }

        travel_dist += mp_.resolution_;
    }
}

Eigen::Matrix3d MapBridge::getRotB2W(const Eigen::Vector3d &att){
    const double s_phi = sin(att[0]);
    const double c_phi = cos(att[0]);
    const double s_theta = sin(att[1]);
    const double c_theta = cos(att[1]);
    const double s_psi = sin(att[2]);
    const double c_psi = cos(att[2]);

    Eigen::Matrix3d rot_mat;
    rot_mat << c_theta * c_psi, s_theta * s_phi * c_psi - s_psi * c_phi, s_theta * c_phi * c_psi + s_psi * s_phi,
        c_theta * s_psi, s_psi * s_theta * s_phi + c_psi * c_phi, s_psi * s_theta * c_phi - c_psi * s_phi,
        -s_theta, s_phi * c_theta, c_phi * c_theta;

    return rot_mat;
}
