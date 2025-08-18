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
 * Created by Zhaohong Liu on 24-12-4.
*/

#ifndef RESCLIB_K_GPEP_H
#define RESCLIB_K_GPEP_H

#include <Eigen/Eigen>

using Corner2d = std::array<Eigen::Vector2f, 4>;
using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

class K_GPEP {
private:
    /* params */
    float collision_threshold_ = 0.3;
    float coll_thresh_sq_ = static_cast<float>(sqrt(2) * collision_threshold_);
    float resolution_ = 0.1;
    float sector_range_ = M_PI / 3;
    double camera_range_max_ = 3.0;
    const float experience_radius_ = 2.0;
    const float experience_gap_dist_ = 0.9;
    const double height_min_ = 0.0;
    const double height_max_ = 2.0;
    bool esdf_updated_ = false;

    /* map data */
    std::vector<double> global_sdf_buffer_;
    Eigen::Vector2i map_voxel_num_;
    double map_size_x_ = 10.0;
    double map_size_y_ = 10.0;

public:
    void set2DESDFMap(const Eigen::Ref<Matrix> &sdf_map, double map_x, double map_y);
    void setCollisionThreshold(float threshold) { collision_threshold_ = threshold; }

    [[nodiscard]] Eigen::VectorXd getKinematicPseudoRaycast(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
                                                            const Eigen::Vector3d &ctrl_p) const;

    [[nodiscard]] Eigen::Vector4d guidedPseudoRaycast(const Eigen::Vector3d &pos, const Eigen::Vector3d &ctrl_p,
                                                      float preset_radius = -1) const;

    [[nodiscard]] Eigen::Vector2d pseudoRaycast(int direction, const Eigen::Vector2f &start, float theta_init,
                                                float d_theta, float radius, const std::string &mode) const;

    /**
     * @brief set a ray from start to end, return the first point that is occupied if obs_free_mode is false
     * @param start start point, Eigen::Vector2f
     * @param end end point, Eigen::Vector2f
     * @param obs_free_mode is false when finding occupied element, true will find the end or edge
     * @return the closest point to start that is occupied on the line from start to end
     */
    [[nodiscard]] Eigen::Vector2f singlePseudoRaycast(const Eigen::Vector2f &start, const Eigen::Vector2f &end,
                                                      bool obs_free_mode) const;

    [[nodiscard]] Eigen::Vector2i pos2Voxel(const Eigen::Vector2f & pos2d) const;
    void clampVoxel(Eigen::Vector2i & voxel) const;
    [[nodiscard]] inline int pos2BufferIndex(const Eigen::Vector2f & pos2d_f) const;

    [[nodiscard]] inline int voxel2BufferIndex(const Eigen::Vector2i &voxel) const;

    [[nodiscard]] inline bool isInflateOccupied(const Eigen::Vector2f & pos2d_f) const;

    [[nodiscard]] double getDistance(const Eigen::Vector3d & pos_3d) const;
    [[nodiscard]] inline double getDistanceSDF(const Eigen::Vector2f & pos_2d_f) const;

    [[nodiscard]] bool isInMap(const Eigen::Vector3d & pos3d) const;
    [[nodiscard]] inline bool isInMap(const Eigen::Vector2f & pos2d_f) const;

    [[nodiscard]] Corner2d getGridCorner(const Eigen::Vector2d & pos_2d) const;
    static Eigen::Vector2d getPeekCorner(const Eigen::Vector2f & start, const Eigen::Vector2f & end,
                                         const Corner2d & corner, int type) ;

    [[nodiscard]] Eigen::VectorXd getESDFPerception(const Eigen::Vector3d & pos) const;
    [[nodiscard]] Eigen::Vector2f voxel2Pos(int row, int col) const;

    void clearGlobalESDFBuffer();

    [[nodiscard]] Eigen::VectorXd getDoublePseudoRaycast(const Eigen::Vector3d &pos,const Eigen::Vector3d &vel,
                                                         const Eigen::Vector3d &ctrl_p1, const Eigen::Vector3d & ctrl_p2) const;
    [[nodiscard]] double getVelDistance(const Eigen::Vector3d & pos, const Eigen::Vector3d & vel) const;
};

inline int K_GPEP::voxel2BufferIndex(const Eigen::Vector2i &voxel) const {
    return voxel.y() * map_voxel_num_.x() + voxel.x();
}

inline bool K_GPEP::isInflateOccupied(const Eigen::Vector2f &pos2d_f) const {
    return global_sdf_buffer_[voxel2BufferIndex(pos2Voxel(pos2d_f))] < collision_threshold_ - 0.01;
}

inline double K_GPEP::getDistanceSDF(const Eigen::Vector2f &pos_2d_f) const {
    return global_sdf_buffer_[voxel2BufferIndex(pos2Voxel(pos_2d_f))];
}

inline bool K_GPEP::isInMap(const Eigen::Vector2f &pos2d_f) const {
    return pos2d_f.x() >= 0 && pos2d_f.x() < map_size_x_ &&
           pos2d_f.y() >= 0 && pos2d_f.y() < map_size_y_;
}

inline int K_GPEP::pos2BufferIndex(const Eigen::Vector2f &pos2d_f) const {
    return voxel2BufferIndex(pos2Voxel(pos2d_f));
}


#endif //RESCLIB_K_GPEP_H
