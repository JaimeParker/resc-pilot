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

#ifndef BACKEND_OPTIMIZER_RL_OPTIMIZE_H
#define BACKEND_OPTIMIZER_RL_OPTIMIZE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <torch/script.h>

using WptPair = std::pair<Eigen::Vector3d, Eigen::Vector3d>;

class RLMotion {
private:
    /* state */
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d acc_;
    Eigen::Vector3d att_;
    Eigen::Vector3d body_rate_;

    /* observation */
    int num_obs_;
    WptPair wpt_pair_;
    Eigen::Vector2d rela_pos_;
    std::vector<double> sdf_values_;
    std::vector<double> pseudo_raycast_;

    /* model */
    std::string model_name_;
    torch::jit::script::Module model_;

    /* params */
    const double height_top_ = 2.0;
    bool use_full_observation = false;

public:
    RLMotion();
    ~RLMotion();
    void init();

    /**
     * @brief Set all states for observation
     * @param pos: position in world frame
     * @param vel: velocity in world frame
     * @param att: attitude in euler angle
     * @param body_rate: angular velocity in body frame
     * @param wpt_pair: pair of waypoints
     */
    void setAllStates(const Eigen::Vector3d & pos, const Eigen::Vector3d & vel,
                      const Eigen::Vector3d & att, const Eigen::Vector3d & body_rate,
                      WptPair& wpt_pair);
    void setAllStates(const Eigen::Vector3d & pos, const Eigen::Vector3d & vel,
                      const Eigen::Vector3d & att, const Eigen::Vector3d & body_rate,
                      WptPair & wpt_pair, const Eigen::Vector2d & rela_pos,
                      const std::vector<double> & sdf_values, const std::vector<double> & pseudo_raycast);

    std::vector<float> getObservation();
    static Eigen::Matrix3d getRotMat(const Eigen::Vector3d& attitude_euler);
    Eigen::Vector4d forwardModel();
    void setModelName(const std::string& model_name);
    void setNumObs(int num_obs);

public:
    using Ptr = std::unique_ptr<RLMotion>;
};


#endif //BACKEND_OPTIMIZER_RL_OPTIMIZE_H
