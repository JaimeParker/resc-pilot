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
 * Created by Zhaohong Liu on 24-10-16.
*/

#ifndef PX4_UTILS_FILTER_H
#define PX4_UTILS_FILTER_H

#include <Eigen/Eigen>
#include <deque>
#include <geometry_msgs/PoseStamped.h>

class KalmanFilter {
private:
    Eigen::VectorXd x_;  // State vector: [phi, theta, psi, phi_dot, theta_dot, psi_dot]
    Eigen::MatrixXd P_;  // Covariance matrix
    Eigen::MatrixXd F_;  // State transition matrix
    Eigen::MatrixXd Q_;  // Process noise covariance matrix
    Eigen::MatrixXd H_;  // Measurement matrix
    Eigen::MatrixXd R_;  // Measurement noise covariance matrix

public:
    KalmanFilter();
    void predict(double dt);
    void update(const Eigen::Vector3d& measurement);
};
// generated purely by ChatGPT

class LowPassFilter {
public:
    // Constructor: alpha is the smoothing factor between 0 (no output) and 1 (no filtering)
    explicit LowPassFilter(double alpha)
            : alpha(alpha), is_initialized(false), prev_output(0.0) {}
    double filter(const std::deque<double> &data);
    void reset();
private:
    double alpha;
    bool is_initialized;
    double prev_output;
};
// generated purely by ChatGPT

class MovingAverageFilter {
private:
    size_t window_size_;
    std::deque<double> data_;
    std::deque<geometry_msgs::PoseStamped> pose_queue_;
    double sum_;
public:
    explicit MovingAverageFilter(size_t window_size) : window_size_(window_size), sum_(0) {}
    double filtering(double current_data);
    geometry_msgs::PoseStamped filtering(const geometry_msgs::PoseStamped &current_pose);
    void reset();
};

class ButterworthFilter {
public:
    ButterworthFilter(double cutoff_freq, int deque_size);
    double filter(const double & current_data, const double & dt);
    Eigen::Vector3d filter(const Eigen::Vector3d & current_vec, const double & dt);
private:
    int deque_size_;
    std::deque<double> original_data_;
    std::deque<double> filtered_data_;
    std::deque<Eigen::Vector3d> origin_vec_;
    std::deque<Eigen::Vector3d> filtered_vec_;
    double cutoff_freq_;
    double b0_, b1_, b2_, a1_, a2_;
private:
    void calculateCoefficients(const double & dt);
};

#endif //PX4_UTILS_FILTER_H
