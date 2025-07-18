//
// Created by Zhaohong Liu on 24-10-16.
//

#include "px4_utils/Filter.h"

KalmanFilter::KalmanFilter() {
    // Initialize state vector (6x1)
    x_ = Eigen::VectorXd(6);
    x_.setZero();

    // Initialize state covariance matrix (6x6)
    P_ = Eigen::MatrixXd(6, 6);
    P_.setIdentity();

    // Initialize state transition matrix (6x6)
    F_ = Eigen::MatrixXd(6, 6);
    F_.setIdentity();

    // Initialize process noise covariance matrix (6x6)
    Q_ = Eigen::MatrixXd(6, 6);
    Q_.setIdentity() * 0.01;  // Small process noise

    // Initialize measurement matrix (3x6) - we measure [phi, theta, psi]
    H_ = Eigen::MatrixXd(3, 6);
    H_.setZero();
    H_(0, 0) = 1;
    H_(1, 1) = 1;
    H_(2, 2) = 1;

    // Initialize measurement noise covariance matrix (3x3)
    R_ = Eigen::MatrixXd(3, 3);
    R_.setIdentity() * 0.1;  // Measurement noise
}

void KalmanFilter::predict(double dt) {
    // Update state transition matrix with time step dt
    F_(0, 3) = dt;
    F_(1, 4) = dt;
    F_(2, 5) = dt;
    // Predict the new state
    x_ = F_ * x_;
    // Predict the new covariance matrix
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector3d &measurement) {
    // Measurement prediction
    Eigen::Vector3d z_pred = H_ * x_;
    // Measurement residual (innovation)
    Eigen::Vector3d y = measurement - z_pred;
    // Innovation covariance
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    // Kalman gain
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    // Update state estimate
    x_ = x_ + K * y;
    // Update covariance matrix
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;
}

double LowPassFilter::filter(const std::deque<double> &data) {
    if (data.empty()) {
        throw std::runtime_error("Data deque is empty.");
    }

    // Get the most recent data point from the deque
    double latest_input = data.back();

    // Initialize the filter with the first data point
    if (!is_initialized) {
        prev_output = latest_input;
        is_initialized = true;
    }

    // Apply the low-pass filter: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    double output = alpha * latest_input + (1.0 - alpha) * prev_output;

    // Save the filtered output for the next iteration
    prev_output = output;

    return output;
}

void LowPassFilter::reset() {
    is_initialized = false;
    prev_output = 0.0;
}

double MovingAverageFilter::filtering(double current_data) {
    if (data_.size() < window_size_) {
        data_.push_back(current_data);
        sum_ += current_data;
    } else {
        sum_ -= data_.front();
        data_.pop_front();
        data_.push_back(current_data);
        sum_ += current_data;
    }

    return sum_ / static_cast<double>(data_.size());
}

void MovingAverageFilter::reset() {
    data_.clear();
    pose_queue_.clear();
    sum_ = 0;
}

geometry_msgs::PoseStamped MovingAverageFilter::filtering(const geometry_msgs::PoseStamped &current_pose) {
    if (pose_queue_.size() < window_size_) {
        pose_queue_.push_back(current_pose);
        return current_pose;
    } else {
        pose_queue_.pop_front();
        pose_queue_.push_back(current_pose);
    }

    auto filtered_pose = current_pose;
    filtered_pose.pose.position.x = 0.0;
    filtered_pose.pose.position.y = 0.0;
    filtered_pose.pose.position.z = 0.0;

    for (const auto &pose : pose_queue_) {
        filtered_pose.pose.position.x += pose.pose.position.x;
        filtered_pose.pose.position.y += pose.pose.position.y;
        filtered_pose.pose.position.z += pose.pose.position.z;
    }

    filtered_pose.pose.position.x /= static_cast<double>(window_size_);
    filtered_pose.pose.position.y /= static_cast<double>(window_size_);
    filtered_pose.pose.position.z /= static_cast<double>(window_size_);

    return filtered_pose;
}

void ButterworthFilter::calculateCoefficients(const double & dt) {
    // Communication might not be stable.
    // We need to calc coefficients of each dt.
    // When the ros topic frequency is stable, you can remove the parameter dt, and run this func in constructor
    double omega = tan(M_PI * cutoff_freq_ * dt);
    double omega2 = omega * omega;
    double sqrt2 = std::sqrt(2);

    // using second order butterworth filter
    double a0 = omega2 + sqrt2 * omega + 1;
    b0_ = omega2 / a0;
    b1_ = 2 * b0_;
    b2_ = b0_;
    a1_ = 2 * (omega2 - 1) / a0;
    a2_ = (omega2 - sqrt2 * omega + 1) / a0;
}

ButterworthFilter::ButterworthFilter(double cutoff_freq, int deque_size) {
    cutoff_freq_ = cutoff_freq;
    deque_size_ = deque_size;
    if (deque_size < 3) {
        throw std::runtime_error("Deque size must be greater than 2, we are using second order butterworth filter.");
    }
    original_data_ = std::deque<double>(deque_size_, 0);
    filtered_data_ = std::deque<double>(deque_size_, 0);
    b0_ = 0;
    b1_ = 0;
    b2_ = 0;
    a1_ = 0;
    a2_ = 0;
}

double ButterworthFilter::filter(const double &current_data, const double &dt) {
    if (original_data_.size() < deque_size_) {
        original_data_.push_back(current_data);
        filtered_data_.push_back(current_data);
        return current_data;
    } else {
        original_data_.pop_front();
        filtered_data_.pop_front();
        original_data_.push_back(current_data);
    }

    calculateCoefficients(dt);
    // Apply the Butterworth filter difference equation:
    // y[i] = b0 * x[i] + b1 * x[i-1] + b2 * x[i-2] - a1 * y[i-1] - a2 * y[i-2]
    double filtered_current_data = b0_ * original_data_[deque_size_ - 1] +
                                   b1_ * original_data_[deque_size_ - 2] +
                                   b2_ * original_data_[deque_size_ - 3] -
                                   a1_ * filtered_data_[deque_size_ - 2] -
                                   a2_ * filtered_data_[deque_size_ - 3];

    filtered_data_.push_back(filtered_current_data);

    return filtered_current_data;
}

Eigen::Vector3d ButterworthFilter::filter(const Eigen::Vector3d &current_vec, const double &dt) {
    if (origin_vec_.size() < deque_size_) {
        origin_vec_.push_back(current_vec);
        filtered_vec_.push_back(current_vec);
        return current_vec;
    } else {
        origin_vec_.pop_front();
        filtered_vec_.pop_front();
        origin_vec_.push_back(current_vec);
    }

    calculateCoefficients(dt);
    // Apply the Butterworth filter difference equation:
    // y[i] = b0 * x[i] + b1 * x[i-1] + b2 * x[i-2] - a1 * y[i-1] - a2 * y[i-2]
    Eigen::Vector3d filtered_current_vec;
    filtered_current_vec[0] = b0_ * origin_vec_[deque_size_ - 1][0] +
                              b1_ * origin_vec_[deque_size_ - 2][0] +
                              b2_ * origin_vec_[deque_size_ - 3][0] -
                              a1_ * filtered_vec_[deque_size_ - 2][0] -
                              a2_ * filtered_vec_[deque_size_ - 3][0];

    filtered_current_vec[1] = b0_ * origin_vec_[deque_size_ - 1][1] +
                              b1_ * origin_vec_[deque_size_ - 2][1] +
                              b2_ * origin_vec_[deque_size_ - 3][1] -
                              a1_ * filtered_vec_[deque_size_ - 2][1] -
                              a2_ * filtered_vec_[deque_size_ - 3][1];

    filtered_current_vec[2] = b0_ * origin_vec_[deque_size_ - 1][2] +
                              b1_ * origin_vec_[deque_size_ - 2][2] +
                              b2_ * origin_vec_[deque_size_ - 3][2] -
                              a1_ * filtered_vec_[deque_size_ - 2][2] -
                              a2_ * filtered_vec_[deque_size_ - 3][2];

    filtered_vec_.push_back(filtered_current_vec);

    return filtered_current_vec;
}
