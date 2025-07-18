//
// Created by Zhaohong Liu on 24-9-21.
//

#include "px4_utils/MocapProcessor.h"

void MocapProcessor::init(ros::NodeHandle &nh) {
    nh_ = nh;

    mocap_pose_sub_ = nh_.subscribe(mocap_pose_topic_, 1, &MocapProcessor::poseCallback, this);
    mocap_accel_sub_ = nh_.subscribe(mocap_accel_topic_, 1, &MocapProcessor::accelCallback, this);
    mocap_rate_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(body_rate_topic_, 1);
    mocap_rate_dot_pub_ = nh_.advertise<geometry_msgs::AccelStamped>(rate_dot_topic_, 1);
    mocap_thrust_pub_ = nh_.advertise<mavros_msgs::Thrust>(thrust_topic_, 1);
    mocap_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>(mocap_pose_filtered_topic_, 1);
}

void MocapProcessor::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    mocap_pose_ = *msg;
    pos_ << mocap_pose_.pose.position.x, mocap_pose_.pose.position.y, mocap_pose_.pose.position.z;
    q_.x() = mocap_pose_.pose.orientation.x;
    q_.y() = mocap_pose_.pose.orientation.y;
    q_.z() = mocap_pose_.pose.orientation.z;
    q_.w() = mocap_pose_.pose.orientation.w;
    Convertor::q2EulerAngle(q_, att_);

    double dt = mocap_pose_.header.stamp.toSec() - last_time_;
    last_time_ = mocap_pose_.header.stamp.toSec();
    pos_ = pos_filter_.filter(pos_, dt);
    att_ = att_filter_.filter(att_, dt);
    // TODO: get vel from pos filtered, filtering the vel and publish in geometry_msgs::TwistStamped
    Convertor::euler2Quaternion(q_, att_);

    if (pose_queue_.size() >= pose_queue_size_) {
        pose_queue_.pop_front();
        time_queue_.pop_front();
        att_queue_.pop_front();
    }

    mocap_pose_.pose.position.x = pos_.x();
    mocap_pose_.pose.position.y = pos_.y();
    mocap_pose_.pose.position.z = pos_.z();
    mocap_pose_.pose.orientation.x = q_.x();
    mocap_pose_.pose.orientation.y = q_.y();
    mocap_pose_.pose.orientation.z = q_.z();
    mocap_pose_.pose.orientation.w = q_.w();
    mocap_pose_pub.publish(mocap_pose_);

    pose_queue_.emplace_back(mocap_pose_);
    time_queue_.emplace_back(mocap_pose_.header.stamp.toSec());
    att_queue_.emplace_back(Convertor::geoMsgsPose2Euler(mocap_pose_));

    if (pose_queue_.size() == pose_queue_size_) {
        ProcessGeoMsgsPose();
    } else {
        ROS_INFO("[Mocap] Waiting for enough pose data to process...");
    }
}

void MocapProcessor::ProcessGeoMsgsPose() {
    if (att_dot_queue_.size() < att_dot_queue_size_) {
        ROS_INFO("[Mocap] Derivative attitude...");
        // fill att_dot_queue_ with att_dot from t1 to t3
        for (size_t i = 0; i < att_queue_.size() - 2; i++) {
            Eigen::Vector3d att_prev = att_queue_[i];
            Eigen::Vector3d att_next = att_queue_[i + 2];
            double t = time_queue_[i + 2] - time_queue_[i];
            att_dot_queue_.emplace_back((att_next - att_prev) / t);
        }
    } else {
        att_dot_queue_.pop_front();
        Eigen::Vector3d att_prev = att_queue_[att_queue_.size() - 2];
        Eigen::Vector3d att_next = att_queue_.back();
        double t = time_queue_.back() - time_queue_[time_queue_.size() - 2];
        att_dot_queue_.emplace_back((att_next - att_prev) / t);
    }

    calculateRate();
}

void MocapProcessor::calculateRate() {
    // TODO: 是否有更好的方法来计算机体角速度？
    //  also filtering current body rate
    if (rate_queue_.size() < rate_queue_size_) {
        ROS_INFO("[Mocap] Calculate rate...");
        for (size_t i = 0; i < att_dot_queue_size_; i++) {
            rate_queue_.emplace_back(Convertor::getRate(att_dot_queue_[i], att_queue_[i + 1]));
        }
    } else {
        rate_queue_.pop_front();
        // Since we have att in t0, t1, and t2, we can calculate att_dot at t1, and then rate at t1
        // We should use att at t1 and att_dot at t1 accordingly
        Eigen::Vector3d body_rate = Convertor::getRate(att_dot_queue_.back(),
                                                       att_queue_[att_queue_.size() - 2]);
        rate_queue_.emplace_back(body_rate);
    }

    if (rate_queue_.size() == rate_queue_size_) {
        // set rate and rate dot
        rate_ = rate_queue_[1];
        double t = time_queue_[time_queue_.size() - 2] - time_queue_[1];
        rate_dot_ = (rate_queue_.back() - rate_queue_.front()) / t;
    }
}

void MocapProcessor::publishRate() {
    body_rate_msg_.header.stamp = pose_queue_[2].header.stamp;
    body_rate_msg_.header.frame_id = body_frame_id_;
    body_rate_msg_.twist.angular.x = rate_[0];
    body_rate_msg_.twist.angular.y = rate_[1];
    body_rate_msg_.twist.angular.z = rate_[2];
    mocap_rate_pub_.publish(body_rate_msg_);

    rate_dot_msg_.header.stamp = pose_queue_[2].header.stamp;
    rate_dot_msg_.header.frame_id = body_frame_id_;
    rate_dot_msg_.accel.angular.x = rate_dot_[0];
    rate_dot_msg_.accel.angular.y = rate_dot_[1];
    rate_dot_msg_.accel.angular.z = rate_dot_[2];
    mocap_rate_dot_pub_.publish(rate_dot_msg_);
}

void MocapProcessor::publishThrust() {
    thrust_msg_.header.stamp = mocap_pose_.header.stamp;
    thrust_msg_.header.frame_id = body_frame_id_;
    thrust_msg_.thrust = thrust_;

    mocap_thrust_pub_.publish(thrust_msg_);
}

void MocapProcessor::accelCallback(const geometry_msgs::TwistStamped ::ConstPtr &msg) {
    // it's strange the Nokov use TwistStamped type for an accel message
    Eigen::Vector3d accel;
    accel << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;

    Eigen::Vector3d att;
    att << att_queue_.back()[0], att_queue_.back()[1], att_queue_.back()[2];

    thrust_ = Convertor::getThrust(att, accel, mass_);
}
