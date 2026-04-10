//UPDATED (zhiyuan 11_28): Two-stage trajectory planning - horizontal movement followed by vertical descent
//When altimeter distance < 2.0m, target is considered reached, notify PX4CtrlFSM to enter GPS correction phase
//Ctrl + F:"range < range_threshold_" to change the threshold

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <cmath>

class SimpleEgoPlanner
{
public:
    SimpleEgoPlanner(ros::NodeHandle& nh) : nh_(nh)
    {
        // Initialize parameters
        initParameters();
        
        // Initialize publishers and subscribers
        initPubSub();
        
        // Initialize state
        current_state_ = IDLE;
        has_odom_ = false;
        has_target_ = false;
        is_in_vertical_phase_ = false;
        
        // Create timer
        plan_timer_ = nh_.createTimer(ros::Duration(0.05), &SimpleEgoPlanner::planTimerCallback, this);
        
        ROS_INFO("[SimpleEgoPlanner] Initialized successfully!");
    }

private:
    enum PlannerState
    {
        IDLE,           // Idle state
        PLANNING,       // Planning
        EXECUTING       // Executing trajectory
    };

    struct TrajectoryPoint
    {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        double yaw;
        double time;
    };

    // ROS related
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber lidar_sub_;  // New LiDAR subscriber
    ros::Publisher pos_cmd_pub_;
    ros::Publisher traj_vis_pub_;
    ros::Publisher goal_vis_pub_;
    ros::Timer plan_timer_;

    // State variables
    PlannerState current_state_;
    bool has_odom_;
    bool has_target_;
    bool is_in_vertical_phase_;  // Whether in vertical descent phase
        
    // Current state
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d current_vel_;
    Eigen::Vector3d target_pos_;
    
    // Trajectory
    std::vector<TrajectoryPoint> trajectory_;
    int current_traj_index_;
    ros::Time traj_start_time_;
    ros::Time last_goal_time_;
    double last_yaw_ = 0.0;  // Record the last published yaw angle
    
    // Parameters
    double max_vel_;           // Maximum velocity
    double max_acc_;           // Maximum acceleration
    double goal_tolerance_;    // Goal tolerance
    double dt_;               // Time step
    double publish_rate_;     // Publish rate
    
    void initParameters()
    {
        nh_.param("max_vel", max_vel_, 1.5);
        nh_.param("max_acc", max_acc_, 2.0);
        nh_.param("goal_tolerance", goal_tolerance_, 0.5);
        nh_.param("dt", dt_, 0.1);
        nh_.param("publish_rate", publish_rate_, 20.0);
        
        ROS_INFO("[SimpleEgoPlanner] Parameters loaded:");
        ROS_INFO("  max_vel: %.2f m/s", max_vel_);
        ROS_INFO("  max_acc: %.2f m/s^2", max_acc_);
        ROS_INFO("  goal_tolerance: %.2f m", goal_tolerance_);
    }

    // Normalize angle to [-pi, pi]
    static double normalizeAngle(double ang)
    {
        while (ang > M_PI) ang -= 2.0 * M_PI;
        while (ang < -M_PI) ang += 2.0 * M_PI;
        return ang;
    }
    
    void initPubSub()
    {
        // Subscribers - use same topics as PX4CtrlFSM
        odom_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &SimpleEgoPlanner::odomCallback, this);
        target_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &SimpleEgoPlanner::targetCallback, this);
        
        // Publishers - publish to topics expected by PX4CtrlFSM
        pos_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);
        traj_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/simple_planner/trajectory", 10);
        goal_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/simple_planner/goal", 10);
    }
    
    void odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_pos_ << msg->pose.position.x, 
                        msg->pose.position.y, 
                        msg->pose.position.z;
        
        // Simple numerical differentiation to estimate velocity
        static Eigen::Vector3d last_pos = current_pos_;
        static ros::Time last_time = ros::Time::now();
        
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        
        if (dt > 0.001 && has_odom_) {
            current_vel_ = (current_pos_ - last_pos) / dt;
        }
        
        last_pos = current_pos_;
        last_time = current_time;
        has_odom_ = true;
    }
    
    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        target_pos_ << msg->pose.position.x,
                       msg->pose.position.y,
                       msg->pose.position.z;
        
        has_target_ = true;
        current_state_ = PLANNING;
        last_goal_time_ = ros::Time::now();
        
        ROS_INFO("[SimpleEgoPlanner] New target received: (%.2f, %.2f, %.2f)", 
                 target_pos_.x(), target_pos_.y(), target_pos_.z());
        
        // Visualize target point
        publishGoalVisualization();
    }
    
    void planTimerCallback(const ros::TimerEvent& /*event*/)
    {
        if (!has_odom_ || !has_target_)
            return;
            
        switch (current_state_)
        {
            case IDLE:
                break;
                
            case PLANNING:
                if (planTrajectory())
                {
                    current_state_ = EXECUTING;
                    current_traj_index_ = 0;
                    traj_start_time_ = ros::Time::now();
                    ROS_INFO("[SimpleEgoPlanner] Trajectory planned, %lu points", trajectory_.size());
                }
                else
                {
                    ROS_WARN("[SimpleEgoPlanner] Trajectory planning failed");
                    current_state_ = IDLE;
                }
                break;
                
            case EXECUTING:
                if (isGoalReached())
                {
                    current_state_ = IDLE;
                    has_target_ = false;
                    ROS_INFO("[SimpleEgoPlanner] Goal reached!");
                    // Publish stop command
                    publishStopCommand();
                }
                else
                {
                    executeTrajectory();
                }
                break;
        }
    }
    
    bool planTrajectory()
    {
        trajectory_.clear();
        is_in_vertical_phase_ = false;
        
        // Check if already reached target
        double distance = (target_pos_ - current_pos_).norm();
        if (distance < goal_tolerance_)
        {
            return false;
        }
        
        // Two-stage trajectory planning
        // First stage: horizontal movement to above target
        Eigen::Vector3d horizontal_start = current_pos_;
        Eigen::Vector3d horizontal_target(target_pos_.x(), target_pos_.y(), current_pos_.z());
        double horizontal_distance = (horizontal_target - horizontal_start).norm();
        
        // Second stage: vertical descent to target
        Eigen::Vector3d vertical_start = horizontal_target;
        Eigen::Vector3d vertical_target(target_pos_.x(), target_pos_.y(), 0.0);
        double vertical_distance = std::abs(vertical_target.z() - vertical_start.z());
        
        // Generate horizontal movement trajectory
        if (horizontal_distance > goal_tolerance_)
        {
            generateHorizontalTrajectory(horizontal_start, horizontal_target, horizontal_distance);
        }
        
        // Generate vertical descent trajectory
        if (vertical_distance > goal_tolerance_)
        {
            generateVerticalTrajectory(vertical_start, vertical_target, vertical_distance);
        }
        
        // If no trajectory points generated, go directly to target
        if (trajectory_.empty())
        {
            TrajectoryPoint final_point;
            final_point.position = target_pos_;
            final_point.velocity = Eigen::Vector3d::Zero();
            final_point.acceleration = Eigen::Vector3d::Zero();
            final_point.yaw = last_yaw_;
            final_point.time = 0.0;
            trajectory_.push_back(final_point);
        }
        else
        {
            // Ensure last point reaches target position
            TrajectoryPoint final_point;
            final_point.position = target_pos_;
            final_point.velocity = Eigen::Vector3d::Zero();
            final_point.acceleration = Eigen::Vector3d::Zero();
            final_point.yaw = trajectory_.back().yaw;
            final_point.time = trajectory_.back().time + dt_;
            trajectory_.push_back(final_point);
        }
        
        // Visualize trajectory
        publishTrajectoryVisualization();
        
        return !trajectory_.empty();
    }
    
    double calculateOptimalTime(double distance)
    {
        // Time calculation for trapezoidal velocity curve
        double acc_time = max_vel_ / max_acc_;
        double acc_dist = 0.5 * max_acc_ * acc_time * acc_time;
        
        if (2 * acc_dist >= distance)
        {
            // Triangular velocity curve
            return 2.0 * sqrt(distance / max_acc_);
        }
        else
        {
            // Trapezoidal velocity curve
            double const_vel_dist = distance - 2 * acc_dist;
            return 2 * acc_time + const_vel_dist / max_vel_;
        }
    }
    
    double calculateOptimalTimeWithMaxVel(double distance, double limited_max_vel)
    {
        // Calculate time using limited maximum velocity
        double acc_time = limited_max_vel / max_acc_;
        double acc_dist = 0.5 * max_acc_ * acc_time * acc_time;
        
        if (2 * acc_dist >= distance)
        {
            // Triangular velocity curve
            return 2.0 * sqrt(distance / max_acc_);
        }
        else
        {
            // Trapezoidal velocity curve
            double const_vel_dist = distance - 2 * acc_dist;
            return 2 * acc_time + const_vel_dist / limited_max_vel;
        }
    }
    
    void generateHorizontalTrajectory(const Eigen::Vector3d& start, 
                                      const Eigen::Vector3d& target, 
                                      double distance)
    {
        if (distance < 1e-3) return;
        
        Eigen::Vector3d direction = (target - start).normalized();
        double total_time = calculateOptimalTime(distance);
        
        // Get time offset for current trajectory
        double time_offset = trajectory_.empty() ? 0.0 : trajectory_.back().time + dt_;
        
        // Generate trajectory points
        int num_points = static_cast<int>(total_time / dt_) + 1;
        
        for (int i = 0; i < num_points; ++i)
        {
            double t = i * dt_;
            if (t > total_time) t = total_time;
            
            TrajectoryPoint point;
            
            // Use trapezoidal velocity curve to calculate position, velocity, acceleration
            calculateTrajectoryPoint(t, total_time, distance, direction, start, point);
            point.time += time_offset;
            
            trajectory_.push_back(point);
        }
        
        // Calculate yaw for horizontal movement trajectory
        updateTrajectoryYaw();
    }
    
    void generateVerticalTrajectory(const Eigen::Vector3d& start, 
                                    const Eigen::Vector3d& target, 
                                    double distance)
    {
        if (distance < 1e-3) return;
        
        Eigen::Vector3d direction = (target - start).normalized();
        
        // For vertical movement, limit maximum velocity to 0.5m/s
        double vertical_max_vel = 0.5;
        double total_time = calculateOptimalTimeWithMaxVel(distance, vertical_max_vel);
        
        // Get time offset for current trajectory
        double time_offset = trajectory_.empty() ? 0.0 : trajectory_.back().time + dt_;
        
        // Generate trajectory points
        int num_points = static_cast<int>(total_time / dt_) + 1;
        
        for (int i = 0; i < num_points; ++i)
        {
            double t = i * dt_;
            if (t > total_time) t = total_time;
            
            TrajectoryPoint point;
            
            // Use trapezoidal velocity curve to calculate position, velocity, acceleration (with limited max velocity)
            calculateTrajectoryPointWithMaxVel(t, total_time, distance, direction, start, point, vertical_max_vel);
            point.time += time_offset;
            
            // Smoothly reduce yaw to zero during vertical movement
            double initial_yaw = trajectory_.empty() ? last_yaw_ : trajectory_.back().yaw;
            double target_yaw = 0.0;
            double yaw_progress = t / total_time;  // Progress from 0 to 1
            
            // Use cosine interpolation for smoother yaw transition
            double smooth_progress = (1.0 - cos(yaw_progress * M_PI)) * 0.5;
            point.yaw = initial_yaw * (1.0 - smooth_progress) + target_yaw * smooth_progress;
            point.yaw = normalizeAngle(point.yaw);
            
            trajectory_.push_back(point);
        }
    }
    
    void updateTrajectoryYaw()
    {
        if (trajectory_.empty()) return;
        
        // Calculate yaw for horizontal movement segment
        size_t start_index = 0;
        
        // Find start position of current segment
        for (size_t i = 1; i < trajectory_.size(); ++i)
        {
            if (std::abs(trajectory_[i].position.z() - trajectory_[i-1].position.z()) > 1e-3)
            {
                start_index = i;
                break;
            }
        }
        
        double yaw_prev = last_yaw_;
        for (size_t i = start_index; i < trajectory_.size(); ++i)
        {
            const auto& pt = trajectory_[i];
            double vx = pt.velocity.x();
            double vy = pt.velocity.y();
            double yaw = yaw_prev;

            // Prioritize velocity direction
            if (std::hypot(vx, vy) > 1e-3)
            {
                yaw = std::atan2(vy, vx);
            }
            else if (i + 1 < trajectory_.size())
            {
                // Use displacement direction between adjacent points
                Eigen::Vector3d d = trajectory_[i + 1].position - pt.position;
                if (std::hypot(d.x(), d.y()) > 1e-3)
                {
                    yaw = std::atan2(d.y(), d.x());
                }
            }

            yaw = normalizeAngle(yaw);
            trajectory_[i].yaw = yaw;
            yaw_prev = yaw;
        }
    }
    
    void calculateTrajectoryPoint(double t, double total_time, double total_distance, 
                                  const Eigen::Vector3d& direction, const Eigen::Vector3d& start_pos,
                                  TrajectoryPoint& point)
    {
        double acc_time = max_vel_ / max_acc_;
        double acc_dist = 0.5 * max_acc_ * acc_time * acc_time;
        
        double s, v, a;  // Position, velocity, acceleration scalar values
        
        if (2 * acc_dist >= total_distance)
        {
            // Triangular velocity curve
            double peak_time = total_time / 2.0;
            if (t <= peak_time)
            {
                s = 0.5 * max_acc_ * t * t;
                v = max_acc_ * t;
                a = max_acc_;
            }
            else
            {
                double dt_from_peak = t - peak_time;
                double peak_vel = max_acc_ * peak_time;
                s = acc_dist + peak_vel * dt_from_peak - 0.5 * max_acc_ * dt_from_peak * dt_from_peak;
                v = peak_vel - max_acc_ * dt_from_peak;
                a = -max_acc_;
            }
        }
        else
        {
            // Trapezoidal velocity curve
            if (t <= acc_time)
            {
                s = 0.5 * max_acc_ * t * t;
                v = max_acc_ * t;
                a = max_acc_;
            }
            else if (t <= total_time - acc_time)
            {
                s = acc_dist + max_vel_ * (t - acc_time);
                v = max_vel_;
                a = 0.0;
            }
            else
            {
                double dt_from_decel = t - (total_time - acc_time);
                s = total_distance - acc_dist + max_vel_ * dt_from_decel - 0.5 * max_acc_ * dt_from_decel * dt_from_decel;
                v = max_vel_ - max_acc_ * dt_from_decel;
                a = -max_acc_;
            }
        }
        
        // Convert to 3D vectors
        point.position = start_pos + direction * s;
        point.velocity = direction * std::max(0.0, v);
        point.acceleration = direction * a;
        // yaw will be calculated uniformly after trajectory generation
        point.yaw = last_yaw_;
        point.time = t;
    }
    
    void calculateTrajectoryPointWithMaxVel(double t, double total_time, double total_distance, 
                                            const Eigen::Vector3d& direction, const Eigen::Vector3d& start_pos,
                                            TrajectoryPoint& point, double limited_max_vel)
    {
        double acc_time = limited_max_vel / max_acc_;
        double acc_dist = 0.5 * max_acc_ * acc_time * acc_time;
        
        double s, v, a;  // Position, velocity, acceleration scalar values
        
        if (2 * acc_dist >= total_distance)
        {
            // Triangular velocity curve
            double peak_time = total_time / 2.0;
            if (t <= peak_time)
            {
                s = 0.5 * max_acc_ * t * t;
                v = max_acc_ * t;
                a = max_acc_;
            }
            else
            {
                double dt_from_peak = t - peak_time;
                double peak_vel = max_acc_ * peak_time;
                s = acc_dist + peak_vel * dt_from_peak - 0.5 * max_acc_ * dt_from_peak * dt_from_peak;
                v = peak_vel - max_acc_ * dt_from_peak;
                a = -max_acc_;
            }
        }
        else
        {
            // Trapezoidal velocity curve
            if (t <= acc_time)
            {
                s = 0.5 * max_acc_ * t * t;
                v = max_acc_ * t;
                a = max_acc_;
            }
            else if (t <= total_time - acc_time)
            {
                s = acc_dist + limited_max_vel * (t - acc_time);
                v = limited_max_vel;
                a = 0.0;
            }
            else
            {
                double dt_from_decel = t - (total_time - acc_time);
                s = total_distance - acc_dist + limited_max_vel * dt_from_decel - 0.5 * max_acc_ * dt_from_decel * dt_from_decel;
                v = limited_max_vel - max_acc_ * dt_from_decel;
                a = -max_acc_;
            }
        }
        
        // Convert to 3D vectors
        point.position = start_pos + direction * s;
        point.velocity = direction * std::max(0.0, v);
        point.acceleration = direction * a;
        // yaw will be calculated uniformly after trajectory generation
        point.yaw = last_yaw_;
        point.time = t;
    }
    
    void executeTrajectory()
    {
        if (trajectory_.empty()) return;
        
        double current_time = (ros::Time::now() - traj_start_time_).toSec();
        
        // Find current trajectory point to execute
        while (current_traj_index_ < trajectory_.size() - 1 && 
               trajectory_[current_traj_index_ + 1].time <= current_time)
        {
            current_traj_index_++;
        }
        
        if (current_traj_index_ >= trajectory_.size() - 1)
        {
            current_traj_index_ = trajectory_.size() - 1;
        }
        
        // Check if entering vertical descent phase
        const TrajectoryPoint& point = trajectory_[current_traj_index_];
        if (!is_in_vertical_phase_ && current_traj_index_ > 0)
        {
            // Determine if starting vertical movement (significant Z velocity and small XY velocity)
            if (std::abs(point.velocity.z()) > 0.1 && 
                std::hypot(point.velocity.x(), point.velocity.y()) < 0.1)
            {
                is_in_vertical_phase_ = true;
                ROS_INFO("[SimpleEgoPlanner] Entering vertical descent phase");
            }
        }
        
        // Check height sensor during vertical descent phase
        if (is_in_vertical_phase_)
        {
            // Altimeter distance < 2.0m indicates reaching target
            current_state_ = IDLE;
            has_target_ = false;
            is_in_vertical_phase_ = false;
            publishStopCommand();
            return;
        }
        
        publishPositionCommand(point);
    }
    
    void publishPositionCommand(const TrajectoryPoint& point)
    {
        quadrotor_msgs::PositionCommand cmd;
        
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "map";
        
        // Position
        cmd.position.x = point.position.x();
        cmd.position.y = point.position.y();
        cmd.position.z = point.position.z();
        
        // Velocity
        cmd.velocity.x = point.velocity.x();
        cmd.velocity.y = point.velocity.y();
        cmd.velocity.z = point.velocity.z();
        
        // Acceleration
        cmd.acceleration.x = point.acceleration.x();
        cmd.acceleration.y = point.acceleration.y();
        cmd.acceleration.z = point.acceleration.z();
        
        // Yaw angle
        cmd.yaw = point.yaw;
        cmd.yaw_dot = 0.0;
        
        pos_cmd_pub_.publish(cmd);

        // Record published yaw angle
        last_yaw_ = point.yaw;
    }
    
    void publishStopCommand()
    {
        is_in_vertical_phase_ = false;  // Reset phase flag
        
        quadrotor_msgs::PositionCommand cmd;
        
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "map";
        
        // Current position, zero velocity and acceleration
        cmd.position.x = current_pos_.x();
        cmd.position.y = current_pos_.y();
        cmd.position.z = current_pos_.z();
        
        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;
        
        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;
        
        // Keep yaw at 0 degrees when stopped
        cmd.yaw = 0.0;
        cmd.yaw_dot = 0.0;
        
        pos_cmd_pub_.publish(cmd);
        
        ROS_INFO("[SimpleEgoPlanner] Trajectory stopped. Ready for PX4CtrlFSM GPS correction.");
    }
    
    bool isGoalReached()
    {
        return (current_pos_ - target_pos_).norm() < goal_tolerance_;
    }
    
    void publishTrajectoryVisualization()
    {
        visualization_msgs::MarkerArray marker_array;
        
        // Trajectory line
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "trajectory";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.scale.x = 0.05;
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.8;
        
        for (const auto& point : trajectory_)
        {
            geometry_msgs::Point p;
            p.x = point.position.x();
            p.y = point.position.y();
            p.z = point.position.z();
            line_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_marker);
        traj_vis_pub_.publish(marker_array);
    }
    
    void publishGoalVisualization()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = target_pos_.x();
        marker.pose.position.y = target_pos_.y();
        marker.pose.position.z = target_pos_.z();
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        goal_vis_pub_.publish(marker);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_ego_planner");
    ros::NodeHandle nh("~");
    
    SimpleEgoPlanner planner(nh);
    
    ROS_INFO("[SimpleEgoPlanner] Node started. Use 2D Nav Goal in RViz to set target.");
    ROS_INFO("[SimpleEgoPlanner] Publishing to /planning/pos_cmd for PX4CtrlFSM");
    ROS_INFO("[SimpleEgoPlanner] Monitoring /scan for height detection");
    
    ros::spin();
    
    return 0;
}
