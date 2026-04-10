#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>

#include <algorithm>
#include <limits>
#include <random>
#include <string>
#include <vector>

class FakeLidarAltimeter {
public:
    FakeLidarAltimeter() : nh_("~"), rng_(std::random_device{}()) {
        nh_.param<std::string>("scan_topic", scan_topic_, std::string("/scan"));
        nh_.param<std::string>("frame_id", frame_id_, std::string("base_link"));
        nh_.param<std::string>("drone_model_name", drone_model_name_, std::string("iris"));
        nh_.param<std::string>("model_states_topic", model_states_topic_, std::string("/gazebo/model_states"));
        nh_.param<std::string>("local_pose_topic", local_pose_topic_, std::string("/mavros/local_position/pose"));
        nh_.param("use_gazebo_model_states", use_gazebo_model_states_, true);
        nh_.param("publish_rate", publish_rate_, 20.0);
        nh_.param("landing_surface_z", landing_surface_z_, 0.0);
        nh_.param("min_range", min_range_, 0.05);
        nh_.param("max_range", max_range_, 30.0);
        nh_.param("noise_stddev", noise_stddev_, 0.0);
        nh_.param("source_timeout_sec", source_timeout_sec_, 0.5);

        normal_dist_ = std::normal_distribution<double>(0.0, noise_stddev_);

        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_, 10);
        model_states_sub_ = nh_.subscribe(model_states_topic_, 10, &FakeLidarAltimeter::modelStatesCallback, this);
        local_pose_sub_ = nh_.subscribe(local_pose_topic_, 10, &FakeLidarAltimeter::localPoseCallback, this);
        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)), &FakeLidarAltimeter::timerCallback, this);

        ROS_INFO("[FakeAltimeter] Publishing simulated altimeter scan on %s", scan_topic_.c_str());
    }

private:
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
        if (!use_gazebo_model_states_) {
            return;
        }

        int index = -1;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == drone_model_name_) {
                index = static_cast<int>(i);
                break;
            }
        }

        if (index < 0 || static_cast<size_t>(index) >= msg->pose.size()) {
            return;
        }

        latest_model_z_ = msg->pose[static_cast<size_t>(index)].position.z;
        latest_model_time_ = ros::Time::now();
        has_model_z_ = true;
    }

    void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        latest_pose_z_ = msg->pose.position.z;
        latest_pose_time_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        has_pose_z_ = true;
    }

    void timerCallback(const ros::TimerEvent &) {
        double vehicle_z = 0.0;
        const ros::Time now = ros::Time::now();
        bool has_valid_height_source = false;

        if (use_gazebo_model_states_ && has_model_z_ && (now - latest_model_time_).toSec() <= source_timeout_sec_) {
            vehicle_z = latest_model_z_;
            has_valid_height_source = true;
        } else if (has_pose_z_ && (now - latest_pose_time_).toSec() <= source_timeout_sec_) {
            vehicle_z = latest_pose_z_;
            has_valid_height_source = true;
        }

        sensor_msgs::LaserScan scan;
        scan.header.stamp = now;
        scan.header.frame_id = frame_id_;
        scan.angle_min = 0.0;
        scan.angle_max = 0.0;
        scan.angle_increment = 0.0;
        scan.time_increment = 0.0;
        scan.scan_time = 1.0 / std::max(1.0, publish_rate_);
        scan.range_min = static_cast<float>(min_range_);
        scan.range_max = static_cast<float>(max_range_);
        scan.ranges.resize(1, std::numeric_limits<float>::quiet_NaN());

        if (has_valid_height_source) {
            double range = vehicle_z - landing_surface_z_;
            if (noise_stddev_ > 0.0) {
                range += normal_dist_(rng_);
            }
            range = std::max(min_range_, std::min(max_range_, range));
            scan.ranges[0] = static_cast<float>(range);
        }

        scan_pub_.publish(scan);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher scan_pub_;
    ros::Subscriber model_states_sub_;
    ros::Subscriber local_pose_sub_;
    ros::Timer timer_;

    std::string scan_topic_;
    std::string frame_id_;
    std::string drone_model_name_;
    std::string model_states_topic_;
    std::string local_pose_topic_;

    bool use_gazebo_model_states_ = true;
    double publish_rate_ = 20.0;
    double landing_surface_z_ = 0.0;
    double min_range_ = 0.05;
    double max_range_ = 30.0;
    double noise_stddev_ = 0.0;
    double source_timeout_sec_ = 0.5;

    bool has_model_z_ = false;
    bool has_pose_z_ = false;
    double latest_model_z_ = 0.0;
    double latest_pose_z_ = 0.0;
    ros::Time latest_model_time_;
    ros::Time latest_pose_time_;

    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_lidar_altimeter_node");
    FakeLidarAltimeter node;
    ros::spin();
    return 0;
}
