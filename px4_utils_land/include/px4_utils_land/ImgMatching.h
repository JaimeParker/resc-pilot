#ifndef PX4_UTILS_LAND_IMAGE_MATCHING_H
#define PX4_UTILS_LAND_IMAGE_MATCHING_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <thread>
#include <atomic>
#include <mutex>

    
namespace px4_utils_land {

class Imgmatching {
private:
    // Non-blocking selection members
    std::atomic<bool> selecting_{false};
    std::atomic<bool> selection_done_{false};
    std::thread selection_thread_;
    std::mutex selection_mtx_;
    cv::Mat selection_image_;  // color image used for selection

    std::string target_path;
    std::string downward_camera_topic_ = "/camera/color/image_raw";
    std::unique_ptr<image_transport::ImageTransport> it_;
    std::vector<cv::KeyPoint> target_kps_;
    image_transport::Subscriber image_sub_;

    cv::Mat target_image_;
    cv::Mat target_desc_;
    cv::Point2f centroid_;
    cv::Point2f target_point_;
    cv::Ptr<cv::AKAZE> orb_;
    cv::BFMatcher matcher_;
    cv::Point2f last_centroid_;
    cv::Point2f offset_;  // pixel plant

    bool matched_;
    bool first_frame_ = true;
    bool use_clahe_ = false; 
    bool enabled_ = true;  // 控制是否进行图像匹配

    float filter_alpha_ = 0.995f; // for centroid filtering
    float z_value; // z value of the hold position 
    float current_depth_;  // 当前深度值

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void preprocessImage(const sensor_msgs::ImageConstPtr& msg, cv::Mat& frame, cv::Mat& gray);
    void initializeTarget(const cv::Mat& frame, const cv::Mat& gray);
    bool computeHomographyInliers(const std::vector<cv::KeyPoint>& target_kps, const cv::Mat& target_desc, const std::vector<cv::KeyPoint>& frame_kps, const cv::Mat& frame_desc, std::vector<cv::DMatch>& inlier_matches, cv::Mat& H);
    void updateOffsetWithFilter(const cv::Mat& gray, const cv::Point2f& centroid);
  
    cv::Point2f projectTargetPoint(const cv::Mat& H, const cv::Point2f& target_point);
    cv::Point2f chooseTargetPoint(const cv::Mat& image);

public:
    Imgmatching();
    void init(ros::NodeHandle& nh);
    void setCameraParams(float fx, float fy);
    void setHoldPos(float pos);
    void setLandPos(float pos);
    void setTargetPath(const std::string& path);
    void disableMatching();                    // 禁用图像匹配

    int frame_count_ = 0;
    bool isTargetMatched() const;
    bool isSelectionDone() const { return selection_done_.load(); }
    cv::Point2f getTargetCentroid() const;
    cv::Point2f getOffset() const;
    
    float fx_; // focal length in x
    float fy_; // focal length in y
    float land_pos_z_; // z position of landing target
    float getCurrentDepth() const;

    template<typename T>
    void getParamWithWarning(ros::NodeHandle& nh, const std::string& param_name, T& param) {
        if (!nh.getParam(param_name, param)) {
            ROS_WARN_STREAM("Failed to get param: " << param_name);
        }
    }
};

} 

#endif // PX4_UTILS_LAND_IMAGE_MATCHING_H