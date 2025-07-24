#ifndef PX4_UTILS_IMAGE_MATCHING_H
#define PX4_UTILS_IMAGE_MATCHING_H

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

namespace px4_utils {
class Imgmatching {
private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::Mat target_image_;
    cv::Mat target_desc_;
    cv::Point2f centroid_;
    std::vector<cv::KeyPoint> target_kps_;

    // Choose AKAZE/ORB for feature matching
    cv::Ptr<cv::AKAZE> orb_;
    cv::BFMatcher matcher_;
    bool matched_;

    bool first_frame_ = true;
    cv::Point2f last_centroid_;
    cv::Point2f offset_;  // pixel plant
    float filter_alpha_ = 0.3f; // for centroid filtering

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void loadTargetImage(const std::string& path);

public:
    Imgmatching(ros::NodeHandle& nh, const std::string& image_topic = "/camera/rgb/image_raw");

    bool isTargetMatched() const;
    cv::Point2f getTargetCentroid() const;
    cv::Point2f getOffset() const;
    void setCameraParams(float fx, float fy, float z);


    float fx_ = 562.94; // focal length in x
    float fy_ = 422.21; // focal length in y
    float land_pos_z_ = 3.0f; // z position of landing target
    float z_ ; 
};

} // namespace px4_utils

#endif // PX4_UTILS_IMAGE_MATCHING_H