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
#include <string>

    
namespace px4_utils_land {

class Imgmatching {
private:
    std::string downward_camera_topic_ = "/camera/color/image_raw";
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_sub_;
    
    int hsv_h_min_ = 0;   
    int hsv_h_max_ = 180;  
    int hsv_s_min_ = 0;     
    int hsv_s_max_ = 30;   
    int hsv_v_min_ = 200;  
    int hsv_v_max_ = 255;  
    
   
    int morph_kernel_size_ = 5;     
    double min_contour_area_ = 100.0; 
    
    cv::Point2f centroid_;        
    cv::Point2f offset_;          
    bool beacon_detected_ = false; 
    bool first_detection_ = true;  
    bool enabled_ = true;          
   
    float filter_alpha_ = 0.8f;   
    float z_value = 0.0f;            
    
    int frame_count_ = 0;        
  
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void updateCentroidAndOffset(const cv::Point2f& detected_centroid, const cv::Size& image_size);

public:
    Imgmatching();
    ~Imgmatching();
    void init(ros::NodeHandle& nh);
    void setCameraParams(float fx, float fy);
    void setHoldPos(float pos);
    void setLandPos(float pos);
    void setHSVThresholds(int h_min, int h_max, int s_min, int s_max, int v_min, int v_max);
    void setMinContourArea(double min_area);
    void setMorphKernelSize(int kernel_size);
    void disableMatching();

    bool isTargetMatched() const;

    cv::Point2f getTargetCentroid() const;
    cv::Point2f getOffset() const;

    float getCurrentDepth() const;
    float fx_ = 525.0f; 
    float fy_ = 525.0f; 
    float land_pos_z_ = 0.0f;  
    
    template<typename T>
    void getParamWithWarning(ros::NodeHandle& nh, const std::string& param_name, T& param) {
        if (!nh.getParam(param_name, param)) {
            ROS_WARN_STREAM("Failed to get param: " << param_name << ", using default value");
        }
    }
};

} 

#endif // PX4_UTILS_LAND_IMAGE_MATCHING_H