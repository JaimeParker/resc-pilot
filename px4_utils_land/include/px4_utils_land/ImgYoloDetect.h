#ifndef PX4_UTILS_LAND_IMAGE_YOLODETECT_H
#define PX4_UTILS_LAND_IMAGE_YOLODETECT_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <onnxruntime_cxx_api.h>
    
namespace px4_utils_land {

class Imgyolodetect {
private:
    std::string downward_camera_topic_ = "/camera/color/image_raw";
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_sub_;

    // 推理并行线程
    int opnumthreads_;

    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "YOLO"};
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    std::string input_name, output_name;
    std::vector<const char*> input_node_names, output_node_names;

    std::string model_path_;    
    double conf_thres_;
    double iou_thres_;
    cv::Size input_size_;
    
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
    Imgyolodetect();
    ~Imgyolodetect();
    void init(ros::NodeHandle& nh);
    void setCameraParams(float fx, float fy);
    void setHoldPos(float pos);
    void setLandPos(float pos);
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

#endif // PX4_UTILS_LAND_IMAGE_YOLODETECT_H