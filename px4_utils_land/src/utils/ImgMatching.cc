#include "px4_utils_land/ImgMatching.h"
#include <iomanip>

namespace px4_utils_land {

void Imgmatching::init(ros::NodeHandle& nh) {  
    // 获取相机话题参数
    getParamWithWarning(nh, "camera/image_topic", downward_camera_topic_);
    
    // 获取相机内参
    if(downward_camera_topic_ == "/camera/color/image_raw") {
        getParamWithWarning(nh, "camera/fx_", fx_);
        getParamWithWarning(nh, "camera/fy_", fy_);
    }
    
    // 获取HSV颜色分割参数
    getParamWithWarning(nh, "hsv/h_min", hsv_h_min_);
    getParamWithWarning(nh, "hsv/h_max", hsv_h_max_);
    getParamWithWarning(nh, "hsv/s_min", hsv_s_min_);
    getParamWithWarning(nh, "hsv/s_max", hsv_s_max_);
    getParamWithWarning(nh, "hsv/v_min", hsv_v_min_);
    getParamWithWarning(nh, "hsv/v_max", hsv_v_max_);
    
    // 获取形态学操作参数
    getParamWithWarning(nh, "morphology/kernel_size", morph_kernel_size_);
    
    // 获取最小面积阈值参数  
    getParamWithWarning(nh, "detection/min_area", min_contour_area_);
    
    // 初始化图像传输和订阅器
    it_ = std::make_unique<image_transport::ImageTransport>(nh);
    image_sub_ = it_->subscribe(downward_camera_topic_, 1, &Imgmatching::imageCallback, this);
    
    ROS_INFO_STREAM("HSV Beacon Detection initialized:");
    ROS_INFO_STREAM("  - Subscribed to: " << downward_camera_topic_);
    ROS_INFO_STREAM("  - HSV Range: H[" << hsv_h_min_ << "," << hsv_h_max_ << "] S[" 
                   << hsv_s_min_ << "," << hsv_s_max_ << "] V[" << hsv_v_min_ << "," << hsv_v_max_ << "]");
    ROS_INFO_STREAM("  - Min contour area: " << min_contour_area_);
}

Imgmatching::Imgmatching() : beacon_detected_(false) {
    // 初始化HSV阈值（亮白色信标的默认值）
    hsv_h_min_ = 0;    
    hsv_h_max_ = 180;  
    hsv_s_min_ = 0;    
    hsv_s_max_ = 30;   
    hsv_v_min_ = 200;  
    hsv_v_max_ = 255;  
    
    // 形态学操作内核大小
    morph_kernel_size_ = 5;
    
    // 最小轮廓面积阈值
    min_contour_area_ = 100.0;
}

Imgmatching::~Imgmatching() {
    // 安全地销毁所有OpenCV窗口
    try {
        cv::destroyAllWindows();
        cv::waitKey(1);
    } catch (const cv::Exception& e) {
        // 静默忽略销毁异常，避免程序崩溃
    }
}

void Imgmatching::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!enabled_) { 
        return;
    }
    
    frame_count_++;
    if (frame_count_ % 3 != 0) {
        return;
    }
    
    beacon_detected_ = false;
    
    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        // HSV颜色空间分割检测亮白色信标
        cv::Mat hsv, mask, result;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        // 创建HSV阈值掩膜
        cv::inRange(hsv, cv::Scalar(hsv_h_min_, hsv_s_min_, hsv_v_min_), 
                        cv::Scalar(hsv_h_max_, hsv_s_max_, hsv_v_max_), mask);
        
        // 形态学操作去除噪声
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                                  cv::Size(morph_kernel_size_, morph_kernel_size_));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        
        // 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        if (!contours.empty()) {
            // 找到最大面积的轮廓作为信标
            double max_area = 0;
            int max_contour_idx = -1;
            
            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > max_area && area > min_contour_area_) {
                    max_area = area;
                    max_contour_idx = static_cast<int>(i);
                }
            }
            
            if (max_contour_idx >= 0) {
                // 计算信标质心
                cv::Moments moments = cv::moments(contours[max_contour_idx]);
                if (moments.m00 > 0) {
                    cv::Point2f detected_centroid(
                        static_cast<float>(moments.m10 / moments.m00),
                        static_cast<float>(moments.m01 / moments.m00)
                    );

                    // update centroid and offset
                    updateCentroidAndOffset(detected_centroid, frame.size());
                    beacon_detected_ = true;
                    
                    cv::Mat display = frame.clone();
                    
                    cv::drawContours(display, contours, max_contour_idx, cv::Scalar(0, 255, 0), 2);

                    // draw centroid
                    cv::circle(display, centroid_, 8, cv::Scalar(0, 0, 255), -1);

                    // draw image center cross
                    cv::Point2f center(static_cast<float>(frame.cols) / 2.0f, 
                                      static_cast<float>(frame.rows) / 2.0f);
                    cv::drawMarker(display, center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);
                    
                    // draw offset vector
                    cv::arrowedLine(display, center, centroid_, cv::Scalar(255, 255, 0), 2);
                    
                    std::string info = "Area: " + std::to_string(static_cast<int>(max_area)) + 
                                      " Offset: (" + std::to_string(static_cast<int>(offset_.x)) + 
                                      "," + std::to_string(static_cast<int>(offset_.y)) + ")";
                    cv::putText(display, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                               0.6, cv::Scalar(255, 255, 255), 2);
                    
                    cv::imshow("Beacon Detection", display);
                    cv::imshow("HSV Mask", mask);
                }
            }
        } else {
            // show center cross even if no beacon detected
            cv::Mat display = frame.clone();
            cv::Point2f center(static_cast<float>(frame.cols) / 2.0f, 
                              static_cast<float>(frame.rows) / 2.0f);
            cv::drawMarker(display, center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);
            cv::putText(display, "No beacon detected", cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            
            cv::imshow("Beacon Detection", display);
            cv::imshow("HSV Mask", mask);
        }
        
        cv::waitKey(1);
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
}

void Imgmatching::updateCentroidAndOffset(const cv::Point2f& detected_centroid, const cv::Size& image_size) {
    // the image center point
    cv::Point2f center(static_cast<float>(image_size.width) / 2.0f, 
                      static_cast<float>(image_size.height) / 2.0f);
    
    if (z_value > land_pos_z_) {
        center.y -= 0.04f * fy_ / (z_value - land_pos_z_);
    }
    
    // filter the centroid to reduce noise
    if (first_detection_) {
        centroid_ = detected_centroid;
        first_detection_ = false;
    } else {
        centroid_.x = filter_alpha_ * detected_centroid.x + (1.0f - filter_alpha_) * centroid_.x;
        centroid_.y = filter_alpha_ * detected_centroid.y + (1.0f - filter_alpha_) * centroid_.y;
    }
    
    offset_ = centroid_ - center;
}

bool Imgmatching::isTargetMatched() const {
    return beacon_detected_;
}

cv::Point2f Imgmatching::getTargetCentroid() const {
    return centroid_;
}

cv::Point2f Imgmatching::getOffset() const {
    if (z_value <= land_pos_z_) {
        return cv::Point2f(0.0f, 0.0f);  // avoid division by zero or negative values
    }
    
    float depth = z_value - land_pos_z_;
    return cv::Point2f(offset_.x * depth / fx_, offset_.y * depth / fy_);
}

void Imgmatching::setCameraParams(float fx, float fy) {
    fx_ = fx;
    fy_ = fy;
    ROS_INFO_STREAM("Camera parameters set: fx=" << fx_ << ", fy=" << fy_);
}

void Imgmatching::setHoldPos(float pos) {
    z_value = pos;
}

void Imgmatching::setLandPos(float pos) {
    land_pos_z_ = pos;
}

// zhiyuan: no use, so ignore. but keep it for future reference
void Imgmatching::setHSVThresholds(int h_min, int h_max, int s_min, int s_max, int v_min, int v_max) {
    hsv_h_min_ = h_min;
    hsv_h_max_ = h_max;
    hsv_s_min_ = s_min;
    hsv_s_max_ = s_max;
    hsv_v_min_ = v_min;
    hsv_v_max_ = v_max;
    
    ROS_INFO_STREAM("HSV thresholds updated: H[" << h_min << "," << h_max 
                   << "] S[" << s_min << "," << s_max 
                   << "] V[" << v_min << "," << v_max << "]");
}

void Imgmatching::setMinContourArea(double min_area) {
    min_contour_area_ = min_area;
    ROS_INFO_STREAM("Min contour area updated: " << min_area);
}

void Imgmatching::setMorphKernelSize(int kernel_size) {
    morph_kernel_size_ = kernel_size;
    ROS_INFO_STREAM("Morphology kernel size updated: " << kernel_size);
}

void Imgmatching::disableMatching() {
    enabled_ = false;
    
    // destroy OpenCV windows safely
    try {
        cv::destroyAllWindows();
        cv::waitKey(1);
    } catch (const cv::Exception& e) {
        ROS_WARN_STREAM("OpenCV window destruction warning: " << e.what());
    }
    
    ROS_INFO("HSV beacon detection disabled");
}

float Imgmatching::getCurrentDepth() const {
    return z_value - land_pos_z_;
}

} // namespace px4_utils_land