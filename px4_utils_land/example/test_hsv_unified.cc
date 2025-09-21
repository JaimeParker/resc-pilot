/**
 * @file test_hsv_unified.cc
 * @brief 统一HSV颜色空间亮白色信标检测测试程序
 * 
 * 该程序整合了ROS实时处理和离线文件处理功能，支持：
 * - ROS话题实时订阅
 * - rosbag文件回放处理
 * - 图片和视频文件处理
 * - 摄像头直接输入
 * 
 * 使用方法：
 * 1. ROS实时模式: rosrun px4_utils_land test_hsv_unified
 * 2. rosbag模式: rosrun px4_utils_land test_hsv_unified _use_rosbag:=true _rosbag_path:=/path/to/bag
 * 3. 离线文件模式: rosrun px4_utils_land test_hsv_unified _use_offline:=true _file_path:=/path/to/file
 * 
 * @author zhiyuan-yang
 * @date 2024
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <memory>

class HSVUnifiedTester {
public:
    enum OperationMode {
        ROS_LIVE,      // 实时ROS话题
        ROSBAG_REPLAY, // rosbag回放
        OFFLINE_FILE   // 离线文件处理
    };

private:
    // ROS相关
    ros::NodeHandle nh_;
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_sub_;
    
    // 操作模式
    OperationMode mode_;
    std::string rosbag_path_;
    std::string file_path_;
    std::string image_topic_;
    
    // HSV参数（默认值用于检测亮白色）
    int hsv_h_min_ = 0;     // 色调最小值
    int hsv_h_max_ = 180;   // 色调最大值
    int hsv_s_min_ = 0;     // 饱和度最小值
    int hsv_s_max_ = 30;    // 饱和度最大值
    int hsv_v_min_ = 200;   // 亮度最小值
    int hsv_v_max_ = 255;   // 亮度最大值
    
    // 图像处理参数
    int morph_kernel_size_ = 5;
    double min_contour_area_ = 100.0;
    
    // 显示控制
    bool show_original_ = true;
    bool show_hsv_mask_ = true;
    bool show_result_ = true;
    bool paused_ = false;
    
    // 离线处理相关
    cv::VideoCapture cap_;
    cv::Mat current_frame_;
    
    // rosbag处理相关
    std::unique_ptr<rosbag::Bag> bag_;
    bool rosbag_playing_ = false;
    std::thread rosbag_thread_;

public:
    HSVUnifiedTester() : nh_("~") {
        // 获取参数
        bool use_rosbag = false;
        bool use_offline = false;
        
        nh_.param<bool>("use_rosbag", use_rosbag, false);
        nh_.param<bool>("use_offline", use_offline, false);
        nh_.param<std::string>("rosbag_path", rosbag_path_, "");
        nh_.param<std::string>("file_path", file_path_, "");
        nh_.param<std::string>("image_topic", image_topic_, "/camera/color/image_raw");
        
        // 如果未指定路径，使用默认路径
        if (use_rosbag && rosbag_path_.empty()) {
            rosbag_path_ = "/home/zhiyuan-yang/Videos/DroneLanding-bag/rgb-indoor-green-lamp.bag";
        }
        
        // 确定操作模式
        if (use_rosbag) {
            mode_ = ROSBAG_REPLAY;
        } else if (use_offline) {
            mode_ = OFFLINE_FILE;
        } else {
            mode_ = ROS_LIVE;
        }
        
        printModeInfo();
        
        // 根据模式初始化
        if (!initialize()) {
            throw std::runtime_error("Failed to initialize HSV tester");
        }
    }
    
    ~HSVUnifiedTester() {
        cleanup();
    }

private:
    void printModeInfo() {
        std::cout << "\n=== HSV Beacon Detection Unified Tester ===" << std::endl;
        
        switch (mode_) {
            case ROS_LIVE:
                std::cout << "Mode: ROS Live Processing" << std::endl;
                std::cout << "Topic: " << image_topic_ << std::endl;
                break;
            case ROSBAG_REPLAY:
                std::cout << "Mode: Rosbag Replay" << std::endl;
                std::cout << "Bag file: " << rosbag_path_ << std::endl;
                std::cout << "Topic: " << image_topic_ << std::endl;
                break;
            case OFFLINE_FILE:
                std::cout << "Mode: Offline File Processing" << std::endl;
                std::cout << "File: " << file_path_ << std::endl;
                break;
        }
        
        std::cout << "\nControls:" << std::endl;
        std::cout << "  - Use trackbars to adjust HSV thresholds" << std::endl;
        std::cout << "  - SPACE: Pause/Resume" << std::endl;
        std::cout << "  - 's': Save HSV parameters" << std::endl;
        std::cout << "  - 'r': Reset to default values" << std::endl;
        std::cout << "  - 'h': Toggle help display" << std::endl;
        std::cout << "  - 'q' or ESC: Exit program" << std::endl;
        std::cout << "==========================================" << std::endl;
    }
    
    bool initialize() {
        // 创建窗口和滑动条
        setupWindows();
        
        switch (mode_) {
            case ROS_LIVE:
                return initializeRosLive();
            case ROSBAG_REPLAY:
                return initializeRosbag();
            case OFFLINE_FILE:
                return initializeOfflineFile();
        }
        return false;
    }
    
    bool initializeRosLive() {
        it_ = std::make_unique<image_transport::ImageTransport>(nh_);
        image_sub_ = it_->subscribe(image_topic_, 1, &HSVUnifiedTester::imageCallback, this);
        std::cout << "Subscribed to ROS topic: " << image_topic_ << std::endl;
        return true;
    }
    
    bool initializeRosbag() {
        try {
            bag_ = std::make_unique<rosbag::Bag>();
            bag_->open(rosbag_path_, rosbag::bagmode::Read);
            
            // 检查话题是否存在
            rosbag::View view(*bag_);
            bool topic_found = false;
            for (const rosbag::ConnectionInfo* info : view.getConnections()) {
                if (info->topic == image_topic_) {
                    topic_found = true;
                    break;
                }
            }
            
            if (!topic_found) {
                std::cerr << "Error: Topic " << image_topic_ << " not found in bag file" << std::endl;
                std::cerr << "Available topics:" << std::endl;
                for (const rosbag::ConnectionInfo* info : view.getConnections()) {
                    std::cerr << "  - " << info->topic << " (" << info->datatype << ")" << std::endl;
                }
                return false;
            }
            
            std::cout << "Loaded rosbag: " << rosbag_path_ << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error opening rosbag: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool initializeOfflineFile() {
        if (file_path_.empty()) {
            std::cerr << "Error: No file path specified for offline mode" << std::endl;
            return false;
        }
        
        // 尝试作为数字（摄像头ID）解析
        try {
            int camera_id = std::stoi(file_path_);
            cap_.open(camera_id);
            if (cap_.isOpened()) {
                std::cout << "Opened camera " << camera_id << std::endl;
                return true;
            }
        } catch (...) {
            // 不是数字，继续尝试作为文件
        }
        
        // 尝试作为视频文件打开
        cap_.open(file_path_);
        if (cap_.isOpened()) {
            std::cout << "Opened video file: " << file_path_ << std::endl;
            return true;
        }
        
        // 尝试作为图片读取
        current_frame_ = cv::imread(file_path_);
        if (!current_frame_.empty()) {
            std::cout << "Loaded image: " << file_path_ << std::endl;
            return true;
        }
        
        std::cerr << "Error: Could not open " << file_path_ << std::endl;
        return false;
    }
    
    void setupWindows() {
        // 创建控制窗口
        cv::namedWindow("HSV Controls", cv::WINDOW_AUTOSIZE);
        
        // 创建HSV阈值滑动条
        cv::createTrackbar("H Min", "HSV Controls", &hsv_h_min_, 179);
        cv::createTrackbar("H Max", "HSV Controls", &hsv_h_max_, 179);
        cv::createTrackbar("S Min", "HSV Controls", &hsv_s_min_, 255);
        cv::createTrackbar("S Max", "HSV Controls", &hsv_s_max_, 255);
        cv::createTrackbar("V Min", "HSV Controls", &hsv_v_min_, 255);
        cv::createTrackbar("V Max", "HSV Controls", &hsv_v_max_, 255);
        
        // 形态学和面积阈值滑动条
        cv::createTrackbar("Kernel Size", "HSV Controls", &morph_kernel_size_, 20);
        int area_trackbar = static_cast<int>(min_contour_area_);
        cv::createTrackbar("Min Area", "HSV Controls", &area_trackbar, 2000, 
                          [](int val, void* userdata) {
                              static_cast<HSVUnifiedTester*>(userdata)->min_contour_area_ = val;
                          }, this);
        
        // 显示选项
        int show_orig = show_original_ ? 1 : 0;
        int show_mask = show_hsv_mask_ ? 1 : 0;
        int show_res = show_result_ ? 1 : 0;
        cv::createTrackbar("Show Original", "HSV Controls", &show_orig, 1,
                          [](int val, void* userdata) {
                              static_cast<HSVUnifiedTester*>(userdata)->show_original_ = val == 1;
                          }, this);
        cv::createTrackbar("Show HSV Mask", "HSV Controls", &show_mask, 1,
                          [](int val, void* userdata) {
                              static_cast<HSVUnifiedTester*>(userdata)->show_hsv_mask_ = val == 1;
                          }, this);
        cv::createTrackbar("Show Result", "HSV Controls", &show_res, 1,
                          [](int val, void* userdata) {
                              static_cast<HSVUnifiedTester*>(userdata)->show_result_ = val == 1;
                          }, this);
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            current_frame_ = cv_bridge::toCvShare(msg, "bgr8")->image;
            processCurrentFrame();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        }
    }
    
    void processCurrentFrame() {
        if (current_frame_.empty()) return;
        
        // HSV颜色空间分割检测
        cv::Mat hsv, mask;
        cv::cvtColor(current_frame_, hsv, cv::COLOR_BGR2HSV);
        
        // 创建HSV阈值掩膜
        cv::inRange(hsv, cv::Scalar(hsv_h_min_, hsv_s_min_, hsv_v_min_), 
                        cv::Scalar(hsv_h_max_, hsv_s_max_, hsv_v_max_), mask);
        
        // 形态学操作去除噪声
        if (morph_kernel_size_ > 0) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                                      cv::Size(morph_kernel_size_, morph_kernel_size_));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        }
        
        // 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // 创建结果图像
        cv::Mat result = current_frame_.clone();
        
        // 绘制图像中心十字线
        cv::Point2f center(static_cast<float>(result.cols) / 2.0f, 
                          static_cast<float>(result.rows) / 2.0f);
        cv::drawMarker(result, center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);
        
        // 分析轮廓
        double max_area = 0;
        int max_contour_idx = -1;
        cv::Point2f beacon_center;
        bool beacon_found = false;
        
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area && area > min_contour_area_) {
                max_area = area;
                max_contour_idx = static_cast<int>(i);
            }
        }
        
        if (max_contour_idx >= 0) {
            // 绘制最大轮廓
            cv::drawContours(result, contours, max_contour_idx, cv::Scalar(0, 255, 0), 2);
            
            // 计算质心
            cv::Moments moments = cv::moments(contours[max_contour_idx]);
            if (moments.m00 > 0) {
                beacon_center.x = static_cast<float>(moments.m10 / moments.m00);
                beacon_center.y = static_cast<float>(moments.m01 / moments.m00);
                beacon_found = true;
                
                // 绘制质心
                cv::circle(result, beacon_center, 8, cv::Scalar(0, 0, 255), -1);
                
                // 绘制偏移向量
                cv::arrowedLine(result, center, beacon_center, cv::Scalar(255, 255, 0), 2);
            }
        }
        
        // 绘制所有检测到的轮廓（灰色）
        for (size_t i = 0; i < contours.size(); i++) {
            if (static_cast<int>(i) != max_contour_idx) {
                double area = cv::contourArea(contours[i]);
                if (area > min_contour_area_) {
                    cv::drawContours(result, contours, static_cast<int>(i), cv::Scalar(128, 128, 128), 1);
                }
            }
        }
        
        // 添加信息文本
        std::vector<std::string> info_lines;
        
        // 显示当前模式
        std::string mode_str;
        switch (mode_) {
            case ROS_LIVE: mode_str = "ROS Live"; break;
            case ROSBAG_REPLAY: mode_str = "Rosbag"; break;
            case OFFLINE_FILE: mode_str = "Offline"; break;
        }
        info_lines.push_back("Mode: " + mode_str);
        
        info_lines.push_back("HSV: [" + std::to_string(hsv_h_min_) + "," + std::to_string(hsv_h_max_) + 
                           "] [" + std::to_string(hsv_s_min_) + "," + std::to_string(hsv_s_max_) + 
                           "] [" + std::to_string(hsv_v_min_) + "," + std::to_string(hsv_v_max_) + "]");
        info_lines.push_back("Contours: " + std::to_string(contours.size()) + 
                           " (min area: " + std::to_string(static_cast<int>(min_contour_area_)) + ")");
        
        if (beacon_found) {
            cv::Point2f offset = beacon_center - center;
            info_lines.push_back("Beacon: (" + std::to_string(static_cast<int>(beacon_center.x)) + 
                               "," + std::to_string(static_cast<int>(beacon_center.y)) + 
                               ") Area: " + std::to_string(static_cast<int>(max_area)));
            info_lines.push_back("Offset: (" + std::to_string(static_cast<int>(offset.x)) + 
                               "," + std::to_string(static_cast<int>(offset.y)) + ")");
        } else {
            info_lines.push_back("No beacon detected");
        }
        
        if (paused_) {
            info_lines.push_back("PAUSED - Press SPACE to continue");
        }
        
        // 绘制信息文本
        for (size_t i = 0; i < info_lines.size(); i++) {
            cv::putText(result, info_lines[i], cv::Point(10, 30 + static_cast<int>(i) * 25), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
            cv::putText(result, info_lines[i], cv::Point(10, 30 + static_cast<int>(i) * 25), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
        }
        
        // 显示窗口
        if (show_original_) {
            cv::imshow("Original", current_frame_);
        }
        if (show_hsv_mask_) {
            cv::imshow("HSV Mask", mask);
        }
        if (show_result_) {
            cv::imshow("Detection Result", result);
        }
        
        // 处理按键
        handleKeyPress();
    }
    
    void handleKeyPress() {
        int key = cv::waitKey(1) & 0xFF;
        
        switch (key) {
            case 'q':
            case 'Q':
            case 27: // ESC
                std::cout << "Exiting..." << std::endl;
                if (mode_ == ROS_LIVE) {
                    ros::shutdown();
                } else {
                    cleanup();
                    exit(0);
                }
                break;
                
            case ' ': // SPACE
                paused_ = !paused_;
                std::cout << (paused_ ? "Paused" : "Resumed") << std::endl;
                break;
                
            case 's':
            case 'S':
                saveParameters();
                break;
                
            case 'r':
            case 'R':
                resetToDefaults();
                break;
                
            case 'h':
            case 'H':
                printHelp();
                break;
        }
    }
    
    void saveParameters() {
        std::string filename = "hsv_unified_params.yaml";
        std::ofstream file(filename);
        
        if (file.is_open()) {
            file << "# HSV Beacon Detection Parameters (Unified Tester)" << std::endl;
            file << "# Generated from mode: ";
            switch (mode_) {
                case ROS_LIVE: file << "ROS Live"; break;
                case ROSBAG_REPLAY: file << "Rosbag Replay"; break;
                case OFFLINE_FILE: file << "Offline File"; break;
            }
            file << std::endl << std::endl;
            
            file << "hsv:" << std::endl;
            file << "  h_min: " << hsv_h_min_ << std::endl;
            file << "  h_max: " << hsv_h_max_ << std::endl;
            file << "  s_min: " << hsv_s_min_ << std::endl;
            file << "  s_max: " << hsv_s_max_ << std::endl;
            file << "  v_min: " << hsv_v_min_ << std::endl;
            file << "  v_max: " << hsv_v_max_ << std::endl;
            file << "morphology:" << std::endl;
            file << "  kernel_size: " << morph_kernel_size_ << std::endl;
            file << "detection:" << std::endl;
            file << "  min_area: " << static_cast<int>(min_contour_area_) << std::endl;
            
            file.close();
            std::cout << "Parameters saved to: " << filename << std::endl;
        } else {
            std::cerr << "Error: Could not save parameters to file" << std::endl;
        }
    }
    
    void resetToDefaults() {
        hsv_h_min_ = 0;
        hsv_h_max_ = 180;
        hsv_s_min_ = 0;
        hsv_s_max_ = 30;
        hsv_v_min_ = 200;
        hsv_v_max_ = 255;
        morph_kernel_size_ = 5;
        min_contour_area_ = 100.0;
        
        // 更新滑动条
        cv::setTrackbarPos("H Min", "HSV Controls", hsv_h_min_);
        cv::setTrackbarPos("H Max", "HSV Controls", hsv_h_max_);
        cv::setTrackbarPos("S Min", "HSV Controls", hsv_s_min_);
        cv::setTrackbarPos("S Max", "HSV Controls", hsv_s_max_);
        cv::setTrackbarPos("V Min", "HSV Controls", hsv_v_min_);
        cv::setTrackbarPos("V Max", "HSV Controls", hsv_v_max_);
        cv::setTrackbarPos("Kernel Size", "HSV Controls", morph_kernel_size_);
        cv::setTrackbarPos("Min Area", "HSV Controls", static_cast<int>(min_contour_area_));
        
        std::cout << "Parameters reset to defaults" << std::endl;
    }
    
    void printHelp() {
        std::cout << "\n=== HSV Unified Tester Help ===" << std::endl;
        std::cout << "Current mode: ";
        switch (mode_) {
            case ROS_LIVE: std::cout << "ROS Live Processing"; break;
            case ROSBAG_REPLAY: std::cout << "Rosbag Replay"; break;
            case OFFLINE_FILE: std::cout << "Offline File Processing"; break;
        }
        std::cout << std::endl << std::endl;
        
        std::cout << "Usage Examples:" << std::endl;
        std::cout << "  ROS Live: rosrun px4_utils_land test_hsv_unified" << std::endl;
        std::cout << "  Rosbag: rosrun px4_utils_land test_hsv_unified _use_rosbag:=true _rosbag_path:=/path/to/bag" << std::endl;
        std::cout << "  Offline: rosrun px4_utils_land test_hsv_unified _use_offline:=true _file_path:=/path/to/file" << std::endl;
        std::cout << std::endl;
        
        std::cout << "HSV Parameters for white beacon:" << std::endl;
        std::cout << "  H: [0, 180] (full range for white)" << std::endl;
        std::cout << "  S: [0, 30] (low saturation for white)" << std::endl;
        std::cout << "  V: [200, 255] (high brightness for white)" << std::endl;
        std::cout << "===============================" << std::endl;
    }
    
    void processRosbag() {
        if (!bag_ || paused_) {
            ros::Duration(0.1).sleep();
            return;
        }
        
        try {
            rosbag::View view(*bag_, rosbag::TopicQuery(image_topic_));
            
            for (const rosbag::MessageInstance& msg : view) {
                if (!ros::ok() || paused_) {
                    ros::Duration(0.1).sleep();
                    continue;
                }
                
                sensor_msgs::Image::ConstPtr image_msg = msg.instantiate<sensor_msgs::Image>();
                if (image_msg) {
                    try {
                        current_frame_ = cv_bridge::toCvShare(image_msg, "bgr8")->image;
                        processCurrentFrame();
                        
                        // 控制播放速度
                        ros::Duration(0.033).sleep(); // ~30 FPS
                    } catch (cv_bridge::Exception& e) {
                        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
                    }
                }
            }
            
            // 循环播放
            std::cout << "Rosbag finished, restarting..." << std::endl;
            
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error processing rosbag: " << e.what());
        }
    }
    
    void processOfflineFile() {
        while (ros::ok()) {
            if (paused_) {
                ros::Duration(0.1).sleep();
                continue;
            }
            
            // 如果是视频或摄像头，读取新帧
            if (cap_.isOpened()) {
                cv::Mat frame;
                if (!cap_.read(frame)) {
                    if (cap_.get(cv::CAP_PROP_POS_FRAMES) == cap_.get(cv::CAP_PROP_FRAME_COUNT)) {
                        // 视频结束，重新开始
                        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                        continue;
                    } else {
                        break;  // 读取失败
                    }
                }
                current_frame_ = frame;
            }
            
            processCurrentFrame();
            ros::Duration(0.033).sleep(); // ~30 FPS
        }
    }
    
    void cleanup() {
        rosbag_playing_ = false;
        if (rosbag_thread_.joinable()) {
            rosbag_thread_.join();
        }
        
        if (bag_) {
            bag_->close();
        }
        
        cv::destroyAllWindows();
    }
    
public:
    void run() {
        switch (mode_) {
            case ROS_LIVE:
                ros::spin();
                break;
            case ROSBAG_REPLAY:
                while (ros::ok()) {
                    processRosbag();
                }
                break;
            case OFFLINE_FILE:
                processOfflineFile();
                break;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hsv_unified_tester");
    
    try {
        HSVUnifiedTester tester;
        tester.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
