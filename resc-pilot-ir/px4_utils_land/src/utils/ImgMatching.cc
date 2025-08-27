#include "px4_utils_land/ImgMatching.h"

namespace px4_utils_land {

void Imgmatching::init(ros::NodeHandle& nh) {  
    getParamWithWarning(nh, "camera/image_topic", downward_camera_topic_);
    getParamWithWarning(nh, "camera/use_clahe_", use_clahe_);
    
    // 添加自动检测相关参数
    bool auto_detect_ir = true;
    getParamWithWarning(nh, "camera/auto_detect_ir", auto_detect_ir);
    
    if(downward_camera_topic_ == "/camera/rgb/image_raw") {
        getParamWithWarning(nh, "camera/fx_", fx_);
        getParamWithWarning(nh, "camera/fy_", fy_);
    }               
    // Initialize image transport and subscriber
    it_ = std::make_unique<image_transport::ImageTransport>(nh);
    image_sub_ = it_->subscribe(downward_camera_topic_, 1, &Imgmatching::imageCallback, this);
    ROS_INFO_STREAM("Imgmatching initialized and subscribed to: " << downward_camera_topic_);
    ROS_INFO_STREAM("Auto IR detection enabled: " << (auto_detect_ir ? "true" : "false"));
}

Imgmatching::Imgmatching()
    : matcher_(cv::NORM_HAMMING), matched_(false) {
    orb_ = cv::AKAZE::create(); 
}

void Imgmatching::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!enabled_) { return;}
    frame_count_++;
    if (frame_count_ % 3 != 0) {
        return;
    }
    matched_ = false;

    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        // 简化的红外灯光跟踪：直接检测当前帧的红外灯光位置
        cv::Point2f detected_ir = detectIRLight(frame);
        
        if (detected_ir.x >= 0 && detected_ir.y >= 0) {
            // 成功检测到红外灯光
            centroid_ = detected_ir;
            updateOffsetWithFilter(frame, centroid_);
            matched_ = true;
            
            // 可视化结果
            cv::Mat vis = frame.clone();
            
            // 绘制轨迹
            static std::vector<cv::Point2f> traj;
            traj.push_back(centroid_);
            
            // 限制轨迹长度以避免内存过度使用
            if (traj.size() > 50) {
                traj.erase(traj.begin());
            }
            
            for (size_t i = 1; i < traj.size(); ++i) {
                cv::line(vis, traj[i - 1], traj[i], cv::Scalar(255, 0, 0), 2);
            }
            
            // 绘制当前检测到的红外灯光位置
            cv::circle(vis, centroid_, 8, cv::Scalar(0, 255, 0), 2);
            cv::circle(vis, centroid_, 3, cv::Scalar(0, 255, 255), -1);
            
            // 绘制图像中心和偏移
            cv::Point2f center(frame.cols / 2.0f, frame.rows / 2.0f);
            cv::circle(vis, center, 5, cv::Scalar(255, 255, 255), 1);
            cv::line(vis, center, centroid_, cv::Scalar(0, 0, 255), 2);
            
            // 显示状态信息
            cv::putText(vis, "IR Light Tracked", cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            cv::putText(vis, "Offset: (" + std::to_string(static_cast<int>(offset_.x)) + 
                       ", " + std::to_string(static_cast<int>(offset_.y)) + ")", 
                       cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
            
            cv::imshow("IR Light Tracking", vis);
            cv::waitKey(1);
            
        } else {
            // 未检测到红外灯光
            ROS_WARN_THROTTLE(1.0, "IR light not detected in current frame");
            matched_ = false;
        }

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
}

bool Imgmatching::computeHomographyInliers(
    const std::vector<cv::KeyPoint>& target_kps, const cv::Mat& target_desc,
    const std::vector<cv::KeyPoint>& frame_kps, const cv::Mat& frame_desc,
    std::vector<cv::DMatch>& inlier_matches, cv::Mat& H) {

    std::vector<cv::DMatch> matches;
    matcher_.match(target_desc, frame_desc, matches);

    const size_t max_matches = 1000;
    if (matches.size() > max_matches) {
        std::partial_sort(matches.begin(), matches.begin() + max_matches, matches.end(),
                          [](const cv::DMatch& a, const cv::DMatch& b) {
                              return a.distance < b.distance;
                          });
        matches.resize(max_matches);
    }

    std::vector<cv::Point2f> pts_target, pts_frame;
    for (const auto& m : matches) {
        pts_target.push_back(target_kps[m.queryIdx].pt);
        pts_frame.push_back(frame_kps[m.trainIdx].pt);
    }

    std::vector<uchar> inliers_mask;
    H = cv::findHomography(pts_target, pts_frame, cv::RANSAC, 3.0, inliers_mask);

    for (size_t i = 0; i < inliers_mask.size(); ++i) {
        if (inliers_mask[i]) {
            inlier_matches.push_back(matches[i]);
        }
    }

    return inlier_matches.size() >= 4;
}

cv::Point2f Imgmatching::projectTargetPoint(const cv::Mat& H, const cv::Point2f& target_point) {
    cv::Mat target_point_H = cv::Mat::ones(3, 1, CV_64F);
    target_point_H.at<double>(0, 0) = target_point.x;
    target_point_H.at<double>(1, 0) = target_point.y;

    cv::Mat frame_point_H = H * target_point_H;

    double w = frame_point_H.at<double>(2, 0);
    return cv::Point2f(
        frame_point_H.at<double>(0, 0) / w,
        frame_point_H.at<double>(1, 0) / w
    );
}

void Imgmatching::updateOffsetWithFilter(const cv::Mat& frame, const cv::Point2f& centroid) {
    cv::Point2f center(static_cast<float>(frame.cols) / 2.0f, static_cast<float>(frame.rows) / 2.0f);

    // Adjust the offset based on the hold position z
    center.y -= 0.04 * fy_ / (z_value - land_pos_z_);

    offset_ = centroid - center;

    if (first_frame_) {
        last_centroid_ = centroid;
        first_frame_ = false;
    } else {
        // 应用低通滤波以平滑跟踪结果
        centroid_.x = (1 - filter_alpha_) * centroid.x + filter_alpha_ * last_centroid_.x;
        centroid_.y = (1 - filter_alpha_) * centroid.y + filter_alpha_ * last_centroid_.y;
        last_centroid_ = centroid_;
    }
}

bool Imgmatching::isTargetMatched() const {
    return matched_;
}

cv::Point2f Imgmatching::getTargetCentroid() const {
    return centroid_;
}

cv::Point2f Imgmatching::getOffset() const {
    //(zhiyuan) convert pixel offset to real-world offset using z and f
    return cv::Point2f(offset_.x * (z_value - land_pos_z_) / fx_, offset_.y * (z_value - land_pos_z_) / fy_);
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
void Imgmatching::setTargetPath(const std::string& path) {
    target_path = path;  
}

// zhiyuan: get target_point_ by mouse click
cv::Point2f Imgmatching::chooseTargetPoint(const cv::Mat& target_image) {
    struct CallbackData {
        cv::Point point;
        bool pointSelected = false;
    } data;

    cv::Mat image = target_image.clone(); 
    bool shouldExit = false;

    cv::namedWindow("Image");
    std::cout << "Click to choose target point..." << std::endl;

    cv::setMouseCallback("Image", [](int event, int x, int y, int, void* userdata) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            auto* data = reinterpret_cast<CallbackData*>(userdata);
            data->point = cv::Point(x, y);
            data->pointSelected = true;
            std::cout << "Set target point at (x=" << x << ", y=" << y << ")" << std::endl;
        }
    }, &data);

    while (!shouldExit) {
        cv::Mat display = image.clone();  

        if (data.pointSelected) {
            cv::circle(display, data.point, 5, cv::Scalar(0, 255, 0), -1);
        }

        cv::putText(display, "Click to select target, press 'Q' to exit", 
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                    cv::Scalar(255, 255, 255), 2);
        cv::imshow("Image", display);

        int key = cv::waitKey(30) & 0xFF;
        if (key == 'q' || key == 'Q' || key == 27) {
            shouldExit = true;
        }
    }

    cv::destroyWindow("Image");
    return data.point;
    
}

// 改进的红外灯光检测方法
cv::Point2f Imgmatching::detectIRLight(const cv::Mat& image) {
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }
    
    // 方法1: 基于最大亮度区域的检测
    cv::Point2f method1_result = detectByMaxBrightness(gray);
    
    // 方法2: 基于Blob检测的方法
    cv::Point2f method2_result = detectByBlobDetection(gray);

    return method2_result;
}

// 基于最大亮度区域的检测方法
cv::Point2f Imgmatching::detectByMaxBrightness(const cv::Mat& gray) {
    // 应用CLAHE增强对比度
    cv::Mat enhanced;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(gray, enhanced);
    
    // 轻微高斯模糊以减少噪声
    cv::Mat blurred;
    cv::GaussianBlur(enhanced, blurred, cv::Size(3, 3), 1.0);
    
    // 找到全局最大值
    cv::Point max_loc;
    double max_val;
    cv::minMaxLoc(blurred, nullptr, &max_val, nullptr, &max_loc);
    
    // 检查最大值是否足够亮
    if (max_val < 180) { // 阈值可调
        return cv::Point2f(-1, -1);
    }
    
    // 在最大值周围进行亚像素精确定位
    cv::Point2f refined_center = refineCenter(blurred, max_loc, 5);
    
    return refined_center;
}

// 基于Blob检测的方法
cv::Point2f Imgmatching::detectByBlobDetection(const cv::Mat& gray) {
    // 设置Blob检测器参数
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 200;
    params.maxThreshold = 255;
    params.filterByArea = true;
    params.minArea = 10;
    params.maxArea = 500;
    params.filterByCircularity = true;
    params.minCircularity = 0.3;
    params.filterByConvexity = true;
    params.minConvexity = 0.5;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.3;
    
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create();
    
    // 检测关键点
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(gray, keypoints);
    
    if (keypoints.empty()) {
        return cv::Point2f(-1, -1);
    }
    
    // 找到最亮的blob
    cv::KeyPoint best_keypoint;
    double max_intensity = 0;
    
    for (const auto& kp : keypoints) {
        if (kp.pt.x >= 0 && kp.pt.y >= 0 && kp.pt.x < gray.cols && kp.pt.y < gray.rows) {
            double intensity = gray.at<uchar>(static_cast<int>(kp.pt.y), static_cast<int>(kp.pt.x));
            if (intensity > max_intensity) {
                max_intensity = intensity;
                best_keypoint = kp;
            }
        }
    }
    
    if (max_intensity > 150) { // 亮度阈值
        return best_keypoint.pt;
    }
    
    return cv::Point2f(-1, -1);
}

// 亚像素精确定位中心点
cv::Point2f Imgmatching::refineCenter(const cv::Mat& image, const cv::Point& rough_center, int window_size) {
    // 确保窗口在图像范围内
    int x1 = std::max(0, rough_center.x - window_size);
    int y1 = std::max(0, rough_center.y - window_size);
    int x2 = std::min(image.cols - 1, rough_center.x + window_size);
    int y2 = std::min(image.rows - 1, rough_center.y + window_size);
    
    if (x2 <= x1 || y2 <= y1) {
        return cv::Point2f(rough_center.x, rough_center.y);
    }
    
    // 提取ROI
    cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
    cv::Mat window = image(roi);
    
    // 计算质心
    cv::Moments m = cv::moments(window, true);
    if (m.m00 == 0) {
        return cv::Point2f(rough_center.x, rough_center.y);
    }
    
    // 转换回全图坐标
    float cx = x1 + m.m10 / m.m00;
    float cy = y1 + m.m01 / m.m00;
    
    return cv::Point2f(cx, cy);
}

void Imgmatching::disableMatching() {
    enabled_ = false;
    cv::destroyWindow("IR Light Tracking");
    cv::destroyAllWindows();  // 销毁所有窗口确保清理
}

float Imgmatching::getCurrentDepth() const {
    return current_depth_;
}

} // namespace px4_utils_land