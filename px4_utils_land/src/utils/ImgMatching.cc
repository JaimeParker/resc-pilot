#include "px4_utils_land/ImgMatching.h"

namespace px4_utils_land {

void Imgmatching::init(ros::NodeHandle& nh) {  
    getParamWithWarning(nh, "camera/image_topic", downward_camera_topic_);
    if(downward_camera_topic_ == "/camera/rgb/image_raw") {
        fx_ = 562.94f; // focal length in x
        fy_ = 422.21f; // focal length in y
    }               
    // Initialize image transport and subscriber
    it_ = std::make_unique<image_transport::ImageTransport>(nh);
    image_sub_ = it_->subscribe(downward_camera_topic_, 1, &Imgmatching::imageCallback, this);
    ROS_INFO_STREAM("Imgmatching initialized and subscribed to: " << downward_camera_topic_);
}

Imgmatching::Imgmatching()
    : matcher_(cv::NORM_HAMMING), matched_(false) {
    orb_ = cv::AKAZE::create(); 
}

void Imgmatching::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    frame_count_++;
    if (frame_count_ % 3 != 0) {
        return;
    }
    matched_ = false;

    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // (zhiyuan) use CLAHE to enhance the image contrast
        // But it may slow down the processing speed                
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.0, cv::Size(4, 4));
        clahe->apply(gray, gray);

        if (first_frame_) {
            initializeTarget(gray, frame);
        } 

        std::vector<cv::KeyPoint> frame_kps;
        cv::Mat frame_desc;
        orb_->detectAndCompute(gray, cv::noArray(), frame_kps, frame_desc);

        if (frame_kps.empty() || target_kps_.empty()) return;

        std::vector<cv::DMatch> inlier_matches;
        cv::Mat H;
        if (!computeHomographyInliers(target_kps_, target_desc_, frame_kps, frame_desc, inlier_matches, H))
            return;

        centroid_ = projectTargetPoint(H, target_point_);
        updateOffsetWithFilter(gray, centroid_);

        matched_ = true;
        cv::Mat vis;
        cv::drawMatches(target_image_, target_kps_, frame, frame_kps, inlier_matches, vis);
        cv::imshow("Match", vis);

        //Add traj of centroid
        static std::vector<cv::Point2f> traj;
        traj.push_back(centroid_);
        for (size_t i = 1; i < traj.size(); ++i) {
            cv::line(frame, traj[i - 1], traj[i], cv::Scalar(255, 0, 0), 2);
        }
        cv::circle(frame, centroid_, 5, cv::Scalar(0, 255, 0), -1);
        cv::imshow("Trajectory", frame);
        cv::waitKey(1);

        target_image_ = frame.clone();
        target_point_ = centroid_;
        target_kps_ = frame_kps;
        target_desc_ = frame_desc.clone();

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
}

void Imgmatching::initializeTarget(const cv::Mat& gray, const cv::Mat& frame) {
    target_image_ = frame.clone();
    target_point_ = chooseTargetPoint(target_image_);
    orb_->detectAndCompute(gray, cv::noArray(), target_kps_, target_desc_);
    ROS_INFO_STREAM("Loaded target image with " << target_kps_.size() << " keypoints.");
}

bool Imgmatching::computeHomographyInliers(
    const std::vector<cv::KeyPoint>& target_kps, const cv::Mat& target_desc,
    const std::vector<cv::KeyPoint>& frame_kps, const cv::Mat& frame_desc,
    std::vector<cv::DMatch>& inlier_matches, cv::Mat& H) {

    std::vector<cv::DMatch> matches;
    matcher_.match(target_desc, frame_desc, matches);

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

    ROS_INFO_STREAM("RANSAC inliers: " << inlier_matches.size());
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

void Imgmatching::updateOffsetWithFilter(const cv::Mat& gray, const cv::Point2f& centroid) {
    cv::Point2f center(static_cast<float>(gray.cols) / 2.0f, static_cast<float>(gray.rows) / 2.0f);

    // Adjust the offset based on the hold position z
    center.x += 0.1 * fx_ / (z_value - land_pos_z_);
    center.y += 0.1 * fy_ / (z_value - land_pos_z_);

    offset_ = centroid - center;

    if (first_frame_) {
        last_centroid_ = centroid;
        first_frame_ = false;
    } else {
        centroid_.x = filter_alpha_ * centroid.x + (1 - filter_alpha_) * last_centroid_.x;
        centroid_.y = filter_alpha_ * centroid.y + (1 - filter_alpha_) * last_centroid_.y;
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

void Imgmatching::setCameraParams(float fx, float fy, float z) {
    fx_ = fx;
    fy_ = fy;
    z_ = z;
    ROS_INFO_STREAM("Camera parameters set: fx=" << fx_ << ", fy=" << fy_ << ", z=" << z_);
}

void Imgmatching::setHoldPos(float pos) {
    z_value = pos;
}

// zhiyuan: no use, so ignore. but keep it for future reference
void Imgmatching::setTargetPath(const std::string& path) {
    target_path = path;  
    ROS_INFO_STREAM("target_image_path: " << target_path);
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



} // namespace px4_utils_land