#include "px4_utils/ImgMatching.h"

namespace px4_utils {

void Imgmatching::init(ros::NodeHandle& nh, const std::string& image_topic) { 
    // Initialize image transport and subscriber
    it_ = std::make_unique<image_transport::ImageTransport>(nh);
    image_sub_ = it_->subscribe(image_topic, 1, &Imgmatching::imageCallback, this);
    ROS_INFO_STREAM("Imgmatching initialized and subscribed to: " << image_topic);
}

Imgmatching::Imgmatching()
    : matcher_(cv::NORM_HAMMING), matched_(false) {
    orb_ = cv::AKAZE::create(); 
}

void Imgmatching::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    matched_ = false;

    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        if(first_frame_) {
            target_image_ = frame.clone();
            target_point_ = chooseTargetPoint(target_image_);
            orb_->detectAndCompute(gray, cv::noArray(), target_kps_, target_desc_);
            ROS_INFO_STREAM("Loaded target image with " << target_kps_.size() << " keypoints.");
        }   

        std::vector<cv::KeyPoint> frame_kps_;
        cv::Mat frame_desc_;
        orb_->detectAndCompute(gray, cv::noArray(), frame_kps_, frame_desc_);

        if (frame_kps_.empty() || target_kps_.empty()) return;

        std::vector<cv::DMatch> matches;
        matcher_.match(target_desc_, frame_desc_, matches);
        std::vector<cv::Point2f> pts_target, pts_frame;
        for (const auto& m : matches) {
            pts_target.push_back(target_kps_[m.queryIdx].pt);
            pts_frame.push_back(frame_kps_[m.trainIdx].pt);
        }

        std::vector<uchar> inliers_mask;
        cv::Mat H = cv::findHomography(pts_target, pts_frame, cv::RANSAC, 3.0, inliers_mask);

        std::vector<cv::DMatch> inlier_matches;
        std::vector<cv::Point2f> inlier_pts;
        for (size_t i = 0; i < inliers_mask.size(); ++i) {
            if (inliers_mask[i]) {
                inlier_matches.push_back(matches[i]);
                inlier_pts.push_back(pts_frame[i]);
            }
        }
        ROS_INFO_STREAM("RANSAC inliers: " << inlier_matches.size());
        if (inlier_pts.size() < 4) return;

        cv::Mat target_point_H = cv::Mat::ones(3, 1, CV_64F);
        target_point_H.at<double>(0, 0) = target_point_.x;
        target_point_H.at<double>(1, 0) = target_point_.y;

        cv::Mat frame_point_H = H * target_point_H;

        double w = frame_point_H.at<double>(2, 0);
        cv::Point2f centroid_(
            frame_point_H.at<double>(0, 0) / w,
            frame_point_H.at<double>(1, 0) / w
        );
        //

        cv::Point2f center(static_cast<float>(gray.cols) / 2.0f, static_cast<float>(gray.rows) / 2.0f);

        // get hold_pos_.z() from PX4CtrlFSM
        center.x += 0.1 * fx_ / (z_value - land_pos_z_);
        center.y += 0.1 * fy_ / (z_value - land_pos_z_);

        offset_ = centroid_ - center;

        if (first_frame_) {
            last_centroid_ = centroid_;
            first_frame_ = false;
        } else {
            centroid_.x = filter_alpha_ * centroid_.x + (1 - filter_alpha_) * last_centroid_.x;
            centroid_.y = filter_alpha_ * centroid_.y + (1 - filter_alpha_) * last_centroid_.y;
            last_centroid_ = centroid_;
        }

        matched_ = true;

        cv::Mat vis;
        cv::drawMatches(target_image_, target_kps_, frame, frame_kps_, inlier_matches, vis);
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

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
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



} // namespace px4_utils