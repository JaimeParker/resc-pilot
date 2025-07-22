#include "px4_utils/ImgMatching.h"
#include "px4_utils/PX4CtrlFSM.h"

namespace px4_utils {

Imgmatching::Imgmatching(ros::NodeHandle& nh, const std::string& image_topic)
    : it_(nh), matcher_(cv::NORM_HAMMING), matched_(false) {

    // (zhiyuan):ORB is not scale invariant, so requires to create manual pyramid 
    // orb_ = cv::ORB::create(500, 1.2f, 8);
    orb_ = cv::AKAZE::create(); 

    // TODO:(zhiyuan)replace with a parameter to load the target image and change the path
    std::string ROS_path = ros::package::getPath("px4_utils");
    std::string target_path = ROS_path + "/../target.png";
    loadTargetImage(target_path);

    image_sub_ = it_.subscribe(image_topic, 1, &Imgmatching::imageCallback, this);
    ROS_INFO_STREAM("Imgmatching subscribed to: " << image_topic);
}

void Imgmatching::loadTargetImage(const std::string& path) {
    target_image_ = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (target_image_.empty()) {
        ROS_ERROR_STREAM("Failed to load target image: " << path);
        return;
    }
    orb_->detectAndCompute(target_image_, cv::noArray(), target_kps_, target_desc_);
    ROS_INFO_STREAM("Loaded target image with " << target_kps_.size() << " keypoints.");
}

void Imgmatching::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    matched_ = false;

    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

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

        //TODO:Calculate centroid_ with H matrix
        //TODO:change this by real target
        cv::Point2f target_point(247, 141);

        cv::Mat target_point_H = cv::Mat::ones(3, 1, CV_64F);
        target_point_H.at<double>(0, 0) = target_point.x;
        target_point_H.at<double>(1, 0) = target_point.y;

        cv::Mat frame_point_H = H * target_point_H;

        double w = frame_point_H.at<double>(2, 0);
        cv::Point2f centroid_(
            frame_point_H.at<double>(0, 0) / w,
            frame_point_H.at<double>(1, 0) / w
        );
        //

        cv::Point2f center(static_cast<float>(gray.cols) / 2.0f, static_cast<float>(gray.rows) / 2.0f);
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
        cv::imshow("Trajectory", frame);
        cv::circle(frame, centroid_, 5, cv::Scalar(0, 255, 0), -1);
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
    return cv::Point2f(offset_.x * PX4CtrlFSM::z_ / PX4CtrlFSM::fx_, offset_.y * PX4CtrlFSM::z_ / PX4CtrlFSM::fy_);
}

} // namespace rese_pilot