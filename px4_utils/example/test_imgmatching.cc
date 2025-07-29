#include "px4_utils/ImgMatching.h"
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_imgmatching");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    if (argc != 1) {
        ROS_ERROR("Usage: rosrun px4_utils test_imgmatching");
        return -1;
    }

    std::string ROS_path = ros::package::getPath("px4_utils");
    std::string video_mp4 = ROS_path + "/assets/test.mp4";

    px4_utils::Imgmatching img_matcher;
    img_matcher.setCameraParams(562.94f, 422.21f, 3.0f);  // fx, fy, z
    img_matcher.setHoldPos(4.0f);  // hold position z

    std::string test_topic = "test_image_topic";
    img_matcher.init(nh, test_topic);

    // Start the ROS spinner, which processes callbacks
    ros::AsyncSpinner spinner(2);
    spinner.start();

    auto img_pub = it.advertise(test_topic, 1);
    cv::VideoCapture cap(video_mp4);
    if (!cap.isOpened()) {
        ROS_ERROR("Could not open video file: %s", video_mp4.c_str());
        return -1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 30.0;
    ros::Rate rate(fps);

    cv::Mat frame;
    while (ros::ok()) {
        if (!cap.read(frame)) break;

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
            std_msgs::Header(), "bgr8", frame
        ).toImageMsg();

        img_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    cap.release();
    cv::destroyAllWindows();
    ROS_INFO("Finished test_imgmatching.");
    return 0;
}
