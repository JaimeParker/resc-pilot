// #include "px4_utils_land/ImgMatching.h"

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "test_imgmatching");
//     ros::NodeHandle nh;

//     px4_utils_land::Imgmatching img_matcher;
//     // img_matcher.setCameraParams(562.94f, 422.21f, 3.0f);
//     img_matcher.setHoldPos(4.0f);

//     img_matcher.init(nh);

//     ROS_INFO("Imgmatching node started, waiting for image callbacks...");

//     ros::spin();  

//     return 0;
// }

#include "px4_utils_land/ImgMatching.h"
#include <sensor_msgs/Image.h>
#include <thread>

void publishVideo(const std::string& video_path, const std::string& topic_name, ros::NodeHandle& nh) {
    ros::Publisher video_pub = nh.advertise<sensor_msgs::Image>(topic_name, 1);
    cv::VideoCapture cap(video_path);

    if (!cap.isOpened()) {
        ROS_ERROR_STREAM("Failed to open video file: " << video_path);
        return;
    }

    ROS_INFO_STREAM("Publishing video from: " << video_path << " to topic: " << topic_name);

    cv::Mat frame;
    ros::Rate loop_rate(30); // Set the frame rate to 30 FPS

    while (ros::ok() && cap.read(frame)) {
        if (frame.empty()) {
            ROS_WARN("Empty frame encountered, stopping video publishing.");
            break;
        }

        // Convert OpenCV frame to ROS Image message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        video_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    cap.release();
    ROS_INFO("Video publishing finished.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_imgmatching");
    ros::NodeHandle nh;

    if (argc < 1) {
        ROS_ERROR("Usage: rosrun px4_utils_land test_imgmatching <video_path>");
        return -1;
    }

    std::string path = ros::package::getPath("px4_utils_land");
    std::string video_path = path + "/assets/test5.mp4";
    std::string topic_name = "/camera/rgb/image_raw"; // Default topic name

    // Start video publishing in a separate thread
    std::thread video_thread(publishVideo, video_path, topic_name, std::ref(nh));

    px4_utils_land::Imgmatching img_matcher;
    img_matcher.setHoldPos(4.0f);
    img_matcher.init(nh);

    ROS_INFO("Imgmatching node started, waiting for image callbacks...");

    ros::spin();

    video_thread.join(); // Ensure the video thread finishes before exiting
    return 0;
}