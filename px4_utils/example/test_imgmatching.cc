#include "px4_utils/ImgMatching.h"

int main(int argc, char**argv) {
    ros::init(argc, argv, "test_imgmatching");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    if (argc != 1) {
        ROS_ERROR("Usage: rosrun px4_utils test_imgmatching");
        return -1;
    }
    std::string ROS_path = ros::package::getPath("px4_utils");
    std::string target_jpg = ROS_path + "/assets/test.jpg";
    std::string video_mp4 = ROS_path + "/assets/test.mp4";    

    // load target image and video file
    px4_utils::Imgmatching img_matcher;
    img_matcher.setTargetPath(target_jpg);
    ROS_INFO("Successfully loaded target image: %s", target_jpg.c_str());

    // set camera parameters
    img_matcher.setCameraParams(562.94f, 422.21f, 3.0f);  // fx, fy, land_z
    img_matcher.setHoldPos(4.0f);  // hold position z

    // initialize Imgmatching
    std::string test_topic = "test_image_topic";  
    img_matcher.init(nh, test_topic);  

    // create image publisher
    auto img_pub = it.advertise(test_topic, 1);

    cv::VideoCapture cap(video_mp4);
    if (!cap.isOpened()) {
        ROS_ERROR("Could not open video file: %s", video_mp4.c_str());
        return -1;
    }

    cv::Mat frame;
    int frame_count = 0;
    while (ros::ok()) {
        if (!cap.read(frame)) break;  

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
            std_msgs::Header(), "bgr8", frame
        ).toImageMsg();
        img_pub.publish(msg);
        frame_count++;

        if (frame_count % 30 == 0) {  
            ROS_INFO("handling frame %d", frame_count);
        }

        ros::spinOnce();
        char key = cv::waitKey(10) & 0xFF;
        if (key == 'q' || key == 'Q' || key == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    ROS_INFO("finished test_imgmatching");
    return 0;
}
