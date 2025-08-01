#include "px4_utils_land/ImgMatching.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_imgmatching");
    ros::NodeHandle nh;

    px4_utils_land::Imgmatching img_matcher;
    // img_matcher.setCameraParams(562.94f, 422.21f, 3.0f);
    img_matcher.setHoldPos(4.0f);

    img_matcher.init(nh);

    ROS_INFO("Imgmatching node started, waiting for image callbacks...");

    ros::spin();  

    return 0;
}
