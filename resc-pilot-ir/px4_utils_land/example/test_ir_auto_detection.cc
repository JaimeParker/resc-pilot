#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "px4_utils_land/ImgMatching.h"
#include <iostream>
#include <iomanip>
#include <unistd.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_ir_auto_detection");
    ros::NodeHandle nh;
    
    std::cout << "=========================================" << std::endl;
    std::cout << "红外灯光自动检测测试程序" << std::endl;
    std::cout << "=========================================" << std::endl;
    
    // 创建图像匹配器实例
    px4_utils_land::Imgmatching img_matcher;
    
    // 设置相机参数
    img_matcher.setCameraParams(640.0, 640.0);
    
    // 测试rosbag文件路径
    std::string bag_file_path;
    if (argc > 1) {
        bag_file_path = argv[1];
    } else {
        // 默认rosbag文件路径
        bag_file_path = std::string(getenv("HOME")) + "/Videos/2025-08-22-12-45-36.bag";
        std::cout << "使用默认rosbag文件: " << bag_file_path << std::endl;
        std::cout << "或者可以指定rosbag路径: ./test_ir_auto_detection <bag_path>" << std::endl;
    }
    
    // 检查rosbag文件是否存在
    if (access(bag_file_path.c_str(), F_OK) == -1) {
        std::cout << "无法访问rosbag文件: " << bag_file_path << std::endl;
        std::cout << "创建一个模拟的红外图像进行测试..." << std::endl;
        
        // 创建一个模拟的红外图像进行fallback测试
        cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC3);
        
        // 添加一些随机噪声
        cv::Mat noise;
        cv::randn(noise, cv::Scalar(50), cv::Scalar(20));
        test_image += noise;
        
        // 添加几个模拟的红外灯光点
        cv::circle(test_image, cv::Point(320, 240), 8, cv::Scalar(255, 255, 255), -1); // 中心亮点
        cv::circle(test_image, cv::Point(200, 150), 5, cv::Scalar(240, 240, 240), -1); // 左上亮点
        cv::circle(test_image, cv::Point(450, 350), 6, cv::Scalar(250, 250, 250), -1); // 右下亮点
        
        std::cout << "创建了模拟红外图像，包含3个亮点" << std::endl;
        
        // 测试模拟图像
        cv::imshow("Simulated IR Image", test_image);
        std::cout << "按任意键开始自动检测..." << std::endl;
        cv::waitKey(0);
        
        cv::Point2f detected_light = img_matcher.detectIRLight(test_image);
        if (detected_light.x >= 0 && detected_light.y >= 0) {
            std::cout << "✓ 模拟测试成功！检测到红外灯光位置: (" << detected_light.x << ", " << detected_light.y << ")" << std::endl;
        }
        cv::destroyAllWindows();
        return 0;
    }
    
    std::cout << "开始处理rosbag文件: " << bag_file_path << std::endl;
    
    try {
        rosbag::Bag bag;
        bag.open(bag_file_path, rosbag::bagmode::Read);
        
        // 创建视图，读取图像话题
        std::vector<std::string> topics;
        topics.push_back("/camera/infra1/image_rect_raw");  // 主要话题
        
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        
        if (view.size() == 0) {
            std::cout << "在rosbag中没有找到图像话题，请检查话题名称" << std::endl;
            bag.close();
            return -1;
        }
        
        std::cout << "找到 " << view.size() << " 条图像消息" << std::endl;
        
        int processed_frames = 0;
        int detected_frames = 0;
        bool first_detection = true;
        
        for (rosbag::MessageInstance const m : view) {
            sensor_msgs::Image::ConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
            if (img_msg != nullptr) {
                processed_frames++;
                
                try {
                    // 转换为OpenCV图像
                    cv::Mat frame = cv_bridge::toCvShare(img_msg, "bgr8")->image;
                    
                    // 执行红外灯光检测
                    cv::Point2f detected_light = img_matcher.detectIRLight(frame);
                    
                    if (detected_light.x >= 0 && detected_light.y >= 0) {
                        detected_frames++;
                        
                        if (first_detection) {
                            std::cout << "✓ 首次检测到红外灯光!" << std::endl;
                            std::cout << "位置: (" << detected_light.x << ", " << detected_light.y << ")" << std::endl;
                            first_detection = false;
                        }
                        
                        // 在图像上标记检测结果
                        cv::Mat result_image = frame.clone();
                        cv::circle(result_image, detected_light, 15, cv::Scalar(0, 255, 0), 3);
                        cv::circle(result_image, detected_light, 5, cv::Scalar(0, 0, 255), -1);
                        
                        // 添加信息文本
                        std::string coord_text = "IR Light (" + std::to_string((int)detected_light.x) + 
                                               ", " + std::to_string((int)detected_light.y) + ")";
                        cv::putText(result_image, coord_text, cv::Point(10, 30), 
                                   cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                        
                        std::string frame_text = "Frame: " + std::to_string(processed_frames);
                        cv::putText(result_image, frame_text, cv::Point(10, 70), 
                                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                        
                        cv::imshow("IR Detection from ROSBag", result_image);
                        
                        // 保存第一个检测结果
                        if (detected_frames == 1) {
                            std::string output_path = "/tmp/ir_detection_from_bag.jpg";
                            cv::imwrite(output_path, result_image);
                            std::cout << "首个检测结果已保存到: " << output_path << std::endl;
                        }
                    } else {
                        // 显示原始图像（无检测）
                        cv::Mat display_img = frame.clone();
                        cv::putText(display_img, "No IR Light Detected", cv::Point(10, 30), 
                                   cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
                        cv::putText(display_img, "Frame: " + std::to_string(processed_frames), 
                                   cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                                   cv::Scalar(255, 255, 255), 2);
                        cv::imshow("IR Detection from ROSBag", display_img);
                    }
                    
                    // 每处理10帧打印一次进度
                    if (processed_frames % 10 == 0) {
                        std::cout << "已处理 " << processed_frames << " 帧，检测到 " 
                                  << detected_frames << " 次红外灯光" << std::endl;
                        sleep(0.1);
                    }
                    
                    // 按ESC退出，按空格暂停
                    int key = cv::waitKey(30) & 0xFF;
                    if (key == 27) {  // ESC
                        std::cout << "用户请求退出" << std::endl;
                        break;
                    } else if (key == 32) {  // 空格键暂停
                        std::cout << "已暂停，按任意键继续..." << std::endl;
                        cv::waitKey(0);
                    }
                    
                } catch (cv_bridge::Exception& e) {
                    std::cout << "cv_bridge异常: " << e.what() << std::endl;
                    continue;
                }
            }
        }
        
        bag.close();
        
        std::cout << "\n=========================================" << std::endl;
        std::cout << "处理完成！" << std::endl;
        std::cout << "总处理帧数: " << processed_frames << std::endl;
        std::cout << "检测到红外灯光的帧数: " << detected_frames << std::endl;
        if (processed_frames > 0) {
            double detection_rate = (double)detected_frames / processed_frames * 100.0;
            std::cout << "检测成功率: " << std::fixed << std::setprecision(1) << detection_rate << "%" << std::endl;
        }
        std::cout << "=========================================" << std::endl;
        
    } catch (rosbag::BagException& e) {
        std::cout << "读取rosbag文件时出错: " << e.what() << std::endl;
        return -1;
    } catch (std::exception& e) {
        std::cout << "处理过程中出错: " << e.what() << std::endl;
        return -1;
    }
    
    cv::destroyAllWindows();
    
    return 0;
}
