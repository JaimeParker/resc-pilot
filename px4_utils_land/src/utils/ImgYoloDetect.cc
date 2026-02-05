#include "px4_utils_land/ImgYoloDetect.h"

namespace px4_utils_land {

void Imgyolodetect::init(ros::NodeHandle& nh) {  
    // 获取相机话题参数
    getParamWithWarning(nh, "camera/image_topic", downward_camera_topic_);
    
    // 获取相机内参
    if(downward_camera_topic_ == "/camera/color/image_raw") {
        getParamWithWarning(nh, "camera/fx_", fx_);
        getParamWithWarning(nh, "camera/fy_", fy_);
    }
    
    // 获取yolo模型参数
    getParamWithWarning(nh, "yolo/model_name", model_name_);
    std::string op_path = ros::package::getPath("px4_utils_land");
    std::string model_path = op_path + "/models/" + model_name_;
    getParamWithWarning(nh, "yolo/conf_thres", conf_thres_);
    getParamWithWarning(nh, "yolo/iou_thres", iou_thres_);
    getParamWithWarning(nh, "yolo/opnumthreads", opnumthreads_);
        
    // 初始化图像传输和订阅器
    it_ = std::make_unique<image_transport::ImageTransport>(nh);
    image_sub_ = it_->subscribe(downward_camera_topic_, 1, &Imgyolodetect::imageCallback, this);
    
    // 初始化YOLO
    try {
        session_options_.SetIntraOpNumThreads(opnumthreads_); 
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options_);
        Ort::AllocatorWithDefaultOptions allocator;
        input_name_ = session_->GetInputName(0, allocator);
        input_node_names_ = {input_name_.c_str()};
        output_name_ = session_->GetOutputName(0, allocator);
        output_node_names_ = {output_name_.c_str()};
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("YOLO Beacon Detection Init Error: " << e.what());
    }

    ROS_INFO_STREAM("YOLO Beacon Detection initialized:");
    ROS_INFO_STREAM("  - Subscribed to: " << downward_camera_topic_);
    ROS_INFO_STREAM("  - Confidence Threshold: " << conf_thres_ );
    ROS_INFO_STREAM("  - IOU Threshold: " << iou_thres_);
}

Imgyolodetect::Imgyolodetect() : beacon_detected_(false) {
    // 初始化YOLO参数
    conf_thres_ = 0.5;
    iou_thres_ = 0.4;
    opnumthreads_ = 4;
    input_size_ = cv::Size(416, 416);
}

Imgyolodetect::~Imgyolodetect() {
    // 安全地销毁所有OpenCV窗口
    try {
        cv::destroyAllWindows();
        cv::waitKey(1);
    } catch (const cv::Exception& e) {
        // 静默忽略销毁异常，避免程序崩溃
    }
}

void Imgyolodetect::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!enabled_) { 
        return;
    }
    
    frame_count_++;
    if (frame_count_ % 3 != 0) {
        return;
    }
    
    beacon_detected_ = false;
    
    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        // 预处理
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0/255.0, input_size_, cv::Scalar(0,0,0), true, false);
        std::vector<int64_t> input_shape = {1, 3, input_size_.height, input_size_.width};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, (float*)blob.data, blob.total(), input_shape.data(), input_shape.size());
        
        // 推理
        auto output_tensors = session_->Run(Ort::RunOptions{nullptr}, input_node_names_.data(), &input_tensor, 1, output_node_names_.data(), 1);

        float* all_data = output_tensors[0].GetTensorMutableData<float>();
        auto shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        int box_num = (int)shape[2]; 

        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;
        boxes.reserve(10); 
        confidences.reserve(10);

        float x_scale = (float)frame.cols / input_size_.width;
        float y_scale = (float)frame.rows / input_size_.height;

        // 过滤与解析
        for (int i = 0; i < box_num; ++i) {
            float score = all_data[4 * box_num + i]; 
            if (score > conf_thres_) {
                float cx = all_data[i];
                float cy = all_data[box_num + i];
                float w  = all_data[2 * box_num + i];
                float h  = all_data[3 * box_num + i];

                int width  = (int)(w * x_scale);
                int height = (int)(h * y_scale);
                int left   = (int)((cx - 0.5f * w) * x_scale);
                int top    = (int)((cy - 0.5f * h) * y_scale);

                boxes.emplace_back(left, top, width, height);
                confidences.push_back(score);
            }
        }
        if (!boxes.empty()) {
            std::vector<int> indices;
            cv::dnn::NMSBoxes(boxes, confidences, conf_thres_, iou_thres_, indices);
            double max_conf = 0;
            int max_conf_idx = -1;
            for (int idx : indices) {
                if (confidences[idx] > max_conf) {
                    max_conf = confidences[idx];
                    max_conf_idx = static_cast<int>(idx);
                }
            }

            if (max_conf_idx >= 0) {
                // 计算信标质心
                cv::Point2f detected_centroid(
                        static_cast<float>(boxes[max_conf_idx].x + boxes[max_conf_idx].width / 2.0f),
                        static_cast<float>(boxes[max_conf_idx].y + boxes[max_conf_idx].height / 2.0f)
                );

                // update centroid and offset
                updateCentroidAndOffset(detected_centroid, frame.size());
                beacon_detected_ = true;

                cv::Mat display = frame.clone();
                cv::rectangle(display, boxes[max_conf_idx], cv::Scalar(0, 255, 255), 2);

                // draw centroid
                cv::circle(display, centroid_, 8, cv::Scalar(0, 0, 255), -1);

                // draw image center cross
                cv::Point2f center(static_cast<float>(frame.cols) / 2.0f, 
                                    static_cast<float>(frame.rows) / 2.0f);
                cv::drawMarker(display, center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);

                // draw offset vector
                cv::arrowedLine(display, center, centroid_, cv::Scalar(255, 255, 0), 2);
                
                std::string info = " Offset: (" + std::to_string(static_cast<int>(offset_.x)) + 
                                    "," + std::to_string(static_cast<int>(offset_.y)) + ")";
                cv::putText(display, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                            0.6, cv::Scalar(255, 255, 255), 2);
                
                cv::imshow("Beacon Detection", display);
            }
        } else {
            // show center cross even if no beacon detected
            cv::Mat display = frame.clone();
            cv::Point2f center(static_cast<float>(frame.cols) / 2.0f, 
                              static_cast<float>(frame.rows) / 2.0f);
            cv::drawMarker(display, center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);
            cv::putText(display, "No beacon detected", cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            
            cv::imshow("Beacon Detection", display);
        }
        
        cv::waitKey(1);
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
}

void Imgyolodetect::updateCentroidAndOffset(const cv::Point2f& detected_centroid, const cv::Size& image_size) {
    // the image center point
    cv::Point2f center(static_cast<float>(image_size.width) / 2.0f, 
                      static_cast<float>(image_size.height) / 2.0f);
    
    if (z_value > land_pos_z_) {
        center.y -= 0.04f * fy_ / (z_value - land_pos_z_);
    }
    
    // filter the centroid to reduce noise
    if (first_detection_) {
        centroid_ = detected_centroid;
        first_detection_ = false;
    } else {
        centroid_.x = filter_alpha_ * detected_centroid.x + (1.0f - filter_alpha_) * centroid_.x;
        centroid_.y = filter_alpha_ * detected_centroid.y + (1.0f - filter_alpha_) * centroid_.y;
    }
    
    offset_ = centroid_ - center;
}

bool Imgyolodetect::isTargetMatched() const {
    return beacon_detected_;
}

cv::Point2f Imgyolodetect::getTargetCentroid() const {
    return centroid_;
}

cv::Point2f Imgyolodetect::getOffset() const {
    if (z_value <= land_pos_z_) {
        return cv::Point2f(0.0f, 0.0f);  // avoid division by zero or negative values
    }
    float depth = z_value - land_pos_z_;
    // float depth = std::max((z_value - land_pos_z_), static_cast<float>(1.5));
    return cv::Point2f(offset_.x * depth / fx_, offset_.y * depth / fy_);
}

void Imgyolodetect::setCameraParams(float fx, float fy) {
    fx_ = fx;
    fy_ = fy;
    ROS_INFO_STREAM("Camera parameters set: fx=" << fx_ << ", fy=" << fy_);
}

void Imgyolodetect::setHoldPos(float pos) {
    z_value = pos;
}

void Imgyolodetect::setLandPos(float pos) {
    land_pos_z_ = pos;
}

void Imgyolodetect::disableMatching() {
    enabled_ = false;
    
    // destroy OpenCV windows safely
    try {
        cv::destroyAllWindows();
        cv::waitKey(1);
    } catch (const cv::Exception& e) {
        ROS_WARN_STREAM("OpenCV window destruction warning: " << e.what());
    }
    
    ROS_INFO("YOLO beacon detection disabled");
}

float Imgyolodetect::getCurrentDepth() const {
    return z_value - land_pos_z_;
}

} // namespace px4_utils_land