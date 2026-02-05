#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

class YOLO_DETECT {
public:
    YOLO_DETECT() : nh_("~") {
        // 获取参数
        nh_.param<std::string>("video_name", video_name_, "test_video.mp4");
        std::string op_path = ros::package::getPath("px4_utils_land");
        video_path_ = op_path + "/videos/" + video_name_;
        nh_.param<std::string>("model_name", model_name_, "best_real.onnx");
        model_path_ = op_path + "/models/" + model_name_;
        nh_.param<float>("conf_thres", conf_thres_, 0.5);
        nh_.param<float>("iou_thres", iou_thres_, 0.4);
        
        std::cout << "Video file: " << video_path_ << std::endl;
        std::cout << "Model: " << model_path_ << std::endl;
        
        // 根据模式初始化
        if (!initialize()) {
            throw std::runtime_error("Failed to initialize YOLO tester");
        }
    }
    
    ~YOLO_DETECT() {
        cleanup();
    }

    void run() {
        cv::VideoCapture cap;
        if (video_path_.size() == 1 && std::isdigit(video_path_[0])) {
            cap.open(std::stoi(video_path_));
        } else {
            cap.open(video_path_);
        }
        
        cv::Mat frame;
        cv::TickMeter tm;

        while (cap.read(frame)) {
            tm.reset();
            tm.start();
            
            cv::Mat result = detect(frame);
            
            tm.stop();

            // 绘制实时帧率
            double fps = 1.0 / tm.getTimeSec();
            cv::putText(result, cv::format("FPS: %.1f", fps), 
                        cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            
            cv::imshow("YOLO Detect", result);
            if (cv::waitKey(1) == 'q') break;
        }
    }

private:
    bool initialize() {
        try {
            input_size_ = cv::Size(416, 416);
            cv::namedWindow("YOLO Detect", cv::WINDOW_NORMAL);
            // 开启多线程推理和全图优化
            session_options.SetIntraOpNumThreads(4); 
            session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            
            // 加载模型
            session = std::make_unique<Ort::Session>(env, model_path_.c_str(), session_options);

            // 获取节点信息
            Ort::AllocatorWithDefaultOptions allocator;
            input_name = session->GetInputName(0, allocator);
            input_node_names = {input_name.c_str()};
            output_name = session->GetOutputName(0, allocator);
            output_node_names = {output_name.c_str()};

            std::cout << "Model Loaded!" << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Load Error: " << e.what() << std::endl;
            return false;
        }
    }

    cv::Mat detect(cv::Mat& frame) {
        int img_h = frame.rows;
        int img_w = frame.cols;

        // 预处理
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0/255.0, input_size_, cv::Scalar(0,0,0), true, false);
        std::vector<int64_t> input_shape = {1, 3, input_size_.height, input_size_.width};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, (float*)blob.data, blob.total(), input_shape.data(), input_shape.size());

        // 推理
        auto output_tensors = session->Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names.data(), 1);
        

        float* all_data = output_tensors[0].GetTensorMutableData<float>();
        auto shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        int box_num = (int)shape[2]; 

        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;
        boxes.reserve(10); 
        confidences.reserve(10);

        float x_scale = (float)img_w / input_size_.width;
        float y_scale = (float)img_h / input_size_.height;

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
            
            for (int idx : indices) {
                cv::Rect box = boxes[idx];
                
                // 可视化
                cv::rectangle(frame, box, color, 2);
                
                std::string label = classes[0] + ": " + cv::format("%.2f", confidences[idx]);
                int baseline;
                cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                
                // 绘制标签背景框
                cv::rectangle(frame, cv::Point(box.x, box.y - label_size.height - 5), 
                              cv::Point(box.x + label_size.width, box.y), color, -1);
                // 绘制标签文字
                cv::putText(frame, label, cv::Point(box.x, box.y - 5), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            }
        }
        return frame;
    }

    void cleanup() {
        cv::destroyAllWindows();
    }

private:
    ros::NodeHandle nh_;
    std::string video_name_;
    std::string model_name_;
    std::string video_path_;
    std::string model_path_;
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "YOLO"};
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    std::string input_name, output_name;
    std::vector<const char*> input_node_names, output_node_names;
    cv::Size input_size_;
    float conf_thres_;
    float iou_thres_;
    std::vector<std::string> classes = {"marker"};
    cv::Scalar color = cv::Scalar(0, 255, 255); 
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "yolo_unified_tester");
    try {
        YOLO_DETECT tester;
        tester.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}