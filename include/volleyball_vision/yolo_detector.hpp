#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

namespace volleyball_vision {

struct Detection {
    cv::Rect2f bbox;
    float confidence;
    int class_id;
    cv::Point2f center;
};

class YoloDetector {
public:
    YoloDetector();
    ~YoloDetector() = default;

    bool loadModel(const std::string& onnx_path, float conf_thresh = 0.4f, float nms_thresh = 0.45f);
    std::vector<Detection> detect(const cv::Mat& image);

private:
    cv::dnn::Net net_;
    float conf_threshold_{0.4f};
    float nms_threshold_{0.45f};
    int input_width_{640};
    int input_height_{640};
    bool is_loaded_{false};
};

} // namespace volleyball_vision
