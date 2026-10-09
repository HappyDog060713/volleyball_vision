#include "volleyball_vision/yolo_detector.hpp"
#include <iostream>

namespace volleyball_vision {

YoloDetector::YoloDetector() {}

bool YoloDetector::loadModel(const std::string& onnx_path, float conf_thresh, float nms_thresh) {
    try {
        net_ = cv::dnn::readNetFromONNX(onnx_path);
        // Prefer OpenCL / Vulkan / iGPU target if available
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        conf_threshold_ = conf_thresh;
        nms_threshold_ = nms_thresh;
        is_loaded_ = true;
        std::cout << "[YoloDetector] Successfully loaded ONNX model: " << onnx_path << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[YoloDetector] Error loading model: " << e.what() << std::endl;
        return false;
    }
}

std::vector<Detection> YoloDetector::detect(const cv::Mat& image) {
    std::vector<Detection> detections;
    if (!is_loaded_ || image.empty()) return detections;

    // Letterbox / Preprocess
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1.0 / 255.0, cv::Size(input_width_, input_height_),
                           cv::Scalar(), true, false);

    net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    if (outputs.empty()) return detections;

    cv::Mat output = outputs[0]; // Shape usually [1, num_features, num_proposals] or [1, num_proposals, num_features]
    if (output.dims == 3 && output.size[1] < output.size[2]) {
        // Transpose [1, 5+C, 8400] -> [8400, 5+C]
        output = output.reshape(1, output.size[1]);
        cv::transpose(output, output);
    } else if (output.dims == 3) {
        output = output.reshape(1, output.size[1]);
    }

    float x_factor = (float)image.cols / input_width_;
    float y_factor = (float)image.rows / input_height_;

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    float* data = (float*)output.data;
    int rows = output.rows;
    int dimensions = output.cols;

    for (int i = 0; i < rows; ++i) {
        float* row = data + i * dimensions;
        float score = (dimensions > 4) ? row[4] : 0.0f;
        int class_id = 0;

        if (dimensions > 5) {
            // Find max score among classes
            float max_class_score = 0.0f;
            for (int c = 4; c < dimensions; ++c) {
                if (row[c] > max_class_score) {
                    max_class_score = row[c];
                    class_id = c - 4;
                }
            }
            score = max_class_score;
        }

        if (score >= conf_threshold_) {
            float cx = row[0] * x_factor;
            float cy = row[1] * y_factor;
            float w = row[2] * x_factor;
            float h = row[3] * y_factor;

            int left = std::max(0, static_cast<int>(cx - w * 0.5f));
            int top = std::max(0, static_cast<int>(cy - h * 0.5f));
            int width = std::min(image.cols - left, static_cast<int>(w));
            int height = std::min(image.rows - top, static_cast<int>(h));

            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(score);
            class_ids.push_back(class_id);
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

    for (int idx : indices) {
        Detection det;
        det.bbox = boxes[idx];
        det.confidence = confidences[idx];
        det.class_id = class_ids[idx];
        det.center = cv::Point2f(boxes[idx].x + boxes[idx].width * 0.5f,
                                 boxes[idx].y + boxes[idx].height * 0.5f);
        detections.push_back(det);
    }

    return detections;
}

} // namespace volleyball_vision
