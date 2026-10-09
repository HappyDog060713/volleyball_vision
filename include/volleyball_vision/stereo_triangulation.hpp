#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "volleyball_vision/yolo_detector.hpp"

namespace volleyball_vision {

struct TriangulatedPoint {
    cv::Point3d point_3d;
    double disparity;
    double epipolar_error;
    float confidence;
    bool is_valid;
};

class StereoTriangulator {
public:
    StereoTriangulator();
    ~StereoTriangulator() = default;

    void setProjectionMatrices(const cv::Mat& P1, const cv::Mat& P2, double baseline);
    void setEpipolarThreshold(double max_vertical_diff_px);

    bool matchAndTriangulate(const std::vector<Detection>& left_dets,
                             const std::vector<Detection>& right_dets,
                             TriangulatedPoint& out_point);

private:
    cv::Mat P1_, P2_;
    double baseline_{1.5};
    double epipolar_threshold_{2.5}; // pixels
};

} // namespace volleyball_vision
