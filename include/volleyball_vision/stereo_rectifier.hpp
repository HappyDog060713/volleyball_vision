#pragma once

#include <string>
#include <opencv2/opencv.hpp>

namespace volleyball_vision {

class StereoRectifier {
public:
    StereoRectifier();
    ~StereoRectifier() = default;

    bool loadCalibration(const std::string& calib_file_path);
    void rectify(const cv::Mat& left_raw, const cv::Mat& right_raw,
                 cv::Mat& left_rect, cv::Mat& right_rect);

    const cv::Mat& getP1() const { return P1_; }
    const cv::Mat& getP2() const { return P2_; }
    const cv::Mat& getQ() const { return Q_; }
    double getBaseline() const { return baseline_; }

private:
    cv::Mat K1_, D1_, K2_, D2_;
    cv::Mat R_, T_;
    cv::Mat R1_, R2_, P1_, P2_, Q_;
    cv::Mat map1_l_, map2_l_;
    cv::Mat map1_r_, map2_r_;
    cv::Size image_size_;
    double baseline_{1.5};
    bool is_initialized_{false};
};

} // namespace volleyball_vision
