#include "volleyball_vision/stereo_rectifier.hpp"
#include <iostream>

namespace volleyball_vision {

StereoRectifier::StereoRectifier() {}

bool StereoRectifier::loadCalibration(const std::string& calib_file_path) {
    cv::FileStorage fs(calib_file_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[StereoRectifier] Failed to open calibration file: " << calib_file_path << std::endl;
        return false;
    }

    int width = 1920, height = 1080;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    image_size_ = cv::Size(width, height);

    fs["K1"] >> K1_;
    fs["D1"] >> D1_;
    fs["K2"] >> K2_;
    fs["D2"] >> D2_;
    fs["R"] >> R_;
    fs["T"] >> T_;

    if (K1_.empty() || K2_.empty() || R_.empty() || T_.empty()) {
        std::cerr << "[StereoRectifier] Calibration parameters incomplete in: " << calib_file_path << std::endl;
        return false;
    }

    baseline_ = cv::norm(T_);

    // Compute rectification transforms
    cv::stereoRectify(K1_, D1_, K2_, D2_, image_size_, R_, T_,
                      R1_, R2_, P1_, P2_, Q_,
                      cv::CALIB_ZERO_DISPARITY, 0, image_size_);

    // Precalculate undistort and rectify maps for fast remap
    cv::initUndistortRectifyMap(K1_, D1_, R1_, P1_, image_size_, CV_16SC2, map1_l_, map2_l_);
    cv::initUndistortRectifyMap(K2_, D2_, R2_, P2_, image_size_, CV_16SC2, map1_r_, map2_r_);

    is_initialized_ = true;
    std::cout << "[StereoRectifier] Stereo calibration successfully initialized. Baseline: " 
              << baseline_ << " m" << std::endl;
    return true;
}

void StereoRectifier::rectify(const cv::Mat& left_raw, const cv::Mat& right_raw,
                             cv::Mat& left_rect, cv::Mat& right_rect) {
    if (!is_initialized_) {
        left_rect = left_raw;
        right_rect = right_raw;
        return;
    }

    cv::remap(left_raw, left_rect, map1_l_, map2_l_, cv::INTER_LINEAR);
    cv::remap(right_raw, right_rect, map1_r_, map2_r_, cv::INTER_LINEAR);
}

} // namespace volleyball_vision
