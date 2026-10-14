#pragma once

#include <string>
#include <memory>
#include <functional>
#include <opencv2/opencv.hpp>
#include "CameraApi.h"

namespace volleyball_vision {

struct CameraFrame {
    cv::Mat image;
    uint64_t timestamp_us{0};
    uint32_t frame_id{0};
};

struct StereoFrame {
    CameraFrame left;
    CameraFrame right;
    bool is_valid{false};
};

class StereoCameraDriver {
public:
    StereoCameraDriver();
    ~StereoCameraDriver();

    bool initialize(int left_camera_index = 0, int right_camera_index = 1);
    bool setExposureTime(double exposure_us);
    bool setAnalogGain(int gain);
    bool setROI(int x, int y, int width, int height);
    bool setSpeedLevel(int level);
    bool setHardwareTrigger(bool enable);
    bool startGrabbing();
    void stopGrabbing();

    bool getStereoFrame(StereoFrame& out_frame, int timeout_ms = 100);

private:
    bool initSingleCamera(int dev_index, int& handle, tSdkCameraDevInfo& dev_info);
    void releaseSingleCamera(int& handle);

    int left_handle_{-1};
    int right_handle_{-1};
    bool is_streaming_{false};

    unsigned char* left_buffer_{nullptr};
    unsigned char* right_buffer_{nullptr};
    tSdkCameraCapbility left_capability_{};
    tSdkCameraCapbility right_capability_{};
};

} // namespace volleyball_vision
