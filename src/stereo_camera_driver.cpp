#include "volleyball_vision/stereo_camera_driver.hpp"
#include <iostream>
#include <vector>
#include <set>
#include <string>

namespace volleyball_vision {

StereoCameraDriver::StereoCameraDriver() {}

StereoCameraDriver::~StereoCameraDriver() {
    stopGrabbing();
    releaseSingleCamera(left_handle_);
    releaseSingleCamera(right_handle_);
}

bool StereoCameraDriver::initSingleCamera(int dev_index, int& handle, tSdkCameraDevInfo& dev_info) {
    CameraSdkStatus status = CameraInit(&dev_info, -1, -1, &handle);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "[StereoCameraDriver] Failed to init camera dev index: " << dev_index
                  << ", code: " << status << std::endl;
        return false;
    }

    // Set raw format and unpack buffer
    CameraGetCapability(handle, (handle == left_handle_ ? &left_capability_ : &right_capability_));
    CameraSetIspOutFormat(handle, CAMERA_MEDIA_TYPE_BGR8);
    return true;
}

void StereoCameraDriver::releaseSingleCamera(int& handle) {
    if (handle > 0) {
        CameraUnInit(handle);
        handle = -1;
    }
}

bool StereoCameraDriver::initialize(int left_camera_index, int right_camera_index) {
    CameraSdkInit(1); // 1 = English/Chinese log

    tSdkCameraDevInfo dev_list[16];
    int camera_count = 16;
    CameraEnumerateDevice(dev_list, &camera_count);

    // Deduplicate physical cameras by SN, preferring subnet matching routes (e.g. 192.168.1.100)
    std::vector<tSdkCameraDevInfo> unique_cameras;
    std::set<std::string> seen_sns;
    for (int i = 0; i < camera_count; ++i) {
        std::string port_str(dev_list[i].acPortType);
        std::string sn(dev_list[i].acSn);
        if (port_str.find("192.168.1.100") != std::string::npos && seen_sns.find(sn) == seen_sns.end()) {
            seen_sns.insert(sn);
            unique_cameras.push_back(dev_list[i]);
        }
    }
    if (unique_cameras.size() < 2) {
        for (int i = 0; i < camera_count; ++i) {
            std::string sn(dev_list[i].acSn);
            if (seen_sns.find(sn) == seen_sns.end()) {
                seen_sns.insert(sn);
                unique_cameras.push_back(dev_list[i]);
            }
        }
    }

    if (unique_cameras.size() < 2) {
        std::cerr << "[StereoCameraDriver] Found only " << unique_cameras.size()
                  << " distinct physical MindVision cameras. At least 2 are required." << std::endl;
        return false;
    }

    if (left_camera_index >= static_cast<int>(unique_cameras.size()) ||
        right_camera_index >= static_cast<int>(unique_cameras.size())) {
        std::cerr << "[StereoCameraDriver] Invalid camera indices." << std::endl;
        return false;
    }

    std::cout << "[StereoCameraDriver] Initializing Left Camera: SN " << unique_cameras[left_camera_index].acSn
              << " (" << unique_cameras[left_camera_index].acPortType << ")" << std::endl;
    if (!initSingleCamera(left_camera_index, left_handle_, unique_cameras[left_camera_index])) {
        return false;
    }

    std::cout << "[StereoCameraDriver] Initializing Right Camera: SN " << unique_cameras[right_camera_index].acSn
              << " (" << unique_cameras[right_camera_index].acPortType << ")" << std::endl;
    if (!initSingleCamera(right_camera_index, right_handle_, unique_cameras[right_camera_index])) {
        releaseSingleCamera(left_handle_);
        return false;
    }

    // 方案 1: 硬件降档至均衡档 (Speed Level 1)，将双目总带宽控制在约 700 Mbps，完全适配 1000M 交换机物理上限
    setSpeedLevel(1);

    // 方案 2: 垂直方向启用 ROI 裁剪 (1280x720, 截取上部 720 行，裁掉底部车体/地面死区，数据量骤降 30%)
    setROI(0, 0, 1280, 720);

    // Allocate RGB frame buffers
    int max_width = 1920;
    int max_height = 1080;
    left_buffer_ = (unsigned char*)malloc(max_width * max_height * 3);
    right_buffer_ = (unsigned char*)malloc(max_width * max_height * 3);

    return true;
}

bool StereoCameraDriver::setROI(int x, int y, int width, int height) {
    if (left_handle_ > 0) {
        CameraSetImageResolutionEx(left_handle_, 0xFF, 0, 0, x, y, width, height, 0, 0);
    }
    if (right_handle_ > 0) {
        CameraSetImageResolutionEx(right_handle_, 0xFF, 0, 0, x, y, width, height, 0, 0);
    }
    return true;
}

bool StereoCameraDriver::setSpeedLevel(int level) {
    if (left_handle_ > 0) {
        CameraSetFrameSpeed(left_handle_, level);
        CameraSetFrameRate(left_handle_, 0);
    }
    if (right_handle_ > 0) {
        CameraSetFrameSpeed(right_handle_, level);
        CameraSetFrameRate(right_handle_, 0);
    }
    return true;
}

bool StereoCameraDriver::setExposureTime(double exposure_us) {
    if (left_handle_ > 0) CameraSetExposureTime(left_handle_, exposure_us);
    if (right_handle_ > 0) CameraSetExposureTime(right_handle_, exposure_us);
    return true;
}

bool StereoCameraDriver::setAnalogGain(int gain) {
    if (left_handle_ > 0) CameraSetAnalogGain(left_handle_, gain);
    if (right_handle_ > 0) CameraSetAnalogGain(right_handle_, gain);
    return true;
}

bool StereoCameraDriver::setHardwareTrigger(bool enable) {
    int trigger_mode = enable ? 1 : 0; // 0 = continuous, 1 = hardware trigger
    if (left_handle_ > 0) {
        CameraSetTriggerMode(left_handle_, trigger_mode);
        if (enable) {
            CameraSetExtTrigSignalType(left_handle_, 0); // 0 = Line 0 / IN
        }
    }
    if (right_handle_ > 0) {
        CameraSetTriggerMode(right_handle_, trigger_mode);
        if (enable) {
            CameraSetExtTrigSignalType(right_handle_, 0);
        }
    }
    return true;
}

bool StereoCameraDriver::startGrabbing() {
    if (left_handle_ > 0) CameraPlay(left_handle_);
    if (right_handle_ > 0) CameraPlay(right_handle_);
    is_streaming_ = true;
    return true;
}

void StereoCameraDriver::stopGrabbing() {
    if (is_streaming_) {
        if (left_handle_ > 0) CameraStop(left_handle_);
        if (right_handle_ > 0) CameraStop(right_handle_);
        is_streaming_ = false;
    }
    if (left_buffer_) { free(left_buffer_); left_buffer_ = nullptr; }
    if (right_buffer_) { free(right_buffer_); right_buffer_ = nullptr; }
}

bool StereoCameraDriver::getStereoFrame(StereoFrame& out_frame, int timeout_ms) {
    if (!is_streaming_) return false;

    tSdkFrameHead head_l, head_r;
    BYTE* raw_buffer_l = nullptr;
    BYTE* raw_buffer_r = nullptr;

    CameraSdkStatus st_l = CameraGetImageBuffer(left_handle_, &head_l, &raw_buffer_l, timeout_ms);
    CameraSdkStatus st_r = CameraGetImageBuffer(right_handle_, &head_r, &raw_buffer_r, timeout_ms);

    if (st_l != CAMERA_STATUS_SUCCESS || st_r != CAMERA_STATUS_SUCCESS) {
        if (st_l == CAMERA_STATUS_SUCCESS) CameraReleaseImageBuffer(left_handle_, raw_buffer_l);
        if (st_r == CAMERA_STATUS_SUCCESS) CameraReleaseImageBuffer(right_handle_, raw_buffer_r);
        return false;
    }

    // Convert raw to BGR
    CameraImageProcess(left_handle_, raw_buffer_l, left_buffer_, &head_l);
    CameraImageProcess(right_handle_, raw_buffer_r, right_buffer_, &head_r);

    out_frame.left.image = cv::Mat(head_l.iHeight, head_l.iWidth, CV_8UC3, left_buffer_).clone();
    out_frame.left.timestamp_us = head_l.uiTimeStamp;
    static uint32_t s_frame_id = 0;
    out_frame.left.frame_id = ++s_frame_id;
    out_frame.right.frame_id = s_frame_id;

    CameraReleaseImageBuffer(left_handle_, raw_buffer_l);
    CameraReleaseImageBuffer(right_handle_, raw_buffer_r);

    out_frame.is_valid = true;
    return true;
}

} // namespace volleyball_vision
