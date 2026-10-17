#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "volleyball_vision/stereo_camera_driver.hpp"
#include "volleyball_vision/stereo_rectifier.hpp"
#include "volleyball_vision/yolo_detector.hpp"
#include "volleyball_vision/stereo_triangulation.hpp"

class VolleyballVisionNode : public rclcpp::Node {
public:
    VolleyballVisionNode() : Node("volleyball_vision_node") {
        // Declare parameters
        this->declare_parameter<std::string>("calib_file", "");
        this->declare_parameter<std::string>("model_path", "");
        this->declare_parameter<double>("conf_thresh", 0.4);
        this->declare_parameter<double>("epipolar_thresh_px", 2.0);
        this->declare_parameter<double>("exposure_us", 2000.0);
        this->declare_parameter<int>("gain", 12);
        this->declare_parameter<int>("speed_level", 1);
        this->declare_parameter<int>("roi_x", 0);
        this->declare_parameter<int>("roi_y", 0);
        this->declare_parameter<int>("roi_width", 1280);
        this->declare_parameter<int>("roi_height", 720);
        this->declare_parameter<bool>("use_hw_trigger", false);
        this->declare_parameter<bool>("enable_debug_image", false);
        this->declare_parameter<bool>("show_opencv_window", true);
        this->declare_parameter<std::string>("frame_id", "camera_optical_frame");

        calib_file_ = this->get_parameter("calib_file").as_string();
        model_path_ = this->get_parameter("model_path").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        enable_debug_image_ = this->get_parameter("enable_debug_image").as_bool();
        show_opencv_window_ = this->get_parameter("show_opencv_window").as_bool();

        point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/volleyball/position", rclcpp::SensorDataQoS());

        if (enable_debug_image_) {
            debug_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/volleyball/debug_image", 10);
        }

        // Initialize modules
        if (!calib_file_.empty()) {
            rectifier_.loadCalibration(calib_file_);
            triangulator_.setProjectionMatrices(rectifier_.getP1(), rectifier_.getP2(), rectifier_.getBaseline());
        }
        triangulator_.setEpipolarThreshold(this->get_parameter("epipolar_thresh_px").as_double());

        if (!model_path_.empty()) {
            detector_.loadModel(model_path_, this->get_parameter("conf_thresh").as_double());
        }

        // Setup camera driver (方案 1: speed_level=1, 方案 2: ROI 1280x720)
        camera_driver_.initialize(0, 1);
        camera_driver_.setSpeedLevel(this->get_parameter("speed_level").as_int());
        camera_driver_.setROI(
            this->get_parameter("roi_x").as_int(),
            this->get_parameter("roi_y").as_int(),
            this->get_parameter("roi_width").as_int(),
            this->get_parameter("roi_height").as_int());
        camera_driver_.setExposureTime(this->get_parameter("exposure_us").as_double());
        camera_driver_.setAnalogGain(this->get_parameter("gain").as_int());
        camera_driver_.setHardwareTrigger(this->get_parameter("use_hw_trigger").as_bool());
        camera_driver_.startGrabbing();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&VolleyballVisionNode::processLoop, this));

        RCLCPP_INFO(this->get_logger(), "VolleyballVisionNode started successfully.");
    }

private:
    void processLoop() {
        volleyball_vision::StereoFrame frame;
        if (!camera_driver_.getStereoFrame(frame, 10)) {
            return;
        }

        if (frame.left.image.empty() || frame.right.image.empty()) {
            return;
        }

        cv::Mat left_rect, right_rect;
        rectifier_.rectify(frame.left.image, frame.right.image, left_rect, right_rect);

        auto left_dets = detector_.detect(left_rect);
        auto right_dets = detector_.detect(right_rect);

        volleyball_vision::TriangulatedPoint tri_pt;
        if (triangulator_.matchAndTriangulate(left_dets, right_dets, tri_pt) && tri_pt.is_valid) {
            geometry_msgs::msg::PointStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = frame_id_;
            msg.point.x = tri_pt.point_3d.x;
            msg.point.y = tri_pt.point_3d.y;
            msg.point.z = tri_pt.point_3d.z;

            point_pub_->publish(msg);
        }

        cv::Mat disp_l = left_rect.clone();
        cv::Mat disp_r = right_rect.clone();

        for (const auto& det : left_dets) {
            cv::rectangle(disp_l, det.bbox, cv::Scalar(0, 255, 0), 2);
            cv::circle(disp_l, det.center, 4, cv::Scalar(0, 0, 255), -1);
            std::string label = "Ball " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";
            cv::putText(disp_l, label, cv::Point(det.bbox.x, std::max(20, (int)det.bbox.y - 5)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        for (const auto& det : right_dets) {
            cv::rectangle(disp_r, det.bbox, cv::Scalar(0, 255, 0), 2);
            cv::circle(disp_r, det.center, 4, cv::Scalar(0, 0, 255), -1);
            std::string label = "Ball " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";
            cv::putText(disp_r, label, cv::Point(det.bbox.x, std::max(20, (int)det.bbox.y - 5)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        if (tri_pt.is_valid) {
            std::string pos_text = "3D Pos: X=" + cv::format("%.2f", tri_pt.point_3d.x) +
                                   " Y=" + cv::format("%.2f", tri_pt.point_3d.y) +
                                   " Z=" + cv::format("%.2f", tri_pt.point_3d.z) + " m";
            cv::putText(disp_l, pos_text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        cv::putText(disp_l, "Left Camera", cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        cv::putText(disp_r, "Right Camera", cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

        cv::Mat small_l, small_r, combined;
        cv::resize(disp_l, small_l, cv::Size(640, 360));
        cv::resize(disp_r, small_r, cv::Size(640, 360));
        cv::hconcat(small_l, small_r, combined);

        if (enable_debug_image_ && debug_img_pub_) {
            auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", combined).toImageMsg();
            img_msg->header.stamp = this->now();
            img_msg->header.frame_id = frame_id_;
            debug_img_pub_->publish(*img_msg);
        }

        if (show_opencv_window_) {
            cv::imshow("Volleyball Vision ROS 2 Node (Direct OpenCV GUI)", combined);
            cv::waitKey(1);
        }
    }

    std::string calib_file_;
    std::string model_path_;
    std::string frame_id_;
    bool enable_debug_image_{false};
    bool show_opencv_window_{true};

    volleyball_vision::StereoCameraDriver camera_driver_;
    volleyball_vision::StereoRectifier rectifier_;
    volleyball_vision::YoloDetector detector_;
    volleyball_vision::StereoTriangulator triangulator_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VolleyballVisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
