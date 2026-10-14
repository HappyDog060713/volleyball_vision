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
        this->declare_parameter<bool>("use_hw_trigger", true);
        this->declare_parameter<bool>("enable_debug_image", false);
        this->declare_parameter<std::string>("frame_id", "camera_optical_frame");

        calib_file_ = this->get_parameter("calib_file").as_string();
        model_path_ = this->get_parameter("model_path").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        enable_debug_image_ = this->get_parameter("enable_debug_image").as_bool();

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

        if (enable_debug_image_ && debug_img_pub_) {
            for (const auto& det : left_dets) {
                cv::rectangle(left_rect, det.bbox, cv::Scalar(0, 255, 0), 2);
            }
            auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_rect).toImageMsg();
            img_msg->header.stamp = this->now();
            img_msg->header.frame_id = frame_id_;
            debug_img_pub_->publish(*img_msg);
        }
    }

    std::string calib_file_;
    std::string model_path_;
    std::string frame_id_;
    bool enable_debug_image_{false};

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
