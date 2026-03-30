#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <atomic>
#include <fstream>

using json = nlohmann::json;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node"), running_(true)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", qos);

        std::string pipeline =
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=1920,height=1080,framerate=30/1 ! "
            "jpegparse ! "
            "nvv4l2decoder mjpeg=1 ! "
            "nvvidconv ! "
            "video/x-raw,width=640,height=360,format=BGRx ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false";

        cap_.open(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir cámara con GStreamer");
            return;
        }

        setup_undistortion("/home/robot/autonomous_navigation/src/perception_stack/params/camera_calibration.json");

        capture_thread_ = std::thread([this]() {
            cv::Mat frame;
            while (rclcpp::ok() && running_) {
                if (!cap_.read(frame) || frame.empty())
                    continue;

                process_and_publish(frame);
            }
        });

        RCLCPP_INFO(this->get_logger(), "Camera node iniciado correctamente.");
    }

    ~CameraNode()
    {
        running_ = false;
        if (capture_thread_.joinable())
            capture_thread_.join();

        if (cap_.isOpened())
            cap_.release();
    }

private:

    void setup_undistortion(const std::string& path)
    {
        std::ifstream f(path);
        if (!f.is_open()) {
            RCLCPP_WARN(this->get_logger(), "No se pudo abrir archivo de calibración.");
            return;
        }

        json calib = json::parse(f);

        cv::Mat K = (cv::Mat_<double>(3,3) <<
            calib["intrinsics"]["fx"], 0, calib["intrinsics"]["cx"],
            0, calib["intrinsics"]["fy"], calib["intrinsics"]["cy"],
            0, 0, 1);

        cv::Mat D = (cv::Mat_<double>(1,5) <<
            calib["distortion"]["k1"],
            calib["distortion"]["k2"],
            calib["distortion"]["p1"],
            calib["distortion"]["p2"],
            calib["distortion"]["k3"]);

        cv::Size size(640, 360);
        cv::Mat newK = cv::getOptimalNewCameraMatrix(K, D, size, 0);

        cv::initUndistortRectifyMap(
            K, D, cv::Mat(), newK, size,
            CV_32FC1, map1_, map2_);

        use_undistortion_ = true;
    }

    void process_and_publish(const cv::Mat& frame)
    {
        cv::Mat output;

        if (use_undistortion_) {
            cv::remap(frame, output, map1_, map2_, cv::INTER_LINEAR);
        } else {
            output = frame;
        }

        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            output
        ).toImageMsg();

        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_link";

        publisher_->publish(*msg);
    }

    cv::VideoCapture cap_;
    std::thread capture_thread_;
    std::atomic<bool> running_;

    cv::Mat map1_, map2_;
    bool use_undistortion_ = false;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}