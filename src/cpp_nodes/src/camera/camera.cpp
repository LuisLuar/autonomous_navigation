#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node"), running_(true) {
        setup_undistortion("/home/raynel/autonomous_navigation/src/perception_stack/params/camera_calibration.json");
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/image_raw/compressed", qos);
        
        int device_id = this->declare_parameter("device", 2);
        std::string device_path = "/dev/video" + std::to_string(device_id);

        cap_.open(device_id, cv::CAP_V4L2); 

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir %s", device_path.c_str());
            return;
        }

        // Configuración de Hardware
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        cap_.set(cv::CAP_PROP_FPS, 30);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        apply_v4l2_settings(device_path);

        // Hilo Único de Ejecución: Captura -> Procesa -> Publica
        // Eliminamos el timer_ y el buffer intermedio con mutex
        capture_thread_ = std::thread([this]() {
            cv::Mat raw_frame;
            RCLCPP_INFO(this->get_logger(), "Hilo de procesamiento en tiempo real iniciado.");
            
            while (rclcpp::ok() && running_) {
                // cap_.read es bloqueante: espera al hardware de la cámara
                if (cap_.read(raw_frame)) {
                    if (raw_frame.empty()) continue;
                    process_and_publish_direct(raw_frame);
                }
            }
        });

        RCLCPP_INFO(this->get_logger(), "Nodo Laptop (Sincronía Directa) iniciado.");
    }

    ~CameraNode() {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (cap_.isOpened()) cap_.release();
    }

private:
    void apply_v4l2_settings(const std::string & device) {
        RCLCPP_INFO(this->get_logger(), "Aplicando ajustes V4L2...");
        std::string base_cmd = "v4l2-ctl -d " + device + " --set-ctrl ";
        
        system((base_cmd + "focus_automatic_continuous=0").c_str());
        system((base_cmd + "focus_absolute=0").c_str());
        
        // Ajuste dinámico: auto_exposure=3 es prioridad de apertura
        // auto_exposure_priority=0 fuerza a mantener el framerate
        system((base_cmd + "auto_exposure=3").c_str()); 
        system((base_cmd + "auto_exposure_priority=0").c_str()); 
        
        system((base_cmd + "white_balance_automatic=0").c_str());
        system((base_cmd + "white_balance_temperature=3700").c_str());
        system((base_cmd + "brightness=128").c_str());
        system((base_cmd + "gain=0").c_str());
    }

    void setup_undistortion(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo de calibración");
            return;
        }
        json calib = json::parse(f);

        cv::Mat K = (cv::Mat_<double>(3,3) << 
            calib["intrinsics"]["fx"], 0.0, calib["intrinsics"]["cx"],
            0.0, calib["intrinsics"]["fy"], calib["intrinsics"]["cy"],
            0.0, 0.0, 1.0);

        cv::Mat D = (cv::Mat_<double>(1,5) << 
            calib["distortion"]["k1"], calib["distortion"]["k2"], 
            calib["distortion"]["p1"], calib["distortion"]["p2"], 
            calib["distortion"]["k3"]);

        cv::Size image_size(640, 360);
        cv::Mat newK = cv::getOptimalNewCameraMatrix(K, D, image_size, 0, image_size);
        cv::initUndistortRectifyMap(K, D, cv::Mat(), newK, image_size, CV_32FC1, map1_, map2_);
        
        use_undistortion_ = true;
        RCLCPP_INFO(this->get_logger(), "Mapas de rectificación calculados.");
    }

    void process_and_publish_direct(const cv::Mat & raw_frame) {
        // 1. Redimensionar (CPU en la laptop)
        cv::resize(raw_frame, resized_frame_, cv::Size(640, 360), 0, 0, cv::INTER_AREA);

        // 2. Rectificar
        cv::Mat final_frame;
        if (use_undistortion_) {
            cv::remap(resized_frame_, final_frame, map1_, map2_, cv::INTER_LINEAR);
        } else {
            final_frame = resized_frame_;
        }

        // 3. Comprimir y Publicar
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_link";
        msg->format = "jpeg";
        
        cv::imencode(".jpg", final_frame, msg->data, compression_params_);
        publisher_->publish(std::move(msg));
    }

    cv::VideoCapture cap_;
    std::atomic<bool> running_;
    std::thread capture_thread_;
    cv::Mat resized_frame_, map1_, map2_;
    bool use_undistortion_ = false;
    
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    std::vector<int> compression_params_ = {cv::IMWRITE_JPEG_QUALITY, 85};
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}