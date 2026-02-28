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
        setup_undistortion("/home/robot/autonomous_navigation/src/perception_stack/params/camera_calibration.json");
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/image_raw/compressed", qos);
        
        int device_id = this->declare_parameter("device", 0);
        std::string device_path = "/dev/video" + std::to_string(device_id);

        // Cambiamos a V4L2 pero con una configuración que la Jetson acepte mejor
        cap_.open(device_id, cv::CAP_V4L2); 

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir %s", device_path.c_str());
            return;
        }

        // Obligamos a la cámara a usar MJPG para no saturar el bus USB
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        cap_.set(cv::CAP_PROP_FPS, 30);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 2); // Un poco de buffer para evitar saltos

        apply_v4l2_settings(device_path);

        // HILO DE CAPTURA: Usamos swap para que sea instantáneo
        capture_thread_ = std::thread([this]() {
            cv::Mat tmp_frame;
            while (rclcpp::ok() && running_) {
                if (cap_.grab()) { // grab() es más rápido que read()
                    cap_.retrieve(tmp_frame);
                    if (tmp_frame.empty()) continue;
                    
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    frame_buffer_ = tmp_frame.clone(); // En Jetson, clone() es seguro para evitar corrupción
                    new_frame_available_ = true;
                }
            }
        });

        // Timer a 30ms (33.3 Hz) para asegurar que procesamos cada frame de los 30fps
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30), 
            std::bind(&CameraNode::process_and_publish, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Nodo Jetson Optimizado: 30 FPS Objetivo.");
    }

    ~CameraNode() {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (cap_.isOpened()) cap_.release();
    }

private:
    void apply_v4l2_settings(const std::string & device) {
        RCLCPP_INFO(this->get_logger(), "Aplicando configuración de hardware a la C922...");
        
        std::string base_cmd = "v4l2-ctl -d " + device + " --set-ctrl ";
        
        // 1. DESACTIVAR EL BLOQUEO DE FPS (Crucial para pasar de 15 a 30 Hz)
        system((base_cmd + "exposure_dynamic_framerate=0").c_str());

        // 2. DESACTIVAR ENFOQUE AUTOMÁTICO (Evita que la imagen baile durante la navegación)
        system((base_cmd + "focus_automatic_continuous=0").c_str());
        system((base_cmd + "focus_absolute=0").c_str());

        // 3. CONFIGURAR EXPOSICIÓN MANUAL
        // 1 = Manual Mode, 3 = Aperture Priority (visto en tu log)
        system((base_cmd + "auto_exposure=1").c_str()); 
        // Ajusta el valor 250 según la luz de tu laboratorio. 
        // Si subes mucho de 333, los FPS volverán a caer.
        system((base_cmd + "exposure_time_absolute=200").c_str());

        // 4. CONFIGURAR BALANCE DE BLANCOS MANUAL
        system((base_cmd + "white_balance_automatic=0").c_str());
        system((base_cmd + "white_balance_temperature=4000").c_str());

        // 5. BRILLO Y GANANCIA (Opcional)
        system((base_cmd + "brightness=128").c_str());
        system((base_cmd + "gain=0").c_str());

        RCLCPP_INFO(this->get_logger(), "Controles V4L2 sincronizados con éxito.");
    }

    void setup_undistortion(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) return;
        json calib = json::parse(f);
        
        cv::Mat K = (cv::Mat_<double>(3,3) << calib["intrinsics"]["fx"], 0, calib["intrinsics"]["cx"], 
                                              0, calib["intrinsics"]["fy"], calib["intrinsics"]["cy"], 
                                              0, 0, 1);
        cv::Mat D = (cv::Mat_<double>(1,5) << calib["distortion"]["k1"], calib["distortion"]["k2"], 
                                              calib["distortion"]["p1"], calib["distortion"]["p2"], 
                                              calib["distortion"]["k3"]);

        cv::Size image_size(640, 360);
        cv::Mat newK = cv::getOptimalNewCameraMatrix(K, D, image_size, 0, image_size);
        cv::initUndistortRectifyMap(K, D, cv::Mat(), newK, image_size, CV_32FC1, map1_, map2_);
        use_undistortion_ = true;
    }

    void process_and_publish() {
        cv::Mat local_frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!new_frame_available_) return;
            local_frame = frame_buffer_;
            new_frame_available_ = false;
        }

        // 1. Redimensionar 1080p -> 360p (Filtrado de ruido INTER_AREA)
        cv::resize(local_frame, resized_frame_, cv::Size(640, 360), 0, 0, cv::INTER_AREA);

        // 2. Rectificar
        cv::Mat undistorted;
        if (use_undistortion_) {
            cv::remap(resized_frame_, undistorted, map1_, map2_, cv::INTER_LINEAR);
        } else {
            undistorted = resized_frame_;
        }

        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_link";
        msg->format = "jpeg";
        
        cv::imencode(".jpg", undistorted, msg->data, {cv::IMWRITE_JPEG_QUALITY, 80});
        publisher_->publish(std::move(msg));
    }

    cv::VideoCapture cap_;
    std::atomic<bool> running_;
    std::thread capture_thread_;
    std::mutex frame_mutex_;
    cv::Mat frame_buffer_, resized_frame_, map1_, map2_;
    bool new_frame_available_ = false;
    bool use_undistortion_ = false;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}