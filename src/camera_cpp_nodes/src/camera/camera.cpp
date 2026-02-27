#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdlib> // Para system()

#include <fstream>
#include <nlohmann/json.hpp>


using json = nlohmann::json;


class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node"), running_(true) {
        setup_undistortion("/home/raynel/autonomous_navigation/src/perception_stack/params/camera_calibration.json");
        // QoS Best Effort para máxima velocidad
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/image_raw/compressed", qos);
        
        int device_id = this->declare_parameter("device", 0);
        std::string device_path = "/dev/video" + std::to_string(device_id);

        // 1. Abrir dispositivo
        cap_.open(device_id, cv::CAP_V4L2); 

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir %s", device_path.c_str());
            return;
        }

        // 2. Configuración de Formato y Resolución vía OpenCV
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        cap_.set(cv::CAP_PROP_FPS, 30);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        // 3. APLICAR CONFIGURACIÓN DIRECTA A V4L2 (Lo que haces en qv4l2)
        apply_v4l2_settings(device_path);

        // Hilo de captura dedicado
        capture_thread_ = std::thread([this]() {
            cv::Mat tmp_frame;
            while (rclcpp::ok() && running_) {
                if (cap_.read(tmp_frame)) {
                    if (tmp_frame.empty()) continue;
                    
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    std::swap(frame_buffer_, tmp_frame); 
                    new_frame_available_ = true;
                }
            }
        });

        // Timer para procesamiento (30ms -> ~33 FPS teóricos)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30), 
            std::bind(&CameraNode::process_and_publish, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Nodo iniciado: 1080p -> 360p con ajustes V4L2 aplicados.");
    }

    ~CameraNode() {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (cap_.isOpened()) cap_.release();
    }

private:
    

    void setup_undistortion(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo de calibración");
            return;
        }
        json calib = json::parse(f);

        fx = calib["intrinsics"]["fx"];
        fy = calib["intrinsics"]["fy"];
        cx = calib["intrinsics"]["cx"];
        cy = calib["intrinsics"]["cy"];

        k1 = calib["distortion"]["k1"];
        k2 = calib["distortion"]["k2"];
        p1 = calib["distortion"]["p1"];
        p2 = calib["distortion"]["p2"];
        k3 = calib["distortion"]["k3"];

        // Valores de tu JSON de calibración
        cv::Mat K = (cv::Mat_<double>(3,3) << 
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0);

        cv::Mat D = (cv::Mat_<double>(1,5) << 
            k1, k2, p1, p2, k3);

        cv::Size image_size(640, 360);
        
        // Obtener la matriz óptima para no perder demasiados píxeles en los bordes
        cv::Mat newK = cv::getOptimalNewCameraMatrix(K, D, image_size, 0, image_size);

        // Pre-calcular los mapas de rectificación (Esto es lo que ahorra CPU)
        cv::initUndistortRectifyMap(K, D, cv::Mat(), newK, image_size, CV_32FC1, map1_, map2_);
        use_undistortion_ = true;
        RCLCPP_INFO(this->get_logger(), "Mapas de rectificación calculados.");
    }

    void apply_v4l2_settings(const std::string & device) {
        RCLCPP_INFO(this->get_logger(), "Configurando controles manuales en %s...", device.c_str());
        
        // Usamos los nombres EXACTOS que reportó tu v4l2-ctl -L
        std::string base_cmd = "v4l2-ctl -d " + device + " --set-ctrl ";
        
        // 1. Desactivar Autofocus (opcional, pero recomendado para robótica)
        system((base_cmd + "focus_automatic_continuous=0").c_str());
        system((base_cmd + "focus_absolute=0").c_str());

        // 2. Configurar Exposición Manual (Crucial para los 30 FPS)
        // auto_exposure: 1 es manual, 3 es Aperture Priority
        system((base_cmd + "auto_exposure=3").c_str()); 
        system((base_cmd + "exposure_time_absolute=0").c_str()); // Baja este valor si hay mucha luz

        // 3. Configurar Balance de Blancos Manual
        system((base_cmd + "white_balance_automatic=0").c_str());
        system((base_cmd + "white_balance_temperature=3700").c_str());

        // 4. Otros ajustes
        system((base_cmd + "brightness=128").c_str());
        system((base_cmd + "gain=0").c_str());

        RCLCPP_INFO(this->get_logger(), "Ajustes V4L2 aplicados con nombres correctos.");
    }

    void process_and_publish() {
        cv::Mat local_frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!new_frame_available_) return;
            std::swap(local_frame, frame_buffer_);
            new_frame_available_ = false;
        }

        // Redimensionamiento con Supersampling (Filtro AREA)
        cv::resize(local_frame, resized_frame_, cv::Size(640, 360), 0, 0, cv::INTER_AREA);

        // 2. Aplicar rectificación (Remap es muy ligero)
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
        
        cv::imencode(".jpg", undistorted, msg->data, compression_params_);
        publisher_->publish(std::move(msg));

    }

    cv::VideoCapture cap_;
    std::atomic<bool> running_;
    std::thread capture_thread_;
    std::mutex frame_mutex_;
    cv::Mat frame_buffer_, resized_frame_;
    bool new_frame_available_ = false;

    // variables para la rectificación
    cv::Mat map1_, map2_;
    double k1, k2, p1, p2, k3;
    double fx, fy, cx, cy;
    bool use_undistortion_ = true;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    std::vector<int> compression_params_ = {cv::IMWRITE_JPEG_QUALITY, 85};
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}