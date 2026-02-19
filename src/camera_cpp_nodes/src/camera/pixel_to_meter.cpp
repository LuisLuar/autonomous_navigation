#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp> 
#include <Eigen/Dense>       

#include "custom_interfaces/msg/pixel_point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using json = nlohmann::json;

class PixelToMeterNode : public rclcpp::Node {
public:
    PixelToMeterNode() : Node("pixel_to_meter_transform") {
        // Cargar calibración completa
        load_calibration("/home/raynel/autonomous_navigation/src/perception_stack/params/camera_calibration.json");

        this->declare_parameter("min_distance", 1.0);
        this->declare_parameter("max_distance", 12.0);

        min_dist_ = this->get_parameter("min_distance").as_double();
        max_dist_ = this->get_parameter("max_distance").as_double();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        sub_pixels_ = this->create_subscription<custom_interfaces::msg::PixelPoint>(
            "/lane/pixel_candidates", qos,
            std::bind(&PixelToMeterNode::process_callback, this, std::placeholders::_1));

        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lane/meter_candidates", 10);

        RCLCPP_INFO(this->get_logger(), "IPM Iniciado: h=%.2f, pitch=%.2f deg", cam_h_, pitch_ * 180.0/M_PI);
    }

private:
    void load_calibration(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo de calibración en: %s", path.c_str());
            return;
        }
        json calib = json::parse(f);

        // Posición física de la cámara respecto a base_footprint
        cam_h_ = calib["camera_height"];
        cam_x_ = calib["camera_x"];
        cam_y_ = calib["camera_y"];
        
        // Intrínsecos
        pitch_ = calib["camera_pitch"];
        fx_ = calib["intrinsics"]["fx"];
        fy_ = calib["intrinsics"]["fy"];
        cx_ = calib["intrinsics"]["cx"];
        cy_ = calib["intrinsics"]["cy"];

        // Matriz de rotación (Rotación alrededor del eje X de la cámara para el pitch)
        double cp = std::cos(pitch_);
        double sp = std::sin(pitch_);
        
        // En el sistema de cámara: X-derecha, Y-abajo, Z-adelante
        // La rotación de pitch inclina la cámara hacia abajo
        rotation_matrix_ << 1,   0,   0,
                            0,  cp,  sp,
                            0, -sp,  cp;
    }

void process_callback(const custom_interfaces::msg::PixelPoint::SharedPtr msg) {
    if (!msg->is_valid || msg->x_coordinates.empty()) return;

    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud_msg->header = msg->header;
    cloud_msg->header.frame_id = "base_footprint";
    
    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(msg->x_coordinates.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

    size_t valid_count = 0;

    for (size_t i = 0; i < msg->x_coordinates.size(); ++i) {
        double u = msg->x_coordinates[i];
        double v = msg->y_coordinates[i];

        // 1. Proyectar pixel a rayo (IGUAL A TU PYTHON)
        double x_ray = (u - cx_) / fx_;
        double y_ray = -(v - cy_) / fy_; // Inversión de signo clave
        Eigen::Vector3d ray_cam(x_ray, y_ray, 1.0);

        // 2. Rotar el rayo
        Eigen::Vector3d ray_robot = rotation_matrix_ * ray_cam;

        // 3. Intersección con suelo (Y < 0 según tu lógica de Python)
        if (ray_robot.y() >= 0) continue; 

        // t es el factor de escala (distancia al plano Y=0)
        double t = cam_h_ / -ray_robot.y();
        
        // Coordenadas en el suelo
        double Xc = ray_robot.x() * t; // Lateral
        double Zc = ray_robot.z() * t; // Adelante (Distancia)

        // 4. Transformar a base_footprint (Marco ROS)
        // x_base = Zc + cam_x (Adelante)
        // y_base = -Xc + cam_y (Izquierda positivo)
        double final_x = Zc + cam_x_;
        double final_y = -Xc + cam_y_; 

        // Filtro de distancia (usando Zc que es la profundidad real)
        if (Zc >= min_dist_ && Zc <= max_dist_) {
            *iter_x = static_cast<float>(final_x);
            *iter_y = static_cast<float>(final_y);
            *iter_z = 0.0f;

            ++iter_x; ++iter_y; ++iter_z;
            valid_count++;
        }
    }

    if (valid_count > 0) {
        modifier.resize(valid_count);
        pub_cloud_->publish(*cloud_msg);
    }
}

    double cam_h_, cam_x_, cam_y_, pitch_, fx_, fy_, cx_, cy_;
    double min_dist_, max_dist_;
    Eigen::Matrix3d rotation_matrix_;

    rclcpp::Subscription<custom_interfaces::msg::PixelPoint>::SharedPtr sub_pixels_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PixelToMeterNode>());
    rclcpp::shutdown();
    return 0;
}