#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp> 
#include <Eigen/Dense>       

#include "custom_interfaces/msg/detection_array.hpp"
#include "custom_interfaces/msg/lane_model.hpp" // Mensaje del EKF
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using json = nlohmann::json;

class DetectorToCloudNode : public rclcpp::Node {
public:
    DetectorToCloudNode() : Node("detector_to_cloud_transform") {        
        load_calibration("/home/raynel/autonomous_navigation/src/perception_stack/params/camera_calibration.json");
        
        this->declare_parameter("min_distance", 0.5);
        this->declare_parameter("max_distance", 12.0);
        min_dist_ = this->get_parameter("min_distance").as_double();
        max_dist_ = this->get_parameter("max_distance").as_double();

        // 1. Suscripción a Detecciones
        sub_detections_ = this->create_subscription<custom_interfaces::msg::DetectionArray>(
            "/detection/results", 10,
            std::bind(&DetectorToCloudNode::process_callback, this, std::placeholders::_1));

        // 2. Suscripción al Modelo de Carril Filtrado (EKF)
        sub_lane_ = this->create_subscription<custom_interfaces::msg::LaneModel>(
            "/lane/model_filtered", 10,
            [this](const custom_interfaces::msg::LaneModel::SharedPtr msg) {
                last_lane_ = msg;
            });

        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detection/meter_clouds", 10);
        RCLCPP_INFO(this->get_logger(), "🚀 IPM + Lane Tracker Debug Integrado");
    }

private:
    // ... (project_pixel y get_class_color se mantienen igual)
    Eigen::Vector2d project_pixel(double u, double v) {
        Eigen::Vector3d pixel(u, v, 1.0);
        Eigen::Vector3d ground = homography_matrix_ * pixel;
        if (std::abs(ground.z()) < 1e-6) return Eigen::Vector2d(-1.0, -1.0); 
        return Eigen::Vector2d(ground.x() / ground.z(), ground.y() / ground.z());
    }

    uint32_t get_class_color(int class_id) {
        uint8_t r = 0, g = 0, b = 0;
        switch (class_id) {
            case 0: r = 0;   g = 0;   b = 255; break; // Person: Azul
            case 1: r = 0;   g = 255; b = 0;   break; // Bicycle: Verde
            case 2: r = 255; g = 0;   b = 0;   break; // Car: Rojo
            case 3: r = 0;   g = 255; b = 255; break; // Motorcycle: Cian
            case 4: r = 255; g = 0;   b = 255; break; // Bus: Magenta
            case 5: r = 255; g = 255; b = 0;   break; // Truck: Amarillo
            case 6: r = 128; g = 0;   b = 128; break; // Speed bump: Púrpura
            case 7: r = 255; g = 128; b = 0;   break; // Crosswalk: Naranja
            case 8: r = 0;   g = 128; b = 255; break; // Speed bump signage: Azul claro
            case 9: r = 128; g = 128; b = 0;   break; // Crosswalk signage: Verde oliva
            default: r = 255; g = 255; b = 255;      
        }
        return ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    }

    // --- CÁLCULO DE INTERSECCIÓN ---
    void calculate_occupancy_percentages(const Eigen::Vector2d& center, double radius, int class_id) {
        if (!last_lane_) return;

        // 1. Definir el intervalo transversal del objeto (Eje Y)
        double obj_y_min = center.y() - radius;
        double obj_y_max = center.y() + radius;

        // 2. Definir intervalos de las zonas
        // Zona de Peligro: Fija entre -0.5 y 0.5 (Ancho total = 1.0m)
        double danger_min = -0.5;
        double danger_max = 0.5;
        double danger_width = 1.0;

        // Zona de Precaución: Basada en el modelo de carril en la posición X del objeto
        double y_lane_center = last_lane_->curvature * std::pow(center.x(), 2) + 
                            std::tan(last_lane_->yaw) * center.x() + 
                            last_lane_->d_lat;
        double half_w = last_lane_->lane_width / 2.0;
        double precaution_min = y_lane_center - half_w;
        double precaution_max = y_lane_center + half_w;
        double precaution_width = last_lane_->lane_width;

        // 3. Calcular Solapamientos
        auto get_overlap = [](double min1, double max1, double min2, double max2) {
            return std::max(0.0, std::min(max1, max2) - std::max(min1, min2));
        };

        double overlap_danger = get_overlap(obj_y_min, obj_y_max, danger_min, danger_max);
        double overlap_precaution = get_overlap(obj_y_min, obj_y_max, precaution_min, precaution_max);

        // 4. Calcular Porcentajes de OCUPACIÓN de la zona
        // ¿Qué porcentaje del carril está bloqueado por el objeto?
        double pct_danger_occupied = (overlap_danger / danger_width) * 100.0;
        double pct_precaution_occupied = (overlap_precaution / precaution_width) * 100.0;

        // Limitar al 100% (si el objeto es más ancho que el carril)
        pct_danger_occupied = std::min(100.0, pct_danger_occupied);
        pct_precaution_occupied = std::min(100.0, pct_precaution_occupied);

        RCLCPP_INFO(this->get_logger(), 
            "ID %d | Ocupación Peligro: %.1f%% | Ocupación Carril: %.1f%%", 
            class_id, pct_danger_occupied, pct_precaution_occupied);
    }

    void process_callback(const custom_interfaces::msg::DetectionArray::SharedPtr msg) {
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header = msg->header;
        cloud_msg->header.frame_id = "base_footprint";
        
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        
        // Puntos estimados: 100 para carriles + 16 por detección
        modifier.resize(msg->detections.size() * 16 + 100);

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(*cloud_msg, "rgb");

        size_t total_points = 0;

        // --- DEBUG: ZONA DE PRECAUCIÓN (BORDES DINÁMICOS DEL CARRIL) ---
        if (last_lane_) {
            uint32_t lane_color = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)0); // Verde para el carril
            double half_w = last_lane_->lane_width / 2.0;

            for (double x = min_dist_; x <= max_dist_; x += 0.4) {
                // Ecuación: y = curvature*x^2 + tan(yaw)*x + d_lat
                double y_center = last_lane_->curvature * std::pow(x, 2) + 
                                  std::tan(last_lane_->yaw) * x + 
                                  last_lane_->d_lat;

                // Dibujar borde izquierdo y derecho del carril
                std::vector<double> offsets = {half_w, -half_w};
                for (double off : offsets) {
                    *iter_x = static_cast<float>(x);
                    *iter_y = static_cast<float>(y_center + off);
                    *iter_z = 0.0f;
                    *iter_rgb = lane_color;
                    ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
                    total_points++;
                }
            }
        }

        // --- DEBUG: ZONA DE PELIGRO (+0.5, -0.5) ---
        uint32_t danger_color = ((uint32_t)200 << 16 | (uint32_t)200 << 8 | (uint32_t)200); // Gris claro
        for (double x = min_dist_; x <= max_dist_; x += 0.5) {
            // Línea Derecha
            *iter_x = static_cast<float>(x); *iter_y = 0.5f; *iter_z = 0.0f; *iter_rgb = danger_color;
            ++iter_x; ++iter_y; ++iter_z; ++iter_rgb; total_points++;
            // Línea Izquierda
            *iter_x = static_cast<float>(x); *iter_y = -0.5f; *iter_z = 0.0f; *iter_rgb = danger_color;
            ++iter_x; ++iter_y; ++iter_z; ++iter_rgb; total_points++;
        }

        // --- DETECCIONES EXISTENTES ---
        for (const auto& det : msg->detections) {
            uint32_t color = get_class_color(det.class_id);
            if (det.class_id == 6 || det.class_id == 7) {
                std::vector<Eigen::Vector2d> corners = {
                    project_pixel(det.u1, det.v1), project_pixel(det.u2, det.v1),
                    project_pixel(det.u2, det.v2), project_pixel(det.u1, det.v2)
                };
                if (corners[2].x() < min_dist_ || corners[2].x() > max_dist_) continue;
                corners.push_back(corners[0]);
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        double t = static_cast<double>(j) / 4.0;
                        *iter_x = corners[i].x() + t * (corners[i+1].x() - corners[i].x());
                        *iter_y = corners[i].y() + t * (corners[i+1].y() - corners[i].y());
                        *iter_z = 0.01f; *iter_rgb = color;
                        ++iter_x; ++iter_y; ++iter_z; ++iter_rgb; total_points++;
                    }
                }
            } 
            else {
                Eigen::Vector2d center = project_pixel((det.u1 + det.u2)/2.0, det.v2);
                Eigen::Vector2d left   = project_pixel(det.u1, det.v2);
                Eigen::Vector2d right  = project_pixel(det.u2, det.v2);
                if (center.x() < min_dist_ || center.x() > max_dist_) continue;

                double radius = (left - right).norm() / 2.0;
                //if (det.class_id == 0) radius *= 0.6;

                calculate_occupancy_percentages(center, radius, det.class_id);
                for (int i = 0; i < 8; ++i) {
                    double angle = i * (2.0 * M_PI / 8.0);
                    *iter_x = center.x() + radius * std::cos(angle);
                    *iter_y = center.y() + radius * std::sin(angle);
                    *iter_z = 0.0f; *iter_rgb = color;
                    ++iter_x; ++iter_y; ++iter_z; ++iter_rgb; total_points++;
                }
            }
        }

        modifier.resize(total_points);
        pub_cloud_->publish(*cloud_msg);
    }
    
    // ... (load_calibration se mantiene igual)
    void load_calibration(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) return;
        json calib = json::parse(f);
        double cam_z = calib["camera_z"], cam_x = calib["camera_x"], cam_y = calib["camera_y"];
        double roll = calib["camera_roll"];
        double fx = calib["intrinsics"]["fx"], fy = calib["intrinsics"]["fy"];
        double cx = calib["intrinsics"]["cx"], cy = calib["intrinsics"]["cy"];
        Eigen::Matrix3d K; K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        Eigen::Matrix3d R_align; R_align << 0, -1, 0, 0, 0, -1, 1, 0, 0;
        double cp = std::cos(roll), sp = std::sin(roll);
        Eigen::Matrix3d R_roll; R_roll << 1, 0, 0, 0, cp, sp, 0, -sp, cp;
        Eigen::Matrix3d R = R_roll * R_align;
        Eigen::Vector3d t = -R * Eigen::Vector3d(cam_x, cam_y, cam_z);
        Eigen::Matrix3d H; H.col(0) = K * R.col(0); H.col(1) = K * R.col(1); H.col(2) = K * t;
        homography_matrix_ = H.inverse();
    }

    Eigen::Matrix3d homography_matrix_; 
    double min_dist_, max_dist_;
    
    // Suscriptores
    rclcpp::Subscription<custom_interfaces::msg::DetectionArray>::SharedPtr sub_detections_;
    rclcpp::Subscription<custom_interfaces::msg::LaneModel>::SharedPtr sub_lane_;
    
    // Almacenamiento del carril
    custom_interfaces::msg::LaneModel::SharedPtr last_lane_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorToCloudNode>());
    rclcpp::shutdown();
    return 0;
}