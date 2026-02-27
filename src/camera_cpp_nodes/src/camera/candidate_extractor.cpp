#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <algorithm>
#include <map>
#include "custom_interfaces/msg/pixel_point.hpp"

class CandidateExtractorNode : public rclcpp::Node {
public:
    CandidateExtractorNode() : Node("candidate_extractor") {
        // Parámetros de refinamiento
        this->declare_parameter("min_width", 10); // Ancho mínimo en píxeles para considerar un carril
        this->declare_parameter("max_width", 50); // Para descartar manchas grandes de ruido

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        // Suscriptor a los puntos "brutos" del segmentador
        sub_pixels_ = this->create_subscription<custom_interfaces::msg::PixelPoint>(
            "/segmentation_data", qos,
            std::bind(&CandidateExtractorNode::pixel_callback, this, std::placeholders::_1));

        // Publicador de puntos refinados (Solo centros)
        pub_refined_ = this->create_publisher<custom_interfaces::msg::PixelPoint>(
            "/lane/pixel_candidates", 10);

        RCLCPP_INFO(this->get_logger(), "Nodo de Refinamiento (Centros de Carril) iniciado.");
    }

private:
    void pixel_callback(const custom_interfaces::msg::PixelPoint::SharedPtr msg) {
        if (msg->u.empty()) return;

        // 1. Agrupar puntos por fila (V)
        // Usamos un mapa donde la llave es la fila V y el valor es una lista de coordenadas U
        std::map<uint16_t, std::vector<uint16_t>> rows_map;
        for (size_t i = 0; i < msg->u.size(); ++i) {
            rows_map[msg->v[i]].push_back(msg->u[i]);
        }

        auto out_msg = custom_interfaces::msg::PixelPoint();
        out_msg.header = msg->header;

        int min_w = this->get_parameter("min_width").as_int();
        int max_w = this->get_parameter("max_width").as_int();

        // 2. Para cada fila, encontrar segmentos contiguos y calcular su centro
        for (auto& [v_row, u_coords] : rows_map) {
            // Ordenar las U para encontrar continuidad
            std::sort(u_coords.begin(), u_coords.end());

            if (u_coords.empty()) continue;

            int start_u = u_coords[0];
            for (size_t i = 1; i <= u_coords.size(); ++i) {
                // Si el pixel no es contiguo o es el final del vector, cerramos segmento
                if (i == u_coords.size() || u_coords[i] > u_coords[i-1] + 2) { 
                    int end_u = u_coords[i-1];
                    int width = end_u - start_u + 1;

                    // 3. Filtrar por ancho y guardar solo el centro
                    if (width >= min_w && width <= max_w) {
                        uint16_t center_u = start_u + (width / 2);
                        out_msg.u.push_back(center_u);
                        out_msg.v.push_back(v_row);
                    }

                    if (i < u_coords.size()) start_u = u_coords[i];
                }
            }
        }

        // 4. Publicar puntos finales
        if (!out_msg.u.empty()) {
            pub_refined_->publish(out_msg);
        }
    }

    rclcpp::Subscription<custom_interfaces::msg::PixelPoint>::SharedPtr sub_pixels_;
    rclcpp::Publisher<custom_interfaces::msg::PixelPoint>::SharedPtr pub_refined_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CandidateExtractorNode>());
    rclcpp::shutdown();
    return 0;
}