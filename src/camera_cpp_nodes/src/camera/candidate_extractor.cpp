#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "custom_interfaces/msg/segmentation_data.hpp"
#include "custom_interfaces/msg/pixel_point.hpp"

class CandidateExtractorNode : public rclcpp::Node {
public:
    CandidateExtractorNode() : Node("candidate_extractor") {
        // Parámetros
        this->declare_parameter("row_step", 2);
        this->declare_parameter("min_width", 2);
        this->declare_parameter("road_kernel", 5);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        // Suscriptor a la segmentación
        sub_seg_ = this->create_subscription<custom_interfaces::msg::SegmentationData>(
            "/segmentation/data", qos,
            std::bind(&CandidateExtractorNode::segmentation_callback, this, std::placeholders::_1));

        // Publicador de candidatos
        pub_pixels_ = this->create_publisher<custom_interfaces::msg::PixelPoint>(
            "/lane/pixel_candidates", 10);

        RCLCPP_INFO(this->get_logger(), "Extractor de Candidatos C++ iniciado a 30Hz.");
    }

private:
    void segmentation_callback(const custom_interfaces::msg::SegmentationData::SharedPtr msg) {
        // 1. Reconstruir la máscara desde los datos serializados
        cv::Mat combined(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(msg->mask_data.data()));
        
        cv::Mat road_mask = (combined == 1);
        cv::Mat lane_mask = (combined == 2);

        // 2. Limpiar Road Mask (Morphological Close)
        int k_size = this->get_parameter("road_kernel").as_int();
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k_size, k_size));
        cv::morphologyEx(road_mask, road_mask, cv::MORPH_CLOSE, kernel);

        // Vectores para el mensaje
        std::vector<uint32_t> x_coords;
        std::vector<uint32_t> y_coords;
        std::vector<float> confidences;

        int row_step = this->get_parameter("row_step").as_int();
        int min_width = this->get_parameter("min_width").as_int();

        // 3. Procesar ROAD MASK (Extracción de Bordes por Gradiente)
        cv::Mat road_edges;
        cv::morphologyEx(road_mask, road_edges, cv::MORPH_GRADIENT, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        for (int y = 0; y < road_edges.rows; y += row_step) {
            uint8_t* row_ptr = road_edges.ptr<uint8_t>(y);
            for (int x = 0; x < road_edges.cols; x++) {
                if (row_ptr[x] > 0) {
                    x_coords.push_back(x);
                    y_coords.push_back(y);
                    confidences.push_back(0.7f); // Confianza para bordes de carretera
                }
            }
        }

        // 4. Procesar LANE MASK (Centro de Masa por fila)
        // Esto es mucho más robusto que Canny para líneas de carril
        for (int y = 0; y < lane_mask.rows; y += row_step) {
            uint8_t* row_ptr = lane_mask.ptr<uint8_t>(y);
            int start_x = -1;
            
            for (int x = 0; x < lane_mask.cols; x++) {
                if (row_ptr[x] > 0) {
                    if (start_x == -1) start_x = x;
                } else {
                    if (start_x != -1) {
                        int end_x = x - 1;
                        int width = end_x - start_x + 1;
                        if (width >= min_width) {
                            x_coords.push_back(start_x + width / 2);
                            y_coords.push_back(y);
                            confidences.push_back(1.0f); // Máxima confianza para líneas pintadas
                        }
                        start_x = -1;
                    }
                }
            }
        }

        // 5. Publicar mensaje PixelPoint
        auto out_msg = custom_interfaces::msg::PixelPoint();
        out_msg.header = msg->header;
        out_msg.x_coordinates = x_coords;
        out_msg.y_coordinates = y_coords;
        out_msg.confidences = confidences;
        out_msg.is_valid = (x_coords.size() > 50);
        out_msg.global_quality = std::min(1.0f, static_cast<float>(x_coords.size()) / 2000.0f);

        pub_pixels_->publish(out_msg);
    }

    rclcpp::Subscription<custom_interfaces::msg::SegmentationData>::SharedPtr sub_seg_;
    rclcpp::Publisher<custom_interfaces::msg::PixelPoint>::SharedPtr pub_pixels_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CandidateExtractorNode>());
    rclcpp::shutdown();
    return 0;
}