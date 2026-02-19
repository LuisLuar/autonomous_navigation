#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <torch/script.h> 
#include <torch/torch.h>
#include "custom_interfaces/msg/segmentation_data.hpp"

class YOLOPv2Segmenter : public rclcpp::Node {
public:
    YOLOPv2Segmenter() : Node("segmenter_yolop") {
        device_ = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
        try {
            // Cargar modelo exportado con torch.jit.save()
            module_ = torch::jit::load("/home/raynel/autonomous_navigation/src/perception_stack/semantic_segmentation/pretrained/yolopv2.pt");
            module_.to(device_);
            module_.eval();
            
            // YOLOPv2 oficial usa FP16 en GPU para velocidad
            if (device_.is_cuda()) module_.to(at::kHalf);

            RCLCPP_INFO(this->get_logger(), "YOLOPv2 cargado exitosamente en %s", device_.is_cuda() ? "CUDA" : "CPU");
        } catch (const c10::Error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error cargando el modelo: %s", e.what());
        }

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", qos,
            std::bind(&YOLOPv2Segmenter::image_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<custom_interfaces::msg::SegmentationData>("/segmentation/data", 1);
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (frame.empty()) return;

            int ori_h = frame.rows;
            int ori_w = frame.cols;

            // 1. PRE-PROCESAMIENTO EXACTO AL DEMO
            cv::Mat resized;
            cv::resize(frame, resized, cv::Size(640, 640), 0, 0, cv::INTER_LINEAR);
            cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);

            torch::Tensor tensor = torch::from_blob(resized.data, {640, 640, 3}, torch::kUInt8).to(device_);
            tensor = tensor.permute({2, 0, 1}).unsqueeze(0);
            
            // Convertir a float y normalizar 1/255
            if (device_.is_cuda()) {
                tensor = tensor.to(at::kHalf).div(255.0);
            } else {
                tensor = tensor.to(torch::kFloat32).div(255.0);
            }

            // 2. INFERENCIA
            torch::NoGradGuard no_grad;
            auto output = module_.forward({tensor});
            
            // YOLOPv2 retorna: [ [pred, anchor_grid], seg, ll ]
            auto outputs = output.toTuple()->elements();
            //torch::Tensor da_logits = outputs[1].toTensor(); // Driving Area
            torch::Tensor ll_logits = outputs[2].toTensor(); // Lane Line

            // 3. POST-PROCESAMIENTO (Basado en utils de YOLOPv2)
            //cv::Mat da_mask = get_da_mask(da_logits, ori_h, ori_w);
            cv::Mat ll_mask = get_ll_mask(ll_logits, ori_h, ori_w);

            // 4. EMPAQUETADO
            cv::Mat combined(ori_h, ori_w, CV_8UC1, cv::Scalar(0));
            //combined.setTo(1, da_mask > 0);
            combined.setTo(2, ll_mask > 0);

            auto out_msg = custom_interfaces::msg::SegmentationData();
            out_msg.header = msg->header;
            out_msg.height = ori_h;
            out_msg.width = ori_w;
            out_msg.mask_data.assign(combined.data, combined.data + (combined.total()));
            
            publisher_->publish(out_msg);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error en proceso: %s", e.what());
        }
    }

    cv::Mat post_process_tensor(torch::Tensor mask, float thresh, int h, int w) {
        // 1. Asegurar que estamos en Float32 para la interpolación (evita errores de bits)
        // y asegurar que es 4D [1, 1, 640, 640]
        torch::Tensor mask_4d = mask.view({1, 1, 640, 640}).to(torch::kFloat32);

        // 2. Reescalar
        auto options = torch::nn::functional::InterpolateFuncOptions()
                            .size(std::vector<int64_t>({h, w}))
                            .mode(torch::kBilinear)
                            .align_corners(false);

        torch::Tensor scaled = torch::nn::functional::interpolate(mask_4d, options);

        // 3. Binarizar y mover a CPU
        // Usamos view({h, w}) en lugar de squeeze() para ser ultra específicos
        torch::Tensor binary_mask = (scaled.view({h, w}) > thresh).to(torch::kUInt8).to(torch::kCPU);
        
        // 4. Clonar para asegurar integridad de memoria en OpenCV
        return cv::Mat(h, w, CV_8UC1, binary_mask.data_ptr()).clone();
    }

    cv::Mat get_da_mask(torch::Tensor logits, int h, int w) {
        // Si logits es [1, 2, 640, 640], seleccionamos el canal 1 (carretera)
        // Forzamos a que sea 2D [640, 640] antes de enviarlo a post_process
        torch::Tensor mask = torch::softmax(logits, 1).select(1, 1).view({640, 640});
        return post_process_tensor(mask, 0.5, h, w);
    }

    cv::Mat get_ll_mask(torch::Tensor logits, int h, int w) {
        // Si logits es [1, 1, 640, 640], quitamos dimensiones extras para tener [640, 640]
        torch::Tensor mask = torch::sigmoid(logits).view({640, 640});
        return post_process_tensor(mask, 0.5, h, w);
    }

    torch::jit::Module module_;
    torch::Device device_{torch::kCPU};
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::SegmentationData>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YOLOPv2Segmenter>());
    rclcpp::shutdown();
    return 0;
}