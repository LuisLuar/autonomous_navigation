#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include "custom_interfaces/msg/pixel_point.hpp"

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) std::cout << msg << std::endl;
    }
} gLogger;

class YOLOPv2Segmenter : public rclcpp::Node {
public:
    YOLOPv2Segmenter() : Node("segmenter_yolop") {
        load_engine("/home/raynel/autonomous_navigation/src/perception_stack/semantic_segmentation/pretrained/yolopv2_lane_OMEN.engine");
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", qos,
            std::bind(&YOLOPv2Segmenter::image_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<custom_interfaces::msg::PixelPoint>("/segmentation_data", 10);
    }

    ~YOLOPv2Segmenter() {
        cudaFree(buffers_[0]);
        cudaFree(buffers_[1]);
    }

private:
    void load_engine(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo .engine");
            return;
        }
        file.seekg(0, file.end);
        size_t size = file.tellg();
        file.seekg(0, file.beg);
        std::vector<char> data(size);
        file.read(data.data(), size);
        file.close();

        runtime_ = nvinfer1::createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(data.data(), size);
        context_ = engine_->createExecutionContext();

        // --- DIAGNÓSTICO DE TENSORES ---
        int nbBindings = engine_->getNbIOTensors();
        RCLCPP_INFO(this->get_logger(), "El modelo tiene %d tensores:", nbBindings);
        
        for (int i = 0; i < nbBindings; ++i) {
            const char* name = engine_->getIOTensorName(i);
            auto mode = engine_->getTensorIOMode(name);
            RCLCPP_INFO(this->get_logger(), "Tensor %d: %s [%s]", 
                        i, name, (mode == nvinfer1::TensorIOMode::kINPUT ? "INPUT" : "OUTPUT"));
        }

        // Reservar memoria
        cudaMalloc(&buffers_[0], 1 * 3 * 640 * 640 * sizeof(float));
        cudaMalloc(&buffers_[1], 1 * 1 * 640 * 640 * sizeof(float));
        
        RCLCPP_INFO(this->get_logger(), "TensorRT 10 Engine cargado correctamente.");
    }

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (frame.empty()) return;

            int ori_h = frame.rows;
            int ori_w = frame.cols;

            cv::Mat resized;
            cv::resize(frame, resized, cv::Size(640, 640));
            cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
            resized.convertTo(resized, CV_32FC3, 1.0 / 255.0);

            std::vector<cv::Mat> channels(3);
            cv::split(resized, channels);
            float input_data[3 * 640 * 640];
            for (int i = 0; i < 3; ++i) {
                memcpy(input_data + i * 640 * 640, channels[i].data, 640 * 640 * sizeof(float));
            }

            // --- INFERENCIA TENSORRT 10 ---
            cudaMemcpy(buffers_[0], input_data, 3 * 640 * 640 * sizeof(float), cudaMemcpyHostToDevice);
            
            // Definir direcciones de tensores (los nombres dependen de cómo se exportó el ONNX)
            // Por defecto YOLOPv2 suele usar "images" y "output"
            context_->setTensorAddress("images", buffers_[0]);
            context_->setTensorAddress("lane_mask", buffers_[1]);
            
            context_->enqueueV3(0); // Ejecución en el stream default
            
            float output_data[640 * 640];
            cudaMemcpy(output_data, buffers_[1], 640 * 640 * sizeof(float), cudaMemcpyDeviceToHost);
            // ------------------------------

            cv::Mat ll_mask_raw(640, 640, CV_32FC1, output_data);
            cv::Mat ll_mask_bin;
            cv::threshold(ll_mask_raw, ll_mask_bin, 0.5, 255, cv::THRESH_BINARY);
            ll_mask_bin.convertTo(ll_mask_bin, CV_8UC1);

            cv::Mat ll_mask;
            cv::resize(ll_mask_bin, ll_mask, cv::Size(ori_w, ori_h), 0, 0, cv::INTER_NEAREST);

            auto out_msg = custom_interfaces::msg::PixelPoint();
            out_msg.header = msg->header;
            for (int y = 0; y < ll_mask.rows; ++y) {
                const uint8_t* row_ptr = ll_mask.ptr<uint8_t>(y);
                for (int x = 0; x < ll_mask.cols; ++x) {
                    if (row_ptr[x] > 0) {
                        out_msg.u.push_back(static_cast<uint16_t>(x));
                        out_msg.v.push_back(static_cast<uint16_t>(y));
                    }
                }
            }
            if (!out_msg.u.empty()) publisher_->publish(out_msg);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    nvinfer1::IRuntime* runtime_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;
    void* buffers_[2]; 
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::PixelPoint>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YOLOPv2Segmenter>());
    rclcpp::shutdown();
    return 0;
}