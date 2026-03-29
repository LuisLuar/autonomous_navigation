#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#include <cuda_fp16.h> // CRÍTICO para manejar FP16
#include <fstream>
#include <iostream>
#include <vector>
#include "custom_interfaces/msg/pixel_point.hpp"

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << "[TensorRT-FP16] " << msg << std::endl;
    }
} gLogger;

class TwinLiteSegmenter : public rclcpp::Node {
public:
    TwinLiteSegmenter() : Node("segmenter_twinlite") {
        input_h_ = 288;
        input_w_ = 512;
        num_classes_ = 2;

        // Reservar Pinned Memory usando __half (2 bytes por valor)
        cudaMallocHost((void**)&input_buffer_, 3 * input_h_ * input_w_ * sizeof(__half));
        cudaMallocHost((void**)&output_buffer_, num_classes_ * input_h_ * input_w_ * sizeof(__half));

        // 1. Declarar el parámetro con un valor por defecto (opcional)
        this->declare_parameter<std::string>("engine_path", "");
        
        // 2. Obtener el valor del parámetro
        std::string engine_path = this->get_parameter("engine_path").as_string();

        //load_engine("/home/raynel/autonomous_navigation/src/params/twinlite_512x288_lines_chunks.engine");
        // 3. Usar la variable en lugar de la ruta estática
        if (engine_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), " No se proporcionó ruta del engine. Usa el parámetro 'engine_path'");
        } else {
            load_engine(engine_path);
        }

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", qos,
            std::bind(&TwinLiteSegmenter::image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<custom_interfaces::msg::PixelPoint>("/segmentation_data", 10);
        
        RCLCPP_INFO(this->get_logger(), " TwinLite FP16 listo (512x288)");
    }

    ~TwinLiteSegmenter() {
        if (context_) delete context_;
        if (engine_) delete engine_;
        if (runtime_) delete runtime_;
        cudaStreamDestroy(stream_);
        cudaFree(buffers_[0]);
        cudaFree(buffers_[1]);
        cudaFreeHost(input_buffer_);
        cudaFreeHost(output_buffer_);
    }

private:
    void load_engine(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.good()) return;
        file.seekg(0, file.end);
        size_t size = file.tellg();
        file.seekg(0, file.beg);
        std::vector<char> data(size);
        file.read(data.data(), size);

        runtime_ = nvinfer1::createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(data.data(), size);
        context_ = engine_->createExecutionContext();

        for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
            auto name = engine_->getIOTensorName(i);
            if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) input_name = name;
            else output_name = name;
        }

        // Buffers en GPU también en FP16
        cudaMalloc(&buffers_[0], 3 * input_h_ * input_w_ * sizeof(__half));
        cudaMalloc(&buffers_[1], num_classes_ * input_h_ * input_w_ * sizeof(__half));
        cudaStreamCreate(&stream_);
    }

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        // 1. Pre-procesamiento
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(input_w_, input_h_));
        cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
        
        // Convertir a FP32 primero para normalizar, luego a FP16 para el buffer
        cv::Mat float_img;
        resized.convertTo(float_img, CV_32FC3, 1.0f / 255.0f);

        int vol = input_h_ * input_w_;
        std::vector<cv::Mat> channels;
        cv::split(float_img, channels);

        // Convertir cada canal a FP16 y copiar al buffer
        for (int c = 0; c < 3; ++c) {
            for (int i = 0; i < vol; ++i) {
                input_buffer_[c * vol + i] = __float2half(channels[c].at<float>(i));
            }
        }

        // 2. Inferencia
        cudaMemcpyAsync(buffers_[0], input_buffer_, 3 * vol * sizeof(__half), cudaMemcpyHostToDevice, stream_);
        context_->setTensorAddress(input_name.c_str(), buffers_[0]); 
        context_->setTensorAddress(output_name.c_str(), buffers_[1]);
        context_->enqueueV3(stream_);
        cudaMemcpyAsync(output_buffer_, buffers_[1], num_classes_ * vol * sizeof(__half), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        // 3. Post-procesamiento (Argmax en FP16)
        __half* ptr_bg = output_buffer_;
        __half* ptr_lane = output_buffer_ + vol;
        
        custom_interfaces::msg::PixelPoint out_msg;
        out_msg.header = msg->header;
        float scale_x = (float)frame.cols / input_w_;
        float scale_y = (float)frame.rows / input_h_;

        for (int i = 0; i < vol; ++i) {
            // Comparamos usando __half2float o directamente si el compilador lo permite
            if (__half2float(ptr_lane[i]) > __half2float(ptr_bg[i])) {
                out_msg.u.push_back(static_cast<int>((i % input_w_) * scale_x));
                out_msg.v.push_back(static_cast<int>((i / input_w_) * scale_y));
            }
        }

        if (!out_msg.u.empty()) publisher_->publish(out_msg);
    }

    int input_h_, input_w_, num_classes_;
    std::string input_name, output_name;
    __half *input_buffer_, *output_buffer_; // Punteros FP16
    nvinfer1::IRuntime* runtime_{nullptr};
    nvinfer1::ICudaEngine* engine_{nullptr};
    nvinfer1::IExecutionContext* context_{nullptr};
    void* buffers_[2];
    cudaStream_t stream_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::PixelPoint>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwinLiteSegmenter>());
    rclcpp::shutdown();
    return 0;
}