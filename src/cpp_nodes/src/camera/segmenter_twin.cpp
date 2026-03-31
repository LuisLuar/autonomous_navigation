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
        cudaMallocHost((void**)&input_buffer_, 3 * input_h_ * input_w_ * sizeof(float));
        cudaMallocHost((void**)&output_buffer_, num_classes_ * input_h_ * input_w_ * sizeof(float));

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

        publisher_ = this->create_publisher<custom_interfaces::msg::PixelPoint>("/lane/pixel_candidates", 10);
        
        //RCLCPP_INFO(this->get_logger(), " TwinLite FP16 listo (512x288)");
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
        cudaMalloc(&buffers_[0], 3 * input_h_ * input_w_ * sizeof(float));
        cudaMalloc(&buffers_[1], num_classes_ * input_h_ * input_w_ * sizeof(float));
        cudaStreamCreate(&stream_);
    }

    // En el callback:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        //auto t_start = std::chrono::high_resolution_clock::now();
        // 1. Decodificar y Pre-procesar (Rápido)
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        cv::Mat resized, float_img;
        cv::resize(frame, resized, cv::Size(input_w_, input_h_));
        cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
        
        // Normalización directa a Float32
        resized.convertTo(float_img, CV_32FC3, 1.0f / 255.0f);

        // HWC a CHW (Copia rápida sin conversión manual a half)
        int vol = input_h_ * input_w_;
        std::vector<cv::Mat> channels(3);
        cv::split(float_img, channels);
        for (int i = 0; i < 3; ++i) {
            memcpy(input_buffer_ + i * vol, channels[i].data, vol * sizeof(float));
        }

        // 2. Inferencia
        cudaMemcpyAsync(buffers_[0], input_buffer_, 3 * vol * sizeof(float), cudaMemcpyHostToDevice, stream_);

        // --- AÑADE ESTAS DOS LÍNEAS AQUÍ ---
        context_->setTensorAddress(input_name.c_str(), buffers_[0]);
        context_->setTensorAddress(output_name.c_str(), buffers_[1]);
        // -----------------------------------

        context_->enqueueV3(stream_);
        cudaMemcpyAsync(output_buffer_, buffers_[1], num_classes_ * vol * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        // 3. Post-proceso con Filtro de Robustez (Igual que en la Jetson)
        float* lane_ptr = output_buffer_ + vol; // Canal de carril
        cv::Mat mask_raw(input_h_, input_w_, CV_32FC1, lane_ptr);
        
        cv::Mat mask_bin;
        cv::threshold(mask_raw, mask_bin, 0.5, 255, cv::THRESH_BINARY);
        mask_bin.convertTo(mask_bin, CV_8UC1);

        custom_interfaces::msg::PixelPoint out_msg;
        out_msg.header = msg->header;
        float scale_x = (float)frame.cols / input_w_;
        float scale_y = (float)frame.rows / input_h_;

        for (int y = 0; y < mask_bin.rows; y++) {
            const uint8_t* row = mask_bin.ptr<uint8_t>(y);
            int start_x = -1;
            for (int x = 0; x < mask_bin.cols; ++x) {
                if (row[x] > 0) {
                    if (start_x == -1) start_x = x;
                } else if (start_x != -1) {
                    int width = x - start_x;
                    // FILTRO DE ROBUSTEZ: Solo acepta anchos lógicos de carril
                    if ( width <= 30) {
                        out_msg.u.push_back(static_cast<int>((start_x + width / 2) * scale_x));
                        out_msg.v.push_back(static_cast<int>(y * scale_y));
                    }
                    start_x = -1;
                }
            }
        }
        publisher_->publish(out_msg);
        // Medir tiempo total del ciclo
        /*auto t_end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Ciclo optimizado: %.2f ms", ms);*/
    }

    int input_h_, input_w_, num_classes_;
    std::string input_name, output_name;
    float *input_buffer_, *output_buffer_;// Punteros FP16
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