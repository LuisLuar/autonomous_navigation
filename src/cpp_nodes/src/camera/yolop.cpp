#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <vector>
#include <map>
#include <algorithm>
#include "custom_interfaces/msg/pixel_point.hpp"
#include <cv_bridge/cv_bridge.h>

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[TensorRT] " << msg << std::endl;
        }
    }
} gLogger;

class YOLOPv2LaneRefiner : public rclcpp::Node {
public:
    YOLOPv2LaneRefiner() : Node("yolop_lane_refiner") {
        // --- PARÁMETROS ---
        this->declare_parameter("min_width", 6);   // Ajustado para 640px
        this->declare_parameter("max_width", 60);  
        this->declare_parameter("engine_path", "/home/robot/Downloads/yolopv2_lane_288_FINAL.engine");

        input_h_ = 288;
        input_w_ = 512;
        
        input_host_buffer_.resize(3 * input_h_ * input_w_);
        output_host_buffer_.resize(input_h_ * input_w_);

        load_engine(this->get_parameter("engine_path").as_string());

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", qos,
            std::bind(&YOLOPv2LaneRefiner::image_callback, this, std::placeholders::_1));

        // Ahora publicamos directamente los candidatos refinados
        publisher_refined_ = this->create_publisher<custom_interfaces::msg::PixelPoint>(
            "/lane/pixel_candidates", 10);
        
        RCLCPP_INFO(this->get_logger(), "Nodo Fusionado: Segmentación + Extracción de Centros iniciado.");
    }

    ~YOLOPv2LaneRefiner() {
        cudaStreamDestroy(stream_);
        cudaFree(device_buffers_[0]);
        cudaFree(device_buffers_[1]);
    }

private:
    void load_engine(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el motor en: %s", path.c_str());
            return;
        }
        file.seekg(0, std::ios::end);
        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> data(size);
        file.read(data.data(), size);

        runtime_ = nvinfer1::createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(data.data(), size);
        context_ = engine_->createExecutionContext();

        cudaMalloc(&device_buffers_[0], 3 * input_h_ * input_w_ * sizeof(float));
        cudaMalloc(&device_buffers_[1], input_h_ * input_w_ * sizeof(float));
        cudaStreamCreate(&stream_);
        
        context_->setTensorAddress("images", device_buffers_[0]);
        context_->setTensorAddress("lane_mask", device_buffers_[1]);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (frame.empty()) return;

        int ori_h = frame.rows;
        int ori_w = frame.cols;

        // 1. PRE-PROCESAMIENTO (Optimizado)
        float scale = std::min((float)input_w_ / ori_w, (float)input_h_ / ori_h);
        int new_w = int(ori_w * scale);
        int new_h = int(ori_h * scale);
        int pad_w = input_w_ - new_w;
        int pad_h = input_h_ - new_h;

        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(new_w, new_h));
        
        cv::Mat canvas = cv::Mat::zeros(input_h_, input_w_, CV_8UC3);
        resized.copyTo(canvas(cv::Rect(0, 0, new_w, new_h)));
        
        cv::cvtColor(canvas, canvas, cv::COLOR_BGR2RGB);
        canvas.convertTo(canvas, CV_32FC3, 1.0 / 255.0);

        std::vector<cv::Mat> channels(3);
        cv::split(canvas, channels);
        for (int i = 0; i < 3; ++i) {
            memcpy(input_host_buffer_.data() + i * input_h_ * input_w_,
                   channels[i].data, input_h_ * input_w_ * sizeof(float));
        }

        // 2. INFERENCIA TENSORRT
        cudaMemcpyAsync(device_buffers_[0], input_host_buffer_.data(), 
                        3 * input_h_ * input_w_ * sizeof(float), cudaMemcpyHostToDevice, stream_);
        context_->enqueueV3(stream_);
        cudaMemcpyAsync(output_host_buffer_.data(), device_buffers_[1], 
                        input_h_ * input_w_ * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        // 3. POST-PROCESAMIENTO + EXTRACCIÓN DE CENTROS (FUSIONADO)
        cv::Mat mask_raw(input_h_, input_w_, CV_32FC1, output_host_buffer_.data());
        cv::Mat mask_bin;
        cv::threshold(mask_raw, mask_bin, 0.5, 255, cv::THRESH_BINARY);
        mask_bin.convertTo(mask_bin, CV_8UC1);

        // Limpieza rápida
        cv::morphologyEx(mask_bin, mask_bin, cv::MORPH_OPEN, 
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        // Solo procesar el área sin padding
        cv::Mat cropped = mask_bin(cv::Rect(0, 0, new_w, new_h));
        
        // Reescalar la máscara a tamaño original para que los puntos sean correctos
        cv::Mat final_mask;
        cv::resize(cropped, final_mask, cv::Size(ori_w, ori_h), 0, 0, cv::INTER_NEAREST);

        // --- LÓGICA DE EXTRACCIÓN DE CANDIDATOS ---
        auto out_msg = custom_interfaces::msg::PixelPoint();
        out_msg.header = msg->header;

        int min_w = this->get_parameter("min_width").as_int();
        int max_w = this->get_parameter("max_width").as_int();

        // Recorremos la máscara fila por fila (escaneando cada 2 o 4 filas para velocidad)
        for (int y = 0; y < final_mask.rows; y += 4) {
            const uint8_t* row_ptr = final_mask.ptr<uint8_t>(y);
            
            int start_x = -1;
            for (int x = 0; x < final_mask.cols; ++x) {
                if (row_ptr[x] > 0) {
                    if (start_x == -1) start_x = x;
                } else {
                    if (start_x != -1) {
                        int width = x - start_x;
                        if (width >= min_w && width <= max_w) {
                            out_msg.u.push_back(start_x + width / 2);
                            out_msg.v.push_back(y);
                        }
                        start_x = -1;
                    }
                }
            }
        }

        if (!out_msg.u.empty()) {
            publisher_refined_->publish(out_msg);
        }
    }

    // Miembros de la clase
    int input_h_, input_w_;
    std::vector<float> input_host_buffer_, output_host_buffer_;
    nvinfer1::IRuntime* runtime_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;
    void* device_buffers_[2];
    cudaStream_t stream_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::PixelPoint>::SharedPtr publisher_refined_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YOLOPv2LaneRefiner>());
    rclcpp::shutdown();
    return 0;
}