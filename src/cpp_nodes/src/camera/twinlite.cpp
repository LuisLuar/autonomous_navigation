#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <cuda_fp16.h> // <--- Crucial para manejar FP16
#include <fstream>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include "custom_interfaces/msg/pixel_point.hpp"

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[TensorRT] " << msg << std::endl;
        }
    }
} gLogger;

class TwinLiteNetNode : public rclcpp::Node {
public:
    TwinLiteNetNode() : Node("twinlite_lane_refiner") {
        // --- PARÁMETROS ---
        this->declare_parameter("engine_path", "/home/robot/Downloads/twinlite_512x288_lines_chunks_ORIN.engine");
        
        input_h_ = 288;
        input_w_ = 512;
        
        // El motor es FP16, pero usaremos float en el host para facilitar OpenCV 
        // y convertiremos a half al subir a la GPU.
        input_host_buffer_.resize(3 * input_h_ * input_w_);
        // Salida: 2 canales (0:bg, 1:lines). Reservamos para ambos.
        output_host_buffer_.resize(2 * input_h_ * input_w_); 

        load_engine(this->get_parameter("engine_path").as_string());

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", qos, std::bind(&TwinLiteNetNode::image_callback, this, std::placeholders::_1));

        publisher_refined_ = this->create_publisher<custom_interfaces::msg::PixelPoint>(
            "/lane/pixel_candidates", 10);
        
        RCLCPP_INFO(this->get_logger(), "TwinLiteNet FP16 iniciado en Jetson Orin.");
    }

    ~TwinLiteNetNode() {
        cudaStreamDestroy(stream_);
        cudaFree(device_buffers_[0]);
        cudaFree(device_buffers_[1]);
    }

private:
    void load_engine(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el motor: %s", path.c_str());
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

        // IMPORTANTE: El tamaño en bytes es * sizeof(__half) porque el motor es FP16
        cudaMalloc(&device_buffers_[0], 3 * input_h_ * input_w_ * sizeof(__half));
        cudaMalloc(&device_buffers_[1], 2 * input_h_ * input_w_ * sizeof(__half));
        cudaStreamCreate(&stream_);
        
        // Ajuste de nombres según tu log de trtexec
        context_->setTensorAddress("input", device_buffers_[0]);
        context_->setTensorAddress("lane_lines", device_buffers_[1]);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (frame.empty()) return;

        // 1. PRE-PROCESAMIENTO (Optimizado para 512x288)
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(input_w_, input_h_));
        cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
        resized.convertTo(resized, CV_32FC3, 1.0 / 255.0);

        // HWC a CHW
        std::vector<cv::Mat> channels(3);
        cv::split(resized, channels);
        for (int i = 0; i < 3; ++i) {
            memcpy(input_host_buffer_.data() + i * input_h_ * input_w_,
                   channels[i].data, input_h_ * input_w_ * sizeof(float));
        }

        // 2. INFERENCIA CON CONVERSIÓN FP16
        // Convertimos de float (host) a half (device) "on the fly" usando un kernel o loop simple
        // Para máxima velocidad en Jetson, aquí podrías usar un kernel de CUDA, 
        // pero por simplicidad usamos una conversión temporal:
        std::vector<__half> input_half(input_host_buffer_.size());
        for(size_t i=0; i<input_half.size(); ++i) input_half[i] = __float2half(input_host_buffer_[i]);

        cudaMemcpyAsync(device_buffers_[0], input_half.data(), 
                        input_half.size() * sizeof(__half), cudaMemcpyHostToDevice, stream_);
        
        context_->enqueueV3(stream_);

        std::vector<__half> output_half(output_host_buffer_.size());
        cudaMemcpyAsync(output_half.data(), device_buffers_[1], 
                        output_half.size() * sizeof(__half), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        // Convertimos de vuelta a float para procesar con OpenCV
        for(size_t i=0; i<output_half.size(); ++i) output_host_buffer_[i] = __half2float(output_half[i]);

        // 3. POST-PROCESAMIENTO (Canal 1: Líneas)
        // El output es [1, 2, 288, 512]. El canal 1 empieza en el offset h*w
        float* lane_channel_ptr = output_host_buffer_.data() + (input_h_ * input_w_);
        cv::Mat mask_raw(input_h_, input_w_, CV_32FC1, lane_channel_ptr);
        
        cv::Mat mask_bin;
        cv::threshold(mask_raw, mask_bin, 0.5, 255, cv::THRESH_BINARY);
        mask_bin.convertTo(mask_bin, CV_8UC1);

        // --- EXTRACCIÓN DE CANDIDATOS (Tu lógica de tesis) ---
        auto out_msg = custom_interfaces::msg::PixelPoint();
        out_msg.header = msg->header;

        // Reescalamos puntos al tamaño original del frame
        float scale_x = (float)frame.cols / input_w_;
        float scale_y = (float)frame.rows / input_h_;

        for (int y = 0; y < mask_bin.rows; y += 1) {
            const uint8_t* row = mask_bin.ptr<uint8_t>(y);
            int start_x = -1;
            for (int x = 0; x < mask_bin.cols; ++x) {
                if (row[x] > 0) {
                    if (start_x == -1) start_x = x;
                } else if (start_x != -1) {
                    int width = x - start_x;
                    if (width >= 2 && width <= 30) { // Ajustado a escala 512
                        out_msg.u.push_back((start_x + width / 2) * scale_x);
                        out_msg.v.push_back(y * scale_y);
                    }
                    start_x = -1;
                }
            }
        }
        publisher_refined_->publish(out_msg);
    }

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
    rclcpp::spin(std::make_shared<TwinLiteNetNode>());
    rclcpp::shutdown();
    return 0;
}