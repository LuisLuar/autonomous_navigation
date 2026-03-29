#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include "custom_interfaces/msg/pixel_point.hpp"

// Clase Logger para capturar mensajes de TensorRT
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << "[TensorRT] " << msg << std::endl;
    }
} gLogger;

class YOLOPv2Segmenter : public rclcpp::Node {
public:
    YOLOPv2Segmenter() : Node("segmenter_yolop") {
        input_h_ = 288;
        input_w_ = 512;

        // Optimización: Pinned Memory (CPU) para transferencias rápidas a GPU
        cudaMallocHost((void**)&input_buffer_, 3 * input_h_ * input_w_ * sizeof(float));
        cudaMallocHost((void**)&output_buffer_, input_h_ * input_w_ * sizeof(float));

        // 1. Declarar el parámetro con un valor por defecto (opcional)
        this->declare_parameter<std::string>("engine_path", "");
        
        // 2. Obtener el valor del parámetro
        std::string engine_path = this->get_parameter("engine_path").as_string();

        //load_engine("/home/raynel/autonomous_navigation/src/params/yolopv2_lane_288_OMEN_TRT10.engine");
        // 3. Usar la variable en lugar de la ruta estática
        if (engine_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), " No se proporcionó ruta del engine. Usa el parámetro 'engine_path'");
        } else {
            load_engine(engine_path);
        }
        

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", qos,
            std::bind(&YOLOPv2Segmenter::image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<custom_interfaces::msg::PixelPoint>("/segmentation_data", 10);
    }

    ~YOLOPv2Segmenter() {
        // En TensorRT 10 se usa delete para limpiar punteros
        if (context_) delete context_;
        if (engine_) delete engine_;
        if (runtime_) delete runtime_;

        cudaStreamDestroy(stream_);
        cudaFree(buffers_[0]);
        cudaFree(buffers_[1]);
        
        if (input_buffer_) cudaFreeHost(input_buffer_);
        if (output_buffer_) cudaFreeHost(output_buffer_);
    }

private:
    void load_engine(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.good()) {
            RCLCPP_FATAL(this->get_logger(), "No se pudo abrir el engine en: %s", path.c_str());
            rclcpp::shutdown();
            return;
        }

        file.seekg(0, file.end);
        size_t size = file.tellg();
        file.seekg(0, file.beg);

        std::vector<char> data(size);
        file.read(data.data(), size);
        file.close();

        // Inicializar plugins (necesario para capas especiales de YOLOPv2)
        initLibNvInferPlugins(&gLogger, "");

        runtime_ = nvinfer1::createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(data.data(), size);
        if (!engine_) {
            RCLCPP_FATAL(this->get_logger(), "Fallo al deserializar el Engine.");
            return;
        }
        context_ = engine_->createExecutionContext();

        // Memoria en el Device (GPU)
        cudaMalloc(&buffers_[0], 3 * input_h_ * input_w_ * sizeof(float));
        cudaMalloc(&buffers_[1], input_h_ * input_w_ * sizeof(float));
        cudaStreamCreate(&stream_);

        RCLCPP_INFO(this->get_logger(), "TensorRT 10 Engine cargado correctamente con Pinned Memory.");
    }

    // -------------------- LETTERBOX --------------------
    cv::Mat letterbox(const cv::Mat& img, float& scale, int& pad_w, int& pad_h) {
        int w = img.cols;
        int h = img.rows;

        scale = std::min((float)input_w_ / w, (float)input_h_ / h);
        int new_w = int(w * scale);
        int new_h = int(h * scale);

        cv::Mat resized;
        cv::resize(img, resized, cv::Size(new_w, new_h));

        pad_w = input_w_ - new_w;
        pad_h = input_h_ - new_h;

        cv::Mat canvas = cv::Mat::zeros(input_h_, input_w_, CV_8UC3);
        resized.copyTo(canvas(cv::Rect(0, 0, new_w, new_h)));

        return canvas;
    }

    // -------------------- CALLBACK --------------------
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        int ori_h = frame.rows;
        int ori_w = frame.cols;
        float scale;
        int pad_w, pad_h;

        // Pre-procesamiento
        cv::Mat input_img = letterbox(frame, scale, pad_w, pad_h);
        cv::cvtColor(input_img, input_img, cv::COLOR_BGR2RGB);
        input_img.convertTo(input_img, CV_32FC3, 1.0 / 255.0);

        // CHW y copia a Pinned Memory
        std::vector<cv::Mat> channels(3);
        for (int i = 0; i < 3; ++i) {
            channels[i] = cv::Mat(input_h_, input_w_, CV_32FC1, input_buffer_ + i * input_h_ * input_w_);
        }
        cv::split(input_img, channels);

        // ----------- INFERENCIA -----------
        cudaMemcpyAsync(buffers_[0], input_buffer_, 3 * input_h_ * input_w_ * sizeof(float), cudaMemcpyHostToDevice, stream_);

        context_->setTensorAddress("images", buffers_[0]);
        context_->setTensorAddress("lane_mask", buffers_[1]);

        context_->enqueueV3(stream_);

        cudaMemcpyAsync(output_buffer_, buffers_[1], input_h_ * input_w_ * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);
        // ----------------------------------

        // Post-procesamiento rápido
        cv::Mat mask_raw(input_h_, input_w_, CV_32FC1, output_buffer_);
        cv::Mat mask_bin;
        cv::threshold(mask_raw, mask_bin, 0.5, 255, cv::THRESH_BINARY);
        mask_bin.convertTo(mask_bin, CV_8UC1);

        // Limpieza de ruido
        cv::morphologyEx(mask_bin, mask_bin, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, {3,3}));

        // Quitar padding (ROI útil)
        cv::Mat cropped = mask_bin(cv::Rect(0, 0, input_w_ - pad_w, input_h_ - pad_h));

        // Optimización: Buscar puntos activos con OpenCV (evita bucles for manuales lentos)
        std::vector<cv::Point> pts;
        cv::findNonZero(cropped, pts);

        if (!pts.empty()) {
            custom_interfaces::msg::PixelPoint out_msg;
            out_msg.header = msg->header;
            
            // Reservar memoria para evitar reallocs
            out_msg.u.reserve(pts.size());
            out_msg.v.reserve(pts.size());

            for (const auto& p : pts) {
                // Escalar puntos al tamaño original de la cámara
                out_msg.u.push_back(static_cast<int>(p.x / scale));
                out_msg.v.push_back(static_cast<int>(p.y / scale));
            }
            publisher_->publish(out_msg);
        }
    }

    // -------------------- VARIABLES --------------------
    int input_h_;
    int input_w_;

    float* input_buffer_{nullptr};  // Pinned Memory
    float* output_buffer_{nullptr}; // Pinned Memory

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
    rclcpp::spin(std::make_shared<YOLOPv2Segmenter>());
    rclcpp::shutdown();
    return 0;
}