#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <sstream>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

using namespace std::chrono_literals;

class ESP32SerialBridge : public rclcpp::Node {
public:
    ESP32SerialBridge()
        : Node("esp32_serial_bridge"),
          io_(),
          serial_(io_),
          work_guard_(boost::asio::make_work_guard(io_))
    {
        this->declare_parameter("port", "/dev/ttyESP32Control");
        this->declare_parameter("baud", 230400);
        this->declare_parameter("frame_id", "base_footprint");

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/unfiltered", 10);
        imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>("imu/unfiltered", 10);
        diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("status/esp32_control", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&ESP32SerialBridge::cmdVelCallback, this, std::placeholders::_1));

        last_rx_time_ = this->now();

        start_io_thread();
        connect_serial();

        diag_timer_ = this->create_wall_timer(1s,
            std::bind(&ESP32SerialBridge::publishDiagnostics, this));
    }

    ~ESP32SerialBridge() {
        work_guard_.reset();
        io_.stop();
        if (io_thread_.joinable())
            io_thread_.join();
    }

private:

    // ================= IO THREAD =================

    void start_io_thread() {
        io_thread_ = std::thread([this]() {
            io_.run();
        });
    }

    // ================= SERIAL =================

    void connect_serial() {
        if (connected_) return;

        try {
            std::string port = this->get_parameter("port").as_string();
            int baud = this->get_parameter("baud").as_int();

            if (serial_.is_open()) {
                serial_.cancel();
                serial_.close();
            }

            serial_.open(port);

            serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
            serial_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            serial_.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            serial_.set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));

            connected_ = true;
            last_rx_time_ = this->now();

            //RCLCPP_INFO(this->get_logger(), "Conectado a %s", port.c_str());

            start_async_read();
            start_watchdog();

        } catch (std::exception &e) {
            connected_ = false;
            //RCLCPP_WARN(this->get_logger(), "No se pudo conectar: %s", e.what());
            schedule_reconnect();
        }
    }

    void handle_disconnect() {
        if (!connected_) return;

        //RCLCPP_WARN(this->get_logger(), "Desconectado. Intentando reconectar...");

        connected_ = false;

        boost::system::error_code ec;
        serial_.cancel(ec);
        serial_.close(ec);

        if (watchdog_timer_)
            watchdog_timer_->cancel();

        schedule_reconnect();
    }

    void schedule_reconnect() {
        reconnect_timer_ = std::make_unique<boost::asio::steady_timer>(
            io_, std::chrono::seconds(2));

        reconnect_timer_->async_wait([this](const boost::system::error_code&) {
            connect_serial();
        });
    }

    void start_async_read() {
        boost::asio::async_read_until(
            serial_, buffer_, '\n',
            [this](boost::system::error_code ec, std::size_t) {

                if (ec) {
                    handle_disconnect();
                    return;
                }

                std::istream is(&buffer_);
                std::string line;
                std::getline(is, line);

                if (line.rfind("DATA,", 0) == 0) {
                    processLine(line);
                    msg_rcv_count_++;
                    last_rx_time_ = this->now();

                    // 🔥 Reiniciar watchdog al recibir datos
                    start_watchdog();
                }

                start_async_read(); // seguir leyendo
            });
    }

    // ================= WATCHDOG =================

    void start_watchdog() {
        if (watchdog_timer_)
            watchdog_timer_->cancel();

        watchdog_timer_ = std::make_unique<boost::asio::steady_timer>(
            io_, std::chrono::milliseconds(1500));

        watchdog_timer_->async_wait(
            [this](const boost::system::error_code& ec) {

                if (!ec && connected_) {
                    /*RCLCPP_WARN(this->get_logger(),
                        "Watchdog timeout - cancelando lectura");*/

                    boost::system::error_code e;
                    serial_.cancel(e);  //  Fuerza salida del async_read
                }
            });
    }

    // ================= CMD VEL =================

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!connected_) return;

        char buf[128];
        int n = snprintf(buf, sizeof(buf), "CMD,%.3f,%.3f\n",
                         msg->linear.x, msg->angular.z);

        std::string data(buf, n);

        boost::asio::post(io_, [this, data]() {
            if (connected_ && serial_.is_open()) {
                boost::system::error_code ec;
                boost::asio::write(serial_,
                                   boost::asio::buffer(data), ec);
                if (ec)
                    handle_disconnect();
            }
        });

        msg_sent_count_++;
    }

    // ================= PARSER =================

    void processLine(const std::string &line) {
        try {
            std::vector<std::string> parts;
            std::stringstream ss(line);
            std::string item;

            while (std::getline(ss, item, ','))
                parts.push_back(item);

            if (parts.size() < 9)
                return;

            auto now = this->now();

            nav_msgs::msg::Odometry odom;
            odom.header.stamp = now;
            odom.header.frame_id = "odom";
            odom.child_frame_id = this->get_parameter("frame_id").as_string();
            odom.twist.twist.linear.x = std::stod(parts[1]);
            odom.twist.twist.angular.z = std::stod(parts[2]);
            odom_pub_->publish(odom);

            sensor_msgs::msg::Imu imu;
            imu.header.stamp = now;
            imu.header.frame_id = "imu_link";
            imu.linear_acceleration.x = std::stod(parts[3]);
            imu.linear_acceleration.y = std::stod(parts[4]);
            imu.linear_acceleration.z = std::stod(parts[5]);
            imu.angular_velocity.x = std::stod(parts[6]);
            imu.angular_velocity.y = std::stod(parts[7]);
            imu.angular_velocity.z = std::stod(parts[8]);
            imu_pub_->publish(imu);

        } catch (...) {}
    }

    // ================= DIAGNOSTICS =================

    void publishDiagnostics() {
        auto diag = diagnostic_msgs::msg::DiagnosticStatus();
        diag.name = "ESP32 Control";
        diag.hardware_id = "ESP32_Robot_v2";

        double dt = (this->now() - last_rx_time_).seconds();

        if (connected_ && dt < 1.5) {
            diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            diag.message = "Conexión Estable";
        } else {
            diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            diag.message = "Sin datos o desconectado";
        }

        diag_pub_->publish(diag);
    }

    // ================= VARIABLES =================

    boost::asio::io_context io_;
    boost::asio::serial_port serial_;
    boost::asio::streambuf buffer_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::unique_ptr<boost::asio::steady_timer> reconnect_timer_;
    std::unique_ptr<boost::asio::steady_timer> watchdog_timer_;
    std::thread io_thread_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    std::atomic<bool> connected_{false};
    uint64_t msg_rcv_count_{0};
    uint64_t msg_sent_count_{0};
    rclcpp::Time last_rx_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ESP32SerialBridge>());
    rclcpp::shutdown();
    return 0;
}
