// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include <libserial/SerialPort.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "scout_base_cpp/conversions.hpp"
#include "scout_base_cpp/kinematics.hpp"
#include "scout_base_cpp/protocol.hpp"

using namespace std::chrono_literals;

namespace scout_base_cpp
{

    namespace
    {
        // Seconds on a clock that never jumps (like Python's time.monotonic()).
        double mono_now()
        {
            return std::chrono::duration<double>(
                       std::chrono::steady_clock::now().time_since_epoch())
                .count();
        }
    } // namespace

    class BaseBridge : public rclcpp::Node
    {
    public:
        using Trigger = std_srvs::srv::Trigger;

        BaseBridge() : Node("base_bridge")
        {
            // 7.4b: start the reader thread (LAST line of the constructor)

            // Parameters
            this->port_name_ = declare_parameter<std::string>("port", "/dev/scout_base");
            this->baud_ = declare_parameter<int>("baud", 115200);
            this->drive_.track_width = declare_parameter<double>("track_width", 0.0);
            this->drive_.pwm_min = declare_parameter<int>("pwm_min", 0);
            this->drive_.max_wheel_speed = declare_parameter<double>("max_wheel_speed", 0.0);
            this->max_linear_ = declare_parameter<double>("max_linear", 0.3);
            this->max_angular_ = declare_parameter<double>("max_angular", 1.5);
            this->cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.25);
            this->send_rate_ = declare_parameter<double>("send_rate", 20.0);
            this->gyro_var_ = declare_parameter<double>("gyro_var", 1.0e-5);
            this->accel_var_ = declare_parameter<double>("accel_var", 2.0e-3);
            this->imu_stale_count_ = declare_parameter<int>("imu_stale_count", 10);
            this->pan_sign_ = declare_parameter<int>("pan_sign", 1);
            this->pan_min_ = declare_parameter<double>("pan_min", -1.5708);
            this->pan_max_ = declare_parameter<double>("pan_max", 1.5708);
            this->cell_count_ = declare_parameter<int>("cell_count", 2);
            this->battery_present_v_ = declare_parameter<double>("battery_present_v", 2.0);
            this->frame_id_imu_ = declare_parameter<std::string>("frame_id_imu", "");
            this->frame_id_base_ = declare_parameter<std::string>("frame_id_base", "");

            // Refuse to start
            if (this->drive_.track_width <= 0.0)
            {
                throw std::invalid_argument("Parameter track_width must be set (> 0)");
            }
            if (this->drive_.max_wheel_speed <= 0.0)
            {
                throw std::invalid_argument("Parameter max_wheel_speed must be set (> 0)");
            }
            if (this->drive_.pwm_min <= 0.0)
            {
                throw std::invalid_argument("Parameter pwm_min must be set (> 0)");
            }

            this->stale_ = StaleDetector(this->imu_stale_count_);

            // Publishers
            this->imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
            this->yaw_pub_ = create_publisher<std_msgs::msg::Float64>("base/debug/yaw", rclcpp::SensorDataQoS());
            this->battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery", rclcpp::QoS(10));
            this->joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));
            this->diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::QoS(10));

            // Subscribers
            this->cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10), std::bind(&BaseBridge::cmd_vel_callback, this, std::placeholders::_1));
            this->pan_sub_ = create_subscription<std_msgs::msg::Float64>("pan/cmd", rclcpp::QoS(10), std::bind(&BaseBridge::pan_cmd_callback, this, std::placeholders::_1));

            // Services
            this->stop_srv_ = create_service<Trigger>("base/stop", std::bind(&BaseBridge::srv_stop, this, std::placeholders::_1, std::placeholders::_2));
            this->calibrate_srv_ = create_service<Trigger>("imu/calibrate", std::bind(&BaseBridge::srv_calibrate, this, std::placeholders::_1, std::placeholders::_2));
            this->zero_yaw_srv_ = create_service<Trigger>("imu/zero_yaw", std::bind(&BaseBridge::srv_zero_yaw, this, std::placeholders::_1, std::placeholders::_2));

            // Timers
            this->send_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / this->send_rate_), std::bind(&BaseBridge::send_tick, this));
            this->battery_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / this->send_rate_), std::bind(&BaseBridge::publish_battery, this));
            this->diag_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / this->send_rate_), std::bind(&BaseBridge::publish_diagnostics, this));

            this->running_ = true;
            this->reader_ = std::thread(&BaseBridge::reader_loop, this);

            RCLCPP_INFO(get_logger(), "base_bridge (C++) started, port %s", this->port_name_.c_str());
        }

        ~BaseBridge() override
        {
            // 7.4b: stop and join the reader thread
            // 7.4f: send S, close the port
            this->running_ = false;
            if (this->reader_.joinable())
            {
                this->reader_.join();
            }
        }

    private:
        std::unique_ptr<LibSerial::SerialPort> serial_; // nullptr = no port object
        std::mutex write_mutex_;                        // guards serial_ open/close/write
        std::atomic<bool> connected_{false};
        std::atomic<bool> running_{false};
        std::thread reader_;

        std::string port_name_;
        int baud_{115200};
        DriveParams drive_; // track_width, max_wheel_speed, pwm_min
        double max_linear_{0.3};
        double max_angular_{1.5};
        double cmd_timeout_{0.25};
        double send_rate_{20.0};
        double gyro_var_{1.0e-5};
        double accel_var_{2.0e-3};
        int imu_stale_count_{10};
        int pan_sign_{1};
        double pan_min_{-1.5708};
        double pan_max_{1.5708};
        int cell_count_{2};
        double battery_present_v_{2.0};
        std::string frame_id_imu_;
        std::string frame_id_base_;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pan_sub_;
        rclcpp::Service<Trigger>::SharedPtr stop_srv_;
        rclcpp::Service<Trigger>::SharedPtr calibrate_srv_;
        rclcpp::Service<Trigger>::SharedPtr zero_yaw_srv_;
        rclcpp::TimerBase::SharedPtr send_timer_;
        rclcpp::TimerBase::SharedPtr battery_timer_;
        rclcpp::TimerBase::SharedPtr diag_timer_;

        StaleDetector stale_{10};

        // ---- 7.4b serial link ----
        bool open_port()
        {
            {                                                  
                std::lock_guard<std::mutex> lock(write_mutex_);
                auto port = std::make_unique<LibSerial::SerialPort>();
                try {
                port->Open(port_name_);
                port->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
                port->FlushInputBuffer();
                } catch (const std::exception &) {
                    return false;
                }
                serial_ = std::move(port);
                connected_ = true;
            }                                                   
            RCLCPP_INFO(get_logger(), "Opened port %s", port_name_.c_str());
            write_line("S\n");
            return true;
        }

        void close_port()
        {
            std::lock_guard<std::mutex> lock(this->write_mutex_);
            this->connected_ = false;
            if (!this->serial_)
            {
                return;
            }

            try
            {
                ::ioctl(this->serial_->GetFileDescriptor(), TIOCNXCL);
                this->serial_->Close();
                this->serial_.reset();
            }
            catch (const std::exception &e)
            {
                ::close(this->serial_->GetFileDescriptor());
                static_cast<void>(this->serial_.release());
            }
        }

        bool write_line(const std::string &text)
        {
            std::lock_guard<std::mutex> lock(this->write_mutex_);
            if (!this->connected_ || !this->serial_)
            {
                return false;
            }

            try
            {
                this->serial_->Write(text);
                return true;
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(get_logger(), "write failed: %s", e.what());
                this->connected_ = false;
            }

            return false;
        }
        void reader_loop()
        {
            bool waiting_logged = false;
            std::string raw;

            while (this->running_)
            {
                if (!this->connected_)
                {
                    this->close_port();
                    if (!this->open_port())
                    {
                        if (!waiting_logged) {
                            RCLCPP_INFO(get_logger(), "Waiting for port %s", port_name_.c_str());
                            waiting_logged = true;
                        }
                        if (this->running_)
                        {
                            for (int i = 0; i < 10 && this->running_; ++i)
                            {
                                std::this_thread::sleep_for(100ms);
                            }
                            continue;
                        }
                        
                    }
                    else
                    {
                        waiting_logged = false;
                    }
                }

                pollfd pfd{serial_->GetFileDescriptor(), POLLIN, 0};
                const int n = ::poll(&pfd, 1, 100);
                if (n <= 0)
                {
                    continue; // 100 ms without data (or EINTR): check running_ again
                }
                if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL))
                {
                    RCLCPP_WARN(get_logger(), "R4 link lost (hang-up)");
                    connected_ = false;
                    continue;
                }

                try
                {
                    this->serial_->ReadLine(raw, '\n', 100);
                }
                catch(const LibSerial::ReadTimeout &){
                    continue;
                }
                catch(const std::exception& e)
                {
                    RCLCPP_WARN(get_logger(), "R4 link lost: %s", e.what());
                    this->connected_ = false;
                    continue;
                }

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "rx: %s", raw.c_str());
            }
        }

        // ---- 7.4c drive ----
        void cmd_vel_callback(const geometry_msgs::msg::Twist &msg) { (void)msg; }
        void send_tick() {}

        // ---- 7.4d telemetry ----
        void handle_line(const std::optional<Line> &line) { (void)line; }
        void on_imu(const ImuSample &s) { (void)s; }
        void publish_battery() {}

        // ---- 7.4e pan + services ----
        void pan_cmd_callback(const std_msgs::msg::Float64 &msg) { (void)msg; }
        void on_ack(const Ack &ack) { (void)ack; }
        void on_watchdog() {}
        std::pair<bool, std::string> send_and_wait(
            const std::string &line, const std::string &expect, double timeout_s)
        {
            (void)line;
            (void)expect;
            (void)timeout_s;
            return {false, "not implemented"};
        }
        void srv_stop(
            const std::shared_ptr<Trigger::Request> request,
            std::shared_ptr<Trigger::Response> response)
        {
            (void)request;
            (void)response;
        }
        void srv_calibrate(
            const std::shared_ptr<Trigger::Request> request,
            std::shared_ptr<Trigger::Response> response)
        {
            (void)request;
            (void)response;
        }
        void srv_zero_yaw(
            const std::shared_ptr<Trigger::Request> request,
            std::shared_ptr<Trigger::Response> response)
        {
            (void)request;
            (void)response;
        }

        // ---- 7.4f diagnostics ----
        void publish_diagnostics() {}

        // ---- state: each sub-task adds its members here ----
    };

} // namespace scout_base_cpp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int code = 0;
    try
    {
        auto node = std::make_shared<scout_base_cpp::BaseBridge>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    { // e.g. refuse-to-start (7.4a)
        RCLCPP_FATAL(rclcpp::get_logger("base_bridge"), "%s", e.what());
        code = 1;
    }
    // `node` was destroyed at the closing brace above, so ~BaseBridge() already ran.
    rclcpp::shutdown();
    return code;
}