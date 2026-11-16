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
            this->frame_id_imu_ = declare_parameter<std::string>("frame_id_imu", "imu_link");
            this->frame_id_base_ = declare_parameter<std::string>("frame_id_base", "base_link");

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
            this->send_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / send_rate_), std::bind(&BaseBridge::send_tick, this));
            this->battery_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / send_rate_), std::bind(&BaseBridge::publish_battery, this));
            this->diag_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / send_rate_), std::bind(&BaseBridge::publish_diagnostics, this));

            this->running_ = true;
            this->reader_ = std::thread(&BaseBridge::reader_loop, this);

            RCLCPP_INFO(get_logger(), "base_bridge (C++) started, port %s", this->port_name_.c_str());
        }

        ~BaseBridge() override
        {
            this->running_ = false;
            if (this->reader_.joinable())
            {
                this->reader_.join();
            }
            write_line("S\n");
            close_port();
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
        double cmd_v_{0.0};
        double cmd_w_{0.0};
        std::optional<double> cmd_time_;
        bool stop_sent_{true};
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
        std::mutex telemetry_mutex_;
        std::deque<double> batt_samples_;
        bool imu_frozen_{false};
        RateMeter imu_rate_{1.0};
        std::optional<double> last_sample_time_;
        std::optional<double> last_battery_v_;
        std::optional<int> last_servo_deg_;
        double last_servo_time_{0.0};
        std::mutex ack_mutex_;
        std::condition_variable ack_cv_;
        std::string last_ack_;
        std::atomic<std::uint64_t> lines_rx_{0};
        std::atomic<std::uint64_t> parse_errors_{0};
        std::atomic<std::uint64_t> watchdog_stops_{0};

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
                        for (int i = 0; i < 10 && this->running_; ++i)
                        {
                            std::this_thread::sleep_for(100ms);
                        }
                        continue;
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
                    continue;
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

                handle_line(parse_line(raw));
                // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "rx: %s", raw.c_str());
            }
        }

        void cmd_vel_callback(const geometry_msgs::msg::Twist &msg) {
            this->cmd_v_ = msg.linear.x;
            this->cmd_w_ = msg.angular.z;
            this->cmd_time_ = mono_now();
        }

        void send_tick() {
            const bool stale = !cmd_time_ || (mono_now() - *cmd_time_) > cmd_timeout_;
            if (stale) {
                if (!stop_sent_) { write_line("S\n"); stop_sent_ = true; }
                return;
            }

            double v = std::clamp(this->cmd_v_, -this->max_linear_, this->max_linear_);
            double w = std::clamp(this->cmd_w_, -this->max_angular_, this->max_angular_);
            const auto [left, right] = twist_to_pwm(v, w, this->drive_);
            write_line(format_drive(left, right));
            this->stop_sent_ = false;
        }

        void handle_line(const std::optional<Line> &line) { 
            if (!line) {
                ++this->parse_errors_;
                return;
            }

            if (std::holds_alternative<ImuSample>(*line)){
                ++this->lines_rx_;
                on_imu(std::get<ImuSample>(*line));
            } 

            else if (std::holds_alternative<Ack>(*line)){
                ++this->lines_rx_;
                on_ack(std::get<Ack>(*line));
            }

            else if (std::holds_alternative<WatchdogStop>(*line)){
                ++this->lines_rx_;
                on_watchdog();
            }
        
        }
        
        void on_imu(const ImuSample &s) { 
            const auto stamp = now();
            sensor_msgs::msg::Imu m;
            m.header.stamp = stamp;
            m.header.frame_id = this->frame_id_imu_;
            m.orientation.w = 1.0;
            m.orientation_covariance[0] = -1.0;
            m.angular_velocity.x = dps_to_radps(s.gyro_x);
            m.angular_velocity.y = dps_to_radps(s.gyro_y);
            m.angular_velocity.z = dps_to_radps(s.gyro_z);
            m.linear_acceleration.x = g_to_mps(s.accel_x);
            m.linear_acceleration.y = g_to_mps(s.accel_y);
            m.linear_acceleration.z = g_to_mps(s.accel_z);
            for (int i: {0, 4, 8}){
                m.angular_velocity_covariance[i] = this->gyro_var_;
                m.linear_acceleration_covariance[i] = this->accel_var_;
            }
            this->imu_pub_->publish(m);

            sensor_msgs::msg::JointState j;
            j.header.stamp = stamp;
            j.name = {"pan_joint"};
            j.position = {servo_deg_to_pan_rad(s.servo_deg, this->pan_sign_)};
            this->joint_pub_->publish(j);

            std_msgs::msg::Float64 yaw;
            yaw.data = s.yaw_deg * kPi / 180.0;
            this->yaw_pub_->publish(yaw);

            std::lock_guard<std::mutex> lock(this->telemetry_mutex_);
            this->batt_samples_.push_back(s.batt_voltage);
            if (this->batt_samples_.size() > 20) { this->batt_samples_.pop_front(); }

            this->imu_frozen_ = this->stale_.update({s.accel_x, s.accel_y, s.accel_z, s.gyro_x, s.gyro_y, s.gyro_z});
            const double t = mono_now();
            this->imu_rate_.tick(t);
            this->last_sample_time_ = t;
        }

        void publish_battery() {
            double v;
            {
                std::lock_guard<std::mutex> lock(this->telemetry_mutex_);
                if (this->batt_samples_.empty()){ return; }
                else {
                    double sum = 0.0; for (double x : batt_samples_) { sum += x; }
                    v = sum / this->batt_samples_.size();
                }
            }
            this->last_battery_v_ = v;
            sensor_msgs::msg::BatteryState b;
            b.header.stamp = now();
            b.header.frame_id = this->frame_id_base_;
            b.voltage = static_cast<float>(v);
            b.present = v > this->battery_present_v_;
            b.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
            const float nan = std::numeric_limits<float>::quiet_NaN();
            b.current = b.charge = b.capacity = b.design_capacity = b.percentage = b.temperature = nan;
            b.cell_voltage.assign(this->cell_count_, static_cast<float>(v / this->cell_count_));
            b.location = "base";
            this->battery_pub_->publish(b);
        }


        void pan_cmd_callback(const std_msgs::msg::Float64 &msg) { 
            int deg = pan_rad_to_servo_deg(msg.data, this->pan_sign_, this->pan_min_, this->pan_max_);

            if (this->last_servo_deg_.has_value() && std::abs(deg - *last_servo_deg_) < 1){ return; }
            if (mono_now() - this->last_servo_time_ < 0.05){ return; }

            if (write_line(format_servo(deg))){
                this->last_servo_deg_ = deg;
                this->last_servo_time_ = mono_now();
            }
        }
        
        void on_ack(const Ack &ack) { 
            {
                std::lock_guard<std::mutex> lock(this->ack_mutex_);
                this->last_ack_ = ack.text;
            }

            this->ack_cv_.notify_all();
        }

        void on_watchdog() {
            ++this->watchdog_stops_;
        }

        std::pair<bool, std::string> send_and_wait(const std::string &line, const std::string &expect, double timeout_s)
        {
            if (!this->connected_) { return {false, "link down"}; }
            { 
                std::lock_guard<std::mutex> lock(this->ack_mutex_);
                this->last_ack_.clear();
            }

            if (!write_line(line)) { return {false, "write failed"}; }

            std::unique_lock<std::mutex> lock(ack_mutex_);
            const bool got = ack_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s), [&] { return last_ack_ == expect; });

            if (got) { return {true, expect}; }
            else {
                char buf[96];
                std::snprintf(buf, sizeof(buf), "no \"%s\" within %.1f s", expect.c_str(), timeout_s);
                return {false, buf};
            }
        }
        
        void srv_stop(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response)
        {
            (void)request;
            std::tie(response->success, response->message) = send_and_wait("S\n", "S ok", 0.5);
        }
        
        void srv_calibrate(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response)
        {
            (void)request;
            std::tie(response->success, response->message) = send_and_wait("C\n", "C ok", 3.0);
        }
        
        void srv_zero_yaw(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response)
        {
            (void)request;
            std::tie(response->success, response->message) = send_and_wait("Z\n", "Z ok", 0.5);
        }

        void publish_diagnostics() {
            const double t = mono_now();
            double rate = 0.0;
            double since = 0.0;
            bool frozen = false;
            {
                std::lock_guard<std::mutex> lock(telemetry_mutex_);
                rate = imu_rate_.rate(t);
                since = last_sample_time_ ? t - *last_sample_time_ : std::numeric_limits<double>::infinity();
                frozen = imu_frozen_;
            }
            const bool connected = this->connected_;
            std::string last_ack;
            {
                std::lock_guard<std::mutex> lk(ack_mutex_);
                last_ack = last_ack_;
            }


            int level = link_level(connected, rate, 20.0, frozen, since, this->last_battery_v_, this->battery_present_v_);
            char rate_txt[16];
            std::snprintf(rate_txt, sizeof(rate_txt), "%.1f", rate);

            std::string message;
            if (!connected) { message = "link down"; }
            else if (frozen) { message = "IMU frozen"; }
            else if (level != 0) {
                if (this->last_battery_v_.has_value() && *last_battery_v_ <= this->battery_present_v_){
                    message = "battery not present";
                } else {
                    message = std::string("telemetry ") + rate_txt + " Hz (expected 20)";
                }
            } else {
                message = "ok";
            }

            diagnostic_msgs::msg::DiagnosticStatus st;
            st.level = static_cast<uint8_t>(level);
            st.name = "base_bridge: R4 link";
            st.message = message;
            st.hardware_id = this->port_name_;
            auto kv = [](const std::string & k, const std::string & v) {
                diagnostic_msgs::msg::KeyValue p;
                p.key = k;
                p.value = v;
                return p;
            };
            auto tf = [](bool b) { return std::string(b ? "True" : "False"); };

            st.values = {
            kv("port", port_name_),
            kv("connected", tf(connected)),
            kv("line_rate_hz", rate_txt),
            kv("lines_rx", std::to_string(lines_rx_.load())),
            kv("parse_errors", std::to_string(parse_errors_.load())),
            kv("watchdog_stops", std::to_string(watchdog_stops_.load())),
            kv("imu_frozen", tf(frozen)),
            kv("last_ack", last_ack),
            kv("protocol", "serial-r4-v1"),
            };

            diagnostic_msgs::msg::DiagnosticArray arr;
            arr.header.stamp = now();
            arr.status.push_back(st);
            this->diag_pub_->publish(arr);
        }
        
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
    {
        RCLCPP_FATAL(rclcpp::get_logger("base_bridge"), "%s", e.what());
        code = 1;
    }
    // `node` was destroyed at the closing brace above, so ~BaseBridge() already ran.
    rclcpp::shutdown();
    return code;
}