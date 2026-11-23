// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "scout_localization/safety.hpp"
#include "scout_localization/scan_math.hpp"

namespace scout_localization
{

class SafetyStop : public rclcpp::Node
{
public:
  SafetyStop()
  : Node("safety_stop")
  {
    // constructor steps 1-5
    this->params_.stop_distance = declare_parameter<double>("stop_distance", 0.15);
    this->params_.laser_to_front = declare_parameter<double>("laser_to_front", 0.0);
    this->params_.scan_timeout = declare_parameter<double>("scan_timeout", 0.3);
    const double hw_deg = declare_parameter<double>("sector_half_width_deg", 30.0);
    this->half_width_ = hw_deg * kPi / 180.0;

    if (this->params_.laser_to_front <= 0.0) {
      throw std::invalid_argument("safety_stop: laser_to_front must be > 0");
    }

    if (this->params_.stop_distance <= 0) {
      throw std::invalid_argument("safety_stop: stop_distance must be > 0");
    }

    if (this->params_.scan_timeout <= 0) {
      throw std::invalid_argument("safety_stop: scan_timeout must be > 0");
    }

    if (hw_deg <= 0 || hw_deg > 90) {
      throw std::invalid_argument("safety_stop: hw_deg must be in range (0:90]");
    }


    this->cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    this->cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel_in", rclcpp::QoS(10),
        std::bind(&SafetyStop::on_cmd, this, std::placeholders::_1));
    this->scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan",
        rclcpp::SensorDataQoS(), std::bind(&SafetyStop::on_scan, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
        "safety_stop: cmd_vel_in -> cmd_vel, stop below %.3f m in the laser frame (+-%.0f deg)",
        params_.laser_to_front + params_.stop_distance, hw_deg);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    const ScanGeometry g{msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max};
    this->front_min_ = sector_min(msg.ranges, g, 0.0, this->half_width_);
    this->last_scan_time_ = now();
  }

  void on_cmd(const geometry_msgs::msg::Twist & msg)
  {
    std::optional<double> age;
    if (this->last_scan_time_) {
      age = (now() - *last_scan_time_).seconds();
    }

    const SafetyState state = classify(this->front_min_, age, this->params_);
    const Velocity v = limit({msg.linear.x, msg.angular.z}, state);
    geometry_msgs::msg::Twist out;
    out.linear.x = v.linear_x;
    out.angular.z = v.angular_z;
    this->cmd_pub_->publish(out);

    if (state != this->last_state_) {
      this->last_state_ = state;
      if (state != SafetyState::kClear) {
        RCLCPP_WARN(get_logger(), "safety_stop: %s (front %.2f m)", to_string(state),
            this->front_min_);
      } else {
        RCLCPP_INFO(get_logger(), "safety_stop: %s (front %.2f m)", to_string(state),
            this->front_min_);
      }
    }
  }

  // ---- state ----
  SafetyParams params_{0.15, 0.0, 0.3};
  double half_width_{30.0 * kPi / 180.0};
  double front_min_{std::numeric_limits<double>::infinity()};
  std::optional<rclcpp::Time> last_scan_time_;
  SafetyState last_state_{SafetyState::kNoScan};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace scout_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<scout_localization::SafetyStop>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("safety_stop"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
