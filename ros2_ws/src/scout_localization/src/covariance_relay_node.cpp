// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "scout_localization/odom_math.hpp"

namespace scout_localization
{

class CovarianceRelay : public rclcpp::Node
{
public:
  CovarianceRelay()
  : Node("covariance_relay")
  {
    const double var_vx = declare_parameter<double>("var_vx", 0.0025);
    const double var_vyaw = declare_parameter<double>("var_vyaw", 0.01);
    this->max_vx_ = declare_parameter<double>("max_vx", 0.6);
    this->max_vyaw_ = declare_parameter<double>("max_vyaw", 3.0);

    // Refuse to start condition
    if (var_vx <= 0.0 || var_vyaw <= 0.0 || this->max_vx_ <= 0.0 || this->max_vyaw_ <= 0.0) {
      throw std::invalid_argument(
          "covariance_relay: var_vx, var_vyaw, max_vx, max_vyaw must be > 0");
    }

    this->twist_cov_ = planar_covariance(var_vx, var_vx, var_vyaw);
    this->pose_cov_ = planar_covariance(kUnused, kUnused, kUnused);

    this->pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    this->sub_ = create_subscription<nav_msgs::msg::Odometry>("odom/rf2o", rclcpp::QoS(10),
        std::bind(&CovarianceRelay::on_odom, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "covariance_relay: odom/rf2o -> odom, var_vx %.4g, var_vyaw %.4g",
        var_vx, var_vyaw);
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const double vx = msg.twist.twist.linear.x;
    const double vyaw = msg.twist.twist.angular.z;

    if (!is_plausible(vx, vyaw, this->max_vx_, this->max_vyaw_)) {
      ++dropped_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "dropped implausible rf2o odometry vx=%.2f vyaw=%.2f (%s dropped so far)", vx, vyaw,
          std::to_string(dropped_).c_str());
      return;
    }
    nav_msgs::msg::Odometry out = msg;
    out.twist.covariance = this->twist_cov_;
    out.pose.covariance = this->pose_cov_;
    this->pub_->publish(out);
  }

  // ---- state ----
  double max_vx_{0.6};
  double max_vyaw_{3.0};
  std::array<double, 36> twist_cov_{};
  std::array<double, 36> pose_cov_{};
  std::uint64_t dropped_{0};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

}  // namespace scout_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<scout_localization::CovarianceRelay>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("covariance_relay"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
