// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "scout_localization/scan_math.hpp"

namespace scout_localization
{

class ScanProbe : public rclcpp::Node
{
public:
  ScanProbe()
  : Node("scan_probe")
  {
    this->period_s_ = declare_parameter<double>("period_s", 0.5);
    const double hw_deg = declare_parameter<double>("sector_half_width_deg", 10.0);

    if (this->period_s_ <= 0.0){
      throw std::invalid_argument("scan_probe: period must be > 0");
    }

    if (hw_deg <= 0.0 || hw_deg > 90.0) {
      throw std::invalid_argument("scan_probe: sector_half_width_deg must be (0:90]");
    }

    this->sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&ScanProbe::on_scan, this, std::placeholders::_1));
    this->timer_ = create_wall_timer(std::chrono::duration<double>(this->period_s_), std::bind(&ScanProbe::report, this));

    RCLCPP_INFO(get_logger(), "scan_probe: every %.2f s, sectors +-%0.f deg", this->period_s_, hw_deg);
  }

private:
  void on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    this->last_ = msg;
  }

  void report()
  {
    if (!this->last_) {
      RCLCPP_INFO(get_logger(), "no scan yet");
      return;
    }

    const auto &s = *last_;
    const ScanGeometry g{s.angle_min, s.angle_increment, s.range_min, s.range_max};
    const std::size_t n = s.ranges.size();
    if (n == 0) { RCLCPP_WARN(get_logger(), "empty scan"); return; }
    double valid_pct = 100.0 * count_valid(s.ranges, g)/n;
    double front = sector_min(s.ranges, g, 0.0, this->half_width_);
    double left = sector_min(s.ranges, g, kPi / 2, this->half_width_);
    double back = sector_min(s.ranges, g, kPi, this->half_width_);
    double right = sector_min(s.ranges, g, -kPi / 2, this->half_width_);
    const auto i = nearest_index(s.ranges, g);
    std::size_t i0 = angle_to_index(g, n, 0.0);

    if (!i.has_value()) {
      RCLCPP_INFO(get_logger(), "%s: n=%zu valid=%.0f%% front=%.2f left=%.2f back=%.2f right=%.2f \nnearest=none m '@' 45 deg (i0=%zu)",
      s.header.frame_id.c_str(), n, valid_pct, front, left, back, right, i0);
    } else {
      RCLCPP_INFO(get_logger(), "%s: n=%zu valid=%.0f%% front=%.2f left=%.2f back=%.2f right=%.2f \nnearest=%.2f m '@' 45 deg (i0=%zu)",
      s.header.frame_id.c_str(), n, valid_pct, front, left, back, right, wrap_angle(beam_angle(g, *i)) * 180.0 /kPi, i0);
    }
  }

  // ---- state ----
  double period_s_{0.5};
  double half_width_{10.0 * kPi / 180.0};
  sensor_msgs::msg::LaserScan::ConstSharedPtr last_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace scout_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<scout_localization::ScanProbe>());
  } catch (const std::exception & e) {   // refuse-to-start (step 1)
    RCLCPP_FATAL(rclcpp::get_logger("scan_probe"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}