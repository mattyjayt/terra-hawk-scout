// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "scout_localization/map_stats.hpp"


namespace scout_localization
{
class MapStats : public rclcpp::Node
{
public:
  MapStats()
  : Node("map_stats")
  {
    this->period_ = declare_parameter<double>("period", 2.0);

    if (this->period_ <= 0.0) {
      throw std::invalid_argument("map_stats: parameter period must be > 0");
    }

    this->sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("map",
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&MapStats::on_map, this, std::placeholders::_1));
    this->timer_ = create_wall_timer(std::chrono::duration<double>(this->period_),
        std::bind(&MapStats::report, this));
  }

private:
  void on_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    this->last_ = msg;
  }

  void report()
  {
    if (!this->last_) {RCLCPP_INFO(get_logger(), "no map yet"); return;}

    const auto & m = *this->last_;
    const double res = m.info.resolution;
    const CellCounts cc = count_cells(m.data);
    const double area = known_area(cc, res);
    const std::optional<CellBox> b = known_bounds(m.data, m.info.width, m.info.height);
    if (!b) {
      RCLCPP_INFO(
        get_logger(), "map %ux%u @ %.3f m: nothing known yet", m.info.width, m.info.height, res);
      return;
    }

    const double ox = m.info.origin.position.x;
    const double oy = m.info.origin.position.y;
    const double x_min = ox + static_cast<double>(b->min_col) * res;
    const double x_max = ox + static_cast<double>(b->max_col + 1) * res;
    const double y_min = oy + static_cast<double>(b->min_row) * res;
    const double y_max = oy + static_cast<double>(b->max_row + 1) * res;

    RCLCPP_INFO(
      get_logger(),
      "map %ux%u @ %.3f m: known %.2f m2 (free %zu, occupied %zu, uncertain %zu), unknown %zu, "
      "box x %.2f..%.2f y %.2f..%.2f m",
      m.info.width, m.info.height, res, area, cc.free, cc.occupied, cc.uncertain, cc.unknown,
      x_min, x_max, y_min, y_max);
  }

  double period_{2.0};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  std::shared_ptr<const nav_msgs::msg::OccupancyGrid> last_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace scout_localization

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<scout_localization::MapStats>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("map_stats"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
