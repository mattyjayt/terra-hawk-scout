// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#ifndef SCOUT_LOCALIZATION__ODOM_MATH_HPP_
#define SCOUT_LOCALIZATION__ODOM_MATH_HPP_

#include <array>

namespace scout_localization
{

// Variance for "this value is not measured": large enough that the EKF ignores it.
constexpr double kUnused = 1.0e6;

// Row-major 6x6 covariance in the order (x, y, z, roll, pitch, yaw), the layout of
// nav_msgs/Odometry pose.covariance and twist.covariance.
// Diagonal: var_x, var_y, kUnused, kUnused, kUnused, var_yaw. Everything else 0.
std::array<double, 36> planar_covariance(double var_x, double var_y, double var_yaw);

// False for speeds the Scout can't reach (a scan-matching glitch).
// Both signs count; the limit itself is still plausible.
bool is_plausible(double vx, double vyaw, double max_vx, double max_vyaw);

}  // namespace scout_localization

#endif  // SCOUT_LOCALIZATION__ODOM_MATH_HPP_