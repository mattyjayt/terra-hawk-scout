// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include "scout_localization/odom_math.hpp"

#include <array>

namespace scout_localization
{

// STUBS (RED). Task 9.6 replaces them.
std::array<double, 36> planar_covariance(double var_x, double var_y, double var_yaw)
{
  (void)var_x; (void)var_y; (void)var_yaw;
  return {};
}
bool is_plausible(double vx, double vyaw, double max_vx, double max_vyaw)
{
  (void)vx; (void)vyaw; (void)max_vx; (void)max_vyaw;
  return true;
}

}  // namespace scout_localization