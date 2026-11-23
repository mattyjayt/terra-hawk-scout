// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include "scout_localization/odom_math.hpp"

#include <array>
#include <cmath>

namespace scout_localization
{

std::array<double, 36> planar_covariance(double var_x, double var_y, double var_yaw)
{
  std::array<double, 36> c{};
  const std::array<double, 6> diag{var_x, var_y, kUnused, kUnused, kUnused, var_yaw};

  for (std::size_t i = 0; i < 6; ++i) {
    c[i * 7] = diag[i];
  }
  return c;
}

bool is_plausible(double vx, double vyaw, double max_vx, double max_vyaw)
{
  return  (std::abs(vx) <= std::abs(max_vx)) && (std::abs(vyaw) <= std::abs(max_vyaw));
}

}  // namespace scout_localization
