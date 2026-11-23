// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include "scout_localization/odom_math.hpp"

using scout_localization::is_plausible;
using scout_localization::kUnused;
using scout_localization::planar_covariance;

TEST(OdomMath, PlanarCovariance)
{
  const auto c = planar_covariance(0.0025, 0.0025, 0.01);
  EXPECT_DOUBLE_EQ(c[0], 0.0025);    // x, x
  EXPECT_DOUBLE_EQ(c[7], 0.0025);    // y, y
  EXPECT_DOUBLE_EQ(c[14], kUnused);  // z, z
  EXPECT_DOUBLE_EQ(c[35], 0.01);     // yaw, yaw: the last diagonal entry
  EXPECT_DOUBLE_EQ(c[1], 0.0);       // off-diagonal stays 0
}

TEST(OdomMath, IsPlausible)
{
  EXPECT_TRUE(is_plausible(0.2, 0.5, 0.6, 3.0));     // normal driving
  EXPECT_TRUE(is_plausible(-0.6, -3.0, 0.6, 3.0));   // the limit itself, reverse
  EXPECT_FALSE(is_plausible(1.0, 0.0, 0.6, 3.0));    // too fast: scan-matching glitch
}
