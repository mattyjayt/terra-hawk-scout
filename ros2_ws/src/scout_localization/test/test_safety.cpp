// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include <limits>
#include <optional>

#include "scout_localization/safety.hpp"

using scout_localization::classify;
using scout_localization::limit;
using scout_localization::SafetyParams;
using scout_localization::SafetyState;

namespace
{
const SafetyParams kP{0.15, 0.10, 0.3};   // stop below 0.25 m in the laser frame
const double kInf = std::numeric_limits<double>::infinity();
}  // namespace

TEST(Safety, Classify)
{
  EXPECT_TRUE(classify(kInf, std::nullopt, kP) == SafetyState::kNoScan);  // no scan yet
  EXPECT_TRUE(classify(2.0, 0.5, kP) == SafetyState::kStale);             // older than 0.3 s
  EXPECT_TRUE(classify(0.20, 0.05, kP) == SafetyState::kObstacle);        // 0.20 < 0.25
  EXPECT_TRUE(classify(2.0, 0.05, kP) == SafetyState::kClear);
}

TEST(Safety, Limit)
{
  const auto clear = limit({0.2, 0.5}, SafetyState::kClear);
  EXPECT_DOUBLE_EQ(clear.linear_x, 0.2);         // passes unchanged
  const auto obst = limit({0.2, 0.5}, SafetyState::kObstacle);
  EXPECT_DOUBLE_EQ(obst.linear_x, 0.0);          // no forward
  EXPECT_DOUBLE_EQ(obst.angular_z, 0.5);         // but may turn
  const auto stale = limit({-0.1, 0.5}, SafetyState::kStale);
  EXPECT_DOUBLE_EQ(stale.linear_x, 0.0);         // blind: everything stops
  EXPECT_DOUBLE_EQ(stale.angular_z, 0.0);
}
