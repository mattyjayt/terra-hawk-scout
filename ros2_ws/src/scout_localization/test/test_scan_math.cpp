// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "scout_localization/scan_math.hpp"

using scout_localization::ScanGeometry;
using scout_localization::angle_to_index;
using scout_localization::beam_angle;
using scout_localization::beam_gap;
using scout_localization::count_valid;
using scout_localization::is_valid;
using scout_localization::kPi;
using scout_localization::nearest_index;
using scout_localization::sector_min;
using scout_localization::wrap_angle;

namespace
{
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr double kDeg = kPi / 180.0;

// tools/fake_lidar.py: 360 beams from -pi, 1 degree apart. Beam 180 = straight ahead.
const ScanGeometry kFake{-kPi, 2.0 * kPi / 360.0, 0.05, 12.0};
// A scan whose first beam is NOT at -pi: 360 beams from -179 deg (sllidar_ros2 publishes
// this when angle_compensate is off). Beam 179 = straight ahead. Code that assumes
// angle_min = -pi is one beam off here.
const ScanGeometry kShifted{kPi - 359.0 * kDeg, kDeg, 0.05, 12.0};
// The real C1 through sllidar_ros2, angle_compensate on (recorded on the Pi, 18 Nov):
// 720 beams, angle_min -pi, angle_max +pi, so angle_increment = 2 pi / 719, not / 720.
// Beam 0 and beam 719 both point backwards; 0 rad lies between beams 359 and 360.
const ScanGeometry kC1{-kPi, 2.0 * kPi / 719.0, 0.05, 16.0};

std::vector<float> empty_scan() {return std::vector<float>(360, kInf);}
}  // namespace

TEST(ScanMath, WrapAngle)
{
  EXPECT_DOUBLE_EQ(wrap_angle(0.0), 0.0);
  EXPECT_NEAR(wrap_angle(1.5 * kPi), -0.5 * kPi, 1e-12);
  EXPECT_NEAR(wrap_angle(-1.5 * kPi), 0.5 * kPi, 1e-12);
  EXPECT_NEAR(wrap_angle(kPi), -kPi, 1e-12);      // [-pi, pi): +pi becomes -pi
  EXPECT_NEAR(wrap_angle(-kPi), -kPi, 1e-12);
  EXPECT_NEAR(wrap_angle(7.0), 7.0 - 2.0 * kPi, 1e-12);
}

TEST(ScanMath, BeamAngle)
{
  EXPECT_NEAR(beam_angle(kFake, 0), -kPi, 1e-12);
  EXPECT_NEAR(beam_angle(kFake, 90), -0.5 * kPi, 1e-12);
  EXPECT_NEAR(beam_angle(kFake, 270), 0.5 * kPi, 1e-12);
  EXPECT_NEAR(beam_angle(kShifted, 179), 0.0, 1e-12);
}

TEST(ScanMath, AngleToIndexWraps)
{
  EXPECT_EQ(angle_to_index(kFake, 360, 0.0), 180u);
  EXPECT_EQ(angle_to_index(kFake, 360, 0.5 * kPi), 270u);    // left
  EXPECT_EQ(angle_to_index(kFake, 360, -0.5 * kPi), 90u);    // right
  EXPECT_EQ(angle_to_index(kFake, 360, -kPi), 0u);
  EXPECT_EQ(angle_to_index(kFake, 360, kPi), 0u);            // +pi is the same beam
  EXPECT_EQ(angle_to_index(kFake, 360, 1.5 * kPi), 90u);     // = -pi/2
  EXPECT_EQ(angle_to_index(kFake, 360, 359.8 * kDeg), 180u);
}

TEST(ScanMath, AngleToIndexUsesAngleMin)
{
  EXPECT_EQ(angle_to_index(kShifted, 360, 0.0), 179u);
  EXPECT_EQ(angle_to_index(kShifted, 360, 0.5 * kPi), 269u);
  EXPECT_EQ(angle_to_index(kShifted, 360, kPi), 359u);
}

TEST(ScanMath, IsValid)
{
  EXPECT_TRUE(is_valid(1.0f, kFake));
  EXPECT_TRUE(is_valid(12.0f, kFake));                       // the limit itself is valid
  EXPECT_FALSE(is_valid(kInf, kFake));                       // no echo
  EXPECT_FALSE(is_valid(-kInf, kFake));
  EXPECT_FALSE(is_valid(std::nanf(""), kFake));
  EXPECT_FALSE(is_valid(0.02f, kFake));                      // dust on the window
  EXPECT_FALSE(is_valid(12.5f, kFake));
}

TEST(ScanMath, CountValid)
{
  const std::vector<float> r{1.0f, kInf, std::nanf(""), 0.02f, 3.0f};
  EXPECT_EQ(count_valid(r, kFake), 2u);
}

TEST(ScanMath, SectorMinFront)
{
  auto r = empty_scan();
  r[180] = 0.01f;    // straight ahead, but below range_min: must be ignored
  r[181] = 1.20f;    // 1 deg left
  r[186] = 1.10f;    // 6 deg left
  EXPECT_NEAR(sector_min(r, kFake, 0.0, 5.0 * kDeg), 1.20, 1e-6);
  EXPECT_NEAR(sector_min(r, kFake, 0.0, 10.0 * kDeg), 1.10, 1e-6);
}

TEST(ScanMath, SectorMinEmptyIsInfinity)
{
  auto r = empty_scan();
  r[180] = 1.0f;
  EXPECT_TRUE(std::isinf(sector_min(r, kFake, 0.5 * kPi, 5.0 * kDeg)));
}

TEST(ScanMath, SectorMinLeftIsPositive)
{
  auto r = empty_scan();
  r[270] = 1.0f;     // +90 deg = left (REP-103: y points left)
  r[90] = 0.5f;      // -90 deg = right
  EXPECT_NEAR(sector_min(r, kFake, 0.5 * kPi, 5.0 * kDeg), 1.0, 1e-6);
  EXPECT_NEAR(sector_min(r, kFake, -0.5 * kPi, 5.0 * kDeg), 0.5, 1e-6);
}

TEST(ScanMath, SectorMinWrapsAtBack)
{
  auto r = empty_scan();
  r[359] = 0.80f;    // +179 deg
  r[1] = 0.90f;      // -179 deg
  EXPECT_NEAR(sector_min(r, kFake, kPi, 3.0 * kDeg), 0.80, 1e-6);
  EXPECT_NEAR(sector_min(r, kFake, -kPi, 3.0 * kDeg), 0.80, 1e-6);
  r[359] = kInf;
  EXPECT_NEAR(sector_min(r, kFake, kPi, 3.0 * kDeg), 0.90, 1e-6);
}

TEST(ScanMath, SectorMinShiftedFront)
{
  std::vector<float> r(360, 2.0f);
  r[179] = 0.50f;    // shifted scan: beam 179 is straight ahead
  EXPECT_NEAR(sector_min(r, kShifted, 0.0, 0.5 * kDeg), 0.50, 1e-6);
}

TEST(ScanMath, RealC1Geometry)
{
  EXPECT_NEAR(beam_angle(kC1, 719), kPi, 1e-9);              // last beam = first direction
  EXPECT_EQ(angle_to_index(kC1, 720, 0.5 * kPi), 539u);      // left
  EXPECT_EQ(angle_to_index(kC1, 720, -0.5 * kPi), 180u);     // right
  EXPECT_EQ(angle_to_index(kC1, 720, kPi), 0u);
  std::vector<float> r(720, kInf);
  r[359] = 0.60f;    // -0.25 deg
  r[360] = 0.40f;    // +0.25 deg
  r[719] = 0.30f;    // straight back
  EXPECT_NEAR(sector_min(r, kC1, 0.0, 1.0 * kDeg), 0.40, 1e-6);
  EXPECT_NEAR(sector_min(r, kC1, kPi, 1.0 * kDeg), 0.30, 1e-6);
}

TEST(ScanMath, NearestIndex)
{
  auto r = empty_scan();
  EXPECT_FALSE(nearest_index(r, kFake).has_value());
  r[45] = 0.70f;
  r[200] = 0.30f;
  r[300] = 0.02f;    // closer, but not valid
  const auto i = nearest_index(r, kFake);
  ASSERT_TRUE(i.has_value());
  EXPECT_EQ(*i, 200u);
}

TEST(ScanMath, BeamGap)
{
  EXPECT_NEAR(beam_gap(5.0, 1.0 * kDeg), 0.0873, 1e-4);
  EXPECT_NEAR(beam_gap(1.0, 0.72 * kDeg), 0.0126, 1e-4);
}