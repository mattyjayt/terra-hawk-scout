// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "scout_base_cpp/conversions.hpp"
#include "scout_base_cpp/kinematics.hpp"
#include "scout_base_cpp/protocol.hpp"

using scout_base_cpp::Ack;
using scout_base_cpp::DriveParams;
using scout_base_cpp::ImuSample;
using scout_base_cpp::Line;
using scout_base_cpp::WatchdogStop;
using scout_base_cpp::parse_line;

// The same robot numbers as the Python tests (Lab 3).
const DriveParams kDrive{0.15, 0.5, 60};   // track_width, max_wheel_speed, pwm_min

// ---------- U1: a valid I line (worked example 1) ----------
TEST(Protocol, ValidImuLine)
{
  const auto r = parse_line("I 123 0.1 0.2 0.3 0.4 0.5 0.6 45.0 90 12.5");
  ASSERT_TRUE(r.has_value());                              // not std::nullopt
  ASSERT_TRUE(std::holds_alternative<ImuSample>(*r));      // and it is an ImuSample
  const auto & s = std::get<ImuSample>(*r);
  EXPECT_EQ(s.time_ms, 123);
  EXPECT_DOUBLE_EQ(s.accel_x, 0.1);
  EXPECT_DOUBLE_EQ(s.accel_z, 0.3);
  EXPECT_DOUBLE_EQ(s.gyro_z, 0.6);
  EXPECT_DOUBLE_EQ(s.yaw_deg, 45.0);
  EXPECT_EQ(s.servo_deg, 90);
  EXPECT_DOUBLE_EQ(s.batt_voltage, 12.5);
}

// ---------- U2: bad lines never throw and give std::nullopt (worked example 2) ----------
class BadLine : public ::testing::TestWithParam<std::string> {};

TEST_P(BadLine, ReturnsNullopt)
{
  std::optional<Line> r;
  EXPECT_NO_THROW(r = parse_line(GetParam()));
  EXPECT_FALSE(r.has_value()) << "line: '" << GetParam() << "'";
}

INSTANTIATE_TEST_SUITE_P(
  U2, BadLine, ::testing::Values(
    std::string(""),
    std::string("I"),
    std::string("\r\n"),
    std::string("I 1 2 3"),
    std::string("I 123 0.1 0.2 0.3 0.4 0.5 0.6 45.0 90"),
    std::string("I 123 0.1 0.2 0.3 0.4 0.5 0.6 45.0 90 12.5 7"),
    std::string("I 123 abc 0.2 0.3 0.4 0.5 0.6 45.0 90 12.5"),
    std::string("I 123 0.1 0.2 0.3 0.4 0.5 0.6 45.0 90.5 12.5"),
    std::string("I 123 0.1x 0.2 0.3 0.4 0.5 0.6 45.0 90 12.5"),
    std::string("M 1 2"),
    std::string(200, 'x')));

// ---------- yours: the cases in the table below ----------