// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include "scout_localization/safety.hpp"

#include <optional>
#include <cmath>

namespace scout_localization
{

SafetyState classify(double front_min, std::optional<double> scan_age, const SafetyParams & p)
{
  if (!scan_age) {return SafetyState::kNoScan;}
  if (*scan_age > p.scan_timeout) {return SafetyState::kStale;}
  if (front_min < p.laser_to_front + p.stop_distance) {return SafetyState::kObstacle;}
  return SafetyState::kClear;
}

Velocity limit(const Velocity & in, SafetyState state)
{
  switch (state) {
    case SafetyState::kClear:
      return {in.linear_x, in.angular_z};

    case SafetyState::kNoScan:
      return {0.0, 0.0};

    case SafetyState::kObstacle:
      return {std::min(in.linear_x, 0.0), in.angular_z};

    case SafetyState::kStale:
      return {0.0, 0.0};
  }
  return {0.0, 0.0};
}

const char * to_string(SafetyState state)
{
  switch (state) {
    case SafetyState::kClear:
      return "clear";

    case SafetyState::kNoScan:
      return "no scan";

    case SafetyState::kObstacle:
      return "obstacle";

    case SafetyState::kStale:
      return "stale scan";
  }
  return "unknown";
}

}  // namespace scout_localization
