// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#ifndef SCOUT_LOCALIZATION__SAFETY_HPP_
#define SCOUT_LOCALIZATION__SAFETY_HPP_

#include <optional>

namespace scout_localization
{

enum class SafetyState
{
  kNoScan,    // no scan received yet          -> stop
  kStale,     // newest scan older than timeout -> stop
  kObstacle,  // something too close in front   -> no forward motion
  kClear      // pass the command through
};

struct SafetyParams
{
  double stop_distance;   // m, REQ-SCOUT-009: 0.15, measured from the front edge
  double laser_to_front;  // m, laser spin axis -> front edge of the robot (Task 9.1)
  double scan_timeout;    // s, scan older than this = stale
};

struct Velocity
{
  double linear_x;   // m/s, + = forward
  double angular_z;  // rad/s, + = left
};

// Decide the state. front_min: sector minimum in the laser frame (m, may be +inf).
// scan_age: seconds since the newest scan, std::nullopt if there was none yet.
// Order: no scan, then stale, then obstacle (front_min < laser_to_front + stop_distance).
SafetyState classify(double front_min, std::optional<double> scan_age, const SafetyParams & p);

// Apply the state to a command: kClear passes it; kObstacle keeps reverse and
// turning but clamps forward speed to 0; kNoScan and kStale stop everything.
Velocity limit(const Velocity & in, SafetyState state);

// "no scan", "stale scan", "obstacle", "clear" (for log lines).
const char * to_string(SafetyState state);

}  // namespace scout_localization

#endif  // SCOUT_LOCALIZATION__SAFETY_HPP_
