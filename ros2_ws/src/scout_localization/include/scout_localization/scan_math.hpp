// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#ifndef SCOUT_LOCALIZATION__SCAN_MATH_HPP_
#define SCOUT_LOCALIZATION__SCAN_MATH_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace scout_localization
{

constexpr double kPi = 3.14159265358979323846;

// The four numbers of a sensor_msgs/LaserScan the maths needs. No ROS types here,
// so the tests need no ROS.
struct ScanGeometry
{
  double angle_min;        // rad, angle of beam 0, counter-clockwise from the laser's +x
  double angle_increment;  // rad between two neighbouring beams
  double range_min;        // m; a smaller value is not a distance
  double range_max;        // m; a larger value is not a distance
};

// Wrap an angle in rad to [-pi, pi).
double wrap_angle(double a);

// Angle of beam i in rad, as the message defines it (not wrapped).
double beam_angle(const ScanGeometry & g, std::size_t i);

// Index of the beam nearest to `angle` (any value; it is wrapped) in a scan of n beams.
std::size_t angle_to_index(const ScanGeometry & g, std::size_t n, double angle);

// True if r is a real echo: finite (not inf, not NaN) and inside [range_min, range_max].
bool is_valid(float r, const ScanGeometry & g);

// Number of valid beams in the scan.
std::size_t count_valid(const std::vector<float> & ranges, const ScanGeometry & g);

// Smallest valid range among the beams whose angle lies within +-half_width of
// centre (the difference is wrapped, so a sector around pi works).
// +infinity if the sector has no valid beam.
double sector_min(
  const std::vector<float> & ranges, const ScanGeometry & g,
  double centre, double half_width);

// Index of the nearest valid beam, or std::nullopt if no beam is valid.
std::optional<std::size_t> nearest_index(
  const std::vector<float> & ranges, const ScanGeometry & g);

// Gap between two neighbouring beams at `distance` (arc length), m.
double beam_gap(double distance, double angle_increment);

}  // namespace scout_localization

#endif  // SCOUT_LOCALIZATION__SCAN_MATH_HPP_
