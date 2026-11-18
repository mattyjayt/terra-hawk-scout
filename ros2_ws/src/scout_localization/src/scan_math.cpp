// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include "scout_localization/scan_math.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace scout_localization
{

double wrap_angle(double a)
{
    return a - (2 * (kPi * std::floor((a + kPi) / (2 * kPi))));
}

double beam_angle(const ScanGeometry & g, std::size_t i)
{
    return g.angle_min + static_cast<double>(i) * g.angle_increment;
}

std::size_t angle_to_index(const ScanGeometry & g, std::size_t n, double angle)
{
  double off = std::fmod(angle - g.angle_min, 2 * kPi);
  if (off < 0.0) { off += 2 * kPi; }
  const auto k = std::lround(off / g.angle_increment);
  return static_cast<std::size_t>(k) % n;
}

bool is_valid(float r, const ScanGeometry & g)
{
    return (std::isfinite(r) && (g.range_min <= r && r <= g.range_max));
}

std::size_t count_valid(const std::vector<float> & ranges, const ScanGeometry & g)
{
  std::size_t counter{0};
  for (int i = 0; i < ranges.size(); i++) {
    if (is_valid(ranges[i], g)) { counter++; }
  }
  return counter;
}

double sector_min(
  const std::vector<float> & ranges, const ScanGeometry & g,
  double centre, double half_width)
{
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < ranges.size(); i++){
    if (!is_valid(ranges[i], g)) continue;
    if (std::abs(wrap_angle(beam_angle(g, i) - centre)) > half_width) continue;

    best = std::min(best, static_cast<double>(ranges[i]));
  }
  return best;
}

std::optional<std::size_t> nearest_index(
  const std::vector<float> & ranges, const ScanGeometry & g)
{
  std::optional<std::size_t> best;
  for (int i = 0; i < ranges.size(); i++){
    if (!is_valid(ranges[i], g)) continue;

    if (!best || ranges[i] < ranges[*best]) { best = i; }
  }
  return best;
}

double beam_gap(double distance, double angle_increment)
{
  return distance * angle_increment;
}

}  // namespace scout_localization