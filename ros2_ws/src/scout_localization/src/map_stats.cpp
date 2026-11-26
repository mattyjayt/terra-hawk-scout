// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include "scout_localization/map_stats.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

namespace scout_localization
{

CellCounts count_cells(const std::vector<int8_t> & data)
{
  CellCounts cc;
  for (const int8_t v : data) {
    if (v < 0) {++cc.unknown;} else if (v <= kFreeThreshold) {
      ++cc.free;
    } else if (v >= kOccupiedThreshold) {++cc.occupied;} else {++cc.uncertain;}
  }
  return cc;
}

double known_area(const CellCounts & c, double resolution)
{
  return static_cast<double>(c.free + c.occupied) * resolution * resolution;
}

std::optional<CellBox> known_bounds(
  const std::vector<int8_t> & data, std::size_t width,
  std::size_t height)
{
  std::optional<CellBox> b;
  for (std::size_t row = 0; row < height; row++) {
    for (std::size_t col = 0; col < width; col++) {
      if (data[row * width + col] == kUnknown) {continue;}
      if (!b) {b = CellBox{col, col, row, row}; continue;}
      b->min_col = std::min(b->min_col, col);
      b->max_col = std::max(b->max_col, col);
      b->min_row = std::min(b->min_row, row);
      b->max_row = std::max(b->max_row, row);
    }
  }
  return b;
}

}  // namespace scout_localization
