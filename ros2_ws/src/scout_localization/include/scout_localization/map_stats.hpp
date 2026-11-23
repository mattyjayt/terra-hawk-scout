// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#ifndef SCOUT_LOCALIZATION__MAP_STATS_HPP_
#define SCOUT_LOCALIZATION__MAP_STATS_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace scout_localization
{

// Occupancy grid cell values (nav_msgs/OccupancyGrid): -1 unknown, 0 free ... 100 occupied.
constexpr int8_t kUnknown = -1;
constexpr int8_t kOccupiedThreshold = 65;   // >= 65 = occupied (map_saver's occupied_thresh 0.65)
constexpr int8_t kFreeThreshold = 19;       // <= 19 = free (map_saver's free_thresh 0.196)

struct CellCounts
{
  std::size_t free = 0;
  std::size_t occupied = 0;
  std::size_t unknown = 0;
  std::size_t uncertain = 0;   // 20 ... 64: seen, but not sure
};

// Count the four kinds of cells.
CellCounts count_cells(const std::vector<int8_t> & data);

// Area of the known cells (free + occupied) in m2. resolution: m per cell.
double known_area(const CellCounts & c, double resolution);

// Bounding box of all known cells, in cell indices (inclusive). nullopt if none is known.
struct CellBox
{
  std::size_t min_col, max_col, min_row, max_row;
};
std::optional<CellBox> known_bounds(
  const std::vector<int8_t> & data, std::size_t width, std::size_t height);

}  // namespace scout_localization

#endif  // SCOUT_LOCALIZATION__MAP_STATS_HPP_