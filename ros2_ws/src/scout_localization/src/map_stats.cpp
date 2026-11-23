// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include "scout_localization/map_stats.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace scout_localization
{

CellCounts count_cells(const std::vector<int8_t> & data) {(void)data; return {};}
double known_area(const CellCounts & c, double resolution) {(void)c; (void)resolution; return 0.0;}
std::optional<CellBox> known_bounds(
  const std::vector<int8_t> & data, std::size_t width, std::size_t height)
{
  (void)data; (void)width; (void)height;
  return std::nullopt;
}

}  // namespace scout_localization