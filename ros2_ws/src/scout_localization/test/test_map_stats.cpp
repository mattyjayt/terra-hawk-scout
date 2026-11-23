// Copyright 2026 Jaime Tellie
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "scout_localization/map_stats.hpp"

using scout_localization::count_cells;
using scout_localization::known_area;
using scout_localization::known_bounds;

TEST(MapStats, CountsEachKind)
{
  // -1 unknown, 0 and 19 free, 20 and 64 uncertain, 65 and 100 occupied
  const std::vector<int8_t> d{-1, 0, 19, 20, 64, 65, 100, -1};
  const auto c = count_cells(d);
  EXPECT_EQ(c.unknown, 2u);
  EXPECT_EQ(c.free, 2u);
  EXPECT_EQ(c.uncertain, 2u);
  EXPECT_EQ(c.occupied, 2u);
}

TEST(MapStats, EmptyMap)
{
  const auto c = count_cells({});
  EXPECT_EQ(c.free + c.occupied + c.unknown + c.uncertain, 0u);
  EXPECT_FALSE(known_bounds({}, 0, 0).has_value());
}

TEST(MapStats, KnownAreaUsesResolutionSquared)
{
  scout_localization::CellCounts c;
  c.free = 300;
  c.occupied = 100;
  c.unknown = 5000;               // unknown cells never count
  EXPECT_NEAR(known_area(c, 0.05), 1.0, 1e-9);   // 400 cells * 0.0025 m2
  EXPECT_NEAR(known_area(c, 0.10), 4.0, 1e-9);
}

TEST(MapStats, BoundsRowMajor)
{
  // 4 wide x 3 high, row-major: index = row * width + col.
  // Known cells at (col 1, row 0) and (col 2, row 2).
  std::vector<int8_t> d(12, -1);
  d[0 * 4 + 1] = 0;
  d[2 * 4 + 2] = 100;
  const auto b = known_bounds(d, 4, 3);
  ASSERT_TRUE(b.has_value());
  EXPECT_EQ(b->min_col, 1u);
  EXPECT_EQ(b->max_col, 2u);
  EXPECT_EQ(b->min_row, 0u);
  EXPECT_EQ(b->max_row, 2u);
}

TEST(MapStats, AllUnknownHasNoBounds)
{
  const std::vector<int8_t> d(20, -1);
  EXPECT_FALSE(known_bounds(d, 5, 4).has_value());
}