#pragma once
#include <cstdint>

// ── Single definition of Direction used by ALL files ─────────────────────────
// executor.h had its own copy with DIR_NORTH/DIR_EAST/... which conflicted.
// All files must include THIS header only. DIR_* aliases kept for compat.

enum Direction : uint8_t {
  NORTH = 0,
  EAST  = 1,
  SOUTH = 2,
  WEST  = 3
};

// Aliases so executor.cpp / test_Executor.ino compile unchanged
static constexpr Direction DIR_NORTH = NORTH;
static constexpr Direction DIR_EAST  = EAST;
static constexpr Direction DIR_SOUTH = SOUTH;
static constexpr Direction DIR_WEST  = WEST;
