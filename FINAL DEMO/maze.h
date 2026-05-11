#pragma once

#include <Arduino.h>
#include <cstdint>
#include <queue>
#include "direction.h"

/* ================= MAZE SIZE ================= */

static constexpr int SIZE = 6;          // full virtual maze: 6x6
static constexpr int INNER_MIN = 1;     // effective maze: x/y = 1..4
static constexpr int INNER_MAX = 4;
static constexpr int LARGEVAL = 32000;

static constexpr int TRUE  = 1;
static constexpr int FALSE = 0;

/* ================= START ================= */

static constexpr int START_X = 1;
static constexpr int START_Y = 1;

/* ================= CELL ================= */

struct Cell {
  int16_t m_distance = LARGEVAL;

  int16_t x = 0;
  int16_t y = 0;

  int16_t visited = FALSE;
  bool is_goal = false;
  bool is_exit = false;

  Cell* north = nullptr;
  Cell* east  = nullptr;
  Cell* south = nullptr;
  Cell* west  = nullptr;
};

/* ================= MAZE ================= */

struct Maze {
  Cell* map[SIZE][SIZE] = {};

  Maze();
  ~Maze();

  Cell* cellAt(int x, int y);

  void closeWall(Cell* c, Direction dir);
  void openWall(Cell* c, Direction dir);

  bool hasOpenWall(Cell* c, Direction dir) const;
  Cell* neighbor(Cell* c, Direction dir) const;

  bool isInnerCell(int x, int y) const;
  bool isOuterCell(int x, int y) const;
  bool isInnerBoundaryFace(int x, int y, Direction dir) const;

  void resetDistances();
  void computeFloodFrom(Cell* goal);
};

Direction oppositeDir(Direction d);
Direction leftOf(Direction d);
Direction rightOf(Direction d);