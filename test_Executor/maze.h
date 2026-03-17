#pragma once

#include <cstdint>
#include <array>
#include <stack>
#include "direction.h"

// ── Maze dimensions ──────────────────────────────────────────────────────────
static constexpr int SIZE      = 16;
static constexpr int STACKSIZE = 256;   // was 80 – too small for 16x16
static constexpr int LARGEVAL  = 32000;

// ── Boolean convenience ───────────────────────────────────────────────────────
static constexpr int TRUE  = 1;
static constexpr int FALSE = 0;

// ── Start position (in array / row-col space):
//    row = SIZE-1 (bottom row), col = 0  →  MMS (x=0, y=0) ───────────────────
static constexpr int START_ROW = SIZE - 1;
static constexpr int START_COL = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Cell
// ─────────────────────────────────────────────────────────────────────────────
struct Cell {
    int16_t m_distance = 0;
    int16_t pos_x      = 0;   // row index (0 = top)
    int16_t pos_y      = 0;   // col index (0 = left)
    int16_t visited    = FALSE;

    Cell* left  = nullptr;   // WEST  neighbour (col - 1)
    Cell* right = nullptr;   // EAST  neighbour (col + 1)
    Cell* up    = nullptr;   // NORTH neighbour (row - 1)
    Cell* down  = nullptr;   // SOUTH neighbour (row + 1)
};

// ─────────────────────────────────────────────────────────────────────────────
// Maze
// ─────────────────────────────────────────────────────────────────────────────
struct Maze {
    Cell* map[SIZE][SIZE] = {};

    Maze();
    ~Maze();
};

// ─────────────────────────────────────────────────────────────────────────────
// Cell / floodfill API
// ─────────────────────────────────────────────────────────────────────────────
// Use std::stack<Cell*> for flood propagation (no fixed-size limit).
using CellStack = std::stack<Cell*>;

Cell*  new_Cell(int i, int j);

void   flood_fill(Cell* cell, CellStack& stk, bool reflood_flag);
void   set_wall(Cell* cell, Direction dir);
void   set_value(Cell* cell, int16_t value);
void   set_visited(Cell* cell);

int16_t get_smallest_neighbor(const Cell* cell);
Direction get_smallest_neighbor_dir(const Cell* cell, Direction preferred_dir);
bool    floodval_check(const Cell* cell);
void    update_floodval(Cell* cell);
void    push_open_neighbors(Cell* cell, CellStack& stk);