#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// maze.h  —  6×6 virtual map for 4×4 escape maze
//
// Virtual grid layout (6×6):
//
//     col  0    1    2    3    4    5
//  row 0 [EX] [EX] [EX] [EX] [EX] [EX]   ← north exit ring
//      1 [EX] [r0] [r1] [r2] [r3] [EX]
//      2 [EX] [r4] [r5] [r6] [r7] [EX]   real 4×4 maze cells
//      3 [EX] [r8] [r9] [rA] [rB] [EX]
//      4 [EX] [rC] [rD] [rE] [rF] [EX]
//      5 [EX] [EX] [EX] [EX] [EX] [EX]   ← south exit ring
//             ↑                   ↑
//          west exits          east exits
//
// Coordinate mapping:
//   MMS pose (x, y): x = col from left, y = row from bottom
//   real cell:       real_row = (INNER-1) - y,  real_col = x
//   virtual:         vrow = real_row + 1,         vcol = real_col + 1
//
// How exit discovery works:
//   All real border cells start with their outer pointer = nullptr (wall closed).
//   When sensors read no wall on an outer border, call mazeOpenExit() which
//   wires that pointer to the exit ring cell and refloods the whole map.
//   After that, every inner cell's m_distance = steps to the nearest exit.
// ─────────────────────────────────────────────────────────────────────────────

#include <cstdint>
#include <stack>
#include "direction.h"

// ── Dimensions ────────────────────────────────────────────────────────────────
static constexpr int INNER    = 4;
static constexpr int VSIZE    = INNER + 2;   // 6×6 virtual grid
static constexpr int LARGEVAL = 32000;

// SIZE alias keeps any other code that references SIZE compiling unchanged
static constexpr int SIZE = VSIZE;

// ── Boolean convenience ───────────────────────────────────────────────────────
static constexpr int TRUE  = 1;
static constexpr int FALSE = 0;

// ── Start position in virtual-map space ──────────────────────────────────────
// MMS (x=0, y=0) → real (row=3, col=0) → virtual (vrow=4, vcol=1)
static constexpr int START_VROW = INNER;     // = 4
static constexpr int START_VCOL = 1;

// Legacy aliases so flood_fill's start-cell guard still compiles
static constexpr int START_ROW = START_VROW;
static constexpr int START_COL = START_VCOL;

// ─────────────────────────────────────────────────────────────────────────────
// Cell
// ─────────────────────────────────────────────────────────────────────────────
struct Cell {
    int16_t m_distance = 0;
    int16_t pos_x      = 0;    // virtual row index
    int16_t pos_y      = 0;    // virtual col index
    int16_t visited    = FALSE;
    bool    is_exit    = false; // true for the 20 outer-ring cells

    Cell* left  = nullptr;    // WEST  neighbour
    Cell* right = nullptr;    // EAST  neighbour
    Cell* up    = nullptr;    // NORTH neighbour
    Cell* down  = nullptr;    // SOUTH neighbour
};

// ─────────────────────────────────────────────────────────────────────────────
// Maze  —  owns the 6×6 grid
// ─────────────────────────────────────────────────────────────────────────────
struct Maze {
    Cell* map[VSIZE][VSIZE] = {};

    Maze();
    ~Maze();

    // Bounds-checked accessor by virtual coordinates (returns nullptr if OOB)
    Cell* at(int vrow, int vcol) const;

    // Accessor by real-maze (row, col)  0..INNER-1
    Cell* atReal(int real_row, int real_col) const;
};

// ─────────────────────────────────────────────────────────────────────────────
// CellStack — unbounded, used for floodfill propagation
// ─────────────────────────────────────────────────────────────────────────────
using CellStack = std::stack<Cell*>;

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

Cell*  new_Cell(int i, int j);   // called internally by Maze constructor

// ── Goal / exit ───────────────────────────────────────────────────────────────

bool mazeGoalKnown();

// Call when an open outer wall is detected on a real border cell.
//   vrow, vcol — virtual coords of that border cell
//   exit_dir   — direction that faces outside (e.g. NORTH if top wall is open)
// Wires pointer bidirectionally and triggers full reflood.
void mazeOpenExit(Maze& maze, int vrow, int vcol, Direction exit_dir);

// Full reflood of the entire virtual map.
//   reflood_flag = false → flood toward exit   (exploration / speed run)
//   reflood_flag = true  → flood toward start  (return journey)
void mazeReflood(Maze& maze, bool reflood_flag);

// ── Wall mutation ─────────────────────────────────────────────────────────────
void set_wall      (Cell* c, Direction dir);
void set_wall_bidir(Maze& maze, int vrow, int vcol, Direction dir);

// ── Cell state ────────────────────────────────────────────────────────────────
void set_value  (Cell* c, int16_t v);
void set_visited(Cell* c);

// ── Floodfill primitives ──────────────────────────────────────────────────────
int16_t   get_smallest_neighbor    (const Cell* c);
bool      floodval_check           (const Cell* c);
void      update_floodval          (Cell* c);
void      push_open_neighbors      (Cell* c, CellStack& stk);
void      flood_fill               (Cell* c, CellStack& stk, bool reflood_flag);

Direction get_smallest_neighbor_dir(const Cell* c, Direction preferred_dir);

// ── Coordinate helpers ────────────────────────────────────────────────────────
inline int mmsToVrow(int x, int y) { return (INNER - 1 - y) + 1; }
inline int mmsToVcol(int x, int y) { return x + 1; }
