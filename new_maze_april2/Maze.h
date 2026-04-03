#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// Maze.h  —  6×6 virtual map for 4×4 escape maze
//
// Layout concept
// ──────────────
//
//   Virtual grid (6×6, rows 0‥5, cols 0‥5):
//
//     col  0    1    2    3    4    5
//  row 0 [EX] [EX] [EX] [EX] [EX] [EX]   ← virtual exit ring (north)
//      1 [EX] [R0] [R1] [R2] [R3] [EX]
//      2 [EX] [R4] [R5] [R6] [R7] [EX]   inner 4×4 = real maze cells
//      3 [EX] [R8] [R9] [RA] [RB] [EX]
//      4 [EX] [RC] [RD] [RE] [RF] [EX]
//      5 [EX] [EX] [EX] [EX] [EX] [EX]   ← virtual exit ring (south)
//             ↑                   ↑
//          west exits          east exits
//
//   A real maze cell at (real_row, real_col) lives at virtual (real_row+1, real_col+1).
//   The 20 outer cells are "virtual exit cells" — the mouse physically steps
//   into one of these to win.
//
// How the exit is represented
// ───────────────────────────
//   All outer-ring cells start with m_distance = 0 and are "always open" to
//   their inner neighbour.  However, the real maze's outer walls are set by
//   default — the border cell of the 4×4 does NOT have its pointer wired to
//   an exit cell initially.
//
//   When the left-wall-follower finds an opening in the outer wall of a real
//   border cell, call mazeOpenExit(vrow, vcol, dir).  This wires the pointer
//   bidirectionally so the floodfill can propagate through the opening into
//   the exit cell (distance 0).  A full reflood then gives every inner cell
//   its correct distance to the exit.
//
// Coordinate systems
// ──────────────────
//   "real"    — (real_row, real_col)  0‥3,  as the robot sees the 4×4 maze
//   "virtual" — (vrow, vcol)          0‥5,  internal map array indices
//   "MMS"     — (x, y) with y=0 at bottom, used by robot_state / executor
//
//   Conversion:
//     vrow = real_row + 1,   vcol = real_col + 1
//     real_row = vrow - 1,   real_col = vcol - 1
//
//   MMS ↔ real:
//     real_row = (INNER-1) - y,   real_col = x
// ─────────────────────────────────────────────────────────────────────────────

#include <cstdint>
#include <stack>

// ── Dimensions ────────────────────────────────────────────────────────────────
static constexpr int INNER = 4;          // real maze is 4×4
static constexpr int VSIZE = INNER + 2;  // virtual map is 6×6
static constexpr int LARGEVAL = 32000;

// ── Boolean convenience ───────────────────────────────────────────────────────
static constexpr int TRUE  = 1;
static constexpr int FALSE = 0;

// ── Directions ────────────────────────────────────────────────────────────────
enum Direction : int { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// ── Start position in virtual-map space ──────────────────────────────────────
// Robot starts at real (row=3, col=0)  →  virtual (vrow=4, vcol=1)
// This is MMS (x=0, y=0) = bottom-left of the real maze.
static constexpr int START_VROW = INNER;       // = 4
static constexpr int START_VCOL = 1;

// ─────────────────────────────────────────────────────────────────────────────
// Cell
// ─────────────────────────────────────────────────────────────────────────────
struct Cell {
    int16_t m_distance = 0;
    int16_t pos_x      = 0;    // virtual row (0 = top of 6×6)
    int16_t pos_y      = 0;    // virtual col (0 = left of 6×6)
    int16_t visited    = FALSE;
    bool    is_exit    = false; // true for the 20 outer-ring cells

    Cell* left  = nullptr;    // WEST  neighbour
    Cell* right = nullptr;    // EAST  neighbour
    Cell* up    = nullptr;    // NORTH neighbour
    Cell* down  = nullptr;    // SOUTH neighbour
};

// ─────────────────────────────────────────────────────────────────────────────
// Maze  (owns the 6×6 grid)
// ─────────────────────────────────────────────────────────────────────────────
struct Maze {
    Cell* map[VSIZE][VSIZE] = {};

    Maze();
    ~Maze();

    // Convenience: access by virtual coordinates (bounds-checked, returns nullptr)
    Cell* at(int vrow, int vcol) const;

    // Convenience: access by real-maze coordinates
    Cell* atReal(int real_row, int real_col) const;
};

// ─────────────────────────────────────────────────────────────────────────────
// CellStack
// ─────────────────────────────────────────────────────────────────────────────
using CellStack = std::stack<Cell*>;

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

// ── Exit / goal ───────────────────────────────────────────────────────────────

// Returns true once at least one exit opening has been wired.
bool mazeGoalKnown();

// Call when the left-wall-follower senses an open outer wall on a real border
// cell.  vrow, vcol are the virtual coordinates of that border cell; exit_dir
// is which side faces outside.
//
// This wires the pointer between the real border cell and its virtual exit
// neighbour, then refloods the entire map so every cell now has its correct
// distance to the nearest reachable exit cell.
void mazeOpenExit(Maze& maze, int vrow, int vcol, Direction exit_dir);

// ── Wall mutation ─────────────────────────────────────────────────────────────

// Set a wall on one side of a cell (nulls that pointer).
// Caller must also call set_wall on the neighbour for bidirectionality.
void set_wall(Cell* c, Direction dir);

// Set wall bidirectionally between two adjacent cells in the virtual map.
// Safe to call with border cells (the outer-ring side is left alone).
void set_wall_bidir(Maze& maze, int vrow, int vcol, Direction dir);

// ── Cell state ────────────────────────────────────────────────────────────────
void set_value  (Cell* c, int16_t v);
void set_visited(Cell* c);

// ── Floodfill primitives ──────────────────────────────────────────────────────
int16_t   get_smallest_neighbor    (const Cell* c);
bool      floodval_check           (const Cell* c);
void      update_floodval          (Cell* c);
void      push_open_neighbors      (Cell* c, CellStack& stk);

// reflood_flag=false → flood toward exit (exploration / speed run)
// reflood_flag=true  → flood toward start cell   (return journey)
void      flood_fill               (Cell* c, CellStack& stk, bool reflood_flag);

// Full reflood of entire map from current wall state.
// Call after mazeOpenExit() or after bulk wall updates.
void      mazeReflood              (Maze& maze, bool reflood_flag);

// ── Direction chooser ─────────────────────────────────────────────────────────
// Returns the best next direction from c toward the floodfill goal.
// preferred_dir used as tiebreaker (prefer straight-ahead).
Direction get_smallest_neighbor_dir(const Cell* c, Direction preferred_dir);

// ── Coordinate helpers ────────────────────────────────────────────────────────
// Convert MMS robot pose (x,y) to virtual map coordinates.
// MMS: x=col, y=row-from-bottom.   Real: row=(INNER-1)-y, col=x.
// Virtual: vrow=real_row+1, vcol=real_col+1.
inline int mmsToVrow(int x, int y) { return (INNER - 1 - y) + 1; }
inline int mmsToVcol(int x, int y) { return x + 1; }
