// Maze.cpp  –  Cell / Maze / floodfill implementation (C++ port)

#include "Maze.h"
#include <cstdlib>
#include <cstdio>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// Cell constructor
// ─────────────────────────────────────────────────────────────────────────────
Cell* new_Cell(int i, int j) {
    Cell* c = new Cell();
    c->pos_x   = static_cast<int16_t>(i);   // row
    c->pos_y   = static_cast<int16_t>(j);   // col
    c->visited = FALSE;

    // Manhattan distance to the 2×2 goal centre (rows SIZE/2-1 .. SIZE/2,
    // cols SIZE/2-1 .. SIZE/2).
    const int half = SIZE / 2;
    int dr = (i < half) ? (half - 1 - i) : (i - half);
    int dc = (j < half) ? (half - 1 - j) : (j - half);
    c->m_distance = static_cast<int16_t>(dr + dc);

    return c;
}

// ─────────────────────────────────────────────────────────────────────────────
// Maze constructor / destructor
// ─────────────────────────────────────────────────────────────────────────────
Maze::Maze() {
    // Allocate all cells
    for (int i = 0; i < SIZE; ++i)
        for (int j = 0; j < SIZE; ++j)
            map[i][j] = new_Cell(i, j);

    // Wire up neighbour pointers
    // UP   = row - 1  (visually north on screen)
    // DOWN = row + 1  (visually south on screen)
    // LEFT = col - 1  (west)
    // RIGHT= col + 1  (east)
    for (int i = 0; i < SIZE; ++i) {
        for (int j = 0; j < SIZE; ++j) {
            map[i][j]->up    = (i == 0)        ? nullptr : map[i-1][j];
            map[i][j]->down  = (i == SIZE-1)   ? nullptr : map[i+1][j];
            map[i][j]->left  = (j == 0)        ? nullptr : map[i][j-1];
            map[i][j]->right = (j == SIZE-1)   ? nullptr : map[i][j+1];
        }
    }
}

Maze::~Maze() {
    for (int i = 0; i < SIZE; ++i)
        for (int j = 0; j < SIZE; ++j) {
            delete map[i][j];
            map[i][j] = nullptr;
        }
}

// ─────────────────────────────────────────────────────────────────────────────
// Wall setting
//
// A wall between two cells is represented by nulling the pointer on BOTH sides.
//
// IMPORTANT boundary semantics:
//   set_wall(NORTH) on a cell at row 0  → outer maze wall, already has up==nullptr
//   set_wall(SOUTH) on a cell at row SIZE-1 → outer wall, already down==nullptr
//   We simply null the pointer regardless; outer-wall pointers are already null.
// ─────────────────────────────────────────────────────────────────────────────
void set_wall(Cell* c, Direction dir) {
    switch (dir) {
        case NORTH: c->up    = nullptr; break;
        case SOUTH: c->down  = nullptr; break;
        case EAST:  c->right = nullptr; break;
        case WEST:  c->left  = nullptr; break;
    }
}

void set_value  (Cell* c, int16_t v) { c->m_distance = v; }
void set_visited(Cell* c)            { c->visited = TRUE; }

// ─────────────────────────────────────────────────────────────────────────────
// Floodfill helpers
// ─────────────────────────────────────────────────────────────────────────────

// Returns the smallest m_distance among open (non-walled) neighbours.
// "Open" = the pointer from THIS cell to the neighbour is non-null, AND the
//          back-pointer from that neighbour back to this cell is also non-null.
// (Both sides are nulled when a wall is added, so checking one side suffices,
//  but checking both is a useful consistency guard.)
int16_t get_smallest_neighbor(const Cell* c) {
    int16_t best = static_cast<int16_t>(LARGEVAL);

    if (c->left  && c->left->right  ) best = std::min(best, c->left ->m_distance);
    if (c->right && c->right->left  ) best = std::min(best, c->right->m_distance);
    if (c->up    && c->up->down     ) best = std::min(best, c->up   ->m_distance);
    if (c->down  && c->down->up     ) best = std::min(best, c->down ->m_distance);

    return best;
}

// Returns true when this cell's flood value is already consistent
// (i.e. == 1 + min open neighbour).
bool floodval_check(const Cell* c) {
    return (get_smallest_neighbor(c) + 1) == c->m_distance;
}

void update_floodval(Cell* c) {
    c->m_distance = static_cast<int16_t>(get_smallest_neighbor(c) + 1);
}

void push_open_neighbors(Cell* c, CellStack& stk) {
    if (c->left  && c->left->right  ) stk.push(c->left);
    if (c->right && c->right->left  ) stk.push(c->right);
    if (c->up    && c->up->down     ) stk.push(c->up);
    if (c->down  && c->down->up     ) stk.push(c->down);
}

void flood_fill(Cell* c, CellStack& stk, bool reflood_flag) {
    // Protect goal cells (forward run): rows SIZE/2-1 and SIZE/2,
    // cols SIZE/2-1 and SIZE/2.
    const int half = SIZE / 2;
    if (!reflood_flag) {
        if ((c->pos_x == half || c->pos_x == half - 1) &&
            (c->pos_y == half || c->pos_y == half - 1))
            return;
    }

    // Protect start cell (return run)
    if (reflood_flag) {
        if (c->pos_x == START_ROW && c->pos_y == START_COL)
            return;
    }

    if (!floodval_check(c)) {
        update_floodval(c);
        push_open_neighbors(c, stk);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Direction chooser
//
// BUG FIX: the original code checked pointer nullity for reachability but
// forgot to use the same "open path" double-pointer check that
// get_smallest_neighbor uses, potentially choosing walled directions.
// ─────────────────────────────────────────────────────────────────────────────
Direction get_smallest_neighbor_dir(const Cell* c, Direction preferred_dir) {
    int16_t best = get_smallest_neighbor(c);

    // Helper lambdas for readability
    auto open_up    = [&]{ return c->up    && c->up->down;    };
    auto open_down  = [&]{ return c->down  && c->down->up;    };
    auto open_left  = [&]{ return c->left  && c->left->right; };
    auto open_right = [&]{ return c->right && c->right->left; };

    // ── Pass 1: prefer unvisited cells ───────────────────────────────────────
    if (open_up()    && c->up->m_distance    == best && !c->up->visited)    return NORTH;
    if (open_right() && c->right->m_distance == best && !c->right->visited) return EAST;
    if (open_down()  && c->down->m_distance  == best && !c->down->visited)  return SOUTH;
    if (open_left()  && c->left->m_distance  == best && !c->left->visited)  return WEST;

    // ── Pass 2: prefer the current heading (no unnecessary turns) ────────────
    switch (preferred_dir) {
        case NORTH: if (open_up()    && c->up->m_distance    == best) return NORTH; break;
        case EAST:  if (open_right() && c->right->m_distance == best) return EAST;  break;
        case SOUTH: if (open_down()  && c->down->m_distance  == best) return SOUTH; break;
        case WEST:  if (open_left()  && c->left->m_distance  == best) return WEST;  break;
    }

    // ── Pass 3: any reachable best-distance cell ──────────────────────────────
    if (open_up()    && c->up->m_distance    == best) return NORTH;
    if (open_right() && c->right->m_distance == best) return EAST;
    if (open_down()  && c->down->m_distance  == best) return SOUTH;
    if (open_left()  && c->left->m_distance  == best) return WEST;

    // Fallback (should not happen in a well-formed maze)
    return preferred_dir;
}