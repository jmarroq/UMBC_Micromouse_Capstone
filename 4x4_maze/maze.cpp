// maze.cpp  —  6×6 virtual map / floodfill for 4×4 escape maze

#include "maze.h"
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// Internal state
// ─────────────────────────────────────────────────────────────────────────────
static bool s_goal_known = false;

bool mazeGoalKnown() { return s_goal_known; }

static bool isExitCell(int vrow, int vcol) {
    return (vrow == 0 || vrow == VSIZE - 1 ||
            vcol == 0 || vcol == VSIZE - 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// Cell constructor
//
// Exit ring:  m_distance = 0, is_exit = true.
// Inner cells: m_distance = Manhattan distance to nearest border = steps to
//              reach any exit cell.  This is the heuristic before the real
//              exit location is known.  It biases the left-wall-follower toward
//              the outside — exactly where the exit will be.
// ─────────────────────────────────────────────────────────────────────────────
Cell* new_Cell(int i, int j) {
    Cell* c    = new Cell();
    c->pos_x   = static_cast<int16_t>(i);
    c->pos_y   = static_cast<int16_t>(j);
    c->visited = FALSE;

    if (isExitCell(i, j)) {
        c->is_exit    = true;
        c->m_distance = 0;
    } else {
        int dist = std::min(std::min(i, VSIZE - 1 - i),
                            std::min(j, VSIZE - 1 - j));
        c->m_distance = static_cast<int16_t>(dist);
    }
    return c;
}

// ─────────────────────────────────────────────────────────────────────────────
// Maze constructor
//
// Neighbour wiring rules:
//   inner ↔ inner            : wired freely (no internal walls at start)
//   exit  → inner            : wired permanently (exit ring always points in)
//   inner → exit ring        : nullptr by default (outer wall closed)
//                              opened only by mazeOpenExit()
//   exit  ↔ exit             : never wired (exit cells don't connect to each other)
// ─────────────────────────────────────────────────────────────────────────────
Maze::Maze() {
    for (int r = 0; r < VSIZE; ++r)
        for (int c = 0; c < VSIZE; ++c)
            map[r][c] = new_Cell(r, c);

    for (int r = 0; r < VSIZE; ++r) {
        for (int c = 0; c < VSIZE; ++c) {
            bool ex = map[r][c]->is_exit;

            // NORTH (r-1)
            if (r > 0) {
                bool nb_ex = map[r-1][c]->is_exit;
                if (!ex && !nb_ex)        map[r][c]->up = map[r-1][c]; // inner↔inner
                else if (ex && !nb_ex)    map[r][c]->up = map[r-1][c]; // exit→inner
                else                      map[r][c]->up = nullptr;      // inner→exit: closed / exit↔exit: closed
            }

            // SOUTH (r+1)
            if (r < VSIZE - 1) {
                bool nb_ex = map[r+1][c]->is_exit;
                if (!ex && !nb_ex)        map[r][c]->down = map[r+1][c];
                else if (ex && !nb_ex)    map[r][c]->down = map[r+1][c];
                else                      map[r][c]->down = nullptr;
            }

            // WEST (c-1)
            if (c > 0) {
                bool nb_ex = map[r][c-1]->is_exit;
                if (!ex && !nb_ex)        map[r][c]->left = map[r][c-1];
                else if (ex && !nb_ex)    map[r][c]->left = map[r][c-1];
                else                      map[r][c]->left = nullptr;
            }

            // EAST (c+1)
            if (c < VSIZE - 1) {
                bool nb_ex = map[r][c+1]->is_exit;
                if (!ex && !nb_ex)        map[r][c]->right = map[r][c+1];
                else if (ex && !nb_ex)    map[r][c]->right = map[r][c+1];
                else                      map[r][c]->right = nullptr;
            }
        }
    }
}

Maze::~Maze() {
    for (int r = 0; r < VSIZE; ++r)
        for (int c = 0; c < VSIZE; ++c) {
            delete map[r][c];
            map[r][c] = nullptr;
        }
}

Cell* Maze::at(int vrow, int vcol) const {
    if (vrow < 0 || vrow >= VSIZE || vcol < 0 || vcol >= VSIZE) return nullptr;
    return map[vrow][vcol];
}

Cell* Maze::atReal(int real_row, int real_col) const {
    return at(real_row + 1, real_col + 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// mazeOpenExit
//
// Called when sensors detect an open outer wall on a real border cell.
// Wires the bidirectional pointer and triggers a full reflood.
// ─────────────────────────────────────────────────────────────────────────────
void mazeOpenExit(Maze& maze, int vrow, int vcol, Direction exit_dir) {
    Cell* border = maze.at(vrow, vcol);
    if (!border || border->is_exit) return;

    int er = vrow, ec = vcol;
    switch (exit_dir) {
        case NORTH: er--; break;
        case SOUTH: er++; break;
        case EAST:  ec++; break;
        case WEST:  ec--; break;
    }

    Cell* exitCell = maze.at(er, ec);
    if (!exitCell || !exitCell->is_exit) return;

    // Wire bidirectionally
    switch (exit_dir) {
        case NORTH: border->up    = exitCell; exitCell->down  = border; break;
        case SOUTH: border->down  = exitCell; exitCell->up    = border; break;
        case EAST:  border->right = exitCell; exitCell->left  = border; break;
        case WEST:  border->left  = exitCell; exitCell->right = border; break;
    }

    s_goal_known = true;
    mazeReflood(maze, false);
}

// ─────────────────────────────────────────────────────────────────────────────
// mazeReflood
//
// Full reflood of the virtual map.
//
// Forward (reflood_flag=false):
//   Resets all inner cells to LARGEVAL.  Seeds from inner border cells that
//   are directly connected to an open exit cell (distance=1).  Pushes their
//   open inner neighbours so flood_fill propagates 2, 3, … inward.
//
// Return (reflood_flag=true):
//   Resets all cells to LARGEVAL.  Seeds start cell at 0 and propagates.
// ─────────────────────────────────────────────────────────────────────────────
void mazeReflood(Maze& maze, bool reflood_flag) {
    CellStack stk;

    if (!reflood_flag) {
        // Reset all inner cells
        for (int r = 0; r < VSIZE; ++r)
            for (int c = 0; c < VSIZE; ++c)
                if (!maze.map[r][c]->is_exit)
                    maze.map[r][c]->m_distance = static_cast<int16_t>(LARGEVAL);

        // Seed: inner cells adjacent to an open exit get distance=1,
        // then we push their inner neighbours (not the seed cells themselves,
        // because floodval_check would pass and skip propagation).
        for (int r = 0; r < VSIZE; ++r) {
            for (int c = 0; c < VSIZE; ++c) {
                Cell* cell = maze.map[r][c];
                if (cell->is_exit) continue;

                bool adj = false;
                if (cell->up    && cell->up->is_exit)    adj = true;
                if (cell->down  && cell->down->is_exit)  adj = true;
                if (cell->left  && cell->left->is_exit)  adj = true;
                if (cell->right && cell->right->is_exit) adj = true;

                if (adj) {
                    cell->m_distance = 1;
                    push_open_neighbors(cell, stk);
                }
            }
        }
    } else {
        // Reset everything
        for (int r = 0; r < VSIZE; ++r)
            for (int c = 0; c < VSIZE; ++c)
                maze.map[r][c]->m_distance = static_cast<int16_t>(LARGEVAL);

        Cell* start = maze.at(START_VROW, START_VCOL);
        if (start) {
            start->m_distance = 0;
            push_open_neighbors(start, stk);
        }
    }

    while (!stk.empty()) {
        Cell* n = stk.top(); stk.pop();
        if (n) flood_fill(n, stk, reflood_flag);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Wall mutation
// ─────────────────────────────────────────────────────────────────────────────
void set_wall(Cell* c, Direction dir) {
    switch (dir) {
        case NORTH: c->up    = nullptr; break;
        case SOUTH: c->down  = nullptr; break;
        case EAST:  c->right = nullptr; break;
        case WEST:  c->left  = nullptr; break;
    }
}

void set_wall_bidir(Maze& maze, int vrow, int vcol, Direction dir) {
    Cell* a = maze.at(vrow, vcol);
    if (!a) return;

    int nr = vrow, nc = vcol;
    Direction opp = NORTH;
    switch (dir) {
        case NORTH: nr--; opp = SOUTH; break;
        case SOUTH: nr++; opp = NORTH; break;
        case EAST:  nc++; opp = WEST;  break;
        case WEST:  nc--; opp = EAST;  break;
    }

    set_wall(a, dir);

    Cell* b = maze.at(nr, nc);
    // Never close an exit cell's pointer — only close real inner-cell pointers
    if (b && !b->is_exit) {
        set_wall(b, opp);
    }
}

void set_value  (Cell* c, int16_t v) { c->m_distance = v; }
void set_visited(Cell* c)            { c->visited = TRUE; }

// ─────────────────────────────────────────────────────────────────────────────
// Floodfill primitives
// ─────────────────────────────────────────────────────────────────────────────
int16_t get_smallest_neighbor(const Cell* c) {
    int16_t best = static_cast<int16_t>(LARGEVAL);

    if (c->left  && c->left->right  ) best = std::min(best, c->left ->m_distance);
    if (c->right && c->right->left  ) best = std::min(best, c->right->m_distance);
    if (c->up    && c->up->down     ) best = std::min(best, c->up   ->m_distance);
    if (c->down  && c->down->up     ) best = std::min(best, c->down ->m_distance);

    return best;
}

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

// flood_fill:
//   forward (reflood_flag=false): protect exit cells (fixed at 0, never update)
//   return  (reflood_flag=true) : protect start cell
void flood_fill(Cell* c, CellStack& stk, bool reflood_flag) {
    if (!reflood_flag) {
        if (c->is_exit) return;
    } else {
        if (c->pos_x == START_ROW && c->pos_y == START_COL) return;
    }

    if (!floodval_check(c)) {
        update_floodval(c);
        push_open_neighbors(c, stk);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Direction chooser — three-pass priority:
//   1. Unvisited neighbours at best distance (explore new territory)
//   2. Current heading at best distance      (avoid unnecessary turns)
//   3. Any open neighbour at best distance
// ─────────────────────────────────────────────────────────────────────────────
Direction get_smallest_neighbor_dir(const Cell* c, Direction preferred_dir) {
    int16_t best = get_smallest_neighbor(c);

    auto open_up    = [&]{ return c->up    && c->up->down;    };
    auto open_down  = [&]{ return c->down  && c->down->up;    };
    auto open_left  = [&]{ return c->left  && c->left->right; };
    auto open_right = [&]{ return c->right && c->right->left; };

    // Pass 1: unvisited at best distance
    if (open_up()    && c->up->m_distance    == best && !c->up->visited)    return NORTH;
    if (open_right() && c->right->m_distance == best && !c->right->visited) return EAST;
    if (open_down()  && c->down->m_distance  == best && !c->down->visited)  return SOUTH;
    if (open_left()  && c->left->m_distance  == best && !c->left->visited)  return WEST;

    // Pass 2: continue straight
    switch (preferred_dir) {
        case NORTH: if (open_up()    && c->up->m_distance    == best) return NORTH; break;
        case EAST:  if (open_right() && c->right->m_distance == best) return EAST;  break;
        case SOUTH: if (open_down()  && c->down->m_distance  == best) return SOUTH; break;
        case WEST:  if (open_left()  && c->left->m_distance  == best) return WEST;  break;
    }

    // Pass 3: any reachable best-distance cell
    if (open_up()    && c->up->m_distance    == best) return NORTH;
    if (open_right() && c->right->m_distance == best) return EAST;
    if (open_down()  && c->down->m_distance  == best) return SOUTH;
    if (open_left()  && c->left->m_distance  == best) return WEST;

    return preferred_dir;
}
