// Maze.cpp  —  6×6 virtual map / floodfill for 4×4 escape maze
//
// See Maze.h for the full coordinate-system and layout description.

#include "Maze.h"
#include <algorithm>
#include <cstring>

// ─────────────────────────────────────────────────────────────────────────────
// Internal state
// ─────────────────────────────────────────────────────────────────────────────
static bool s_goal_known = false;   // set to true by mazeOpenExit()

bool mazeGoalKnown() { return s_goal_known; }

// ─────────────────────────────────────────────────────────────────────────────
// Helpers — is a virtual cell on the outer exit ring?
// ─────────────────────────────────────────────────────────────────────────────
static bool isExitCell(int vrow, int vcol) {
    return (vrow == 0 || vrow == VSIZE - 1 ||
            vcol == 0 || vcol == VSIZE - 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// Maze constructor
//
// Grid initialisation:
//   • Outer ring (20 cells): is_exit=true, m_distance=0, no outer-wall pointers.
//     They are wired to their single inner neighbour only.  Inner real-border
//     cells start with their outer pointer = nullptr (wall exists by default).
//     mazeOpenExit() wires them when the robot discovers an opening.
//
//   • Inner 4×4 cells: m_distance = Manhattan distance to nearest exit cell
//     (= min(vrow, VSIZE-1-vrow, vcol, VSIZE-1-vcol) ).
//     This is their distance to the virtual border, which equals the number of
//     steps to reach any exit cell.  It is a valid heuristic before the real
//     exit location is known.
//
//   • All interior walls start open (pointers wired).
//   • All real outer-border walls start closed (pointer = nullptr on the
//     border cell's outward side).
// ─────────────────────────────────────────────────────────────────────────────
Maze::Maze() {
    // ── Allocate all 6×6 cells ───────────────────────────────────────────────
    for (int r = 0; r < VSIZE; ++r) {
        for (int c = 0; c < VSIZE; ++c) {
            Cell* cell = new Cell();
            cell->pos_x   = static_cast<int16_t>(r);
            cell->pos_y   = static_cast<int16_t>(c);
            cell->visited = FALSE;

            if (isExitCell(r, c)) {
                // Exit ring: always distance 0, flagged
                cell->is_exit    = true;
                cell->m_distance = 0;
            } else {
                // Inner cell: heuristic distance = steps to nearest border
                // = min(r, VSIZE-1-r, c, VSIZE-1-c)
                // which equals min(vrow, VSIZE-1-vrow, vcol, VSIZE-1-vcol).
                int dist = std::min({r, VSIZE - 1 - r, c, VSIZE - 1 - c});
                cell->m_distance = static_cast<int16_t>(dist);
            }

            map[r][c] = cell;
        }
    }

    // ── Wire neighbour pointers ───────────────────────────────────────────────
    //
    // Rules:
    //   (A) Exit ring ↔ exit ring: never wired (outer-ring cells don't connect
    //       to each other — the mouse only cares about entering/leaving the
    //       real maze, not navigating around the exterior).
    //
    //   (B) Exit ring ↔ inner cell (real border): pointer from exit cell TO
    //       inner cell is wired permanently (exit cells are always reachable
    //       from inside once an opening exists).  Pointer from inner cell TO
    //       exit cell starts as nullptr (wall closed) and is opened by
    //       mazeOpenExit().
    //
    //   (C) Inner cell ↔ inner cell: wired freely (no internal walls at start;
    //       set_wall_bidir() adds them as the robot explores).
    //
    for (int r = 0; r < VSIZE; ++r) {
        for (int c = 0; c < VSIZE; ++c) {
            Cell* cell = map[r][c];
            bool  exit = cell->is_exit;

            // NORTH neighbour (r-1)
            if (r > 0) {
                Cell* nb = map[r-1][c];
                if (!exit && !nb->is_exit) {
                    cell->up = nb;          // (C) inner ↔ inner
                } else if (exit && !nb->is_exit) {
                    cell->up = nb;          // (B) exit→inner (exit side)
                    // nb->down stays nullptr until mazeOpenExit()
                } else if (!exit && nb->is_exit) {
                    cell->up = nullptr;     // real border: wall closed by default
                    // nb->down = cell done below when nb is processed
                }
                // exit ↔ exit: both stay nullptr
            }

            // SOUTH neighbour (r+1)
            if (r < VSIZE - 1) {
                Cell* nb = map[r+1][c];
                if (!exit && !nb->is_exit) {
                    cell->down = nb;
                } else if (exit && !nb->is_exit) {
                    cell->down = nb;
                } else if (!exit && nb->is_exit) {
                    cell->down = nullptr;
                }
            }

            // WEST neighbour (c-1)
            if (c > 0) {
                Cell* nb = map[r][c-1];
                if (!exit && !nb->is_exit) {
                    cell->left = nb;
                } else if (exit && !nb->is_exit) {
                    cell->left = nb;
                } else if (!exit && nb->is_exit) {
                    cell->left = nullptr;
                }
            }

            // EAST neighbour (c+1)
            if (c < VSIZE - 1) {
                Cell* nb = map[r][c+1];
                if (!exit && !nb->is_exit) {
                    cell->right = nb;
                } else if (exit && !nb->is_exit) {
                    cell->right = nb;
                } else if (!exit && nb->is_exit) {
                    cell->right = nullptr;
                }
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
// Called when the left-wall-follower senses an open outer wall on a real
// border cell.
//
//   vrow, vcol — virtual coordinates of the REAL border cell (not the exit cell)
//   exit_dir   — which side of that cell faces the exit ring
//
// What it does:
//   1. Finds the exit cell adjacent in exit_dir.
//   2. Wires the pointer FROM the real border cell TO the exit cell.
//      (The reverse pointer from exit cell to border cell is already wired from
//       the constructor — exit cells always point to their inner neighbour.)
//   3. Sets s_goal_known = true.
//   4. Does a full reflood so every inner cell now has its correct shortest
//      distance to this (or any previously opened) exit.
// ─────────────────────────────────────────────────────────────────────────────
void mazeOpenExit(Maze& maze, int vrow, int vcol, Direction exit_dir) {
    Cell* border = maze.at(vrow, vcol);
    if (!border || border->is_exit) return;

    // Find the virtual exit cell in exit_dir
    int er = vrow, ec = vcol;
    switch (exit_dir) {
        case NORTH: er = vrow - 1; break;
        case SOUTH: er = vrow + 1; break;
        case EAST:  ec = vcol + 1; break;
        case WEST:  ec = vcol - 1; break;
    }

    Cell* exitCell = maze.at(er, ec);
    if (!exitCell || !exitCell->is_exit) return;  // not actually an exit cell

    // Wire the opening bidirectionally
    switch (exit_dir) {
        case NORTH:
            border->up    = exitCell;
            exitCell->down = border;
            break;
        case SOUTH:
            border->down  = exitCell;
            exitCell->up  = border;
            break;
        case EAST:
            border->right = exitCell;
            exitCell->left = border;
            break;
        case WEST:
            border->left  = exitCell;
            exitCell->right = border;
            break;
    }

    s_goal_known = true;

    // Full reflood — exit cells are already at distance 0, so the flood
    // propagates naturally from them through any open border into the inner map.
    mazeReflood(maze, false);
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
    // Don't close the exit cell's pointer — only close real inner-cell pointers
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

// ─────────────────────────────────────────────────────────────────────────────
// flood_fill
//
// reflood_flag = false  →  propagate toward exit
//   Protect exit cells (m_distance=0, never updated) and the start cell once
//   we need to return.  Exit cells act as the fixed "seed" of the flood.
//
// reflood_flag = true   →  propagate toward start (return run)
//   Protect the start cell.  Exit cells are not protected — on the return run
//   we want the floodfill to propagate from the start outward.
// ─────────────────────────────────────────────────────────────────────────────
void flood_fill(Cell* c, CellStack& stk, bool reflood_flag) {
    if (!reflood_flag) {
        // Forward run: exit cells are the fixed seeds — never update them
        if (c->is_exit) return;
    } else {
        // Return run: protect the start cell
        if (c->pos_x == START_VROW && c->pos_y == START_VCOL) return;
    }

    if (!floodval_check(c)) {
        update_floodval(c);
        push_open_neighbors(c, stk);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// mazeReflood
//
// Full reflood of the entire virtual map.
//
// reflood_flag=false (forward):
//   Reset all non-exit cells to LARGEVAL, then seed from exit cells (distance
//   already 0) and propagate inward through any open border pointers.
//
// reflood_flag=true (return):
//   Reset all non-start cells to LARGEVAL, seed the start cell at 0, propagate.
//
// This is called:
//   • After mazeOpenExit() — a new exit path is available
//   • After plannerUpdateCurrentCell() — new walls discovered during exploration
// ─────────────────────────────────────────────────────────────────────────────
void mazeReflood(Maze& maze, bool reflood_flag) {
    CellStack stk;

    if (!reflood_flag) {
        // ── Forward reflood: seed from inner cells adjacent to open exits ─────
        //
        // Reset all inner (non-exit) cells to LARGEVAL.
        for (int r = 0; r < VSIZE; ++r)
            for (int c = 0; c < VSIZE; ++c)
                if (!maze.map[r][c]->is_exit)
                    maze.map[r][c]->m_distance = static_cast<int16_t>(LARGEVAL);

        // Identify every inner border cell that has an open connection to an
        // exit cell.  Set it to distance 1 and push its open inner neighbours
        // directly — we cannot rely on flood_fill() to push from these seed
        // cells because flood_fill() checks floodval_check() first, and a seed
        // cell already set to 1 (consistent with its exit neighbour at 0) passes
        // the check and returns without pushing anything.
        for (int r = 0; r < VSIZE; ++r) {
            for (int c = 0; c < VSIZE; ++c) {
                Cell* cell = maze.map[r][c];
                if (cell->is_exit) continue;

                bool seeds = false;
                if (cell->up    && cell->up->is_exit)    seeds = true;
                if (cell->down  && cell->down->is_exit)  seeds = true;
                if (cell->left  && cell->left->is_exit)  seeds = true;
                if (cell->right && cell->right->is_exit) seeds = true;

                if (seeds) {
                    cell->m_distance = 1;
                    // Push this cell's open inner neighbours so they get updated
                    push_open_neighbors(cell, stk);
                }
            }
        }
    } else {
        // ── Return reflood: seed from start cell ──────────────────────────────
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
        Cell* n = stk.top();
        stk.pop();
        if (n) flood_fill(n, stk, reflood_flag);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Direction chooser
//
// Three-pass priority:
//   1. Unvisited neighbours at best distance  (explore new territory first)
//   2. Current heading at best distance       (avoid unnecessary turns)
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

    // Pass 2: continue straight if possible
    switch (preferred_dir) {
        case NORTH: if (open_up()    && c->up->m_distance    == best) return NORTH; break;
        case EAST:  if (open_right() && c->right->m_distance == best) return EAST;  break;
        case SOUTH: if (open_down()  && c->down->m_distance  == best) return SOUTH; break;
        case WEST:  if (open_left()  && c->left->m_distance  == best) return WEST;  break;
    }

    // Pass 3: any open neighbour at best distance
    if (open_up()    && c->up->m_distance    == best) return NORTH;
    if (open_right() && c->right->m_distance == best) return EAST;
    if (open_down()  && c->down->m_distance  == best) return SOUTH;
    if (open_left()  && c->left->m_distance  == best) return WEST;

    return preferred_dir;
}
