// Interface.cpp  –  Glue between MMS API and Maze/Floodfill core

#include "Interface.h"
#include "API.h"
#include <cstdio>

// ─────────────────────────────────────────────────────────────────────────────
// Direction arithmetic helpers
// ─────────────────────────────────────────────────────────────────────────────
static inline Direction turn_left_of (Direction d) { return static_cast<Direction>((d + 3) & 3); }
static inline Direction turn_right_of(Direction d) { return static_cast<Direction>((d + 1) & 3); }

// ─────────────────────────────────────────────────────────────────────────────
// Coordinate mapping: MMS ↔ internal map[row][col]
//   MMS:    (0,0) = bottom-left, y increases upward
//   Internal: map[row][col], row 0 = top, row increases downward
//
//   row = (SIZE - 1) - y
//   col = x
// ─────────────────────────────────────────────────────────────────────────────
static inline int row_of(int y) { return (SIZE - 1) - y; }
static inline int col_of(int x) { return x; }

static Cell* cell_at(Maze& maze, int x, int y) {
    return maze.map[row_of(y)][col_of(x)];
}

// Const overload for read-only contexts
static const Cell* cell_at(const Maze& maze, int x, int y) {
    return maze.map[row_of(y)][col_of(x)];
}

// ─────────────────────────────────────────────────────────────────────────────
// Bidirectional wall setting
// A wall must be set on BOTH adjacent cells so floodfill sees a consistent map.
// ─────────────────────────────────────────────────────────────────────────────
static void set_wall_bidir(Maze& maze, int x, int y, Direction dir) {
    Cell* a = cell_at(maze, x, y);
    if (!a) return;

    set_wall(a, dir);

    // Inform MMS visualiser
    const char dir_char[] = { 'n', 'e', 's', 'w' };
    API_setWall(x, y, dir_char[dir]);

    // Mirror on the neighbouring cell
    switch (dir) {
        case NORTH:
            if (y + 1 < SIZE) if (Cell* b = cell_at(maze, x, y + 1)) set_wall(b, SOUTH);
            break;
        case EAST:
            if (x + 1 < SIZE) if (Cell* b = cell_at(maze, x + 1, y)) set_wall(b, WEST);
            break;
        case SOUTH:
            if (y - 1 >= 0)   if (Cell* b = cell_at(maze, x, y - 1)) set_wall(b, NORTH);
            break;
        case WEST:
            if (x - 1 >= 0)   if (Cell* b = cell_at(maze, x - 1, y)) set_wall(b, EAST);
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Rotation  –  BUG FIX
//
// Original code turned right in a loop, meaning e.g. NORTH → WEST required
// three right turns instead of one left turn.
//
// Fix: compute the signed angular difference, then issue turns accordingly.
//   diff = 1 → 1 right
//   diff = 2 → 2 rights  (or 2 lefts – same cost; we use 2 rights)
//   diff = 3 → 1 left  ← THIS was the bug: 3 rights instead of 1 left
// ─────────────────────────────────────────────────────────────────────────────
static void rotate_to(Pose& pose, Direction target) {
    // Number of right-turns to reach target: 0, 1, 2, or 3
    int diff = ((int)target - (int)pose.heading + 4) & 3;

    if (diff == 0) {
        // Already facing the right way
    } else if (diff == 1) {
        API_turnRight();
        pose.heading = turn_right_of(pose.heading);
    } else if (diff == 2) {
        // 180°: two right turns (or two lefts – cost is equal)
        API_turnRight(); pose.heading = turn_right_of(pose.heading);
        API_turnRight(); pose.heading = turn_right_of(pose.heading);
    } else {  // diff == 3  →  one left turn
        API_turnLeft();
        pose.heading = turn_left_of(pose.heading);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Public interface functions
// ─────────────────────────────────────────────────────────────────────────────

void iface_sense_and_update(Maze& maze, const Pose& pose) {
    bool wf = API_wallFront();
    bool wl = API_wallLeft();
    bool wr = API_wallRight();

    Direction fwd   = pose.heading;
    Direction left  = turn_left_of(pose.heading);
    Direction right = turn_right_of(pose.heading);

    if (wf) set_wall_bidir(maze, pose.x, pose.y, fwd);
    if (wl) set_wall_bidir(maze, pose.x, pose.y, left);
    if (wr) set_wall_bidir(maze, pose.x, pose.y, right);

    set_visited(cell_at(maze, pose.x, pose.y));
}

void iface_reflood_from_current(Maze& maze, const Pose& pose, bool reflood_flag) {
    CellStack stk;
    stk.push(cell_at(maze, pose.x, pose.y));

    while (!stk.empty()) {
        Cell* c = stk.top();
        stk.pop();
        if (c) flood_fill(c, stk, reflood_flag);
    }
}

Direction iface_choose_next_dir(const Maze& maze, const Pose& pose) {
    const Cell* cur = cell_at(maze, pose.x, pose.y);
    return get_smallest_neighbor_dir(cur, pose.heading);
}

bool iface_execute_step(Pose& pose, Direction target_dir) {
    rotate_to(pose, target_dir);

    bool moved = (API_moveForward() == 1);
    if (moved) {
        switch (pose.heading) {
            case NORTH: pose.y++; break;
            case EAST:  pose.x++; break;
            case SOUTH: pose.y--; break;
            case WEST:  pose.x--; break;
        }
    }
    return moved;
}
