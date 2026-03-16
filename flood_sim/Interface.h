#pragma once

#include "Maze.h"

// ── Pose: mouse position in MMS coordinate space ─────────────────────────────
// MMS: (0,0) = bottom-left corner, y increases upward.
struct Pose {
    int x       = 0;           // MMS x  (column, 0..SIZE-1)
    int y       = 0;           // MMS y  (row from bottom, 0..SIZE-1)
    Direction heading = NORTH; // current facing direction
};

// 1) Read wall sensors and update the internal Maze model.
void iface_sense_and_update(Maze& maze, const Pose& pose);

// 2) Propagate floodfill values from the current cell.
//    reflood_flag=false → flood toward goal centre.
//    reflood_flag=true  → flood toward start cell.
void iface_reflood_from_current(Maze& maze, const Pose& pose, bool reflood_flag);

// 3) Choose the best next absolute direction using the floodfill values.
Direction iface_choose_next_dir(const Maze& maze, const Pose& pose);

// 4) Rotate to target_dir (choosing shortest turn path) then move forward.
//    Returns true if the mouse successfully moved, false on wall collision.
bool iface_execute_step(Pose& pose, Direction target_dir);
