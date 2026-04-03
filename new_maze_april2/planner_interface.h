#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// planner_interface.h  —  Escape-maze planner API
//
// Responsibilities
// ────────────────
//   Phase 1 — Exploration (left-wall-follow)
//     The mouse follows the left wall around the maze until it finds an open
//     outer wall (the exit).  Every cell it visits is recorded in the 6×6
//     virtual map.  The moment the exit is found, plannerOnExitFound() triggers
//     a full floodfill reseeded from that exit cell.
//
//   Phase 2 — Speed run (floodfill-guided)
//     The mouse returns to start, then runs the optimal path found by the
//     floodfill.  plannerChooseNextDirection() returns the direction that
//     descends the flood gradient toward the exit.
//
// Call sequence (from your main loop / test harness)
// ───────────────────────────────────────────────────
//   setup():
//     plannerInit();
//
//   After each executorDone() event (robot stopped at cell centre):
//     plannerArriveAtCell();       // sense walls, update map, check for exit
//
//     if (plannerPhase() == PHASE_EXPLORE)
//         direction = plannerLeftWallDirection();
//     else
//         direction = plannerChooseNextDirection();
//
//     executorBeginStep(direction);
//
//   Checking if done:
//     plannerAtExit()   — true once the robot has stepped into a virtual exit cell
//     plannerPhase()    — PHASE_EXPLORE / PHASE_SPEEDRUN
// ─────────────────────────────────────────────────────────────────────────────

#include "Maze.h"
#include "robot_state.h"   // RobotState robotState, SensorsFrame, PerceptionFrame

// ── Phase ─────────────────────────────────────────────────────────────────────
enum PlannerPhase {
    PHASE_EXPLORE,    // following left wall, mapping as we go
    PHASE_RETURN,     // floodfill found; returning to start for speed run
    PHASE_SPEEDRUN    // running optimal path toward exit
};

// ── Global maze (defined in planner_interface.cpp) ────────────────────────────
extern Maze maze;

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

// Call once in setup().  Resets all state.
void plannerInit();

// ─────────────────────────────────────────────────────────────────────────────
// Per-cell update  (call after executorDone(), before choosing next direction)
//
// plannerArriveAtCell() does all of the following in order:
//   1. Converts robot pose (robotState.x, robotState.y, robotState.heading)
//      to virtual map coordinates.
//   2. Reads robotState.perceptionFrame (wall_front, wall_left, wall_right).
//   3. Commits those walls to the virtual map (set_wall_bidir).
//   4. Checks whether any sensed open outer wall is an exit — if so, calls
//      mazeOpenExit() and transitions to PHASE_RETURN.
//   5. Marks the current cell visited.
//   6. Refloods from the current cell (incremental, not full).
// ─────────────────────────────────────────────────────────────────────────────
void plannerArriveAtCell();

// ─────────────────────────────────────────────────────────────────────────────
// Direction selection
// ─────────────────────────────────────────────────────────────────────────────

// Left-wall-follow direction — call during PHASE_EXPLORE.
// Returns the direction the robot should move next, following the left wall.
// Priority: turn left if open, else go straight if open, else turn right,
// else turn around (dead end).
Direction plannerLeftWallDirection();

// Floodfill-optimal direction — call during PHASE_SPEEDRUN or PHASE_RETURN.
// Descends the m_distance gradient toward whichever goal is active.
Direction plannerChooseNextDirection();

// ─────────────────────────────────────────────────────────────────────────────
// State queries
// ─────────────────────────────────────────────────────────────────────────────

PlannerPhase plannerPhase();

// True once the robot has stepped off the 4×4 maze into a virtual exit cell.
// Your main loop should stop calling executorBeginStep() when this is true.
bool plannerAtExit();

// Virtual-map coordinates of current robot position (updated by plannerArriveAtCell).
int plannerCurrentVrow();
int plannerCurrentVcol();
