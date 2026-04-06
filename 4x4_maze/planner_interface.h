#ifndef PLANNER_INTERFACE_H
#define PLANNER_INTERFACE_H

// ─────────────────────────────────────────────────────────────────────────────
// planner_interface.h  —  escape maze planner
//
// Public API is backward-compatible with the original test_Executor.ino
// calling pattern:
//
//   plannerInit()
//   plannerUpdateCurrentCell()
//   plannerChooseNextDirection()
//
// Additional calls exposed for the new three-phase logic:
//   plannerPhase()       — which phase the robot is in
//   plannerAtExit()      — true once the robot has exited the maze
//   plannerGoalKnown()   — true once an exit opening has been discovered
// ─────────────────────────────────────────────────────────────────────────────

#include "direction.h"

// ── Phase ─────────────────────────────────────────────────────────────────────
enum PlannerPhase {
    PHASE_EXPLORE,    // left-wall-follow until exit found
    PHASE_RETURN,     // return to start for clean speed run
    PHASE_SPEEDRUN    // follow floodfill gradient to exit
};

// ── Lifecycle ─────────────────────────────────────────────────────────────────
void plannerInit();

// ── Per-cell update ───────────────────────────────────────────────────────────
// Call after executorDone() is true (robot stopped at cell centre).
// Senses walls, updates the virtual map, checks for exit, marks cell visited,
// and refloods incrementally.
void plannerUpdateCurrentCell();

// ── Direction selection ───────────────────────────────────────────────────────
// Returns the best next direction for the current phase.
// During PHASE_EXPLORE this is left-wall-follow.
// During PHASE_RETURN and PHASE_SPEEDRUN this is floodfill-optimal.
Direction plannerChooseNextDirection();

// ── State queries ─────────────────────────────────────────────────────────────
PlannerPhase plannerPhase();
bool         plannerAtExit();
bool         plannerGoalKnown();

#endif
