// planner_interface.cpp  —  Escape-maze planner implementation
//
// Integrates:
//   • Maze.h/cpp   — 6×6 virtual map + floodfill
//   • robot_state  — live sensor readings, pose, heading
//   • Left-wall-follow algorithm during exploration
//   • Floodfill-guided path during speed run

#include "planner_interface.h"
#include "Maze.h"
#include "robot_state.h"

#include <Arduino.h>   // Serial, millis — remove if building for host/test

// ─────────────────────────────────────────────────────────────────────────────
// Global maze instance
// ─────────────────────────────────────────────────────────────────────────────
Maze maze;

// ─────────────────────────────────────────────────────────────────────────────
// Internal state
// ─────────────────────────────────────────────────────────────────────────────
static PlannerPhase s_phase    = PHASE_EXPLORE;
static bool         s_at_exit  = false;
static int          s_vrow     = START_VROW;
static int          s_vcol     = START_VCOL;

// ─────────────────────────────────────────────────────────────────────────────
// Direction helpers
// ─────────────────────────────────────────────────────────────────────────────
static Direction turnLeft (Direction d) { return static_cast<Direction>((d + 3) % 4); }
static Direction turnRight(Direction d) { return static_cast<Direction>((d + 1) % 4); }
static Direction opposite (Direction d) { return static_cast<Direction>((d + 2) % 4); }

// Absolute direction of each sensor relative to robot heading
static Direction absoluteFront(Direction heading) { return heading;            }
static Direction absoluteLeft (Direction heading) { return turnLeft(heading);  }
static Direction absoluteRight(Direction heading) { return turnRight(heading); }

// ─────────────────────────────────────────────────────────────────────────────
// Coordinate helpers
// ─────────────────────────────────────────────────────────────────────────────
// Convert current robot MMS pose to virtual-map row/col
static int poseToVrow() { return mmsToVrow(robotState.x, robotState.y); }
static int poseToVcol() { return mmsToVcol(robotState.x, robotState.y); }

// Given a virtual cell (vr,vc) and direction, return the neighbour's virtual coords.
static void neighbourCoords(int vr, int vc, Direction dir, int& nr, int& nc) {
    nr = vr; nc = vc;
    switch (dir) {
        case NORTH: nr--; break;
        case SOUTH: nr++; break;
        case EAST:  nc++; break;
        case WEST:  nc--; break;
    }
}

// Is (vr,vc) on the border of the inner 4×4 (i.e. it has an outer edge)?
static bool isRealBorderCell(int vr, int vc) {
    return (vr == 1 || vr == INNER || vc == 1 || vc == INNER);
}

// Which direction from (vr,vc) faces the exit ring?
// Returns true and sets exit_dir if this border cell faces that direction.
static bool outerWallDirection(int vr, int vc, Direction dir) {
    switch (dir) {
        case NORTH: return (vr == 1);
        case SOUTH: return (vr == INNER);
        case EAST:  return (vc == INNER);
        case WEST:  return (vc == 1);
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Wall sensing helpers
//
// The PerceptionFrame gives wall_front, wall_left, wall_right relative to the
// robot's current heading.  This converts them to absolute directions and
// checks each one.
//
// "Outer wall open" = we are at a border cell, the sensor in that direction
// reads NO wall, AND that direction faces the exit ring.  That is the exit.
// ─────────────────────────────────────────────────────────────────────────────
static void senseAndUpdateWalls() {
    Direction heading = robotState.heading;
    int vr = s_vrow;
    int vc = s_vcol;

    const bool wf = robotState.perceptionFrame.wall_front;
    const bool wl = robotState.perceptionFrame.wall_left;
    const bool wr = robotState.perceptionFrame.wall_right;
    // Note: back wall (behind robot) is always the wall we came through —
    // already set when the previous cell was processed.

    struct { bool wall; Direction abs_dir; } sides[3] = {
        { wf, absoluteFront(heading) },
        { wl, absoluteLeft (heading) },
        { wr, absoluteRight(heading) },
    };

    for (auto& s : sides) {
        if (s.wall) {
            // Sensor sees a wall — commit it bidirectionally.
            // set_wall_bidir is safe on border cells (won't close exit-ring side).
            set_wall_bidir(maze, vr, vc, s.abs_dir);

        } else {
            // No wall in this direction.
            // Check: are we a border cell, and does this direction face outside?
            if (!mazeGoalKnown() && isRealBorderCell(vr, vc) &&
                outerWallDirection(vr, vc, s.abs_dir)) {

                // This is the exit opening.
                Serial.print("PLANNER: exit found at virtual (");
                Serial.print(vr); Serial.print(","); Serial.print(vc);
                Serial.print(") dir="); Serial.println(s.abs_dir);

                mazeOpenExit(maze, vr, vc, s.abs_dir);
                // mazeOpenExit triggers a full reflood internally.

                s_phase = PHASE_RETURN;
                Serial.println("PLANNER: -> PHASE_RETURN (head back to start)");
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Exit detection
//
// After the robot executes a step, check whether it has moved into a virtual
// exit cell.  This happens when the executor moves the robot in a direction
// that was an open outer wall.
// ─────────────────────────────────────────────────────────────────────────────
static void checkIfExited() {
    int vr = poseToVrow();
    int vc = poseToVcol();
    Cell* cur = maze.at(vr, vc);
    if (cur && cur->is_exit) {
        s_at_exit = true;
        Serial.println("PLANNER: robot has exited the maze!");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Incremental reflood from current cell
//
// After committing new walls, propagate the floodfill outward from the current
// cell only — much faster than a full reflood on a 4×4 map, and sufficient
// because wall additions can only increase neighbouring distances.
// ─────────────────────────────────────────────────────────────────────────────
static void incrementalReflood() {
    Cell* cur = maze.at(s_vrow, s_vcol);
    if (!cur || cur->is_exit) return;

    CellStack stk;
    stk.push(cur);
    while (!stk.empty()) {
        Cell* n = stk.top(); stk.pop();
        if (n) flood_fill(n, stk, /*reflood_flag=*/false);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase transition helpers
// ─────────────────────────────────────────────────────────────────────────────
static void checkPhaseTransition() {
    if (s_phase == PHASE_RETURN) {
        // Check if we are back at the start cell
        if (s_vrow == START_VROW && s_vcol == START_VCOL) {
            s_phase = PHASE_SPEEDRUN;
            // Full reflood toward exit to get the cleanest gradient for speed run
            mazeReflood(maze, false);
            Serial.println("PLANNER: -> PHASE_SPEEDRUN");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// plannerInit
// ─────────────────────────────────────────────────────────────────────────────
void plannerInit() {
    s_phase   = PHASE_EXPLORE;
    s_at_exit = false;
    s_vrow    = START_VROW;
    s_vcol    = START_VCOL;
    // maze is default-constructed with all initial heuristic distances.
}

// ─────────────────────────────────────────────────────────────────────────────
// plannerArriveAtCell
//
// Call this after executorDone() is true (robot has stopped at cell centre).
// Updates the virtual map with freshly sensed walls, checks for the exit,
// and keeps the floodfill values current.
// ─────────────────────────────────────────────────────────────────────────────
void plannerArriveAtCell() {
    // 1. Update virtual-map position from robot pose
    s_vrow = poseToVrow();
    s_vcol = poseToVcol();

    // 2. Check if the robot has stepped into an exit cell
    //    (happens when the previous step moved through an open outer wall)
    checkIfExited();
    if (s_at_exit) return;  // no more planning needed

    Cell* cur = maze.at(s_vrow, s_vcol);
    if (!cur) return;

    // 3. Sense walls and update the virtual map
    //    Also detects the exit if an outer wall is open
    senseAndUpdateWalls();

    // 4. Mark this cell visited
    set_visited(cur);

    // 5. Incremental reflood from current cell
    //    (mazeOpenExit already did a full reflood if the exit was just found)
    if (!mazeGoalKnown()) {
        incrementalReflood();
    }

    // 6. Check phase transitions
    checkPhaseTransition();

    // 7. Debug
    Serial.print("PLANNER: at virtual (");
    Serial.print(s_vrow); Serial.print(","); Serial.print(s_vcol);
    Serial.print(") dist="); Serial.print(cur->m_distance);
    Serial.print(" phase="); Serial.println(s_phase);
}

// ─────────────────────────────────────────────────────────────────────────────
// plannerLeftWallDirection
//
// Standard left-wall-follow decision.
// The robot always tries to turn left first, then go straight, then turn right,
// then turn around.  This guarantees it hugs the left wall and will eventually
// find the exit in any simply-connected maze.
//
// "Open" here means the sensor read no wall AND the map pointer is non-null
// (consistent).  We use the PerceptionFrame for real-time sensing.
// ─────────────────────────────────────────────────────────────────────────────
Direction plannerLeftWallDirection() {
    Direction heading = robotState.heading;

    // Sensor readings relative to heading
    bool wall_front = robotState.perceptionFrame.wall_front;
    bool wall_left  = robotState.perceptionFrame.wall_left;
    bool wall_right = robotState.perceptionFrame.wall_right;
    // Back is always closed (where we came from — we don't need to check it
    // until we reach a dead end).

    // Left-wall priority:
    //   1. Can we turn left?   → turn left
    //   2. Can we go forward?  → go straight
    //   3. Can we turn right?  → turn right
    //   4. Dead end            → turn around

    if (!wall_left)  return turnLeft(heading);
    if (!wall_front) return heading;
    if (!wall_right) return turnRight(heading);
    return opposite(heading);  // dead end
}

// ─────────────────────────────────────────────────────────────────────────────
// plannerChooseNextDirection
//
// Follow the floodfill gradient.  Used during PHASE_RETURN (gradient toward
// start) and PHASE_SPEEDRUN (gradient toward exit).
//
// The reflood_flag used for the current gradient:
//   PHASE_RETURN   → mazeReflood was called with reflood_flag=true (toward start)
//   PHASE_SPEEDRUN → mazeReflood was called with reflood_flag=false (toward exit)
// ─────────────────────────────────────────────────────────────────────────────
Direction plannerChooseNextDirection() {
    Cell* cur = maze.at(s_vrow, s_vcol);
    if (!cur) return robotState.heading;  // fallback

    if (s_phase == PHASE_RETURN) {
        // Gradient is currently set toward start (reflood_flag=true was used).
        // Descend that gradient.
        return get_smallest_neighbor_dir(cur, robotState.heading);
    }

    // PHASE_SPEEDRUN: gradient set toward exit.
    return get_smallest_neighbor_dir(cur, robotState.heading);
}

// ─────────────────────────────────────────────────────────────────────────────
// State queries
// ─────────────────────────────────────────────────────────────────────────────
PlannerPhase plannerPhase()       { return s_phase;   }
bool         plannerAtExit()      { return s_at_exit; }
int          plannerCurrentVrow() { return s_vrow;    }
int          plannerCurrentVcol() { return s_vcol;    }
