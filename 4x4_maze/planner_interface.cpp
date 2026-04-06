// planner_interface.cpp  —  escape maze planner
//
// Wires together:
//   • maze.h/cpp      — 6×6 virtual map + floodfill
//   • robot_state     — live sensor readings and pose
//   • Left-wall-follow during PHASE_EXPLORE
//   • Floodfill-guided path during PHASE_RETURN and PHASE_SPEEDRUN

#include "planner_interface.h"
#include "maze.h"
#include "robot_state.h"
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// Global maze instance
// ─────────────────────────────────────────────────────────────────────────────
static Maze maze;

// ─────────────────────────────────────────────────────────────────────────────
// Internal state
// ─────────────────────────────────────────────────────────────────────────────
static PlannerPhase s_phase   = PHASE_EXPLORE;
static bool         s_at_exit = false;
static int          s_vrow    = START_VROW;
static int          s_vcol    = START_VCOL;

// ─────────────────────────────────────────────────────────────────────────────
// Direction helpers
// ─────────────────────────────────────────────────────────────────────────────
static Direction turnLeft (Direction d) { return static_cast<Direction>((d + 3) % 4); }
static Direction turnRight(Direction d) { return static_cast<Direction>((d + 1) % 4); }
static Direction opposite (Direction d) { return static_cast<Direction>((d + 2) % 4); }

// ─────────────────────────────────────────────────────────────────────────────
// Coordinate conversion from robot MMS pose to virtual-map row/col
// ─────────────────────────────────────────────────────────────────────────────
static int poseVrow() { return mmsToVrow(robotState.x, robotState.y); }
static int poseVcol() { return mmsToVcol(robotState.x, robotState.y); }

// ─────────────────────────────────────────────────────────────────────────────
// Wall direction helpers
// ─────────────────────────────────────────────────────────────────────────────
static Direction absLeft (Direction h) { return turnLeft(h);  }
static Direction absFront(Direction h) { return h;             }
static Direction absRight(Direction h) { return turnRight(h); }

// Is (vrow, vcol) on the real 4×4 border — i.e. it has an outer edge?
static bool isBorderCell(int vr, int vc) {
    return (vr == 1 || vr == INNER || vc == 1 || vc == INNER);
}

// Does direction `dir` from this border cell face outside the maze?
static bool facesOutside(int vr, int vc, Direction dir) {
    if (dir == NORTH && vr == 1)     return true;
    if (dir == SOUTH && vr == INNER) return true;
    if (dir == EAST  && vc == INNER) return true;
    if (dir == WEST  && vc == 1)     return true;
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Exit cell detection
// After any step the robot might have moved into a virtual exit cell.
// ─────────────────────────────────────────────────────────────────────────────
static void checkIfExited() {
    Cell* cur = maze.at(s_vrow, s_vcol);
    if (cur && cur->is_exit) {
        s_at_exit = true;
        Serial.println("PLANNER: robot has exited the maze!");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Wall sensing and map update
//
// Reads the three sensor directions from robotState.perceptionFrame.
// For each direction:
//   • Wall present  → commit to virtual map bidirectionally.
//   • No wall AND this is a border cell facing outside → exit discovered.
// ─────────────────────────────────────────────────────────────────────────────
static void senseAndUpdateWalls() {
    Direction heading = robotState.heading;
    int vr = s_vrow;
    int vc = s_vcol;

    struct { bool wall; Direction abs_dir; } sides[3] = {
        { robotState.perceptionFrame.wall_front, absFront(heading) },
        { robotState.perceptionFrame.wall_left,  absLeft (heading) },
        { robotState.perceptionFrame.wall_right, absRight(heading) },
    };

    for (auto& s : sides) {
        if (s.wall) {
            set_wall_bidir(maze, vr, vc, s.abs_dir);
        } else {
            // Sensor reads open — check if this is the outer border (exit)
            if (!mazeGoalKnown() && isBorderCell(vr, vc) &&
                facesOutside(vr, vc, s.abs_dir)) {

                Serial.print("PLANNER: exit found at v(");
                Serial.print(vr); Serial.print(",");
                Serial.print(vc); Serial.print(") dir=");
                Serial.println((int)s.abs_dir);

                mazeOpenExit(maze, vr, vc, s.abs_dir);
                // mazeOpenExit does a full reflood internally

                // Switch to return phase — go back to start for speed run
                s_phase = PHASE_RETURN;
                // Reverse the gradient: reflood toward start
                mazeReflood(maze, true);
                Serial.println("PLANNER: -> PHASE_RETURN");
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Incremental reflood from current cell
// Faster than a full reflood; sufficient after adding a single wall.
// ─────────────────────────────────────────────────────────────────────────────
static void incrementalReflood() {
    Cell* cur = maze.at(s_vrow, s_vcol);
    if (!cur || cur->is_exit) return;

    CellStack stk;
    stk.push(cur);
    while (!stk.empty()) {
        Cell* n = stk.top(); stk.pop();
        if (n) flood_fill(n, stk, false);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase transition check
// ─────────────────────────────────────────────────────────────────────────────
static void checkPhaseTransition() {
    if (s_phase == PHASE_RETURN) {
        if (s_vrow == START_VROW && s_vcol == START_VCOL) {
            s_phase = PHASE_SPEEDRUN;
            // Restore exit gradient for the speed run
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
    // maze default-constructed with all initial heuristic distances
}

// ─────────────────────────────────────────────────────────────────────────────
// plannerUpdateCurrentCell
//
// Called by test_Executor.ino after executorDone() is true.
// ─────────────────────────────────────────────────────────────────────────────
void plannerUpdateCurrentCell() {
    // 1. Update virtual-map position from robot pose
    s_vrow = poseVrow();
    s_vcol = poseVcol();

    // 2. Check if robot stepped into an exit cell
    checkIfExited();
    if (s_at_exit) return;

    Cell* cur = maze.at(s_vrow, s_vcol);
    if (!cur) return;

    // 3. Sense walls; detect exit if outer wall is open
    senseAndUpdateWalls();

    // 4. Mark visited
    set_visited(cur);

    // 5. Incremental reflood (only if goal not yet known — mazeOpenExit
    //    already does a full reflood when the exit is first found)
    if (!mazeGoalKnown()) {
        incrementalReflood();
    }

    // 6. Check for phase transitions (return→speedrun on reaching start)
    checkPhaseTransition();

    Serial.print("PLANNER: v(");
    Serial.print(s_vrow); Serial.print(",");
    Serial.print(s_vcol); Serial.print(") dist=");
    Serial.print(cur->m_distance);
    Serial.print(" phase="); Serial.println((int)s_phase);
}

// ─────────────────────────────────────────────────────────────────────────────
// plannerChooseNextDirection
//
// PHASE_EXPLORE : left-wall-follow
//   Priority: turn left if open, else straight, else turn right, else reverse.
//
// PHASE_RETURN / PHASE_SPEEDRUN : floodfill gradient descent
//   get_smallest_neighbor_dir() handles all three passes (unvisited, straight,
//   any) so we don't need to duplicate that logic here.
// ─────────────────────────────────────────────────────────────────────────────
Direction plannerChooseNextDirection() {
    if (s_at_exit) return robotState.heading;  // no further movement needed

    if (s_phase == PHASE_EXPLORE) {
        Direction heading = robotState.heading;

        bool open_left  = !robotState.perceptionFrame.wall_left;
        bool open_front = !robotState.perceptionFrame.wall_front;
        bool open_right = !robotState.perceptionFrame.wall_right;

        if (open_left)  return turnLeft(heading);
        if (open_front) return heading;
        if (open_right) return turnRight(heading);
        return opposite(heading);  // dead end
    }

    // PHASE_RETURN or PHASE_SPEEDRUN: follow floodfill gradient
    Cell* cur = maze.at(s_vrow, s_vcol);
    if (!cur) return robotState.heading;
    return get_smallest_neighbor_dir(cur, robotState.heading);
}

// ─────────────────────────────────────────────────────────────────────────────
// State queries
// ─────────────────────────────────────────────────────────────────────────────
PlannerPhase plannerPhase()    { return s_phase;        }
bool         plannerAtExit()   { return s_at_exit;      }
bool         plannerGoalKnown(){ return mazeGoalKnown(); }
