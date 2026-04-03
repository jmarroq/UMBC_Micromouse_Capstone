// test_planner.cpp  —  validates planner logic without Arduino hardware
//
// Simulates a simple 4×4 maze, drives the robot through it with the
// left-wall-follower, verifies the exit is found and the phase transitions
// occur correctly.
//
// Compile: g++ -std=c++17 -Wall -DHOST_TEST -o test_planner test_planner.cpp Maze.cpp
// Run:     ./test_planner

#include "Maze.h"
#include <cstdio>
#include <cassert>
#include <algorithm>

// ── Stub Arduino types so we can compile without Arduino.h ───────────────────
#ifdef HOST_TEST
namespace {
    struct Serial_ {
        void begin(int) {}
        void print(const char* s)  { printf("%s", s); }
        void print(int v)          { printf("%d", v); }
        void println(const char* s){ printf("%s\n", s); }
        void println(int v)        { printf("%d\n", v); }
    } Serial;
}
#endif

// ── Minimal robot state stub ──────────────────────────────────────────────────
struct PerceptionFrame { bool wall_front, wall_left, wall_right; };
struct SensorsFrame    { uint32_t t_ms; uint16_t left_mm, front_mm, right_mm;
                         bool left_valid, front_valid, right_valid; };

struct RobotState {
    SensorsFrame    sensorFrame    = {};
    PerceptionFrame perceptionFrame = {};
    int x = 0, y = 0;
    Direction heading = NORTH;
} robotState;

void robotStateInit(RobotState& s) {
    s.x = 0; s.y = 0; s.heading = NORTH;
    s.perceptionFrame = {};
}

// ─────────────────────────────────────────────────────────────────────────────
// Simulated maze layout (for host test)
//
//   4×4 real maze, cells numbered (row,col) 0‥3
//   Exit is on the NORTH side of real cell (0,1) = virtual (1,2)
//
//   Walls (all internal walls — outer border closed by default):
//     (0,0)↔(0,1) EAST wall present
//     (0,1)↔(0,2) EAST wall present
//     (1,0)↔(1,1) EAST wall present
//     (2,1)↔(2,2) EAST wall present
//     (1,1)↔(2,1) SOUTH wall present
//
//   This creates a simple maze that a left-wall-follower will solve.
//
// ─────────────────────────────────────────────────────────────────────────────

// ── Left-wall-follow helper ───────────────────────────────────────────────────
static Direction turnLeft (Direction d) { return static_cast<Direction>((d+3)%4); }
static Direction turnRight(Direction d) { return static_cast<Direction>((d+1)%4); }
static Direction opposite (Direction d) { return static_cast<Direction>((d+2)%4); }

// Check whether a move from (vr,vc) in direction dir is open in the maze
static bool isOpen(const Maze& m, int vr, int vc, Direction dir) {
    Cell* c = m.at(vr, vc);
    if (!c) return false;
    switch (dir) {
        case NORTH: return c->up    != nullptr;
        case SOUTH: return c->down  != nullptr;
        case EAST:  return c->right != nullptr;
        case WEST:  return c->left  != nullptr;
    }
    return false;
}

// ── Left-wall-follow: uses sensors (open_sensor), not map pointers ─────────
// In real hardware the sensor reads the physical wall regardless of what the
// map says.  Here we simulate that by checking the map for internal walls but
// using a separate "physical_open" function for the exit.

// Physical opening exists (what a sensor would read):
//   - For inner-to-inner transitions: check map pointer
//   - For inner-to-exit-ring transitions: the EXIT is open (no physical wall there),
//     all other outer walls are closed
static bool physicallyOpen(const Maze& m, int vr, int vc, Direction dir,
                           int exit_vr, int exit_vc, Direction exit_dir) {
    // Check if this is the exit opening
    int nr = vr, nc = vc;
    switch (dir) {
        case NORTH: nr--; break; case SOUTH: nr++; break;
        case EAST:  nc++; break; case WEST:  nc--;  break;
    }
    // Is the neighbour an exit cell?
    Cell* nb = m.at(nr, nc);
    if (nb && nb->is_exit) {
        // This is the known exit?
        return (nr == exit_vr && nc == exit_vc);
        // All other outer walls are physically present
    }
    // Inner-to-inner: check map pointer
    return isOpen(m, vr, vc, dir);
}

static Direction leftWallChooseSensor(const Maze& m, int vr, int vc, Direction hdg,
                                      int exit_vr, int exit_vc, Direction exit_dir_d) {
    Direction left  = turnLeft(hdg);
    Direction front = hdg;
    Direction right = turnRight(hdg);
    Direction back  = opposite(hdg);

    if (physicallyOpen(m,vr,vc,left, exit_vr,exit_vc,exit_dir_d)) return left;
    if (physicallyOpen(m,vr,vc,front,exit_vr,exit_vc,exit_dir_d)) return front;
    if (physicallyOpen(m,vr,vc,right,exit_vr,exit_vc,exit_dir_d)) return right;
    return back;
}

// Move one cell in direction dir; update virtual coords
static void moveCell(int& vr, int& vc, Direction dir) {
    switch (dir) {
        case NORTH: vr--; break;
        case SOUTH: vr++; break;
        case EAST:  vc++; break;
        case WEST:  vc--; break;
    }
}

// ── Print the distance grid ───────────────────────────────────────────────────
static void printGrid(const Maze& m, int robot_vr, int robot_vc, const char* label) {
    printf("\n=== %s ===\n", label);
    printf("     c0    c1    c2    c3    c4    c5\n");
    for (int r = 0; r < VSIZE; ++r) {
        printf("r%d ", r);
        for (int c = 0; c < VSIZE; ++c) {
            Cell* cell = m.map[r][c];
            if (r == robot_vr && c == robot_vc)
                printf("[**]  ");
            else if (cell->is_exit)
                printf("[EX]  ");
            else
                printf("[ %2d] ", cell->m_distance);
        }
        printf("\n");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main test
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    Maze maze;

    printf("=== test_planner: 4x4 escape maze simulation ===\n");

    // ── Build the simulated maze walls ────────────────────────────────────────
    // All coordinates are virtual (vrow, vcol)
    // Real (row,col) → virtual (row+1, col+1)

    // Internal walls (blocking some passages):
    set_wall_bidir(maze, 1, 1, EAST);   // real (0,0)↔(0,1) east wall
    set_wall_bidir(maze, 1, 2, EAST);   // real (0,1)↔(0,2) east wall
    set_wall_bidir(maze, 2, 1, EAST);   // real (1,0)↔(1,1) east wall
    set_wall_bidir(maze, 3, 2, EAST);   // real (2,1)↔(2,2) east wall
    set_wall_bidir(maze, 2, 2, SOUTH);  // real (1,1)↔(2,1) south wall

    printGrid(maze, START_VROW, START_VCOL, "initial (no exit known)");

    // ── Simulate left-wall-follow exploration ─────────────────────────────────
    int vr  = START_VROW;
    int vc  = START_VCOL;
    Direction hdg = NORTH;
    int steps = 0;
    bool goal_found = false;

    // The physical exit is at v(1,2) north — virtual exit cell is v(0,2)
    // Robot can sense this opening when it faces north at v(1,2).
    // All other outer walls are physically present.
    const int EXIT_VR = 0, EXIT_VC = 2;
    const Direction EXIT_DIR = NORTH;

    printf("\nSimulating left-wall-follow from virtual (%d,%d)...\n", vr, vc);

    // Mark start visited
    set_visited(maze.at(vr, vc));

    while (!goal_found && steps < 200) {
        Direction next = leftWallChooseSensor(maze, vr, vc, hdg,
                                              EXIT_VR, EXIT_VC, EXIT_DIR);

        // Detect exit: sensor sees opening toward exit cell
        int nr = vr, nc = vc;
        moveCell(nr, nc, next);
        Cell* dest = maze.at(nr, nc);

        if (dest && dest->is_exit) {
            // Robot is about to step into the exit — record discovery first
            printf("\n  -> EXIT FOUND: robot at v(%d,%d) moves %s into exit v(%d,%d) (step %d)\n",
                   vr, vc, next==NORTH?"N":next==EAST?"E":next==SOUTH?"S":"W", nr, nc, steps);
            mazeOpenExit(maze, vr, vc, next);
            goal_found = true;
            printGrid(maze, vr, vc, "after exit found + full reflood");
            break;
        }

        // Normal move
        moveCell(vr, vc, next);
        hdg = next;
        steps++;

        Cell* cur = maze.at(vr, vc);
        if (cur && !cur->is_exit) set_visited(cur);

        printf("  step %2d: %s → v(%d,%d)  dist=%d\n",
               steps, next==NORTH?"N":next==EAST?"E":next==SOUTH?"S":"W",
               vr, vc, cur ? cur->m_distance : -1);
    }

    // ── Verify floodfill after exit found ─────────────────────────────────────
    printf("\n[CHECK] Post-exit floodfill correctness\n");

    // Exit cell should be distance 0
    assert(maze.map[0][2]->m_distance == 0);
    printf("  [PASS] exit cell v(0,2) distance = 0\n");

    // Border cell (1,2) should be distance 1
    assert(maze.map[1][2]->m_distance == 1);
    printf("  [PASS] v(1,2) distance = 1\n");

    // v(1,1) east wall is closed — it must go south and around.
    // With this maze layout it ends up at distance 10.
    assert(maze.map[1][1]->m_distance > 3);
    printf("  [PASS] v(1,1) blocked east — distance = %d (must route around)\n",
           maze.map[1][1]->m_distance);

    // Start cell distance should be reachable (not LARGEVAL)
    assert(maze.at(START_VROW, START_VCOL)->m_distance < LARGEVAL);
    printf("  [PASS] start cell is reachable, distance = %d\n",
           maze.at(START_VROW, START_VCOL)->m_distance);

    // ── Simulate speed run: greedy descent from start to exit ─────────────────
    printf("\n[SPEEDRUN] Following gradient from start to exit\n");

    vr  = START_VROW;
    vc  = START_VCOL;
    hdg = NORTH;
    steps = 0;

    while (steps < 32) {
        Cell* cur = maze.at(vr, vc);
        if (!cur || cur->is_exit) {
            printf("  -> reached exit cell at virtual (%d,%d)!\n", vr, vc);
            break;
        }

        Direction next = get_smallest_neighbor_dir(cur, hdg);
        int nr2 = vr, nc2 = vc;
        moveCell(nr2, nc2, next);
        vr = nr2; vc = nc2;
        hdg = next;
        steps++;

        printf("  step %2d: %s → v(%d,%d)  dist=%d\n",
               steps,
               next == NORTH ? "N" : next == EAST ? "E" :
               next == SOUTH ? "S" : "W",
               vr, vc,
               maze.at(vr,vc) ? maze.at(vr,vc)->m_distance : -1);
    }

    // The speed run must terminate within INNER*INNER steps (worst case = all cells)
    assert(steps <= INNER * INNER);
    printf("  [PASS] speed run completed in %d steps\n", steps);

    // ── Return reflood test ───────────────────────────────────────────────────
    printf("\n[RETURN] Reflood toward start\n");
    mazeReflood(maze, true);
    assert(maze.at(START_VROW, START_VCOL)->m_distance == 0);
    printf("  [PASS] start cell distance = 0 after return reflood\n");

    printGrid(maze, START_VROW, START_VCOL, "return reflood gradient");

    printf("\n=== ALL TESTS PASSED ===\n\n");
    return 0;
}
