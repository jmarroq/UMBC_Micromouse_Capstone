#include "planner_interface.h"
#include "maze.h"
#include "robot_state.h"
#include "wall_detector.h"

/* ================= GLOBAL MAZE + DETECTOR ================= */

static Maze maze;

// One WallDetector instance — shared between plannerUpdateCurrentCell
// and the sensor task that calls wallDetector.update().
// Declared extern in planner_interface.h so test_Executor.ino can call update().
WallDetector wallDetector;

/* ================= COORDINATE HELPERS ================= */

// Robot pose: (x,y) origin bottom-left, y increases upward.
// Maze array: map[row][col], row 0 = top, increases downward.
static int rowFromY(int y) { return (SIZE - 1) - y; }
static int colFromX(int x) { return x; }

static Cell* cellAt(int x, int y) {
  int row = rowFromY(y);
  int col = colFromX(x);
  if (row < 0 || row >= SIZE || col < 0 || col >= SIZE) return nullptr;
  return maze.map[row][col];
}

static Direction turnLeft(Direction d) {
  return static_cast<Direction>((static_cast<int>(d) + 3) % 4);
}

static Direction turnRight(Direction d) {
  return static_cast<Direction>((static_cast<int>(d) + 1) % 4);
}

static Direction oppositeDir(Direction d) {
  return static_cast<Direction>((static_cast<int>(d) + 2) % 4);
}

static Cell* neighborInDirection(int x, int y, Direction dir) {
  switch (dir) {
    case NORTH: return cellAt(x, y + 1);
    case EAST:  return cellAt(x + 1, y);
    case SOUTH: return cellAt(x, y - 1);
    case WEST:  return cellAt(x - 1, y);
    default:    return nullptr;
  }
}

static void setWallBidirectional(int x, int y, Direction dir) {
  Cell* a = cellAt(x, y);
  if (!a) return;
  set_wall(a, dir);
  Cell* b = neighborInDirection(x, y, dir);
  if (b) set_wall(b, oppositeDir(dir));
}

/* ================= PUBLIC FUNCTIONS ================= */

void plannerInit() {
  wallDetector.reset();
  // Maze constructor allocates and wires all cells — nothing else needed.
}

// ─────────────────────────────────────────────────────────────────────────────
// plannerUpdateCurrentCell
//
// THIS IS THE ONLY PLACE WALLS ARE COMMITTED TO THE MAZE MAP.
//
// Design contract:
//   • Only runs when the executor signals the robot has reached cell center
//     (i.e. after executorDone() returns true, before executorBeginStep()).
//   • Uses ValidatedWallFrame from wallDetector — NOT the raw perceptionFrame.
//   • Only writes walls when frame.ready == true (VOTE_WINDOW frames collected).
//   • set_wall() is irreversible — one bad write permanently corrupts floodfill.
//
// Caller (test_Executor.ino / main loop):
//   if (executorDone()) {
//     plannerUpdateCurrentCell();   // ← walls committed here, safe
//     Direction next = plannerChooseNextDirection();
//     executorBeginStep(next);
//   }
// ─────────────────────────────────────────────────────────────────────────────
void plannerUpdateCurrentCell() {
  int x = robotState.x;
  int y = robotState.y;
  Direction heading = robotState.heading;

  Cell* cur = cellAt(x, y);
  if (!cur) return;

  // ── Guard: only commit walls when the vote window is fully settled ─────────
  const ValidatedWallFrame& vf = wallDetector.getFrame();

  if (!vf.ready) {
    // Not enough history yet — mark visited but don't write walls.
    // Floodfill will retry once we have a confident reading.
    Serial.println("PLANNER: wall frame not ready, skipping wall commit");
    set_visited(cur);
    return;
  }

  // ── Commit confirmed walls ─────────────────────────────────────────────────
  // WALL_YES = confident wall.  WALL_NO or WALL_UNSURE = do not write.
  // We never write "no wall" — the maze starts fully open; walls are only added.

  if (vf.front == ValidatedWallFrame::WALL_YES) {
    setWallBidirectional(x, y, heading);
    Serial.println("PLANNER: wall FRONT committed");
  }

  if (vf.left == ValidatedWallFrame::WALL_YES) {
    setWallBidirectional(x, y, turnLeft(heading));
    Serial.println("PLANNER: wall LEFT committed");
  }

  if (vf.right == ValidatedWallFrame::WALL_YES) {
    setWallBidirectional(x, y, turnRight(heading));
    Serial.println("PLANNER: wall RIGHT committed");
  }

  set_visited(cur);

  // ── Reflood from current cell ──────────────────────────────────────────────
  CellStack stk;
  stk.push(cur);
  while (!stk.empty()) {
    Cell* n = stk.top();
    stk.pop();
    if (n) flood_fill(n, stk, false);
  }
}

Direction plannerChooseNextDirection() {
  int x = robotState.x;
  int y = robotState.y;

  Cell* cur = cellAt(x, y);
  if (!cur) return robotState.heading;

  return get_smallest_neighbor_dir(cur, robotState.heading);
}
