#include "planner_interface.h"

#include "Maze.h"
#include "robot_state.h"

/* ================= GLOBAL MAZE ================= */

static Maze maze;

/* ================= HELPERS ================= */

// Robot / planner coordinates:
// robotState uses (x,y) with origin at bottom-left, y increasing upward
// Maze array uses [row][col] with row 0 at top

static int rowFromY(int y) {
  return (SIZE - 1) - y;
}

static int colFromX(int x) {
  return x;
}

static Cell* cellAt(int x, int y) {
  int row = rowFromY(y);
  int col = colFromX(x);

  if (row < 0 || row >= SIZE || col < 0 || col >= SIZE) {
    return nullptr;
  }

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
  if (b) {
    set_wall(b, oppositeDir(dir));
  }
}

/* ================= PUBLIC FUNCTIONS ================= */

void plannerInit() {
  // Maze constructor already allocates and wires cells.
  // Nothing else needed here for now.
}

void plannerUpdateCurrentCell() {
  int x = robotState.x;
  int y = robotState.y;
  Direction heading = robotState.heading;

  Cell* cur = cellAt(x, y);
  if (!cur) return;

  const bool wf = robotState.perceptionFrame.wall_front;
  const bool wl = robotState.perceptionFrame.wall_left;
  const bool wr = robotState.perceptionFrame.wall_right;

  if (wf) {
    setWallBidirectional(x, y, heading);
  }

  if (wl) {
    setWallBidirectional(x, y, turnLeft(heading));
  }

  if (wr) {
    setWallBidirectional(x, y, turnRight(heading));
  }

  set_visited(cur);

  CellStack stk;
  stk.push(cur);

  while (!stk.empty()) {
    Cell* n = stk.top();
    stk.pop();

    if (n) {
      flood_fill(n, stk, false);
    }
  }
}

Direction plannerChooseNextDirection() {
  int x = robotState.x;
  int y = robotState.y;

  Cell* cur = cellAt(x, y);
  if (!cur) {
    return robotState.heading;
  }

  return get_smallest_neighbor_dir(cur, robotState.heading);
}