#include "planner_interface.h"
#include "maze.h"
#include "robot_state.h"
#include "Arduino.h"

/* ================= GLOBAL MAZE ================= */

static Maze maze;

static bool goalKnown = false;
static Cell* goalCell = nullptr;
static constexpr uint16_t EXIT_OPEN_CONFIRM_MM = 80;

/* ================= HELPERS ================= */

static Cell* currentCell() {
  return maze.cellAt(robotState.x, robotState.y);
}

static Direction sensorDirFront(Direction h) {
  return h;
}

static Direction sensorDirLeft(Direction h) {
  return leftOf(h);
}

static Direction sensorDirRight(Direction h) {
  return rightOf(h);
}

void plannerRouteToStart() {
  Cell* start = maze.cellAt(START_X, START_Y);

  if (!start) {
    Serial.println("ERROR: plannerRouteToStart failed, start cell missing");
    return;
  }

  maze.computeFloodFrom(start);

  Serial.println("Planner route target: START");
}

void plannerRouteToKnownExit() {
  if (!goalKnown || !goalCell) {
    Serial.println("ERROR: plannerRouteToKnownExit failed, goal unknown");
    return;
  }

  maze.computeFloodFrom(goalCell);

  Serial.println("Planner route target: KNOWN EXIT");
}

static void markWallFromSensor(
  Cell* cur,
  Direction dir,
  bool sensorValid,
  uint16_t sensorMm,
  bool wallDetected
) {
  if (!cur) return;

  // If sensor is invalid, do NOT map anything.
  // Invalid should never mean open.
  if (!sensorValid) {
    Serial.println("Planner skipped invalid sensor reading");
    return;
  }

  bool boundaryFace = maze.isInnerBoundaryFace(cur->x, cur->y, dir);

  // Normal wall mapping
  if (wallDetected) {
    maze.closeWall(cur, dir);
    return;
  }

  // Exit detection only happens on inner boundary faces.
  // It must be a valid and clearly far/open reading.
  if (boundaryFace && !goalKnown && sensorMm >= EXIT_OPEN_CONFIRM_MM) {
    maze.openWall(cur, dir);

    goalCell = maze.neighbor(cur, dir);

    if (goalCell) {
      goalCell->is_exit = true;
      goalKnown = true;
      maze.computeFloodFrom(goalCell);

      Serial.println("EXIT FOUND");
      Serial.print("Exit from cell: (");
      Serial.print(cur->x);
      Serial.print(", ");
      Serial.print(cur->y);
      Serial.print(") dir=");
      Serial.println((int)dir);

      Serial.print("Goal outer cell: (");
      Serial.print(goalCell->x);
      Serial.print(", ");
      Serial.print(goalCell->y);
      Serial.println(")");
    }

    return;
  }

  // If open but not boundary, leave pointer open.
}

static Direction chooseExploreDirection(Cell* cur) {
  Direction h = robotState.heading;

  Direction options[4] = {
    leftOf(h),
    h,
    rightOf(h),
    oppositeDir(h)
  };

  for (int i = 0; i < 4; i++) {
    Direction d = options[i];

    if (!maze.hasOpenWall(cur, d)) continue;

    Cell* n = maze.neighbor(cur, d);
    if (!n) continue;

    if (maze.isInnerCell(n->x, n->y) && !n->visited) {
      return d;
    }
  }

  for (int i = 0; i < 4; i++) {
    Direction d = options[i];

    if (!maze.hasOpenWall(cur, d)) continue;

    Cell* n = maze.neighbor(cur, d);
    if (!n) continue;

    if (maze.isInnerCell(n->x, n->y)) {
      return d;
    }
  }

  return oppositeDir(h);
}

static Direction chooseFloodDirection(Cell* cur) {
  Direction h = robotState.heading;

  Direction options[4] = {
    h,
    leftOf(h),
    rightOf(h),
    oppositeDir(h)
  };

  Direction bestDir = h;
  int16_t bestDist = LARGEVAL;

  for (int i = 0; i < 4; i++) {
    Direction d = options[i];

    if (!maze.hasOpenWall(cur, d)) continue;

    Cell* n = maze.neighbor(cur, d);
    if (!n) continue;

    if (n->m_distance < bestDist) {
      bestDist = n->m_distance;
      bestDir = d;
    }
  }

  return bestDir;
}

/* ================= PUBLIC FUNCTIONS ================= */

void plannerInit() {
  goalKnown = false;
  goalCell = nullptr;

  Serial.println("Planner initialized for 6x6 virtual / 4x4 effective maze");
}

void plannerUpdateCurrentCell() {
  Cell* cur = currentCell();
  if (!cur) return;

  Direction h = robotState.heading;

  markWallFromSensor(
    cur,
    sensorDirFront(h),
    robotState.sensorFrame.front_valid,
    robotState.sensorFrame.front_mm,
    robotState.perceptionFrame.wall_front
  );

  markWallFromSensor(
    cur,
    sensorDirLeft(h),
    robotState.sensorFrame.left_valid,
    robotState.sensorFrame.left_mm,
    robotState.perceptionFrame.wall_left
  );

  markWallFromSensor(
    cur,
    sensorDirRight(h),
    robotState.sensorFrame.right_valid,
    robotState.sensorFrame.right_mm,
    robotState.perceptionFrame.wall_right
  );

  cur->visited = TRUE;

  if (goalKnown && goalCell) {
    maze.computeFloodFrom(goalCell);
  }
}

Direction plannerChooseNextDirection() {
  Cell* cur = currentCell();

  if (!cur) {
    return robotState.heading;
  }

  if (goalKnown) {
    return chooseFloodDirection(cur);
  }

  return chooseExploreDirection(cur);
}

bool plannerGoalKnown() {
  return goalKnown;
}

void plannerDebugPrint() {
  Serial.println();
  Serial.println("MAPPED MAZE FROM PLANNER CELLS");
  Serial.println("| = vertical wall, ___ = horizontal wall");
  Serial.println("? = unknown edge, R = robot, v = visited, G = discovered exit");

  auto physicalNeighbor = [&](int x, int y, Direction d) -> Cell* {
    if (d == NORTH) return maze.cellAt(x, y + 1);
    if (d == EAST)  return maze.cellAt(x + 1, y);
    if (d == SOUTH) return maze.cellAt(x, y - 1);
    if (d == WEST)  return maze.cellAt(x - 1, y);
    return nullptr;
  };

  auto cellVisited = [&](int x, int y) {
    Cell* c = maze.cellAt(x, y);
    return c && c->visited;
  };

  auto edgeKnown = [&](int x, int y, Direction d) {
    // Inner boundary walls are assumed known from the start.
    if (maze.isInnerBoundaryFace(x, y, d)) return true;

    // If current cell was visited, its sensed left/front/right walls are known.
    if (cellVisited(x, y)) return true;

    // Use physical neighbor, NOT pointer neighbor.
    // Pointer neighbor may be nullptr if wall was mapped.
    Cell* n = physicalNeighbor(x, y, d);
    if (n && n->visited) return true;

    return false;
  };

  auto edgeWall = [&](int x, int y, Direction d) {
    Cell* c = maze.cellAt(x, y);
    if (!c) return true;

    // If planner pointer is missing, planner believes wall exists.
    return !maze.hasOpenWall(c, d);
  };

  for (int y = INNER_MAX; y >= INNER_MIN; y--) {
    // North horizontal edges
    for (int x = INNER_MIN; x <= INNER_MAX; x++) {
      Serial.print("+");

      if (!edgeKnown(x, y, NORTH)) {
        Serial.print("???");
      } else {
        Serial.print(edgeWall(x, y, NORTH) ? "___" : "   ");
      }
    }
    Serial.println("+");

    // West/East vertical edges and cell contents
    for (int x = INNER_MIN; x <= INNER_MAX; x++) {
      if (!edgeKnown(x, y, WEST)) {
        Serial.print("?");
      } else {
        Serial.print(edgeWall(x, y, WEST) ? "|" : " ");
      }

      Cell* c = maze.cellAt(x, y);

      if (robotState.x == x && robotState.y == y) {
        Serial.print(" R ");
      }
      else if (c && c->is_exit) {
        Serial.print(" G ");
      }
      else if (c && c->visited) {
        Serial.print(" v ");
      }
      else {
        Serial.print("   ");
      }
    }

    if (!edgeKnown(INNER_MAX, y, EAST)) {
      Serial.print("?");
    } else {
      Serial.print(edgeWall(INNER_MAX, y, EAST) ? "|" : " ");
    }

    Serial.println();
  }

  // South horizontal edges
  for (int x = INNER_MIN; x <= INNER_MAX; x++) {
    Serial.print("+");

    if (!edgeKnown(x, INNER_MIN, SOUTH)) {
      Serial.print("???");
    } else {
      Serial.print(edgeWall(x, INNER_MIN, SOUTH) ? "___" : "   ");
    }
  }
  Serial.println("+");

  Serial.print("Robot: (");
  Serial.print(robotState.x);
  Serial.print(", ");
  Serial.print(robotState.y);
  Serial.print("), heading=");
  Serial.print((int)robotState.heading);
  Serial.print(", goalKnown=");
  Serial.println(goalKnown ? "YES" : "NO");
}

void plannerReset() {
  static bool firstInit = true;

  if (!firstInit) {
    maze.~Maze();
    new (&maze) Maze();
  }

  firstInit = false;
  goalKnown = false;
  goalCell = nullptr;

  Serial.println("Planner reset for new test");
}
