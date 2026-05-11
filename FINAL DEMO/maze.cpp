#include "maze.h"

/* ================= DIRECTION HELPERS ================= */

Direction oppositeDir(Direction d) {
  return (Direction)(((int)d + 2) % 4);
}

Direction leftOf(Direction d) {
  return (Direction)(((int)d + 3) % 4);
}

Direction rightOf(Direction d) {
  return (Direction)(((int)d + 1) % 4);
}

/* ================= MAZE CONSTRUCTOR ================= */

Maze::Maze() {
  for (int x = 0; x < SIZE; x++) {
    for (int y = 0; y < SIZE; y++) {
      map[x][y] = new Cell();
      map[x][y]->x = x;
      map[x][y]->y = y;
      map[x][y]->m_distance = LARGEVAL;
    }
  }

  for (int x = 0; x < SIZE; x++) {
    for (int y = 0; y < SIZE; y++) {
      Cell* c = map[x][y];

      c->north = (y < SIZE - 1) ? map[x][y + 1] : nullptr;
      c->east  = (x < SIZE - 1) ? map[x + 1][y] : nullptr;
      c->south = (y > 0)        ? map[x][y - 1] : nullptr;
      c->west  = (x > 0)        ? map[x - 1][y] : nullptr;
    }
  }

  // Assume the inner 4x4 boundary is closed at the beginning.
  for (int x = INNER_MIN; x <= INNER_MAX; x++) {
    closeWall(map[x][INNER_MIN], SOUTH);
    closeWall(map[x][INNER_MAX], NORTH);
  }

  for (int y = INNER_MIN; y <= INNER_MAX; y++) {
    closeWall(map[INNER_MIN][y], WEST);
    closeWall(map[INNER_MAX][y], EAST);
  }
}

/* ================= DESTRUCTOR ================= */

Maze::~Maze() {
  for (int x = 0; x < SIZE; x++) {
    for (int y = 0; y < SIZE; y++) {
      delete map[x][y];
      map[x][y] = nullptr;
    }
  }
}

/* ================= BASIC ACCESS ================= */

Cell* Maze::cellAt(int x, int y) {
  if (x < 0 || x >= SIZE || y < 0 || y >= SIZE) return nullptr;
  return map[x][y];
}

Cell* Maze::neighbor(Cell* c, Direction dir) const {
  if (!c) return nullptr;

  switch (dir) {
    case NORTH: return c->north;
    case EAST:  return c->east;
    case SOUTH: return c->south;
    case WEST:  return c->west;
  }

  return nullptr;
}

bool Maze::hasOpenWall(Cell* c, Direction dir) const {
  return neighbor(c, dir) != nullptr;
}

/* ================= WALL CONTROL ================= */

void Maze::closeWall(Cell* c, Direction dir) {
  if (!c) return;

  Cell* n = neighbor(c, dir);

  switch (dir) {
    case NORTH: c->north = nullptr; break;
    case EAST:  c->east  = nullptr; break;
    case SOUTH: c->south = nullptr; break;
    case WEST:  c->west  = nullptr; break;
  }

  if (!n) return;

  switch (oppositeDir(dir)) {
    case NORTH: n->north = nullptr; break;
    case EAST:  n->east  = nullptr; break;
    case SOUTH: n->south = nullptr; break;
    case WEST:  n->west  = nullptr; break;
  }
}

void Maze::openWall(Cell* c, Direction dir) {
  if (!c) return;

  int nx = c->x;
  int ny = c->y;

  switch (dir) {
    case NORTH: ny++; break;
    case EAST:  nx++; break;
    case SOUTH: ny--; break;
    case WEST:  nx--; break;
  }

  Cell* n = cellAt(nx, ny);
  if (!n) return;

  switch (dir) {
    case NORTH: c->north = n; break;
    case EAST:  c->east  = n; break;
    case SOUTH: c->south = n; break;
    case WEST:  c->west  = n; break;
  }

  switch (oppositeDir(dir)) {
    case NORTH: n->north = c; break;
    case EAST:  n->east  = c; break;
    case SOUTH: n->south = c; break;
    case WEST:  n->west  = c; break;
  }
}

/* ================= CELL TYPE CHECKS ================= */

bool Maze::isInnerCell(int x, int y) const {
  return x >= INNER_MIN && x <= INNER_MAX &&
         y >= INNER_MIN && y <= INNER_MAX;
}

bool Maze::isOuterCell(int x, int y) const {
  return x == 0 || x == SIZE - 1 || y == 0 || y == SIZE - 1;
}

bool Maze::isInnerBoundaryFace(int x, int y, Direction dir) const {
  if (!isInnerCell(x, y)) return false;

  if (x == INNER_MIN && dir == WEST)  return true;
  if (x == INNER_MAX && dir == EAST)  return true;
  if (y == INNER_MIN && dir == SOUTH) return true;
  if (y == INNER_MAX && dir == NORTH) return true;

  return false;
}

/* ================= FLOODFILL ================= */

void Maze::resetDistances() {
  for (int x = 0; x < SIZE; x++) {
    for (int y = 0; y < SIZE; y++) {
      map[x][y]->m_distance = LARGEVAL;
      map[x][y]->is_goal = false;
    }
  }
}

void Maze::computeFloodFrom(Cell* goal) {
  if (!goal) return;

  resetDistances();

  goal->m_distance = 0;
  goal->is_goal = true;

  std::queue<Cell*> q;
  q.push(goal);

  while (!q.empty()) {
    Cell* c = q.front();
    q.pop();

    Direction dirs[4] = { NORTH, EAST, SOUTH, WEST };

    for (int i = 0; i < 4; i++) {
      Cell* n = neighbor(c, dirs[i]);

      if (!n) continue;

      int16_t newDist = c->m_distance + 1;

      if (newDist < n->m_distance) {
        n->m_distance = newDist;
        q.push(n);
      }
    }
  }
}