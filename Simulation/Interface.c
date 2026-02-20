// Interface.c
// Glue layer between MMS (stdin/stdout API) and Maze/Floodfill core.

#include <stdio.h>
#include "Interface.h"
#include "Maze.h"
#include "Stack.h"
#include "API.h"

// ----------------- Direction helpers -----------------

static inline int turn_left_dir(int d)  { return (d + 3) & 3; }
static inline int turn_right_dir(int d) { return (d + 1) & 3; }
static inline int turn_back_dir(int d)  { return (d + 2) & 3; }

// ----------------- Coordinate mapping -----------------
// MMS: (0,0) bottom-left, y increases up
// Typical C 2D array: map[row][col] with row 0 at top, row increases down
static inline int I_from_Y(int y) { return (SIZE - 1) - y; }
static inline int J_from_X(int x) { return x; }

static inline Node* node_at(Maze* m, int x, int y) {
  return m->map[I_from_Y(y)][J_from_X(x)];
}

// ----------------- Bidirectional wall setting -----------------
// Floodfill checks expect BOTH sides to be blocked/open consistently.
// This wrapper updates both the current cell and the neighbor cell.

static void set_wall_bidir(Maze* m, int x, int y, int dir_abs) {
  Node* a = node_at(m, x, y);
  if (!a) return;

  switch (dir_abs) {
    case NORTH: {
      // wall between (x,y) and (x,y+1)
      set_wall(a, NORTH);
      if (y + 1 < SIZE) {
        Node* b = node_at(m, x, y + 1);
        if (b) set_wall(b, SOUTH);
      }
      API_setWall(x, y, 'n'); // visualization in MMS
    } break;

    case EAST: {
      set_wall(a, EAST);
      if (x + 1 < SIZE) {
        Node* b = node_at(m, x + 1, y);
        if (b) set_wall(b, WEST);
      }
      API_setWall(x, y, 'e');
    } break;

    case SOUTH: {
      set_wall(a, SOUTH);
      if (y - 1 >= 0) {
        Node* b = node_at(m, x, y - 1);
        if (b) set_wall(b, NORTH);
      }
      API_setWall(x, y, 's');
    } break;

    case WEST: {
      set_wall(a, WEST);
      if (x - 1 >= 0) {
        Node* b = node_at(m, x - 1, y);
        if (b) set_wall(b, EAST);
      }
      API_setWall(x, y, 'w');
    } break;
  }
}

// ----------------- Pose / motion helpers -----------------

static void rotate_to(Pose* pose, int target_dir) {
  //  turn right until heading matches.
  while (pose->heading != target_dir) {
    API_turnRight();
    pose->heading = turn_right_dir(pose->heading);
  }
}

static int move_forward_and_update_pose(Pose* pose) {
  int ok = API_moveForward();   // returns 1 if moved, 0 if hit wall
  if (!ok) return 0;

  if (pose->heading == NORTH) pose->y++;
  else if (pose->heading == EAST) pose->x++;
  else if (pose->heading == SOUTH) pose->y--;
  else if (pose->heading == WEST) pose->x--;

  return 1;
}

// ----------------- Interface functions -----------------

void iface_sense_and_update(Maze* maze, const Pose* pose) {
  // Read walls relative to current heading
  int wf = API_wallFront();
  int wl = API_wallLeft();
  int wr = API_wallRight();

  // Convert relative -> absolute and update Maze (bidirectional)
  if (wf) set_wall_bidir(maze, pose->x, pose->y, pose->heading);
  if (wl) set_wall_bidir(maze, pose->x, pose->y, turn_left_dir(pose->heading));
  if (wr) set_wall_bidir(maze, pose->x, pose->y, turn_right_dir(pose->heading));

  // Mark visited your internal map
  Node* cur = node_at(maze, pose->x, pose->y);
  set_visited(cur);

}

void iface_reflood_from_current(Maze* maze, const Pose* pose, int reflood_flag) {
  Node* cur = node_at(maze, pose->x, pose->y);

  Stack* s = new_Stack();
  push(s, cur);

  while (!is_empty_Stack(s)) {
    Node* n = NULL;
    pop(s, &n);
    if (n) {
      flood_fill(n, s, (short)reflood_flag);
    }
  }

  delete_Stack(&s);
}


int iface_choose_next_dir(Maze* maze, const Pose* pose) {
  Node* cur = node_at(maze, pose->x, pose->y);

  // Prefer continuing forward (pose->heading)
  return (int)get_smallest_neighbor_dir(cur, (short)pose->heading);
}

void iface_execute_step(Pose* pose, int target_dir) {
  rotate_to(pose, target_dir);

  // Attempt forward move
  int ok = move_forward_and_update_pose(pose);

  // If crashed, we do NOT update pose. Marking the wall + reflood
  if (!ok) {
    // Optional debug
    // fprintf(stderr, "Bump! wall ahead at (%d,%d) heading=%d\n",
    //         pose->x, pose->y, pose->heading);
  }
}
