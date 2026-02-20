#ifndef INTERFACE_H
#define INTERFACE_H

#include "Maze.h"

// Direction enum must match what Maze code expects
#ifndef NORTH
enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
#endif

typedef struct {
  int x;        // MMS x coordinate (0..SIZE-1)
  int y;        // MMS y coordinate (0..SIZE-1)
  int heading;  // NORTH/EAST/SOUTH/WEST
} Pose;

// 1) Pull wall data from simulator and update internal Maze model.
void iface_sense_and_update(Maze* maze, const Pose* pose);

// 2) Run reflood (stack-based propagation) starting from current cell.
void iface_reflood_from_current(Maze* maze, const Pose* pose, int reflood_flag);

// 3) Use your floodfill policy to select the next absolute direction.
int iface_choose_next_dir(Maze* maze, const Pose* pose);

// 4) Execute one step (rotate + move) in the simulator and update pose.
void iface_execute_step(Pose* pose, int target_dir);

#endif
