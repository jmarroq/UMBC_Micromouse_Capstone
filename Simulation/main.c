// main.c
// Controls: sense -> updates -> reflood -> direction -> move

#include "Maze.h"
#include "Interface.h"

int main(void) {

  // Initialize maze and mouse pose
  Maze* maze = new_Maze();
  Pose pose;
  pose.x = 0;
  pose.y = 0;
  pose.heading = NORTH;


  while (1) {
    iface_sense_and_update(maze, &pose);

    iface_reflood_from_current(maze, &pose, 0);

    int next_dir = iface_choose_next_dir(maze, &pose);

    int old_x = pose.x;
    int old_y = pose.y;

    iface_execute_step(&pose, next_dir);

    if (pose.x == old_x && pose.y == old_y) {
      iface_sense_and_update(maze, &pose);
      iface_reflood_from_current(maze, &pose, 0);
      next_dir = iface_choose_next_dir(maze, &pose);
      iface_execute_step(&pose, next_dir);
    }
  }

  return 0;
}
