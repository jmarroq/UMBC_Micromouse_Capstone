#ifndef PLANNER_INTERFACE_H
#define PLANNER_INTERFACE_H

#include "direction.h"

// Initialize maze planner
void plannerInit();

// Update the maze with the robot's current sensed walls
void plannerUpdateCurrentCell();

// Ask the maze for the best next direction
Direction plannerChooseNextDirection();

#endif