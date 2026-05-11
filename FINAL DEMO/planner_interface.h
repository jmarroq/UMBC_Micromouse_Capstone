#ifndef PLANNER_INTERFACE_H
#define PLANNER_INTERFACE_H

#include "direction.h"

void plannerInit();

void plannerUpdateCurrentCell();

Direction plannerChooseNextDirection();

bool plannerGoalKnown();

void plannerDebugPrint();

// NEW mission routing functions
void plannerRouteToStart();
void plannerRouteToKnownExit();

#endif