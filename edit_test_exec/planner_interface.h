#ifndef PLANNER_INTERFACE_H
#define PLANNER_INTERFACE_H

#include "direction.h"
#include "wall_detector.h"

// The single WallDetector instance — defined in planner_interface.cpp.
// test_Executor.ino calls wallDetector.update(sensorFrame) inside taskSensors.
extern WallDetector wallDetector;

// Initialize maze planner and wall detector
void plannerInit();

// Commit walls to maze map for the current cell (only call at cell center,
// after executorDone() is true). Uses wallDetector.getFrame() internally.
void plannerUpdateCurrentCell();

// Ask the floodfill for the best next absolute direction
Direction plannerChooseNextDirection();

#endif
