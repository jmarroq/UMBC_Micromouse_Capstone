#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Arduino.h>
#include "Sensors.h"
#include "perception.h"
#include "direction.h"

struct RobotState {
  // Latest low-level sensing
  SensorsFrame sensorFrame;
  PerceptionFrame perceptionFrame;

  // Robot pose in maze coordinates
  int x;
  int y;
  Direction heading;
};

void robotStateInit(RobotState& state);
extern RobotState robotState;

#endif