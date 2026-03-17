#include "robot_state.h"

RobotState robotState;

void robotStateInit(RobotState& state) {
  state.sensorFrame = {};
  state.perceptionFrame = {};

  state.x = 0;
  state.y = 0;
  state.heading = NORTH;
}