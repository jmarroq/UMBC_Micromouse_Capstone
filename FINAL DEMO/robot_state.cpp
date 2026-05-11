#include "robot_state.h"
#include "maze.h"

RobotState robotState;

void robotStateInit(RobotState& state) {
  state.sensorFrame = {};
  state.perceptionFrame = {};

  state.x = START_X;
  state.y = START_Y;
  state.heading = NORTH;
}