#include "robot_state.h"

void robotStateInit(RobotState& state) {
  state.sequenceState = SEQ_START_FORWARD;
  state.waitStart = 0;
  state.sensorFrame = {};
  state.perceptionFrame = {};
}