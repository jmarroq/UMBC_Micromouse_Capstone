#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Arduino.h>
#include "Sensors.h"
#include "perception.h"

enum SequenceState {
  SEQ_START_FORWARD,
  SEQ_RUN_FORWARD,
  SEQ_WAIT_AFTER_FORWARD,
  SEQ_START_TURN_AROUND,
  SEQ_RUN_TURN_AROUND,
  SEQ_WAIT_AFTER_TURN_AROUND
};

struct RobotState {
  SensorsFrame sensorFrame;
  PerceptionFrame perceptionFrame;

  SequenceState sequenceState;
  unsigned long waitStart;
};

void robotStateInit(RobotState& state);
extern RobotState robotState;

#endif