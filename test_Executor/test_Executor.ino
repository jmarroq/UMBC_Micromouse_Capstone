#include <Arduino.h>

#include "Sensors.h"
#include "perception.h"
#include "robot_state.h"
#include "executor.h"
#include "encoders.h"
#include "motors.h"

/* ================= GLOBALS ================= */

Sensors sensors;

/* ================= TEST STATE ================= */

enum TestState {
  TEST_START_FORWARD,
  TEST_WAIT_FORWARD_DONE,
  TEST_START_TURN_AROUND_STEP,
  TEST_WAIT_TURN_AROUND_STEP_DONE,
  TEST_FINISHED
};

static TestState testState = TEST_START_FORWARD;

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Boot");

  motorsInit();
  encodersInit();
  robotStateInit(robotState);
  executorInit();

  if (!sensors.begin()) {
    Serial.println("Sensors begin failed");
    while (1) { delay(100); }
  }

  Serial.println("Setup complete");
}

/* ================= LOOP ================= */

void loop() {
  /* ---------- Update sensing ---------- */
  if (sensors.update20ms()) {
    sensors.getLatest(robotState.sensorFrame);
    robotState.perceptionFrame = computePerception(robotState.sensorFrame);

    Serial.print("SENS L=");
    Serial.print(robotState.sensorFrame.left_mm);
    Serial.print(" F=");
    Serial.print(robotState.sensorFrame.front_mm);
    Serial.print(" R=");
    Serial.print(robotState.sensorFrame.right_mm);
    Serial.print(" | WL=");
    Serial.print(robotState.perceptionFrame.wall_left);
    Serial.print(" WF=");
    Serial.print(robotState.perceptionFrame.wall_front);
    Serial.print(" WR=");
    Serial.println(robotState.perceptionFrame.wall_right);
  }

  /* ---------- Test harness ---------- */
  switch (testState) {
    case TEST_START_FORWARD:
      if (executorGetCurrentPrimitive() == PRIM_IDLE && !executorDone()) {
        Serial.println("TEST: begin step toward NORTH");
        executorBeginStep(DIR_NORTH);   // heading starts NORTH, so this should just go forward
        testState = TEST_WAIT_FORWARD_DONE;
      }
      break;

    case TEST_WAIT_FORWARD_DONE:
      if (executorDone()) {
        Serial.print("TEST: forward done, pose = (");
        Serial.print(robotState.x);
        Serial.print(", ");
        Serial.print(robotState.y);
        Serial.print("), heading = ");
        Serial.println((int)robotState.heading);

        testState = TEST_START_TURN_AROUND_STEP;
      }
      break;

    case TEST_START_TURN_AROUND_STEP:
      if (executorGetCurrentPrimitive() == PRIM_IDLE) {
        Serial.println("TEST: begin step toward SOUTH");
        executorBeginStep(DIR_SOUTH);   // from NORTH, should turn around then go forward
        testState = TEST_WAIT_TURN_AROUND_STEP_DONE;
      }
      break;

    case TEST_WAIT_TURN_AROUND_STEP_DONE:
      if (executorDone()) {
        Serial.print("TEST: turn-around step done, pose = (");
        Serial.print(robotState.x);
        Serial.print(", ");
        Serial.print(robotState.y);
        Serial.print("), heading = ");
        Serial.println((int)robotState.heading);

        testState = TEST_FINISHED;
      }
      break;

    case TEST_FINISHED:
      stopMotors();
      break;

    default:
      break;
  }

  /* ---------- Always update executor ---------- */
  executorUpdate();
}