#include <Arduino.h>

#include "Sensors.h"
#include "perception.h"
#include "robot_state.h"
#include "planner_interface.h"
#include "executor.h"
#include "encoders.h"
#include "motors.h"

/* ================= GLOBAL OBJECTS ================= */

Sensors sensors;

/* ================= LOCAL FLAGS ================= */

static bool haveSensorFrame = false;

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Micromouse boot");

  motorsInit();
  encodersInit();
  robotStateInit(robotState);
  plannerInit();
  executorInit();

  if (!sensors.begin()) {
    Serial.println("ERROR: sensors.begin() failed");
    while (1) {
      delay(100);
    }
  }

  Serial.println("Setup complete");
}

/* ================= LOOP ================= */

void loop() {
  /* ---------- Update sensors / perception ---------- */
  if (sensors.update20ms()) {
    sensors.getLatest(robotState.sensorFrame);
    robotState.perceptionFrame = computePerception(robotState.sensorFrame);
    haveSensorFrame = true;

    Serial.print("SENS  L=");
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

  /* ---------- Plan next step when executor is idle ---------- */
  if (haveSensorFrame && executorGetCurrentPrimitive() == PRIM_IDLE) {
    plannerUpdateCurrentCell();

    Direction nextDir = plannerChooseNextDirection();

    Serial.print("PLAN nextDir = ");
    Serial.println((int)nextDir);

    executorBeginStep(nextDir);
  }

  /* ---------- Always advance active step ---------- */
  executorUpdate();
}