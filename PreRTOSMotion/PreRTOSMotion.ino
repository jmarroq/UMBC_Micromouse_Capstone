#include "motion.h"
#include "motors.h"
#include "encoders.h"
#include "Sensors.h"
#include "perception.h"
#include "robot_state.h"

const unsigned long CONTROL_PERIOD_MS = 10;
const unsigned long SENSOR_PERIOD_MS  = 20;
const unsigned long PLAN_PERIOD_MS    = 20;
const unsigned long DEBUG_PERIOD_MS   = 500;

unsigned long lastControlTime = 0;
unsigned long lastSensorTime  = 0;
unsigned long lastPlanTime    = 0;
unsigned long lastDebugTime   = 0;

Sensors sensors(0x70);
RobotState robotState;

void taskSensors() {
  sensors.update20ms();
  sensors.getLatest(robotState.sensorFrame);
  robotState.perceptionFrame = computePerception(robotState.sensorFrame);
}

void taskControl() {
  motionUpdate();
}

void taskPlanner() {
  switch (robotState.sequenceState) {

    case SEQ_START_FORWARD:
      motionBeginPrimitive(PRIM_FORWARD_ONE_CELL);
      robotState.sequenceState = SEQ_RUN_FORWARD;
      break;

    case SEQ_RUN_FORWARD:
      if (motionDone()) {
        robotState.waitStart = millis();
        robotState.sequenceState = SEQ_WAIT_AFTER_FORWARD;
      }
      break;

    case SEQ_WAIT_AFTER_FORWARD:
      if (millis() - robotState.waitStart >= 3000) {
        robotState.sequenceState = SEQ_START_TURN_AROUND;
      }
      break;

    case SEQ_START_TURN_AROUND:
      motionBeginPrimitive(PRIM_TURN_AROUND);
      robotState.sequenceState = SEQ_RUN_TURN_AROUND;
      break;

    case SEQ_RUN_TURN_AROUND:
      if (motionDone()) {
        robotState.waitStart = millis();
        robotState.sequenceState = SEQ_WAIT_AFTER_TURN_AROUND;
      }
      break;

    case SEQ_WAIT_AFTER_TURN_AROUND:
      if (millis() - robotState.waitStart >= 3000) {
        robotState.sequenceState = SEQ_START_FORWARD;
      }
      break;
  }
}

void taskDebug() {
  Serial.print("L=");
  Serial.print(robotState.sensorFrame.left_mm);
  Serial.print(" F=");
  Serial.print(robotState.sensorFrame.front_mm);
  Serial.print(" R=");
  Serial.print(robotState.sensorFrame.right_mm);

  Serial.print(" | walls ");
  Serial.print(robotState.perceptionFrame.wall_left);
  Serial.print(",");
  Serial.print(robotState.perceptionFrame.wall_front);
  Serial.print(",");
  Serial.print(robotState.perceptionFrame.wall_right);

  Serial.print(" | seq=");
  Serial.println(robotState.sequenceState);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  motorsInit();
  encodersInit();
  motionInit();
  robotStateInit(robotState);

  if (!sensors.begin()) {
    Serial.println("Sensors begin failed");
    while (1) {}
  }

  Serial.println("SETUP DONE");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSensorTime >= SENSOR_PERIOD_MS) {
    lastSensorTime = now;
    taskSensors();
  }

  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    lastControlTime = now;
    taskControl();
  }

  if (now - lastPlanTime >= PLAN_PERIOD_MS) {
    lastPlanTime = now;
    taskPlanner();
  }

  if (now - lastDebugTime >= DEBUG_PERIOD_MS) {
    lastDebugTime = now;
    taskDebug();
  }
}