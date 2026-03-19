#include <Arduino.h>

#include "sensors.h"
#include "perception.h"
#include "wall_detector.h"       // ← new
#include "planner_interface.h"   // ← exposes wallDetector extern
#include "robot_state.h"
#include "executor.h"
#include "encoders.h"
#include "motors.h"

/* ================= TIMING ================= */

static const unsigned long SENSOR_PERIOD_MS  = 20;
static const unsigned long CONTROL_PERIOD_MS = 10;
static const unsigned long DEBUG_PERIOD_MS   = 500;

static unsigned long lastSensorTime  = 0;
static unsigned long lastControlTime = 0;
static unsigned long lastDebugTime   = 0;

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

// BUG FIX: original test checked executorDone() in the same loop() iteration
// that executorUpdate() sets stepDoneFlag = true, which is fine. But it also
// checked executorGetCurrentPrimitive() == PRIM_IDLE as a gate — that's
// redundant with executorDone() and can miss the transition frame.
// Cleaner: gate only on executorDone() and a local "stepWasStarted" flag.
static bool stepStarted = false;

/* ================= TASK: SENSORS (every 20ms) ================= */

void taskSensors() {
  if (sensors.update20ms()) {
    sensors.getLatest(robotState.sensorFrame);

    // Still compute the fast perceptionFrame for motion_control wall-following
    robotState.perceptionFrame = computePerception(robotState.sensorFrame);

    // Feed the vote window — this is what plannerUpdateCurrentCell will use
    wallDetector.update(robotState.sensorFrame);
  }
}

/* ================= TASK: CONTROL (every 10ms) ================= */

void taskControl() {
  executorUpdate();
}

/* ================= TASK: PLANNER (runs on executorDone event) ================= */

// Called once per completed step — not on a fixed timer.
// This is the correct place for plannerUpdateCurrentCell() because:
//   1. The robot is stationary at cell center — sensor readings are stable.
//   2. wallDetector has had 20ms * VOTE_WINDOW frames to settle.
//   3. We are NOT inside the 10ms control ISR.
void taskPlanner() {
  switch (testState) {

    case TEST_START_FORWARD:
      if (!stepStarted) {
        Serial.println("TEST: begin step toward NORTH");
        executorBeginStep(NORTH);
        stepStarted = true;
        testState = TEST_WAIT_FORWARD_DONE;
      }
      break;

    case TEST_WAIT_FORWARD_DONE:
      if (executorDone()) {
        // ── Commit walls now — robot is at cell center, detector is settled ──
        plannerUpdateCurrentCell();

        Serial.print("TEST: forward done, pose = (");
        Serial.print(robotState.x);
        Serial.print(", ");
        Serial.print(robotState.y);
        Serial.print("), heading = ");
        Serial.println((int)robotState.heading);

        stepStarted = false;
        testState = TEST_START_TURN_AROUND_STEP;
      }
      break;

    case TEST_START_TURN_AROUND_STEP:
      if (!stepStarted) {
        Serial.println("TEST: begin step toward SOUTH");
        executorBeginStep(SOUTH);  // NORTH → turn around → forward
        stepStarted = true;
        testState = TEST_WAIT_TURN_AROUND_STEP_DONE;
      }
      break;

    case TEST_WAIT_TURN_AROUND_STEP_DONE:
      if (executorDone()) {
        plannerUpdateCurrentCell();

        Serial.print("TEST: turn-around step done, pose = (");
        Serial.print(robotState.x);
        Serial.print(", ");
        Serial.print(robotState.y);
        Serial.print("), heading = ");
        Serial.println((int)robotState.heading);

        stepStarted = false;
        testState = TEST_FINISHED;
      }
      break;

    case TEST_FINISHED:
      stopMotors();
      break;

    default:
      break;
  }
}

/* ================= TASK: DEBUG (every 500ms) ================= */

void taskDebug() {
  // Sensor raw values
  Serial.print("SENS L=");
  Serial.print(robotState.sensorFrame.left_mm);
  Serial.print(" F=");
  Serial.print(robotState.sensorFrame.front_mm);
  Serial.print(" R=");
  Serial.print(robotState.sensorFrame.right_mm);

  // Raw perception (fast, single-frame)
  Serial.print(" | raw WL=");
  Serial.print(robotState.perceptionFrame.wall_left);
  Serial.print(" WF=");
  Serial.print(robotState.perceptionFrame.wall_front);
  Serial.print(" WR=");
  Serial.print(robotState.perceptionFrame.wall_right);

  // Vote window state (what will actually be written to maze map)
  uint8_t vl, vf, vr;
  wallDetector.getVoteCounts(vl, vf, vr);
  Serial.print(" | votes L=");
  Serial.print(vl);
  Serial.print("/");
  Serial.print(VOTE_WINDOW);
  Serial.print(" F=");
  Serial.print(vf);
  Serial.print("/");
  Serial.print(VOTE_WINDOW);
  Serial.print(" R=");
  Serial.print(vr);
  Serial.print("/");
  Serial.print(VOTE_WINDOW);
  Serial.print(" ready=");
  Serial.print(wallDetector.getFrame().ready);

  // Pose
  Serial.print(" | pose (");
  Serial.print(robotState.x);
  Serial.print(",");
  Serial.print(robotState.y);
  Serial.print(") hdg=");
  Serial.println((int)robotState.heading);
}

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Boot");

  motorsInit();
  encodersInit();
  robotStateInit(robotState);
  executorInit();
  plannerInit();   // ← added: initializes maze + wallDetector

  if (!sensors.begin()) {
    Serial.println("Sensors begin failed");
    while (1) { delay(100); }
  }

  Serial.println("Setup complete");
}

/* ================= LOOP ================= */

void loop() {
  unsigned long now = millis();

  // Sensor task — drives wallDetector.update() internally
  if (now - lastSensorTime >= SENSOR_PERIOD_MS) {
    lastSensorTime = now;
    taskSensors();
  }

  // Control task — runs executorUpdate() at 10ms
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    lastControlTime = now;
    taskControl();
  }

  // Planner task — event-driven, not timer-driven
  taskPlanner();

  // Debug task
  if (now - lastDebugTime >= DEBUG_PERIOD_MS) {
    lastDebugTime = now;
    taskDebug();
  }
}
