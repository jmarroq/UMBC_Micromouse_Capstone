#include <Arduino.h>

#include "Sensors.h"
#include "perception.h"
#include "robot_state.h"
#include "planner_interface.h"
#include "executor.h"
#include "encoders.h"
#include "motors.h"
#include "maze.h"

/* ================= GLOBAL OBJECTS ================= */

Sensors sensors;

/* ================= FLAGS ================= */

static bool haveSensorFrame = false;
static bool settling = false;

static uint32_t settleStartMs = 0;
static uint32_t raceDelayStartMs = 0;

static bool lastStepHadTurn = false;

/* ================= SETTINGS ================= */

static constexpr int LED_PIN = LED_BUILTIN;

static constexpr uint32_t EXPLORE_SETTLE_MS = 500;
static constexpr uint32_t RETURN_SETTLE_MS  = 500;
static constexpr uint32_t RACE_CORNER_SETTLE_MS = 50;
static constexpr uint32_t RACE_STRAIGHT_SETTLE_MS = 0;

static constexpr uint32_t DELAY_BEFORE_RACE_MS = 3000;

/* ================= MISSION MODE ================= */

enum MissionMode {
  MODE_EXPLORE_TO_EXIT,
  MODE_RETURN_TO_START,
  MODE_FACE_NORTH_AT_START,
  MODE_DELAY_BEFORE_RACE,
  MODE_RUN_TO_KNOWN_EXIT,
  MODE_FINISHED
};

static MissionMode missionMode = MODE_EXPLORE_TO_EXIT;

/* ================= HELPERS ================= */

static bool atStartCell() {
  return robotState.x == START_X && robotState.y == START_Y;
}

static bool outsideInnerMaze() {
  return robotState.x < INNER_MIN || robotState.x > INNER_MAX ||
         robotState.y < INNER_MIN || robotState.y > INNER_MAX;
}

static uint32_t settleTimeForCurrentMode() {
  if (missionMode == MODE_EXPLORE_TO_EXIT) {
    return EXPLORE_SETTLE_MS;
  }

  if (missionMode == MODE_RETURN_TO_START) {
    return RETURN_SETTLE_MS;
  }

  if (missionMode == MODE_RUN_TO_KNOWN_EXIT) {
    if (lastStepHadTurn) return RACE_CORNER_SETTLE_MS;
    return RACE_STRAIGHT_SETTLE_MS;
  }

  return 300;
}

static void updateSensorsIfReady() {
  if (sensors.update20ms()) {
    sensors.getLatest(robotState.sensorFrame);
    robotState.perceptionFrame = computePerception(robotState.sensorFrame);
    haveSensorFrame = true;
  }
}

static void printSensors() {
  Serial.print("Sensors: L=");
  if (robotState.sensorFrame.left_valid) Serial.print(robotState.sensorFrame.left_mm);
  else Serial.print("X");

  Serial.print(" F=");
  if (robotState.sensorFrame.front_valid) Serial.print(robotState.sensorFrame.front_mm);
  else Serial.print("X");

  Serial.print(" R=");
  if (robotState.sensorFrame.right_valid) Serial.print(robotState.sensorFrame.right_mm);
  else Serial.print("X");

  Serial.println();

  Serial.print("Perception: L=");
  Serial.print(robotState.perceptionFrame.wall_left ? "WALL" : "OPEN");
  Serial.print(" F=");
  Serial.print(robotState.perceptionFrame.wall_front ? "WALL" : "OPEN");
  Serial.print(" R=");
  Serial.println(robotState.perceptionFrame.wall_right ? "WALL" : "OPEN");
}

static void printMissionMode() {
  Serial.print("Mission mode: ");

  if (missionMode == MODE_EXPLORE_TO_EXIT) Serial.println("EXPLORE_TO_EXIT");
  else if (missionMode == MODE_RETURN_TO_START) Serial.println("RETURN_TO_START");
  else if (missionMode == MODE_FACE_NORTH_AT_START) Serial.println("FACE_NORTH_AT_START");
  else if (missionMode == MODE_DELAY_BEFORE_RACE) Serial.println("DELAY_BEFORE_RACE");
  else if (missionMode == MODE_RUN_TO_KNOWN_EXIT) Serial.println("RUN_TO_KNOWN_EXIT");
  else if (missionMode == MODE_FINISHED) Serial.println("FINISHED");
}

static void finalStopForever() {
  stopMotors();

  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(80);
    digitalWrite(LED_PIN, LOW);
    delay(80);

    Serial.println();
    Serial.println("================================");
    Serial.println("MISSION COMPLETE / ROBOT STOPPED");
    Serial.print("Final pose: (");
    Serial.print(robotState.x);
    Serial.print(", ");
    Serial.print(robotState.y);
    Serial.print("), heading=");
    Serial.println((int)robotState.heading);
    Serial.println("================================");

    printMissionMode();
    printSensors();
    plannerDebugPrint();

    delay(700);
  }
}

/* ================= SETUP ================= */

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  Serial.println("Micromouse mission mode boot");

  motorsInit();
  encodersInit();
  robotStateInit(robotState);
  plannerInit();
  executorInit();

  if (!sensors.begin()) {
    Serial.println("ERROR: sensors.begin() failed");
    stopMotors();
    digitalWrite(LED_PIN, HIGH);

    while (1) {
      delay(100);
    }
  }

  Serial.println("Setup complete");
}

/* ================= LOOP ================= */

void loop() {
  /* ---------- Always update sensors / perception ---------- */
  updateSensorsIfReady();

  if (!haveSensorFrame) {
    executorUpdate();
    return;
  }

  /* ---------- If executor is busy, only advance executor ---------- */
  if (executorBusy()) {
    executorUpdate();

    if (executorDone()) {
      stopMotors();
      settling = true;
      settleStartMs = millis();
    }

    return;
  }

  /* ---------- Settling happens only after executor is fully idle ---------- */
  if (settling) {
    stopMotors();

    uint32_t neededSettle = settleTimeForCurrentMode();

    if (millis() - settleStartMs >= neededSettle) {
      settling = false;
    } else {
      return;
    }
  }

  /* ---------- Mission control while executor is fully idle ---------- */

  if (missionMode == MODE_EXPLORE_TO_EXIT) {
    Serial.println();
    Serial.println("===== EXPLORE CELL =====");

    printMissionMode();
    printSensors();

    plannerUpdateCurrentCell();
    plannerDebugPrint();

    if (plannerGoalKnown() && outsideInnerMaze()) {
      Serial.println("Exit reached. Switching to RETURN_TO_START.");

      plannerRouteToStart();
      missionMode = MODE_RETURN_TO_START;
      return;
    }

    Direction nextDir = plannerChooseNextDirection();

    Serial.print("Next direction: ");
    Serial.println((int)nextDir);

    lastStepHadTurn = (nextDir != robotState.heading);

    executorBeginStep(nextDir);
    return;
  }

  if (missionMode == MODE_RETURN_TO_START) {
    Serial.println();
    Serial.println("===== RETURN TO START =====");

    printMissionMode();
    plannerDebugPrint();

    if (atStartCell()) {
      Serial.println("Returned to start. Facing NORTH.");

      missionMode = MODE_FACE_NORTH_AT_START;
      executorBeginAlignOnly(NORTH);
      return;
    }

    Direction nextDir = plannerChooseNextDirection();

    Serial.print("Return direction: ");
    Serial.println((int)nextDir);

    lastStepHadTurn = (nextDir != robotState.heading);

    executorBeginStep(nextDir);
    return;
  }

  if (missionMode == MODE_FACE_NORTH_AT_START) {
    Serial.println("Aligned at start. Delaying before race.");

    stopMotors();
    raceDelayStartMs = millis();
    missionMode = MODE_DELAY_BEFORE_RACE;
    return;
  }

  if (missionMode == MODE_DELAY_BEFORE_RACE) {
    stopMotors();

    if (millis() - raceDelayStartMs >= DELAY_BEFORE_RACE_MS) {
      Serial.println("Starting known-exit run.");

      plannerRouteToKnownExit();
      missionMode = MODE_RUN_TO_KNOWN_EXIT;
    }

    return;
  }

  if (missionMode == MODE_RUN_TO_KNOWN_EXIT) {
    Serial.println();
    Serial.println("===== RUN TO KNOWN EXIT =====");

    printMissionMode();

    if (plannerGoalKnown() && outsideInnerMaze()) {
      Serial.println("Known exit reached. Mission complete.");
      missionMode = MODE_FINISHED;
      return;
    }

    Direction nextDir = plannerChooseNextDirection();

    Serial.print("Race direction: ");
    Serial.println((int)nextDir);

    lastStepHadTurn = (nextDir != robotState.heading);

    executorBeginStep(nextDir);
    return;
  }

  if (missionMode == MODE_FINISHED) {
    finalStopForever();
  }
}