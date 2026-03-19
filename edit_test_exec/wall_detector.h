#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// wall_detector.h
//
// Answers the question: "Is there actually a wall here?"
//
// Problem with using computePerception() directly for maze map writes:
//   • A single noisy reading (I2C glitch, reflection artefact) flips wall=true
//     for one 20ms frame.
//   • plannerUpdateCurrentCell() calls set_wall() which is irreversible on the
//     current map — a false wall permanently corrupts the floodfill.
//
// Solution — per-sensor vote window:
//   Each sensor independently accumulates the last VOTE_WINDOW binary results
//   (wall / no-wall). A wall is "confirmed" only when WALL_CONFIRM_VOTES out of
//   VOTE_WINDOW frames agree. Similarly, a wall is "cleared" only when
//   WALL_CLEAR_VOTES frames agree it is absent (hysteresis).
//
// Usage:
//   1. Call WallDetector::update() every time a new SensorsFrame arrives
//      (i.e. inside taskSensors, right after computePerception).
//   2. Call WallDetector::getFrame() in plannerUpdateCurrentCell() instead of
//      reading robotState.perceptionFrame directly.
//   3. Only commit walls to the maze map when getFrame().ready == true.
//
// Threading / timing:
//   update() runs in taskSensors (every 20 ms).
//   getFrame() is called from taskPlanner (every 20 ms) and the control loop.
//   Both run on the same single Arduino loop() thread — no mutex needed.
//   If you move to RTOS tasks, guard _frame with a mutex.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include "sensors.h"
#include "perception.h"

// ── Tuning constants ──────────────────────────────────────────────────────────

// How many consecutive frames to accumulate before making a decision.
// At 20ms/frame: VOTE_WINDOW=5 → 100ms of history before committing.
static constexpr uint8_t VOTE_WINDOW = 5;

// A wall is CONFIRMED when this many of the last VOTE_WINDOW frames said wall.
static constexpr uint8_t WALL_CONFIRM_VOTES = 4;  // 4/5 — high confidence

// A wall is CLEARED when this many of the last VOTE_WINDOW frames said clear.
// Lower than CONFIRM to make it harder to accidentally un-see a real wall.
static constexpr uint8_t WALL_CLEAR_VOTES = 5;    // 5/5 — must be unanimous clear

// Distance thresholds — adjust per physical maze geometry.
// These mirror perception.h but are owned here so WallDetector is self-contained.
// You can change them at runtime with setThresholds().
struct WallThresholds {
  uint16_t left_mm  = 70;
  uint16_t front_mm = 90;
  uint16_t right_mm = 70;
};

// ── Validated wall state (output of WallDetector) ────────────────────────────

struct ValidatedWallFrame {
  uint32_t t_ms;

  // Tri-state per sensor:
  //   WALL_YES    — enough votes confirm a wall; safe to commit to maze map
  //   WALL_NO     — enough votes confirm no wall
  //   WALL_UNSURE — not enough history yet, or votes are split
  enum WallVote : uint8_t { WALL_YES = 0, WALL_NO = 1, WALL_UNSURE = 2 };

  WallVote left;
  WallVote front;
  WallVote right;

  // Convenience: true only when ALL three sensors have a definitive vote.
  // plannerUpdateCurrentCell() should check this before writing walls.
  bool ready;
};

// ── WallDetector class ───────────────────────────────────────────────────────

class WallDetector {
public:
  WallDetector() { reset(); }

  // Call once to set non-default thresholds (e.g. read from EEPROM at boot).
  void setThresholds(uint16_t left_mm, uint16_t front_mm, uint16_t right_mm) {
    _thresholds.left_mm  = left_mm;
    _thresholds.front_mm = front_mm;
    _thresholds.right_mm = right_mm;
  }

  // Call every time a new SensorsFrame is available (inside taskSensors).
  // Returns true if the validated frame changed state this call.
  bool update(const SensorsFrame& s) {
    // ── Step 1: compute raw binary wall/no-wall from thresholds ─────────────
    bool raw_left  = s.left_valid  && (s.left_mm  < _thresholds.left_mm);
    bool raw_front = s.front_valid && (s.front_mm < _thresholds.front_mm);
    bool raw_right = s.right_valid && (s.right_mm < _thresholds.right_mm);

    // ── Step 2: push into rolling vote windows ───────────────────────────────
    _pushVote(_leftVotes,  _leftIdx,  _leftCount,  raw_left);
    _pushVote(_frontVotes, _frontIdx, _frontCount, raw_front);
    _pushVote(_rightVotes, _rightIdx, _rightCount, raw_right);

    _frameCount++;

    // ── Step 3: build ValidatedWallFrame ────────────────────────────────────
    ValidatedWallFrame prev = _frame;

    _frame.t_ms  = s.t_ms;
    _frame.left  = _vote(_leftCount);
    _frame.front = _vote(_frontCount);
    _frame.right = _vote(_rightCount);

    // ready = we have seen at least VOTE_WINDOW frames AND all three are decided
    _frame.ready = (_frameCount >= VOTE_WINDOW) &&
                   (_frame.left  != ValidatedWallFrame::WALL_UNSURE) &&
                   (_frame.front != ValidatedWallFrame::WALL_UNSURE) &&
                   (_frame.right != ValidatedWallFrame::WALL_UNSURE);

    return (_frame.left  != prev.left  ||
            _frame.front != prev.front ||
            _frame.right != prev.right);
  }

  // Returns the latest validated frame.
  // plannerUpdateCurrentCell() calls this and checks frame.ready before writing.
  const ValidatedWallFrame& getFrame() const { return _frame; }

  // Returns the raw vote counts (0..VOTE_WINDOW) — useful for Serial debug.
  void getVoteCounts(uint8_t& left, uint8_t& front, uint8_t& right) const {
    left  = _leftCount;
    front = _frontCount;
    right = _rightCount;
  }

  // Reset all history — call when the robot is reset or re-positioned.
  void reset() {
    memset(_leftVotes,  0, sizeof(_leftVotes));
    memset(_frontVotes, 0, sizeof(_frontVotes));
    memset(_rightVotes, 0, sizeof(_rightVotes));
    _leftIdx = _frontIdx = _rightIdx = 0;
    _leftCount = _frontCount = _rightCount = 0;
    _frameCount = 0;
    _frame = {};
  }

private:
  WallThresholds _thresholds;
  ValidatedWallFrame _frame = {};

  // Per-sensor rolling vote buffers
  bool    _leftVotes [VOTE_WINDOW] = {};
  bool    _frontVotes[VOTE_WINDOW] = {};
  bool    _rightVotes[VOTE_WINDOW] = {};
  uint8_t _leftIdx = 0, _frontIdx = 0, _rightIdx = 0;
  uint8_t _leftCount = 0, _frontCount = 0, _rightCount = 0;
  uint32_t _frameCount = 0;

  // Push a new binary sample into a rolling window, maintaining the running sum.
  static void _pushVote(bool* buf, uint8_t& idx, uint8_t& count, bool val) {
    // Subtract the outgoing sample from the running sum
    if (buf[idx]) count--;
    // Write and advance
    buf[idx] = val;
    if (val) count++;
    idx = (idx + 1) % VOTE_WINDOW;
  }

  // Classify vote count into tri-state decision.
  static ValidatedWallFrame::WallVote _vote(uint8_t count) {
    if (count >= WALL_CONFIRM_VOTES) return ValidatedWallFrame::WALL_YES;
    if ((VOTE_WINDOW - count) >= WALL_CLEAR_VOTES) return ValidatedWallFrame::WALL_NO;
    return ValidatedWallFrame::WALL_UNSURE;
  }
};
