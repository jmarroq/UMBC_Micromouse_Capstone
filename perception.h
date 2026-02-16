#pragma once
#include <Arduino.h>
#include "Sensors.h"


// Internal thresholds 
static constexpr uint16_t LEFT_THRESHOLD_MM  = 70;
static constexpr uint16_t FRONT_THRESHOLD_MM = 90;
static constexpr uint16_t RIGHT_THRESHOLD_MM = 70;

// Output (robot-relative)
struct PerceptionFrame {
  uint32_t t_ms;
  bool wall_left;
  bool wall_front;
  bool wall_right;
};

// Stateless computation
static inline PerceptionFrame computePerception(const SensorsFrame& s) {
  PerceptionFrame p;
  p.t_ms = s.t_ms;

  // invalid reading => wall=false
  p.wall_left  = (s.left_valid  && (s.left_mm  < LEFT_THRESHOLD_MM));
  p.wall_front = (s.front_valid && (s.front_mm < FRONT_THRESHOLD_MM));
  p.wall_right = (s.right_valid && (s.right_mm < RIGHT_THRESHOLD_MM));

  return p;
}
