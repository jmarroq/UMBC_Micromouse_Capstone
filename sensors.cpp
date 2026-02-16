#include "Sensors.h"
#include <Wire.h>
#include <Adafruit_VL6180X.h>

static constexpr uint8_t CH_LEFT  = 0;  // mux channel 0
static constexpr uint8_t CH_FRONT = 1;  // mux channel 1
static constexpr uint8_t CH_RIGHT = 2;  // mux channel 2

static constexpr uint16_t INIT_FAR_MM = 255; // VL6180X  max reading range

// Helper: select a channel on TCA9548A
static bool muxSelect(uint8_t mux_addr, uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(mux_addr);
  Wire.write(1u << channel);
  return (Wire.endTransmission() == 0);
}

Sensors::Sensors(uint8_t mux_addr)
: _mux_addr(mux_addr),
  _latest{},
  _next_due_ms(0)
{
  // Initialize minimal defaults 
  _latest.t_ms = 0;

  _latest.left_mm = INIT_FAR_MM;
  _latest.left_valid = false;

  _latest.front_mm = INIT_FAR_MM;
  _latest.front_valid = false;

  _latest.right_mm = INIT_FAR_MM;
  _latest.right_valid = false;
}

bool Sensors::begin() {
  Wire.begin();

  // Adafruit driver expects device at 0x29 on the selected channel.
  Adafruit_VL6180X vl;
  
 // Initialize each sensor through its mux channel.
  for (uint8_t ch : {CH_LEFT, CH_FRONT, CH_RIGHT}) {
    if (!muxSelect(_mux_addr, ch)) return false;
    if (!vl.begin()) return false;
  }

  // Schedule first update immediately
  _next_due_ms = millis();
  return true;
}

bool Sensors::readOne(uint8_t mux_channel, uint16_t& mm_out, bool& valid_out) {
  valid_out = false;

  if (!muxSelect(_mux_addr, mux_channel)) {
    return false; // mux select failed
  }

  Adafruit_VL6180X vl;
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  // Treat "no error" as valid
  valid_out = (status == VL6180X_ERROR_NONE);
  mm_out = (uint16_t)range; // range is in mm (0..255)

  return true;
}

bool Sensors::update20ms() {
  const uint32_t now = millis();

  // Only run at ~50 Hz using a "next due" schedule
  if ((int32_t)(now - _next_due_ms) < 0) {
    return false; // not due yet
  }
  _next_due_ms += 20; // keep a steady cadence

  // Read each sensor; on failure, keep last mm but mark invalid for that sensor
  uint16_t mm;
  bool ok;

  // LEFT
  ok = readOne(CH_LEFT, mm, _latest.left_valid);
  if (ok) {
    if (_latest.left_valid) _latest.left_mm = mm;
    // if read succeeded but status not valid: keep last mm, valid=false
  } else {
    _latest.left_valid = false;
  }

  // FRONT
  ok = readOne(CH_FRONT, mm, _latest.front_valid);
  if (ok) {
    if (_latest.front_valid) _latest.front_mm = mm;
  } else {
    _latest.front_valid = false;
  }

  // RIGHT
  ok = readOne(CH_RIGHT, mm, _latest.right_valid);
  if (ok) {
    if (_latest.right_valid) _latest.right_mm = mm;
  } else {
    _latest.right_valid = false;
  }

  _latest.t_ms = now;
  return true;
}

void Sensors::getLatest(SensorsFrame& out) const {
  out = _latest; // simple struct copy
}
