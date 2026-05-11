#include "Sensors.h"
#include <Wire.h>

static constexpr uint8_t CH_LEFT  = 1;
static constexpr uint8_t CH_FRONT = 0;
static constexpr uint8_t CH_RIGHT = 7;

static constexpr uint16_t INIT_FAR_MM = 255;
static constexpr uint16_t FORCE_FAR_MM = 255;

// TEMPORARY TEST MODE:
// Treat every sensor frame as valid.
// If sensor returns an error/out-of-range, store 255 mm instead of invalid.
static constexpr bool FORCE_ALL_READINGS_VALID = true;

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
    _next_due_ms(0),
    _vl() {

  _latest.t_ms = 0;

  _latest.left_mm = INIT_FAR_MM;
  _latest.left_valid = true;

  _latest.front_mm = INIT_FAR_MM;
  _latest.front_valid = true;

  _latest.right_mm = INIT_FAR_MM;
  _latest.right_valid = true;
}

bool Sensors::begin() {
  Wire.begin();
  delay(50);

  Serial.println("Sensors.begin()...");
  Serial.print("Trying mux 0x");
  Serial.println(_mux_addr, HEX);

  for (uint8_t ch : {CH_LEFT, CH_FRONT, CH_RIGHT}) {
    Serial.print("Selecting channel ");
    Serial.println(ch);

    if (!muxSelect(_mux_addr, ch)) {
      Serial.print("Mux select failed on channel ");
      Serial.println(ch);
      return false;
    }

    delay(10);

    Serial.print("Calling _vl.begin() on channel ");
    Serial.println(ch);

    if (!_vl.begin()) {
      Serial.print("_vl.begin failed on channel ");
      Serial.println(ch);
      return false;
    }

    Serial.print("Channel ");
    Serial.print(ch);
    Serial.println(" OK");
    delay(10);
  }

  _next_due_ms = millis();
  Serial.println("Sensors.begin success");
  return true;
}

bool Sensors::readOne(uint8_t mux_channel, uint16_t& mm_out, bool& valid_out) {
  valid_out = false;
  mm_out = FORCE_FAR_MM;

  if (!muxSelect(_mux_addr, mux_channel)) {
    Serial.print("muxSelect failed on read channel ");
    Serial.println(mux_channel);

    if (FORCE_ALL_READINGS_VALID) {
      valid_out = true;
      mm_out = FORCE_FAR_MM;
    }

    return false;
  }

  delay(2);

  uint8_t range = _vl.readRange();
  uint8_t status = _vl.readRangeStatus();

  bool sensorOk = (status == VL6180X_ERROR_NONE);

  if (sensorOk) {
    mm_out = (uint16_t)range;
    valid_out = true;
  } else {
    if (FORCE_ALL_READINGS_VALID) {
      mm_out = FORCE_FAR_MM;
      valid_out = true;
    } else {
      mm_out = (uint16_t)range;
      valid_out = false;
    }
  }


  return true;
}

bool Sensors::update20ms() {
  const uint32_t now = millis();

  if ((int32_t)(now - _next_due_ms) < 0) {
    return false;
  }

  _next_due_ms += 20;

  uint16_t mm;
  bool valid;
  bool ok;

  ok = readOne(CH_LEFT, mm, valid);
  if (ok || FORCE_ALL_READINGS_VALID) {
    _latest.left_mm = mm;
    _latest.left_valid = valid;
  } else {
    _latest.left_mm = FORCE_FAR_MM;
    _latest.left_valid = false;
  }

  ok = readOne(CH_FRONT, mm, valid);
  if (ok || FORCE_ALL_READINGS_VALID) {
    _latest.front_mm = mm;
    _latest.front_valid = valid;
  } else {
    _latest.front_mm = FORCE_FAR_MM;
    _latest.front_valid = false;
  }

  ok = readOne(CH_RIGHT, mm, valid);
  if (ok || FORCE_ALL_READINGS_VALID) {
    _latest.right_mm = mm;
    _latest.right_valid = valid;
  } else {
    _latest.right_mm = FORCE_FAR_MM;
    _latest.right_valid = false;
  }

  _latest.t_ms = now;


  return true;
}

void Sensors::getLatest(SensorsFrame& out) const {
  out = _latest;
}