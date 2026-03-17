#include "Sensors.h"
#include <Wire.h>

static constexpr uint8_t CH_LEFT  = 0;
static constexpr uint8_t CH_FRONT = 3;
static constexpr uint8_t CH_RIGHT = 6;

static constexpr uint16_t INIT_FAR_MM = 255;

// Helper: select a channel on TCA9548A
static bool muxSelect(uint8_t mux_addr, uint8_t channel) {
  if (channel > 7) return false;

  Wire.beginTransmission(mux_addr);
  Wire.write(1u << channel);
  return (Wire.endTransmission() == 0);
}

Sensors::Sensors(uint8_t mux_addr): _mux_addr(mux_addr),_latest{},_next_due_ms(0),_vl(){
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

  if (!muxSelect(_mux_addr, mux_channel)) {
    Serial.print("muxSelect failed on read channel ");
    Serial.println(mux_channel);
    return false;
  }

  delay(2);

  uint8_t range = _vl.readRange();
  uint8_t status = _vl.readRangeStatus();

  valid_out = (status == VL6180X_ERROR_NONE);
  mm_out = (uint16_t)range;

  return true;
}

bool Sensors::update20ms() {
  const uint32_t now = millis();

  if ((int32_t)(now - _next_due_ms) < 0) {
    return false;
  }
  _next_due_ms += 20;

  uint16_t mm;
  bool ok;

  ok = readOne(CH_LEFT, mm, _latest.left_valid);
  if (ok) {
    if (_latest.left_valid) _latest.left_mm = mm;
  } else {
    _latest.left_valid = false;
  }

  ok = readOne(CH_FRONT, mm, _latest.front_valid);
  if (ok) {
    if (_latest.front_valid) _latest.front_mm = mm;
  } else {
    _latest.front_valid = false;
  }

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
  out = _latest;
}