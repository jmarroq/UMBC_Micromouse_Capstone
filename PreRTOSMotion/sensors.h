#pragma once
#include <Arduino.h>
#include <Adafruit_VL6180X.h>

struct SensorsFrame {
  uint32_t t_ms;

  uint16_t left_mm;
  bool     left_valid;

  uint16_t front_mm;
  bool     front_valid;

  uint16_t right_mm;
  bool     right_valid;
};

class Sensors {
public:
  // Establish Mux address0x70 for TCA9548A
  explicit Sensors(uint8_t mux_addr = 0x70);

  bool begin();
  bool update20ms();
  void getLatest(SensorsFrame& out) const;

private:
  bool readOne(uint8_t mux_channel, uint16_t& mm_out, bool& valid_out);
  uint8_t _mux_addr;
  SensorsFrame _latest;
  Adafruit_VL6180X _vl;
  uint32_t _next_due_ms;
  
};