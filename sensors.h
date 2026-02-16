#pragma once
#include <Arduino.h>

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

  // Call once in setup()
  bool begin();

  // Returns true if a new frame was produced this call.
  bool update20ms();

  // Copy out the latest frame (fast, simple).
  void getLatest(SensorsFrame& out) const;

private:
  bool readOne(uint8_t mux_channel, uint16_t& mm_out, bool& valid_out);

  uint8_t _mux_addr;
  SensorsFrame _latest;

  uint32_t _next_due_ms;   // for steady 50 Hz scheduling
};
