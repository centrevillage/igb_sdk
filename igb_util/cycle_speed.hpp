#pragma once

#include <cstdint>
#include <igb_util/macro.hpp>

namespace igb {

constexpr uint16_t cycle_speed_table_count = 12*100;
extern const uint16_t cycle_speed_table[cycle_speed_table_count];

// sampling rate: 48KHz
// if you want 96kHz, you can set ovs = 1
IGB_FAST_INLINE float cycle_speed_pitch_to_delta_f(float pitch_oct_v, uint32_t ovs = 0) {
  if (pitch_oct_v < 0.0f) {
    pitch_oct_v = 0.0f;
  }
  uint8_t oct = (uint8_t)pitch_oct_v;
  float key = pitch_oct_v - (float)oct;
  float value = key * (float)cycle_speed_table_count;
  uint32_t index = (uint32_t)value;
  const float scaling = (float)((uint32_t)1 << (uint32_t)(20 + 5 + ovs)); // C0(16.351Hz) on oct = 0
  return (float)((uint32_t)(cycle_speed_table[index] << oct)) / scaling;
}

}

