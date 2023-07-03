#pragma once

#include <cstdint>
#include <igb_util/macro.hpp>

namespace igb::dsp {

IGB_FAST_INLINE float saw_wave_uni(float x) {
  return x;
}

IGB_FAST_INLINE float saw_wave_bi(float x) {
  x -= (uint32_t)x;
  return (x * 2.0f) - 1.0f;
}

IGB_FAST_INLINE float sqr_wave_uni(float x, float duty = 0.5f) {
  if (x > duty) {
    return 0.0f;
  }
  return 1.0f;
}

IGB_FAST_INLINE float sqr_wave_bi(float x, float duty = 0.5f) {
  if (x > duty) {
    return -1.0f;
  }
  return 1.0f;
}

IGB_FAST_INLINE float tri_wave_uni(float x, float duty = 0.5f) {
  if (x > duty) {
    const float w = 1.0f - duty;
    const float a = x - duty;
    return 1.0f - (a / w);
  }
  return x / duty;
}

IGB_FAST_INLINE float tri_wave_bi(float x) {
  x -= (uint32_t)x;
  if (x < -0.5f) {
    return tri_wave_uni(((-x) - 0.5f) * 2.0f);
  } else if (x < 0.0f) {
    return -tri_wave_uni((-x) * 2.0f);
  } else if (x < 0.5f) {
    return tri_wave_uni(x * 2.0f);
  }
  return -tri_wave_uni((x - 0.5f) * 2.0f);
}

IGB_FAST_INLINE float softclip(float v /* -1.0 ~ 1.0 */) /* -> -1.0 ~ 1.0 */ {
  v *= 2.0f / 3.0f;
  if (v < -1.0f) {
    return -1.0f;
  } else if (v > 1.0f) {
    return 1.0f;
  }
  return (v - (v * v * v) / 3.0f) * 3.0f / 2.0f;
}

};

