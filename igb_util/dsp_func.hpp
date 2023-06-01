#ifndef IGB_UTIL_DSP_FUNC_H
#define IGB_UTIL_DSP_FUNC_H

#include <cstdint>
#include <igb_util/macro.hpp>

namespace igb {

IGB_FAST_INLINE float df_saw(float x) {
  return x;
}

IGB_FAST_INLINE float df_saw_biphase(float x) {
  x -= (uint32_t)x;
  return (x * 2.0f) - 1.0f;
}

IGB_FAST_INLINE float df_sqr(float x, float duty = 0.5f) {
  if (x > duty) {
    return 0.0f;
  }
  return 1.0f;
}

IGB_FAST_INLINE float df_sqr_biphase(float x, float duty = 0.5f) {
  if (x > duty) {
    return -1.0f;
  }
  return 1.0f;
}

IGB_FAST_INLINE float df_tri(float x, float duty = 0.5f) {
  if (x > duty) {
    const float w = 1.0f - duty;
    const float a = x - duty;
    return 1.0f - (a / w);
  }
  return x / duty;
}

IGB_FAST_INLINE float df_tri_biphase(float x) {
  x -= (uint32_t)x;
  if (x < -0.5f) {
    return df_tri(((-x) - 0.5f) * 2.0f);
  } else if (x < 0.0f) {
    return -df_tri((-x) * 2.0f);
  } else if (x < 0.5f) {
    return df_tri(x * 2.0f);
  }
  return -df_tri((x - 0.5f) * 2.0f);
}

IGB_FAST_INLINE float df_softclip(float v /* -1.0 ~ 1.0 */) /* -> -1.0 ~ 1.0 */ {
  v *= 2.0f / 3.0f;
  if (v < -1.0f) {
    return -1.0f;
  } else if (v > 1.0f) {
    return 1.0f;
  }
  return (v - (v * v * v) / 3.0f) * 3.0f / 2.0f;
}

};

#endif /* IGB_UTIL_DSP_FUNC_H */
