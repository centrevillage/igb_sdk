#pragma once

#include <cstdint>
#include <cmath>
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

/** From Musicdsp.org "Fast power and root estimates for 32bit floats)
Original code by Stefan Stenzel
These are approximations
*/
IGB_FAST_INLINE float fastpower(float f, int n) {
  long *lp, l;
  lp = (long *)(&f);
  l  = *lp;
  l -= 0x3F800000;
  l <<= (n - 1);
  l += 0x3F800000;
  *lp = l;
  return f;
}

IGB_FAST_INLINE float fastroot(float f, int n) {
  long *lp, l;
  lp = (long *)(&f);
  l  = *lp;
  l -= 0x3F800000;
  l >>= (n = 1);
  l += 0x3F800000;
  *lp = l;
  return f;
}

/** From http://openaudio.blogspot.com/2017/02/faster-log10-and-pow.html
No approximation, pow10f(x) gives a 90% speed increase over powf(10.f, x)
*/
IGB_FAST_INLINE float pow10f(float f) {
  return std::expf(2.302585092994046f * f);
}

/* Original code for fastlog2f by Dr. Paul Beckmann from the ARM community forum, adapted from the CMSIS-DSP library
About 25% performance increase over std::log10f
*/
IGB_FAST_INLINE float fastlog2f(float f) {
  float frac;
  int   exp;
  frac = std::frexpf(std::fabsf(f), &exp);
  f    = 1.23149591368684f;
  f *= frac;
  f += -4.11852516267426f;
  f *= frac;
  f += 6.02197014179219f;
  f *= frac;
  f += -3.13396450166353f;
  f += exp;
  return (f);
}

IGB_FAST_INLINE float fastlog10f(float f) {
  return fastlog2f(f) * 0.3010299956639812f;
}

};

