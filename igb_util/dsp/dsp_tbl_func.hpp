#pragma once

#include <cstdint>
#include <igb_util/macro.hpp>
#include <igb_util/algorithm.hpp>

namespace igb::dsp {

#if defined(USE_SMALL_DSP_TABLE)
constexpr uint16_t dsp_func_tbl_size = 256;
#else
constexpr uint16_t dsp_func_tbl_size = 1024;
#endif

#if defined(USE_SMALL_DSP_TABLE)

#include "_dsp_tbl_func_256.hpp"

#else

#include "_dsp_tbl_func_1024.hpp"

#endif

IGB_FAST_INLINE float bi2uni(float x) {
  return x * 0.5f + 0.5f;
}

IGB_FAST_INLINE float uni2bi(float x) {
  return x * 2.0f - 1.0f;
}

IGB_FAST_INLINE float read_tbl(const float* tbl, float x /* 0.0 ~ 1.0 */) {
  x -= (int32_t)x;
  if (x < 0.0f) {
    x += 1.0f;
  }

  float v = x * (float)(dsp_func_tbl_size);
  uint16_t sp = (uint16_t)v;
  float diff = v - (float)sp;
  uint16_t ep = sp + 1;
  if (ep >= dsp_func_tbl_size) {
    return tbl[sp];
  }
  float sv = tbl[sp];
  float ev = tbl[ep];

  return lerp(sv, ev, diff);
}

IGB_FAST_INLINE float read_tbl_cyclic(const float* tbl, float x /* 0.0 ~ 1.0 */) {
  x -= (int32_t)x;
  if (x < 0.0f) {
    x += 1.0f;
  }

  float v = x * (float)(dsp_func_tbl_size);
  uint16_t sp = (uint16_t)v;
  float diff = v - (float)sp;
  uint16_t ep = (sp + 1) % dsp_func_tbl_size;
  float sv = tbl[sp];
  float ev = tbl[ep];

  return lerp(sv, ev, diff);
}

IGB_FAST_INLINE float read_tbl_fast(const float* tbl, float x /* 0.0 ~ 1.0 */) {
  x -= (int32_t)x;
  if (x < 0.0f) {
    x += 1.0f;
  }
  uint16_t v = x * (float)(dsp_func_tbl_size-1);
  return tbl[v];
}

IGB_FAST_INLINE float sin_wave_bi(float x /* 0.0 ~ 1.0 */) {
  return read_tbl_cyclic(dsp_func_sinusoid_tbl, x);
}
IGB_FAST_INLINE float sin_wave_uni(float x /* 0.0 ~ 1.0 */) {
  return bi2uni(sin_wave_bi(x));
}
IGB_FAST_INLINE float cos_wave_bi(float x /* 0.0 ~ 1.0 */) {
  return read_tbl_cyclic(dsp_func_sinusoid_tbl, x + 0.25f);
}
IGB_FAST_INLINE float cos_wave_uni(float x /* 0.0 ~ 1.0 */) {
  return bi2uni(cos_wave_bi(x));
}
IGB_FAST_INLINE float sigmoid_curve(float x /* 0.0 ~ 1.0 */) {
  return read_tbl(dsp_func_sigmoid_tbl, x);
}
IGB_FAST_INLINE float exp_curve(float x /* 0.0 ~ 1.0 */) {
  return read_tbl(dsp_func_exp_tbl, x);
}
IGB_FAST_INLINE float log_curve(float x /* 0.0 ~ 1.0 */) {
  return read_tbl(dsp_func_log_tbl, x);
}
IGB_FAST_INLINE float perlin_5order(float x /* -1.0 ~ 1.0 */) {
  if (x > 1.0f || x < -1.0f) {
    return 0.0f;
  } else if (x < 0.0f) {
    return read_tbl(dsp_func_perlin_5order_tbl, -x);
  }
  return read_tbl(dsp_func_perlin_5order_tbl, x);
}
IGB_FAST_INLINE float sin_wave_bi_fast(float x /* 0.0 ~ 1.0 */) {
  return read_tbl_fast(dsp_func_sinusoid_tbl, x);
}
IGB_FAST_INLINE float sin_wave_uni_fast(float x /* 0.0 ~ 1.0 */) {
  return bi2uni(sin_wave_bi_fast(x));
}
IGB_FAST_INLINE float cos_wave_bi_fast(float x /* 0.0 ~ 1.0 */) {
  return read_tbl_fast(dsp_func_sinusoid_tbl, x + 0.25f);
}
IGB_FAST_INLINE float cos_wave_uni_fast(float x /* 0.0 ~ 1.0 */) {
  return bi2uni(cos_wave_bi_fast(x));
}
IGB_FAST_INLINE float sigmoid_curve_fast(float x) {
  return read_tbl_fast(dsp_func_sigmoid_tbl, x);
}
IGB_FAST_INLINE float exp_curve_fast(float x) {
  return read_tbl_fast(dsp_func_exp_tbl, x);
}
IGB_FAST_INLINE float log_curve_fast(float x) {
  return read_tbl_fast(dsp_func_log_tbl, x);
}
IGB_FAST_INLINE float perlin_5order_fast(float x /* -1.0 ~ 1.0 */) {
  if (x > 1.0f || x < -1.0f) {
    return 0.0f;
  } else if (x < 0.0f) {
    return read_tbl_fast(dsp_func_perlin_5order_tbl, -x);
  }
  return read_tbl_fast(dsp_func_perlin_5order_tbl, x);
}

}

