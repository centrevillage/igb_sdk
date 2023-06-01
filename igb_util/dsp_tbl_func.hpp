#ifndef IGB_UTIL_DSP_TBL_FUNC_H
#define IGB_UTIL_DSP_TBL_FUNC_H

#include <cstdint>
#include <igb_util/macro.hpp>
#include <igb_util/algorithm.hpp>

namespace igb {

constexpr uint16_t dsp_func_tbl_size = 1024;

extern const float dsp_func_sigmoid_tbl[dsp_func_tbl_size];
extern const float dsp_func_sinusoid_tbl[dsp_func_tbl_size];
extern const float dsp_func_log_tbl[dsp_func_tbl_size];
extern const float dsp_func_exp_tbl[dsp_func_tbl_size];

IGB_FAST_INLINE float dsp_read_tbl(const float* tbl, float x /* 0.0 ~ 1.0 */) {
  x -= (uint32_t)x;
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

IGB_FAST_INLINE float dsp_read_tbl_fast(const float* tbl, float x /* 0.0 ~ 1.0 */) {
  x -= (uint32_t)x;
  if (x < 0.0f) {
    x += 1.0f;
  }
  uint16_t v = x * (float)(dsp_func_tbl_size-1);
  return tbl[v];
}

IGB_FAST_INLINE float dsp_sin(float x /* 0.0 ~ 1.0 */) {
  return dsp_read_tbl(dsp_func_sinusoid_tbl, x);
}
IGB_FAST_INLINE float dsp_sigmoid(float x) {
  return dsp_read_tbl(dsp_func_sigmoid_tbl, x);
}
IGB_FAST_INLINE float dsp_exp(float x) {
  return dsp_read_tbl(dsp_func_exp_tbl, x);
}
IGB_FAST_INLINE float dsp_log(float x) {
  return dsp_read_tbl(dsp_func_log_tbl, x);
}
IGB_FAST_INLINE float dsp_sin_fast(float x /* 0.0 ~ 1.0 */) {
  return dsp_read_tbl_fast(dsp_func_sinusoid_tbl, x);
}
IGB_FAST_INLINE float dsp_sigmoid_fast(float x) {
  return dsp_read_tbl_fast(dsp_func_sigmoid_tbl, x);
}
IGB_FAST_INLINE float dsp_exp_fast(float x) {
  return dsp_read_tbl_fast(dsp_func_exp_tbl, x);
}
IGB_FAST_INLINE float dsp_log_fast(float x) {
  return dsp_read_tbl_fast(dsp_func_log_tbl, x);
}

}

#endif /* IGB_UTIL_DSP_TBL_FUNC_H */
