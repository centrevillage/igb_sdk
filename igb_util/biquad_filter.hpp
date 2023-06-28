#pragma once

#include <cstdint>
#include <cstddef>
#include <tuple>
#include <cmath>
#include <igb_util/algorithm.hpp>
#include <igb_util/math.hpp>
#include <igb_util/dsp_tbl_func.hpp>

namespace igb {

template<uint32_t sampling_rate, bool use_dsp_tbl = false>
struct BiQuadFilter {
  struct Context {
    float x1 = 0.0f;
    float x2 = 0.0f;
    float y1 = 0.0f;
    float y2 = 0.0f;
  };

  float a1 = 0.0f;
  float a2 = 0.0f;
  float b0 = 0.0f;
  float b1 = 0.0f;
  float b2 = 0.0f;

  inline static float freq2w0(float freq) {
    if (use_dsp_tbl) {
      return freq / (float)sampling_rate;
    }
    return 2.0f * (float)igb::numbers::pi * freq / (float)sampling_rate;
  }

  inline static float _cos_w0(float w0) {
    if (use_dsp_tbl) {
      return igb::dsp_cos(w0);
    }
    return std::cos(w0);
  }

  inline static float _sin_w0(float w0) {
    if (use_dsp_tbl) {
      return igb::dsp_sin(w0);
    }
    return std::sin(w0);
  }

  //LPF:  H(s) = 1 / (s^2 + s/Q + 1)
  //            b0 =  (1 - cos(w0))/2
  //            b1 =   1 - cos(w0)
  //            b2 =  (1 - cos(w0))/2
  //            a0 =   1 + alpha
  //            a1 =  -2*cos(w0)
  //            a2 =   1 - alpha
  void lpf(float freq, float q) {
    float w0 = freq2w0(freq);
    float cos_w0 = _cos_w0(w0);
    float sin_w0 = _sin_w0(w0);
    float alpha = sin_w0 / (2.0f * q);
    float a0 = 1.0f + alpha;
    b1 = (1.0f - cos_w0) / a0;
    b0 = b2 = b1 / 2.0f;
    a1 = -2.0f * cos_w0 / a0;
    a2 = (1.0f - alpha) / a0;
  }

  //HPF:  H(s) = s^2 / (s^2 + s/Q + 1)
  //            b0 =  (1 + cos(w0))/2
  //            b1 = -(1 + cos(w0))
  //            b2 =  (1 + cos(w0))/2
  //            a0 =   1 + alpha
  //            a1 =  -2*cos(w0)
  //            a2 =   1 - alpha
  void hpf(float freq, float q) {
    float w0 = freq2w0(freq);
    float cos_w0 = _cos_w0(w0);
    float sin_w0 = _sin_w0(w0);
    float alpha = sin_w0 / (2.0f * q);
    float a0 = 1.0f + alpha;
    b1 = -(1.0f + cos_w0) / a0;
    b0 = b2 = -b1 / 2.0f;
    a1 = -2.0f * cos_w0 / a0;
    a2 = (1.0f - alpha) / a0;
  }

  //BPF: H(s) = s / (s^2 + s/Q + 1)
  //               (constant skirt gain, peak gain = Q)
  //
  //            b0 =   sin(w0)/2  =   Q*alpha
  //            b1 =   0
  //            b2 =  -sin(w0)/2  =  -Q*alpha
  //            a0 =   1 + alpha
  //            a1 =  -2*cos(w0)
  //            a2 =   1 - alpha
  void bpf(float freq, float q) {
    float w0 = freq2w0(freq);
    float cos_w0 = _cos_w0(w0);
    float sin_w0 = _sin_w0(w0);
    float alpha = sin_w0 / (2.0f * q);
    float a0 = 1.0f + alpha;
    b1 = 0;
    b0 = q * alpha / a0;
    b2 = -q * alpha / a0;
    a1 = -2.0f * cos_w0 / a0;
    a2 = (1.0f - alpha) / a0;
  }

  //BPF:  H(s) = (s/Q) / (s^2 + s/Q + 1) 
  //               (constant 0 dB peak gain)
  //
  //            b0 =   alpha
  //            b1 =   0
  //            b2 =  -alpha
  //            a0 =   1 + alpha
  //            a1 =  -2*cos(w0)
  //            a2 =   1 - alpha
  void bpf_0db_peak(float freq, float q) {
    float w0 = freq2w0(freq);
    float cos_w0 = _cos_w0(w0);
    float sin_w0 = _sin_w0(w0);
    float alpha = sin_w0 / (2.0f * q);
    float a0 = 1.0f + alpha;
    b1 = 0;
    b0 = alpha / a0;
    b2 = -alpha / a0;
    a1 = -2.0f * cos_w0 / a0;
    a2 = (1.0f - alpha) / a0;
  }

  //notch:  H(s) = (s^2 + 1) / (s^2 + s/Q + 1)
  //            b0 =   1
  //            b1 =  -2*cos(w0)
  //            b2 =   1
  //            a0 =   1 + alpha
  //            a1 =  -2*cos(w0)
  //            a2 =   1 - alpha
  void notch(float freq, float q) {
    float w0 = freq2w0(freq);
    float cos_w0 = _cos_w0(w0);
    float sin_w0 = _sin_w0(w0);
    float alpha = sin_w0 / (2.0f * q);
    float a0 = 1.0f + alpha;
    b0 = b2 = 1.0f / a0;
    b1 = a1 =  -2.0f * cos_w0 / a0;
    a2 = (1.0f - alpha) / a0;
  }

  //APF:  H(s) = (s^2 - s/Q + 1) / (s^2 + s/Q + 1)
  //
  //            b0 =   1 - alpha
  //            b1 =  -2*cos(w0)
  //            b2 =   1 + alpha
  //            a0 =   1 + alpha
  //            a1 =  -2*cos(w0)
  //            a2 =   1 - alpha
  void apf(float freq , float q) {
    float w0 = freq2w0(freq);
    float cos_w0 = _cos_w0(w0);
    float sin_w0 = _sin_w0(w0);
    float alpha = sin_w0 / (2.0f * q);
    float a0 = 1.0f + alpha;
    b0 = (1.0f - alpha) / a0;
    b1 = -2.0f * cos_w0 / a0;
    b2 = 1.0f;
    a1 = b1;
    a2 = b0;
  }

  //peakingEQ:  H(s) = (s^2 + s*(A/Q) + 1) / (s^2 + s/(A*Q) + 1)
  //
  //            b0 =   1 + alpha*A
  //            b1 =  -2*cos(w0)
  //            b2 =   1 - alpha*A
  //            a0 =   1 + alpha/A
  //            a1 =  -2*cos(w0)
  //            a2 =   1 - alpha/A
  void peakingEq(float freq, float q, float gain) {
    float w0 = freq2w0(freq);
    float amp = std::pow(10.0f, gain / 40.0f);
    float cos_w0 = _cos_w0(w0);
    float sin_w0 = _sin_w0(w0);
    float alpha = sin_w0 / (2.0f * q);
    float a0 = 1.0f + alpha / amp;
    b0 = (1.0f + alpha * amp) / a0;
    b1 = -2.0f * cos_w0 / a0;
    b2 = (1.0f - alpha * amp) / a0;
    a1 = b1;
    a2 = (1.0f - alpha / amp) / a0;
  }

  //lowShelf:
  //  H(s) = A * (s^2 + (sqrt(A)/Q)*s + A)/(A*s^2 + (sqrt(A)/Q)*s + 1)
  //
  //            b0 =    A*( (A+1) - (A-1)*cos(w0) + 2*sqrt(A)*alpha )
  //            b1 =  2*A*( (A-1) - (A+1)*cos(w0)                   )
  //            b2 =    A*( (A+1) - (A-1)*cos(w0) - 2*sqrt(A)*alpha )
  //            a0 =        (A+1) + (A-1)*cos(w0) + 2*sqrt(A)*alpha
  //            a1 =   -2*( (A-1) + (A+1)*cos(w0)                   )
  //            a2 =        (A+1) + (A-1)*cos(w0) - 2*sqrt(A)*alpha
  void lowShelf(float freq, float q, float gain) {
    float amp = std::pow(10.0f, gain / 40.0f);
    float w0 = freq2w0(freq);
    float cos_w0 = _cos_w0(w0);
    float sin_w0 = _sin_w0(w0);
    float alpha = sin_w0 / (2.0f * q);
    float sqrt_amp = std::sqrt(amp);
    float a0 = (amp + 1.0f) + (amp - 1.0f) * cos_w0 + 2.0f * sqrt_amp * alpha;
    b0 = amp * ((amp + 1.0f) - (amp - 1.0f) *cos_w0 + 2.0f * sqrt_amp * alpha) / a0;
    b1 = 2.0f * amp *((amp - 1.0f) - (amp +1.0f) * cos_w0) / a0;
    b2 = amp * ((amp + 1.0f) - (amp - 1.0f) * cos_w0 - 2.0f * sqrt_amp * alpha) / a0;
    a1 = -2.0f * ((amp - 1.0f) + (amp + 1.0f) * cos_w0) / a0;
    a2 = ((amp + 1.0f) + (amp - 1.0f) * cos_w0 - 2.0f * sqrt_amp * alpha) / a0;
  }

  //highShelf:
  //  H(s) = A * (A*s^2 + (sqrt(A)/Q)*s + 1)/(s^2 + (sqrt(A)/Q)*s + A)
  //
  //            b0 =    A*( (A+1) + (A-1)*cos(w0) + 2*sqrt(A)*alpha )
  //            b1 = -2*A*( (A-1) + (A+1)*cos(w0)                   )
  //            b2 =    A*( (A+1) + (A-1)*cos(w0) - 2*sqrt(A)*alpha )
  //            a0 =        (A+1) - (A-1)*cos(w0) + 2*sqrt(A)*alpha
  //            a1 =    2*( (A-1) - (A+1)*cos(w0)                   )
  //            a2 =        (A+1) - (A-1)*cos(w0) - 2*sqrt(A)*alpha
  void highShelf(float freq, float q, float gain) {
    float amp = std::pow(10.0f, gain / 40.0f);
    float w0 = freq2w0(freq);
    float cos_w0 = _cos_w0(w0);
    float sin_w0 = _sin_w0(w0);
    float alpha = sin_w0 / (2.0f * q);
    float sqrt_amp = std::sqrt(amp);
    float a0 = (amp + 1.0f) + (amp - 1.0f) * cos_w0 + 2.0f * sqrt_amp * alpha;
    b0 = amp * ((amp + 1.0f) + (amp - 1.0f) * cos_w0 + 2.0f * sqrt_amp * alpha) / a0;
    b1 = -2.0f * amp * ((amp - 1.0f) + (amp + 1.0f) * cos_w0) / a0;
    b2 = amp * ((amp + 1.0f) + (amp - 1.0f) * cos_w0 - 2.0f * sqrt_amp * alpha) / a0;
    a1 = 2.0f * ((amp - 1.0f) - (amp + 1.0f) * cos_w0) / a0;
    a2 = ((amp + 1.0f) - (amp - 1.0f) * cos_w0 - 2.0f * sqrt_amp * alpha) / a0;
  }

  float process(Context& ctx, float input_value) {
    const float x0 = input_value;

    float y0 = b0 * x0 + b1 * ctx.x1 + b2 * ctx.x2 - a1 * ctx.y1 - a2 * ctx.y2;
    ctx.x2 = ctx.x1;
    ctx.x1 = x0;
    ctx.y2 = ctx.y1;
    ctx.y1 = y0;

    return y0;
  }
};

}
