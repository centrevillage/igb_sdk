#pragma once

#include <cmath>
#include <igb_util/macro.hpp>
#include <igb_util/math.hpp>
#include <igb_util/dsp/config.hpp>

namespace igb::dsp {

IGB_FAST_INLINE float tau2pole(float tau, uint32_t sampling_rate) {
  if (tau < igb::numbers::epsilon_f) {
    return 0.0f;
  }
  return std::exp(-1.0f / (tau * (float)sampling_rate));
}

IGB_FAST_INLINE float tau2pole(float tau) {
  return tau2pole(tau, Config::getSamplingRateF());
}

IGB_FAST_INLINE float linear2db(float g) {
  return 20.0f * std::log10(std::max(igb::numbers::min_f, g));
}

IGB_FAST_INLINE float db2linear(float l) {
  return std::pow(10.0f, l / 20.0f);
}

}
