#pragma once

#include <cmath>
#include <igb_util/macro.hpp>
#include <igb_util/math.hpp>

namespace igb {
namespace numbers {

constexpr inline static double pi    = 3.1415926535897932;
constexpr inline static double sqrt2 = 1.4142135623730951;
constexpr inline static float epsilon_f = 1.192092896e-07;
constexpr inline static float min_f = 1.175494351e-38;

}

IGB_FAST_INLINE float tau2pole(float tau, float sampling_rate) {
  if (tau < igb::numbers::epsilon_f) {
    return 0.0f;
  }
  return std::exp(-1.0f / (tau * (float)sampling_rate));
}

IGB_FAST_INLINE float linear2db(float g) {
  return 20.0f * std::log10(std::max(igb::numbers::min_f, g));
}

IGB_FAST_INLINE float db2linear(float l) {
  return std::pow(10.0f, l / 20.0f);
}

}
