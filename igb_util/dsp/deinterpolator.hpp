#pragma once

#include <cstdint>
#include <cstddef>
#include <utility>
#include <array>
#include <igb_util/macro.hpp>

namespace igb::dsp {

struct DeinterpNo {
  template<typename T>
  IGB_FAST_INLINE T operator()(const T& value, float) const { return value; }
  template<typename T>
  IGB_FAST_INLINE void update(const T&) {}
};

template<typename T>
struct DeinterpLinear {
  T _prev = {};

  IGB_FAST_INLINE T operator()(const T& value, float t) const {
    return _interp(_prev, value, t);
  }
  IGB_FAST_INLINE void update(const T& value) { _prev = value; }

  static IGB_FAST_INLINE float _interp(float a, float b, float t) {
    return a + t * (b - a);
  }
  static IGB_FAST_INLINE std::pair<float, float> _interp(
      const std::pair<float, float>& a,
      const std::pair<float, float>& b, float t) {
    return {a.first  + t * (b.first  - a.first),
            a.second + t * (b.second - a.second)};
  }
};

template<typename T>
struct DeinterpCubic {
  std::array<T, 4> _prev = {};
  uint8_t _idx = 0;

  IGB_FAST_INLINE T operator()(const T& value, float t) const {
    const T& pm1 = _prev[(_idx + 2) & 3];
    const T& p0  = _prev[(_idx + 3) & 3];
    return _interp(pm1, p0, value, value, t);
  }
  IGB_FAST_INLINE void update(const T& value) {
    _prev[_idx] = value;
    _idx = (_idx + 1) & 3;
  }

  static IGB_FAST_INLINE float _interp(
      float pm1, float p0, float p1, float p2, float t) {
    float c0 = t * (-0.5f + t * (1.0f - 0.5f * t));
    float c1 = 1.0f + t * t * (1.5f * t - 2.5f);
    float c2 = t * (0.5f + t * (2.0f - 1.5f * t));
    float c3 = 0.5f * t * t * (t - 1.0f);
    return c0 * pm1 + c1 * p0 + c2 * p1 + c3 * p2;
  }
  static IGB_FAST_INLINE std::pair<float, float> _interp(
      const std::pair<float, float>& pm1, const std::pair<float, float>& p0,
      const std::pair<float, float>& p1, const std::pair<float, float>& p2,
      float t) {
    float c0 = t * (-0.5f + t * (1.0f - 0.5f * t));
    float c1 = 1.0f + t * t * (1.5f * t - 2.5f);
    float c2 = t * (0.5f + t * (2.0f - 1.5f * t));
    float c3 = 0.5f * t * t * (t - 1.0f);
    return {c0 * pm1.first  + c1 * p0.first  + c2 * p1.first  + c3 * p2.first,
            c0 * pm1.second + c1 * p0.second + c2 * p1.second + c3 * p2.second};
  }
};

}
