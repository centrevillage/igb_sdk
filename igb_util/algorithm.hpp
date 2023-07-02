#pragma once

// std lib algorithm for specific compilers

namespace igb {

template <class T>
constexpr const T& clamp(const T& v, const T& low, const T& high) noexcept {
  return (v < low) ? low : (v > high ? high : v);
}

template <class T>
constexpr T lerp(T a, T b, T t) noexcept {
  return a + (b - a) * t;
}

}

