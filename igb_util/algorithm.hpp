#pragma once

// std lib algorithm for specific compilers

#include <igb_util/macro.hpp>

namespace igb {

// always_inline (LilaC #202): these run on audio-IRQ (ITCM) hot paths.
// constexpr implies inline but NOT always_inline — once the caller grows past
// GCC's inline budget these get outlined to flash and long-called through
// veneers per frame (the #62 class, caught by the nm veneer audit).
template <class T>
IGB_FAST_INLINE constexpr const T& clamp(const T& v, const T& low, const T& high) noexcept {
  return (v < low) ? low : (v > high ? high : v);
}

template <class T>
IGB_FAST_INLINE constexpr T lerp(T a, T b, T t) noexcept {
  return a + (b - a) * t;
}

}

