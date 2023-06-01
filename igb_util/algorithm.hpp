#ifndef IGB_UTIL_ALGORITHM_H
#define IGB_UTIL_ALGORITHM_H

// std lib algorithm for specific compilers

namespace igb {

template <class T>
constexpr const T& clamp(const T& v, const T& low, const T& high) noexcept {
  return (v < low) ? low : (v > high ? high : v);
}

constexpr float lerp(float a, float b, float t) noexcept {
  return (1.0f - t) * a + t * b;
}

}

#endif /* IGB_UTIL_ALGORITHM_H */
