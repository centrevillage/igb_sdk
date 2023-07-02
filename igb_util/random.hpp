#pragma once

#include <cstdint>
#include <igb_util/macro.hpp>

namespace igb {

struct RandomXorshift {
  uint32_t x = 123456789;
  uint32_t y = 362436069;
  uint32_t z = 521288629;
  uint32_t w = 88675123;

  inline void init(uint32_t seed) {
    z = seed;
    w = x ^ z;
  }

  inline uint32_t get() {
    uint32_t t = x ^ (x << 11);
    x = y;
    y = z;
    z = w;
    w = (w ^ (w >> 19)) ^ (t ^ (t >> 8)); 
    return w;
  }

  inline float getf() {
    return (float)get() / (float)0xFFFFFFFF;
  }

  // singleton
  static RandomXorshift& instance() {
    static RandomXorshift random;
    return random;
  }
};

IGB_FAST_INLINE uint32_t rand_u32() {
  return RandomXorshift::instance().get();
}

IGB_FAST_INLINE float rand_f() {
  return RandomXorshift::instance().getf();
}

}

