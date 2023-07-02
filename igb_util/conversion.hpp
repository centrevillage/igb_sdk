#pragma once

#include <igb_util/macro.hpp>
#include <functional>

namespace igb {

IGB_FAST_INLINE auto create_bits(auto&& container, auto&& f) {
  uint32_t bits = 0;
  uint8_t i = 0;
  for (const auto& elem : container) {
    if (f(elem)) {
      bits |= 1UL << i;
    }
    ++i;
  }
  return bits;
}

}

