#ifndef IGB_UTIL_DSP_FUNC_H
#define IGB_UTIL_DSP_FUNC_H

#include <igb_util/macro.hpp>

namespace igb {

IGB_FAST_INLINE float df_softclip(float v) {
  v *= 2.0f / 3.0f;
  if (v < -1.0f) {
    return -1.0f;
  } else if (v > 1.0f) {
    return 1.0f;
  }
  return (v - (v * v * v) / 3.0f) * 3.0f / 2.0f;
}

};

#endif /* IGB_UTIL_DSP_FUNC_H */
