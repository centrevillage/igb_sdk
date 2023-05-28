#ifndef IGB_UTIL_KNOB_MODE_H
#define IGB_UTIL_KNOB_MODE_H

#include <cstdint>
#include <optional>

namespace igb {

enum class KnobModeType : uint8_t {
  jump = 0,
  catching
};

template<typename ValType>
struct KnobMode {
  KnobModeType type = KnobModeType::jump;

  void changeType(KnobModeType t) {
    type = t;
  }

  std::optional<ValType> newValue(ValType param_v, ValType current_knob_v, ValType prev_knob_v) {
    if (current_knob_v == prev_knob_v) {
      // force loading
      return current_knob_v;
    }
    std::optional<ValType> ret = std::nullopt;
    switch (type) {
      case KnobModeType::jump:
        ret = current_knob_v;
        break;
      case KnobModeType::catching:
        if (prev_knob_v == param_v) {
          ret = current_knob_v;
        } else {
          if (current_knob_v > prev_knob_v) {
            if (current_knob_v > param_v) {
              ret = current_knob_v;
            }
          } else {
            if (current_knob_v < param_v) {
              ret = current_knob_v;
            }
          }
        }
        break;
      default:
        break;
    }
    return ret;
  }
};

}

#endif /* IGB_UTIL_KNOB_MODE_H */
