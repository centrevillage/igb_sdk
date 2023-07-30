#pragma once

#include <cstddef>
#include <cstdint>

namespace igb {

struct ValueStabilizerConf {
  uint8_t chattering_count = 2;
  float filter_coeff = 0.1f;
  float chattering_threshold = 2.0f;
  float value_change_threshold = 0.4f;
};

struct ValueStabilizerContext {
  float new_value = 0.0f;
  float current_value = 0.0f;
  uint8_t change_count = 0;
};

struct ValueStabilizerMethod {
  constexpr static bool isOverThreshold(float old_value, float new_value, float threshold_offset) {
    return new_value > (old_value + 1.0f + threshold_offset) || new_value < (old_value - threshold_offset);
  }

  static bool updateContext(float value, ValueStabilizerContext& context, const ValueStabilizerConf& conf) {
    if (isOverThreshold(context.new_value, value, conf.chattering_threshold)) {
      if (context.change_count >= conf.chattering_count) {
        float fc = conf.filter_coeff * (float)(conf.chattering_count + 1);
        if (fc > 1.0f) {
          fc = 1.0f;
        }
        context.new_value = (context.new_value * (1.0f - fc)) + (value * fc);
        context.change_count = 0;
      } else {
        ++context.change_count;
      }
    } else {
      context.new_value = (context.new_value * (1.0f - conf.filter_coeff)) + (value * conf.filter_coeff);
      context.change_count = 0;
    }

    return context.change_count == 0;
  }

  static bool updateCurrentValue(float value, ValueStabilizerContext& context, const ValueStabilizerConf& conf) {
    if (!updateContext(value, context, conf)) {
      return false;
    }
    if (isOverThreshold(context.current_value, context.new_value, conf.value_change_threshold)) {
      context.current_value = (float)((int32_t)context.new_value);
      return true;
    }
    return false;
  }
};

}

