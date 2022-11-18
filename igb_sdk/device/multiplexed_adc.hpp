#ifndef IGB_SDK_DEVICE_MULTIPLEXED_ADC_HPP
#define IGB_SDK_DEVICE_MULTIPLEXED_ADC_HPP

#include <array>
#include <functional>
#include <cmath>
#include <igb_sdk/base.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace sdk {

template<typename ADC_TYPE, uint8_t ADC_RESOLUTION, typename GPIO_PIN_TYPE, size_t gpio_pin_count>
struct MultiplexedAdc {
  static constexpr uint8_t address_size = (1 << gpio_pin_count);

  ADC_TYPE adc;
  std::array<GPIO_PIN_TYPE, gpio_pin_count> gpios;
  std::function<void(uint8_t, uint32_t, float)> on_update;
  float threshold = 0.4f;
  float filter_coeff = 0.1f;
  uint8_t chattering_count = 2;
  float chattering_threshold = 2.0f;

  std::array<float, address_size> _buf;
  std::array<float, address_size> _values;
  std::array<uint8_t, address_size> _change_counts; // to prevent chattering
  bool _is_conversioning = false;
  uint8_t process_idx = 0;

  constexpr uint32_t resolutionBits(uint32_t resolution) {
    return (1UL << (uint32_t)resolution) - 1;
  }

  constexpr bool isOverThreshold(float old_value, float new_value, float threshold_offset) {
    return new_value > (old_value + 1.0f + threshold_offset) || new_value < (old_value - threshold_offset);
  }

  IGB_FAST_INLINE void prepare() {
    for (uint8_t i = 0; i < gpio_pin_count; ++i) {
      gpios[i].write(!!(process_idx & (1 << i)));
    }
  }

  IGB_FAST_INLINE void complete(uint32_t value) {
    const float v = (float)value;
    const float prev_v = _buf[process_idx];
    if (isOverThreshold(prev_v, v, chattering_threshold)) {
      if (_change_counts[process_idx] >= chattering_count) {
        float fc = filter_coeff * (float)(chattering_count + 1);
        if (fc > 1.0f) {
          fc = 1.0f;
        }
        _buf[process_idx] = (prev_v * (1.0f - fc)) + (v * fc);
        _change_counts[process_idx] = 0;
      } else {
        ++_change_counts[process_idx];
      }
    } else {
      _buf[process_idx] = (prev_v * (1.0f - filter_coeff)) + (v * filter_coeff);
    }
  }
  
  IGB_FAST_INLINE void next() {
    process_idx = (process_idx + 1) % address_size;
    if (process_idx == 0) {
      // complete all
      update();
    }
  }

  IGB_FAST_INLINE bool nextWithoutUpdate() {
    process_idx = (process_idx + 1) % address_size;
    if (process_idx == 0) {
      return true; // need to update
    }
    return false;
  }

  IGB_FAST_INLINE void update() {
    for (uint8_t i = 0; i < address_size; ++i) {
      float new_value = _buf[i];
      const auto old_value = _values[i];
      if (isOverThreshold(old_value, new_value, threshold)) {
        _values[i] = (float)((int32_t)new_value);
        if (on_update) {
          on_update(i, getValue(i), getValueFloat(i));
        }
      }
    }
  }

  IGB_FAST_INLINE void forceUpdate() {
    if (on_update) {
      for (uint8_t i = 0; i < address_size; ++i) {
        on_update(i, getValue(i), getValueFloat(i));
      }
    }
  }

  IGB_FAST_INLINE void start() {
    adc.startConversion();
  }

  IGB_FAST_INLINE void process() {
    if (!_is_conversioning && adc.checkReady()) {
      prepare();
      start();
      _is_conversioning = true;
    } else {
      if (adc.checkEndOfConversion()) {
        complete(adc.readData());
        next();
        _is_conversioning = false;
      }
    }
  }

  IGB_FAST_INLINE uint32_t getValue(uint8_t address_idx) {
    return _values[address_idx];
  }
  IGB_FAST_INLINE float getValueFloat(uint8_t address_idx) {
    return _values[address_idx] / (float)(resolutionBits(ADC_RESOLUTION));
  }
};

}
}


#endif /* IGB_SDK_DEVICE_MULTIPLEXED_ADC_HPP */
