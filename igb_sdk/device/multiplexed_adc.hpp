#ifndef IGB_SDK_DEVICE_MULTIPLEXED_ADC_HPP
#define IGB_SDK_DEVICE_MULTIPLEXED_ADC_HPP

#include <array>
#include <functional>
#include <igb_sdk/base.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace sdk {

template<typename ADC_TYPE, uint8_t ADC_RESOLUTION, typename GPIO_PIN_TYPE, size_t GPIO_PIN_COUNT>
struct MultiplexedAdc {
  static constexpr uint8_t address_size = (1 << GPIO_PIN_COUNT);

  ADC_TYPE adc;
  std::array<GPIO_PIN_TYPE, GPIO_PIN_COUNT> gpios;
  std::function<void(uint8_t, uint32_t, float)> on_update;

  std::array<uint32_t, address_size> _values;
  bool _is_conversioning = false;
  uint8_t process_idx = 0;

  constexpr uint32_t resolutionBits(uint32_t resolution) {
    return (1UL << (uint32_t)resolution) - 1;
  }

  void process() {
    if (!_is_conversioning && adc.checkReady()) {
      for (uint8_t i = 0; i < GPIO_PIN_COUNT; ++i) {
        gpios[i].write(!!(process_idx & (1 << i)));
      }
      adc.startConversion();
      _is_conversioning = true;
    } else {
      if (adc.checkEndOfConversion()) {
        _values[process_idx] = adc.readData();
        if (on_update) {
          on_update(process_idx, getValue(process_idx), getValueFloat(process_idx));
        }
        process_idx = (process_idx + 1) % address_size;
        _is_conversioning = false;
      }
    }
  }

  IGB_FAST_INLINE uint32_t getValue(uint8_t address_idx) {
    return _values[address_idx];
  }
  IGB_FAST_INLINE float getValueFloat(uint8_t address_idx) {
    return (float)getValue(address_idx) / (float)(resolutionBits(ADC_RESOLUTION));
  }
};

}
}


#endif /* IGB_SDK_DEVICE_MULTIPLEXED_ADC_HPP */
