#pragma once

#include <igb_util/macro.hpp>
#include <igb_stm32/base/mcu/stm32h750xx.hpp>

namespace igb {

enum class DaisyGpioPinType : uint8_t {
  d0  = static_cast<uint8_t>(stm32::GpioPinType::pb12),
  d1  = static_cast<uint8_t>(stm32::GpioPinType::pc11),
  d2  = static_cast<uint8_t>(stm32::GpioPinType::pc10),
  d3  = static_cast<uint8_t>(stm32::GpioPinType::pc9),
  d4  = static_cast<uint8_t>(stm32::GpioPinType::pc8),
  d5  = static_cast<uint8_t>(stm32::GpioPinType::pd2),
  d6  = static_cast<uint8_t>(stm32::GpioPinType::pc12),
  d7  = static_cast<uint8_t>(stm32::GpioPinType::pg10),
  d8  = static_cast<uint8_t>(stm32::GpioPinType::pg11),
  d9  = static_cast<uint8_t>(stm32::GpioPinType::pb4),
  d10 = static_cast<uint8_t>(stm32::GpioPinType::pb5),
  d11 = static_cast<uint8_t>(stm32::GpioPinType::pb8),
  d12 = static_cast<uint8_t>(stm32::GpioPinType::pb9),
  d13 = static_cast<uint8_t>(stm32::GpioPinType::pb6),
  d14 = static_cast<uint8_t>(stm32::GpioPinType::pb7),
  d15 = static_cast<uint8_t>(stm32::GpioPinType::pc0),
  d16 = static_cast<uint8_t>(stm32::GpioPinType::pa3),
  d17 = static_cast<uint8_t>(stm32::GpioPinType::pb1),
  d18 = static_cast<uint8_t>(stm32::GpioPinType::pa7),
  d19 = static_cast<uint8_t>(stm32::GpioPinType::pa6),
  d20 = static_cast<uint8_t>(stm32::GpioPinType::pc1),
  d21 = static_cast<uint8_t>(stm32::GpioPinType::pc4),
  d22 = static_cast<uint8_t>(stm32::GpioPinType::pa5),
  d23 = static_cast<uint8_t>(stm32::GpioPinType::pa4),
  d24 = static_cast<uint8_t>(stm32::GpioPinType::pa1),
  d25 = static_cast<uint8_t>(stm32::GpioPinType::pa0),
  d26 = static_cast<uint8_t>(stm32::GpioPinType::pd11),
  d27 = static_cast<uint8_t>(stm32::GpioPinType::pg9),
  d28 = static_cast<uint8_t>(stm32::GpioPinType::pa2),
  d29 = static_cast<uint8_t>(stm32::GpioPinType::pb14),
  d30 = static_cast<uint8_t>(stm32::GpioPinType::pb15),
};

constexpr IGB_FAST_INLINE stm32::GpioPinType daisy_pin_to_stm32_pin(DaisyGpioPinType pin_type) {
  return static_cast<stm32::GpioPinType>(static_cast<uint8_t>(pin_type));
}

}

