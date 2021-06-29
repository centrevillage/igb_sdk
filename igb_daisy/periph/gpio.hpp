#ifndef IGB_DAISY_PERIPH_GPIO_H
#define IGB_DAISY_PERIPH_GPIO_H

#include <igb_util/macro.hpp>
#include <igb_stm32/base/mcu/stm32h750xx.hpp>

namespace igb {
namespace daisy {

enum class DaisyGpioPinType : uint8_t {
  p1  = static_cast<uint8_t>(stm32::GpioPinType::pb12),
  p2  = static_cast<uint8_t>(stm32::GpioPinType::pc11),
  p3  = static_cast<uint8_t>(stm32::GpioPinType::pc10),
  p4  = static_cast<uint8_t>(stm32::GpioPinType::pc9),
  p5  = static_cast<uint8_t>(stm32::GpioPinType::pc8),
  p6  = static_cast<uint8_t>(stm32::GpioPinType::pd2),
  p7  = static_cast<uint8_t>(stm32::GpioPinType::pc12),
  p8  = static_cast<uint8_t>(stm32::GpioPinType::pg10),
  p9  = static_cast<uint8_t>(stm32::GpioPinType::pg11),
  p10 = static_cast<uint8_t>(stm32::GpioPinType::pb4),
  p11 = static_cast<uint8_t>(stm32::GpioPinType::pb5),
  p12 = static_cast<uint8_t>(stm32::GpioPinType::pb8),
  p13 = static_cast<uint8_t>(stm32::GpioPinType::pb9),
  p14 = static_cast<uint8_t>(stm32::GpioPinType::pb6),
  p15 = static_cast<uint8_t>(stm32::GpioPinType::pb7),
  p16 = static_cast<uint8_t>(stm32::GpioPinType::pc0),
  p17 = static_cast<uint8_t>(stm32::GpioPinType::pa3),
  p18 = static_cast<uint8_t>(stm32::GpioPinType::pb1),
  p19 = static_cast<uint8_t>(stm32::GpioPinType::pa7),
  p20 = static_cast<uint8_t>(stm32::GpioPinType::pa6),
  p21 = static_cast<uint8_t>(stm32::GpioPinType::pc1),
  p22 = static_cast<uint8_t>(stm32::GpioPinType::pc4),
  p23 = static_cast<uint8_t>(stm32::GpioPinType::pa5),
  p24 = static_cast<uint8_t>(stm32::GpioPinType::pa4),
  p25 = static_cast<uint8_t>(stm32::GpioPinType::pa1),
  p26 = static_cast<uint8_t>(stm32::GpioPinType::pa0),
  p27 = static_cast<uint8_t>(stm32::GpioPinType::pd11),
  p28 = static_cast<uint8_t>(stm32::GpioPinType::pg9),
  p29 = static_cast<uint8_t>(stm32::GpioPinType::pa2),
  p30 = static_cast<uint8_t>(stm32::GpioPinType::pb14),
  p31 = static_cast<uint8_t>(stm32::GpioPinType::pb15),
};

constexpr IGB_FAST_INLINE stm32::GpioPinType daisy_pin_to_stm32_pin(DaisyGpioPinType pin_type) {
  return static_cast<stm32::GpioPinType>(static_cast<uint8_t>(pin_type));
}

}
}

#endif /* IGB_DAISY_PERIPH_GPIO_H */
