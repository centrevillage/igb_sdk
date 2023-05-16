#ifndef IGB_DAISY_PERIPH_SPI_H
#define IGB_DAISY_PERIPH_SPI_H


#include <igb_stm32/base/mcu/stm32h750xx.hpp>
#include <igb_stm32/periph/spi.hpp>
#include <igb_daisy/periph/gpio.hpp>

namespace igb {

typedef stm32::Spi<
  stm32::SpiType::spi1,
  daisy_pin_to_stm32_pin(DaisyGpioPinType::d10), // MOSI
  daisy_pin_to_stm32_pin(DaisyGpioPinType::d9), // MISO
  daisy_pin_to_stm32_pin(DaisyGpioPinType::d8) // SCK
  > DaisySpi;

}

#endif /* IGB_DAISY_PERIPH_SPI_H */
