#pragma once

#include <igb_stm32/base/mcu/stm32h750xx.hpp>
#include <igb_stm32/periph/i2c.hpp>
#include <igb_daisy/periph/gpio.hpp>

namespace igb {

typedef stm32::I2c<
  stm32::I2cType::i2c1,
  daisy_pin_to_stm32_pin(DaisyGpioPinType::d11), // SCL
  daisy_pin_to_stm32_pin(DaisyGpioPinType::d12)  // SDA
  > DaisyI2c;
}

