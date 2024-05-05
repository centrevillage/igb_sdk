#pragma once

#include <igb_stm32/periph/systick.hpp>
#include <igb_sdk/base.hpp>
#include <functional>
#include <igb_util/macro.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/null_functor.hpp>

namespace igb {
namespace sdk {

template<typename SPI_TYPE, typename GPIO_PIN_TYPE, typename WAIT_FUNC = NullFunctor>
struct Mcp33151SPI {
  // public attributes {{{
  SPI_TYPE spi;
  GPIO_PIN_TYPE cs_pin;
  // }}}

  WAIT_FUNC _wait_func;

  void init(uint8_t address = 0) {
    cs_pin.enable();
    cs_pin.initOutputDefault();
    cs_pin.high();
  }

  IGB_FAST_INLINE uint16_t getData() {
    // full duplex
    cs_pin.low();
    uint16_t msb = spi.transferU8sync(0, _wait_func);
    uint16_t lsb = spi.transferU8sync(0, _wait_func);
    cs_pin.high();
    return  ((msb << 8) | lsb);;
  }
};

}
}
