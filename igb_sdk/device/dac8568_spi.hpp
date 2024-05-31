#pragma once

#include <igb_sdk/base.hpp>
#include <functional>
#include <igb_util/macro.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/null_functor.hpp>

namespace igb {
namespace sdk {

template<typename SPI_TYPE, typename GPIO_PIN_TYPE, typename WAIT_FUNC = NullFunctor>
struct Dac8568SPI {
  // public attributes {{{
  SPI_TYPE spi;
  GPIO_PIN_TYPE cs_pin; // = sync_pin
  // }}}

  WAIT_FUNC _wait_func;

  enum class LoadCmd : uint8_t {
    noLoad = 0,
    single = 0x03,
    all = 0x02,
  };

  void init() {
    cs_pin.enable();
    cs_pin.initOutputDefault();
    cs_pin.high();
  }

  IGB_FAST_INLINE void _writeByte(uint8_t byte) {
    volatile uint8_t tmp IGB_UNUSED = spi.transferU8sync(byte, _wait_func);
  }

  IGB_FAST_INLINE void writeData(uint8_t ch /* 0 ~ 7 */, uint16_t data, LoadCmd load_cmd = LoadCmd::single) {
    cs_pin.low();
    _writeByte(as<uint8_t>(load_cmd));
    _writeByte(ch << 4 | (data >> 12));
    _writeByte((data >> 4) & 0xFF);
    _writeByte((data << 4) & 0xF0);
    cs_pin.high();
  }
};

}
}
