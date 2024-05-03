#pragma once

#include <igb_sdk/base.hpp>
#include <functional>
#include <igb_util/macro.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/null_functor.hpp>

namespace igb {
namespace sdk {

template<typename SPI_TYPE, typename GPIO_PIN_TYPE, typename WAIT_FUNC = NullFunctor>
struct Dac8554SPI {
  // public attributes {{{
  SPI_TYPE spi;
  GPIO_PIN_TYPE cs_pin; // = sync_pin
  // }}}

  WAIT_FUNC _wait_func;
  uint8_t _address = 0;

  enum class LoadCmd : uint8_t {
    noLoad = 0,
    single = 0x10,
    all = 0x20,
    broadCast = 0x30
  };

  void init(uint8_t address = 0) {
    _address = address << 6;

    cs_pin.enable();
    cs_pin.initOutputDefault();
    cs_pin.high();
  }

  // TODO: for DMA operation
  // void initForDMA() {}

  IGB_FAST_INLINE void _writeByte(uint8_t byte) {
    volatile uint8_t tmp IGB_UNUSED = spi.transferU8sync(byte, _wait_func);
  }

  IGB_FAST_INLINE void writeData(uint8_t ch /* 0 ~ 3 */, uint16_t data, LoadCmd load_cmd = LoadCmd::single) {
    const uint8_t head = _address | as<uint8_t>(load_cmd) | (ch << 1);
    cs_pin.low();
    _writeByte(head);
    _writeByte(data >> 8);
    _writeByte(data & 0xFF);
    cs_pin.high();
  }
};

}
}
