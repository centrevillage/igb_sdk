#pragma once


#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/usart.hpp>
#include <igb_sdk/util/midi_usart.hpp>

// MIDI for igb_stm32
namespace igb {
namespace sdk {

template<typename usart_t>
using MidiStm32 = MidiUsart<usart_t, igb::stm32::GpioPinType, igb::stm32::UsartConf, igb::stm32::UsartState>;

}
}

