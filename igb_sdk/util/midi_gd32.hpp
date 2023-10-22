#pragma once

#include <igb_gd32/base.hpp>
#include <igb_gd32/periph/gpio.hpp>
#include <igb_gd32/periph/usart.hpp>
#include <igb_sdk/util/midi_usart.hpp>

// MIDI for igb_gd32
namespace igb {
namespace sdk {

template<typename usart_t>
using MidiGd32 = MidiUsart<usart_t, igb::gd32::GpioPinType, igb::gd32::UsartConf, igb::gd32::UsartState>;

}
}

