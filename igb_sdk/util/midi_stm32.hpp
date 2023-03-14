#ifndef IGB_SDK_UTIL_MIDI_STM32_H
#define IGB_SDK_UTIL_MIDI_STM32_H


#include <optional>
#include <functional>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/usart.hpp>
#include <igb_sdk/util/midi.hpp>

// MIDI for igb_stm32

namespace igb {
namespace sdk {

template<typename usart_t>
struct MidiStm32 {
  const std::optional<igb::stm32::GpioPinType> rx_pin_type = std::nullopt;
  const std::optional<igb::stm32::GpioPinType> tx_pin_type = std::nullopt;
  std::function<void(const MidiEvent& event)> on_receive;
  std::function<void(void)> on_sysex_start;
  std::function<void(void)> on_sysex_end;
  std::function<void(uint8_t)> on_sysex_receive;
  
  uint8_t is_sysex_mode = false;

  usart_t usart;
  Midi midi;

  IGB_FAST_INLINE void send(auto&& event) {
    midi.addEvent(event);
  }

  void initUsart(uint32_t usart_clock_freq, uint8_t interrupt_priority = 1) {
    usart.init(
      igb::stm32::UsartConf {
        .rx_pin_type = rx_pin_type,
        .tx_pin_type = tx_pin_type,
        .enable_rx = rx_pin_type ? true : false,
        .enable_tx = tx_pin_type ? true : false,
        .enable_parity = false,
        .data_width = igb::stm32::UsartDataWidth::_8bit,
        .base_clock_freq = usart_clock_freq,
        .baudrate = 31250,
        .enable_it_rx_not_empty = rx_pin_type ? true : false,
        //.enable_it_tx_empty = tx_pin_type ? true : false,
        .interrupt_priority = interrupt_priority
      }
    );
  }

  // TODO: USARTのクロックを自動計算
  void init(uint32_t usart_clock_freq) {
    initUsart(usart_clock_freq);
  }

  IGB_FAST_INLINE void _startSysEx() {
    is_sysex_mode = true;
    if (on_sysex_start) {
      on_sysex_start();
    }
  }

  IGB_FAST_INLINE void _endSysEx() {
    is_sysex_mode = false;
    if (on_sysex_end) {
      on_sysex_end();
    }
  }

  IGB_FAST_INLINE void _receiveSysEx(uint8_t data) {
    if (on_sysex_receive) {
      on_sysex_receive(data);
    }
  }

  IGB_FAST_INLINE std::optional<MidiEvent> process() {
    std::optional<MidiEvent> event = std::nullopt;
    if (is_sysex_mode) {
      std::optional<uint8_t> tmp = std::nullopt;
      while ((tmp = midi.rx_buffer.get())) {
        uint8_t recv_byte = tmp.value();
        if (recv_byte & 0x80) {
          _endSysEx();
          if (recv_byte == IGB_MIDI_SYS_EX_START) {
            _startSysEx();
          }
        } else {
          // sysex data
          _receiveSysEx(recv_byte);
        }
      }
    } else {
      event = midi.getEvent();
      if (event) {
        auto v = event.value();
        if (v.status == IGB_MIDI_SYS_EX_START) {
          _startSysEx();
        } else if (v.status == IGB_MIDI_SYS_EX_END) {
          _endSysEx();
        } else if (on_receive) {
          _endSysEx();
          on_receive(v);
        }
      }
    }

    if (usart.is(igb::stm32::UsartState::txEmpty)) {
      txHandler();
    }

    return event;
  }

  IGB_FAST_INLINE void rxHandler() {
    midi.rx_buffer.add(usart.rxData());
  }

  IGB_FAST_INLINE void txHandler() {
    std::optional<uint8_t> send_byte = midi.tx_buffer.get();
    if (send_byte) {
      usart.txData(send_byte.value());
    }
  }

  // for high pripority data (clock etc...)
  IGB_FAST_INLINE void sendDataDirect(uint8_t data) {
    while (!usart.is(igb::stm32::UsartState::txEmpty)) {} // wait
    usart.txData(data);
  }

  // Call this in USARTx_IRQHandler
  IGB_FAST_INLINE void irqHandler() {
    if (usart.is(igb::stm32::UsartState::rxNotEmpty)) {
      rxHandler();
      return;
    }

    usart.enable(false);
    usart.enable(true);
  }
};

}
}

#endif /* IGB_SDK_UTIL_MIDI_STM32_H */
