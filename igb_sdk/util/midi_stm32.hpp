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

template<typename USART>
struct MidiStm32 {
  const std::optional<igb::stm32::GpioPinType> rx_pin_type = std::nullopt;
  const std::optional<igb::stm32::GpioPinType> tx_pin_type = std::nullopt;
  std::function<void(const MidiEvent& event)> on_receive;

  USART usart;
  Midi midi;

  bool _midi_transfer_interrupt_enabled = false;

  IGB_FAST_INLINE void send(auto&& event) {
    midi.addEvent(event);
  }

  void initUsart(uint32_t usart_clock_freq) {
    usart.init(
      igb::stm32::UsartConf {
        .rx_pin_type = rx_pin_type,
        .tx_pin_type = tx_pin_type,
        .enable_rx = rx_pin_type ? true : false,
        .enable_tx = tx_pin_type ? true : false,
        .enable_it_rx_not_empty = rx_pin_type ? true : false,
        //.enable_it_tx_empty = tx_pin_type ? true : false,
        .enable_parity = false,
        .data_width = igb::stm32::UsartDataWidth::_8bit,
        .base_clock_freq = usart_clock_freq,
        .baudrate = 31250,
        .interrupt_priority = 0
      }
    );
  }

  // TODO: USARTのクロックを自動計算
  void init(uint32_t usart_clock_freq) {
    initUsart(usart_clock_freq);
  }

  IGB_FAST_INLINE std::optional<MidiEvent> process() {
    std::optional<MidiEvent> event = midi.getEvent();
    if (on_receive && event) {
      on_receive(event.value());
    }

    if (!_midi_transfer_interrupt_enabled) {
      if (usart.is(igb::stm32::UsartState::txEmpty)) {
        txHandler();
      }
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
      if (!_midi_transfer_interrupt_enabled) {
        usart.enableItTxEmpty(true);
        _midi_transfer_interrupt_enabled = true;
      }
    } else {
      if (_midi_transfer_interrupt_enabled) {
        usart.enableItTxEmpty(false);
        _midi_transfer_interrupt_enabled = false;
      }
    }
  }

  // Call this in USARTx_IRQHandler
  IGB_FAST_INLINE void irqHandler() {
    if (usart.is(igb::stm32::UsartState::rxNotEmpty)) {
      rxHandler();
      // usart.clear(UsartState::rxNotEmpty);
      return;
    }
    if (usart.is(igb::stm32::UsartState::txEmpty)) {
      txHandler();
      // usart.clear(UsartState::txEmpty);
      return;
    }

    usart.enable(false);
    usart.enable(true);
  }
};

}
}

#endif /* IGB_SDK_UTIL_MIDI_STM32_H */
