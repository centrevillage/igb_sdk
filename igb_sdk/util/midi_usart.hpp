#pragma once

#include <optional>
#include <functional>
#include <igb_sdk/util/midi.hpp>

namespace igb {
namespace sdk {

template<typename usart_t, typename gpio_pin_type, typename usart_conf_t, typename usart_state_t>
struct MidiUsart {
  const std::optional<gpio_pin_type> rx_pin_type = std::nullopt;
  const std::optional<gpio_pin_type> tx_pin_type = std::nullopt;
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
      usart_conf_t {
        .rx_pin_type = rx_pin_type,
        .tx_pin_type = tx_pin_type,
        .enable_rx = rx_pin_type ? true : false,
        .enable_tx = tx_pin_type ? true : false,
        .enable_parity = false,
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
    // Idempotent: the normal receive path calls this defensively before
    // dispatching, which must not fire the user callback again.
    if (!is_sysex_mode) {
      return;
    }
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
      // Re-check is_sysex_mode every byte: once the sysex ends mid-batch the
      // remaining bytes belong to the normal parse path (next process() call)
      // — draining on here would eat the messages that followed the sysex.
      while (is_sysex_mode && (tmp = midi.rx_buffer.get())) {
        uint8_t recv_byte = tmp.value();
        if (recv_byte >= 0xF8) {
          // System Real-Time may interleave a sysex stream WITHOUT
          // terminating it — dispatch it and keep receiving the body.
          if (auto e = midi.feedByte(recv_byte)) {
            event = e;
            if (on_receive) {
              on_receive(e.value());
            }
          }
        } else if (recv_byte & 0x80) {
          _endSysEx();
          if (recv_byte == IGB_MIDI_SYS_EX_START) {
            _startSysEx();
          } else if (recv_byte != IGB_MIDI_SYS_EX_END) {
            // Any other status byte terminates the sysex AND starts a new
            // message — hand it to the byte parser instead of dropping it.
            // (Only a 1-byte message completes here, e.g. Tune Request; a
            // channel/system-common status just seeds the parser state.)
            if (auto e = midi.feedByte(recv_byte)) {
              event = e;
              if (on_receive) {
                on_receive(e.value());
              }
            }
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

    if (usart.is(usart_state_t::txEmpty)) {
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
    while (!usart.is(usart_state_t::txEmpty)) {} // wait
    usart.txData(data);
  }

  // Call this in USARTx_IRQHandler
  IGB_FAST_INLINE void irqHandler() {
    if (usart.is(usart_state_t::rxNotEmpty)) {
      rxHandler();
      return;
    }

    usart.enable(false);
    usart.enable(true);
  }
};

}
}

