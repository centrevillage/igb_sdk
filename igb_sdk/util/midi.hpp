#ifndef IGB_SDK_UTIL_MIDI_H
#define IGB_SDK_UTIL_MIDI_H

#include <igb_sdk/base.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/ring_buf.hpp>
#include <optional>

namespace igb {
namespace sdk {

typedef enum {
  IGB_MIDI_NOTE_ON = 0x90,
  IGB_MIDI_NOTE_OFF = 0x80,
  IGB_MIDI_CLOCK = 0xF8,
  IGB_MIDI_START = 0xFA,
  IGB_MIDI_STOP = 0xFC,
  IGB_MIDI_RESET = 0xFF,
  IGB_MIDI_CC = 0xB0,
  IGB_MIDI_PROG_CHG = 0xC0,
  IGB_MIDI_SYS_EX_START = 0xF0,
  IGB_MIDI_SYS_EX_END = 0xF7,
  IGB_MIDI_PITCHBEND = 0xE0,
  IGB_MIDI_CH_PRESSURE = 0xD0,
  IGB_MIDI_POLY_KEY_PRESSURE= 0xA0,
} MidiStatusByte;

enum class MidiStatus : uint8_t {
  noteOn = IGB_MIDI_NOTE_ON,
  noteOff = IGB_MIDI_NOTE_OFF,
  clock = IGB_MIDI_CLOCK,
  start = IGB_MIDI_START,
  stop = IGB_MIDI_STOP,
  reset = IGB_MIDI_RESET,
  cc = IGB_MIDI_CC,
  progChg = IGB_MIDI_PROG_CHG,
  sysExStart = IGB_MIDI_SYS_EX_START,
  sysExEnd = IGB_MIDI_SYS_EX_END,
  pitchbend = IGB_MIDI_PITCHBEND,
  chPressure = IGB_MIDI_CH_PRESSURE,
  polyKeyPressure = IGB_MIDI_POLY_KEY_PRESSURE
};

struct MidiEvent {
  constexpr static uint8_t noData = 255;

  uint8_t status;
  uint8_t data1 = noData;
  uint8_t data2 = noData;

  IGB_FAST_INLINE bool is(const MidiStatus _status) const {
    if (status >= 0xF0) {
      return status == static_cast<uint8_t>(_status);
    }
    // channel message
    return (status & 0xF0) == static_cast<uint8_t>(_status);
  }

  inline static bool isStatusByte(uint8_t byte) {
    return byte & 0x80;
  }
};

struct Midi {
  RingBuf256<uint8_t> rx_buffer;
  RingBuf256<uint8_t> tx_buffer;
  MidiEvent event;

  IGB_FAST_INLINE void addEvent(auto&& event) {
    tx_buffer.add(event.status);
    if (event.data1 != MidiEvent::noData) {
      tx_buffer.add(event.data1);
    }
    if (event.data2 != MidiEvent::noData) {
      tx_buffer.add(event.data2);
    }
  }

  // TODO: processing sysex
  std::optional<MidiEvent> getEvent() {
    std::optional<uint8_t> tmp = std::nullopt;
    while ((tmp = rx_buffer.get())) {
      uint8_t recv_byte = tmp.value();
      if (recv_byte & 0x80) {
        event.data1 = MidiEvent::noData;
        event.data2 = MidiEvent::noData;
        // status byte
        switch (recv_byte) {
          case IGB_MIDI_START:
          case IGB_MIDI_STOP:
          case IGB_MIDI_CLOCK:
          case IGB_MIDI_RESET:
            return MidiEvent { .status = recv_byte };
          case IGB_MIDI_SYS_EX_START:
            // TODO:
            break;
          default:
            if ((recv_byte & 0xF0) != 0xF0) {
              event.status = recv_byte;
            }
            break;
        }
      } else {
        // data byte
        if (event.data1 == MidiEvent::noData) {
          event.data1 = recv_byte;
        } else if (event.data2 == MidiEvent::noData) {
          event.data2 = recv_byte;
        }
        if (event.data2 != MidiEvent::noData) {
          return event;
        } else {
          switch(event.status) {
            case IGB_MIDI_PROG_CHG:
            case IGB_MIDI_CH_PRESSURE:
              return event;
            default:
              break;
          }
        }
      }
    }
    return std::nullopt;
  }
};

}
}

#endif /* IGB_SDK_UTIL_MIDI_H */
