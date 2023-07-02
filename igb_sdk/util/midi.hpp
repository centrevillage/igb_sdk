#pragma once

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
  IGB_MIDI_CONTINUE = 0xFB,
  IGB_MIDI_STOP = 0xFC,
  IGB_MIDI_RESET = 0xFF,
  IGB_MIDI_CC = 0xB0,
  IGB_MIDI_PROG_CHG = 0xC0,
  IGB_MIDI_SYS_EX_START = 0xF0,
  IGB_MIDI_SYS_EX_END = 0xF7,
  IGB_MIDI_PITCHBEND = 0xE0,
  IGB_MIDI_CH_PRESSURE = 0xD0,
  IGB_MIDI_POLY_KEY_PRESSURE= 0xA0,
  IGB_MIDI_TIME_CODE = 0xF1,
  IGB_MIDI_SONG_POSITION = 0xF2,
  IGB_MIDI_SONG_SELECT = 0xF3,
  IGB_MIDI_TUNE = 0xF6,
  IGB_MIDI_ACTIVE_SENSE = 0xFE,
} MidiStatusByte;

enum class MidiStatus : uint8_t {
  noteOn = IGB_MIDI_NOTE_ON,
  noteOff = IGB_MIDI_NOTE_OFF,
  clock = IGB_MIDI_CLOCK,
  start = IGB_MIDI_START,
  _continue = IGB_MIDI_CONTINUE,
  stop = IGB_MIDI_STOP,
  reset = IGB_MIDI_RESET,
  cc = IGB_MIDI_CC,
  progChg = IGB_MIDI_PROG_CHG,
  sysExStart = IGB_MIDI_SYS_EX_START,
  sysExEnd = IGB_MIDI_SYS_EX_END,
  pitchbend = IGB_MIDI_PITCHBEND,
  chPressure = IGB_MIDI_CH_PRESSURE,
  polyKeyPressure = IGB_MIDI_POLY_KEY_PRESSURE,
  timeCode = IGB_MIDI_TIME_CODE,
  songPosition = IGB_MIDI_SONG_POSITION,
  songSelect = IGB_MIDI_SONG_SELECT,
  tune = IGB_MIDI_TUNE,
  activeSense = IGB_MIDI_ACTIVE_SENSE,
};
constexpr uint8_t to_idx(MidiStatus status) {
  switch (status) {
    case MidiStatus::noteOn:
      return IGB_MIDI_NOTE_ON;
      break;
    case MidiStatus::noteOff:
      return IGB_MIDI_NOTE_OFF;
      break;
    case MidiStatus::clock:
      return IGB_MIDI_CLOCK;
      break;
    case MidiStatus::start:
      return IGB_MIDI_START;
      break;
    case MidiStatus::_continue:
      return IGB_MIDI_CONTINUE;
      break;
    case MidiStatus::stop:
      return IGB_MIDI_STOP;
      break;
    case MidiStatus::reset:
      return IGB_MIDI_RESET;
      break;
    case MidiStatus::cc:
      return IGB_MIDI_CC;
      break;
    case MidiStatus::progChg:
      return IGB_MIDI_PROG_CHG;
      break;
    case MidiStatus::sysExStart:
      return IGB_MIDI_SYS_EX_START;
      break;
    case MidiStatus::sysExEnd:
      return IGB_MIDI_SYS_EX_END;
      break;
    case MidiStatus::pitchbend:
      return IGB_MIDI_PITCHBEND;
      break;
    case MidiStatus::chPressure:
      return IGB_MIDI_CH_PRESSURE;
      break;
    case MidiStatus::polyKeyPressure:
      return IGB_MIDI_POLY_KEY_PRESSURE;
      break;
    case MidiStatus::timeCode:
      return IGB_MIDI_TIME_CODE;
      break;
    case MidiStatus::songPosition:
      return IGB_MIDI_SONG_POSITION;
      break;
    case MidiStatus::songSelect:
      return IGB_MIDI_SONG_SELECT;
      break;
    case MidiStatus::tune:
      return IGB_MIDI_TUNE;
      break;
    case MidiStatus::activeSense:
      return IGB_MIDI_ACTIVE_SENSE;
      break;
    default:
      break;
  }
  return 0;
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
          case IGB_MIDI_CONTINUE:
          case IGB_MIDI_STOP:
          case IGB_MIDI_CLOCK:
          case IGB_MIDI_RESET:
          case IGB_MIDI_SYS_EX_START:
          case IGB_MIDI_SYS_EX_END:
          case IGB_MIDI_TUNE:
          case IGB_MIDI_ACTIVE_SENSE:
            // 1 byte message
            return MidiEvent { .status = recv_byte };
            break;
          case 0xF4:
          case 0xF5:
          case 0xF9:
          case 0xFD:
            // undefined
            break;
          default:
            event.status = recv_byte;
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
          // 2 byte message?
          switch(event.status) {
            case IGB_MIDI_PROG_CHG:
            case IGB_MIDI_CH_PRESSURE:
            case IGB_MIDI_TIME_CODE:
            case IGB_MIDI_SONG_SELECT:
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

