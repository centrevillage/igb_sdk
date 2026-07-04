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

  // 0 = no status seen yet (parser start state); real statuses are >= 0x80.
  uint8_t status = 0;
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

  uint8_t _use_running_status = false;
  uint8_t _last_status = 0;

  void useRunningStatus(bool flag) {
    if (_use_running_status != flag) {
      _use_running_status = flag;
      _last_status = 0;
    }
  }

  IGB_FAST_INLINE void addEvent(auto&& event) {
    if (_use_running_status) {
      if (event.status < 0xF8) { // not realtime message
        if (_last_status != event.status) {
          tx_buffer.add(event.status);
          _last_status = event.status;
        }
      } else { // realtime message
        tx_buffer.add(event.status);
      }
    } else {
      tx_buffer.add(event.status);
    }
    if (event.data1 != MidiEvent::noData) {
      tx_buffer.add(event.data1);
    }
    if (event.data2 != MidiEvent::noData) {
      tx_buffer.add(event.data2);
    }
  }

  // Feed one byte to the parser state machine; returns a completed event if
  // this byte finished one. Used by getEvent() (RX ring drain) and by
  // MidiUsart, which hands back a sysex-terminating status byte so the next
  // message isn't dropped.
  std::optional<MidiEvent> feedByte(uint8_t recv_byte) {
    if (recv_byte >= 0xF8) {
      // System Real-Time may interleave ANY other message (even between a
      // status byte and its data bytes) — never touch the partial-message
      // state. Undefined realtime (0xF9 / 0xFD) is discarded the same way.
      switch (recv_byte) {
        case IGB_MIDI_CLOCK:
        case IGB_MIDI_START:
        case IGB_MIDI_CONTINUE:
        case IGB_MIDI_STOP:
        case IGB_MIDI_RESET:
        case IGB_MIDI_ACTIVE_SENSE:
          return MidiEvent { .status = recv_byte };
        default:
          return std::nullopt;
      }
    }
    if (recv_byte & 0x80) {
      // Non-realtime status byte: starts a new message (aborting a partial
      // one) — except the undefined System Common bytes, which are dropped
      // without disturbing the pending state.
      switch (recv_byte) {
        case 0xF4:
        case 0xF5:
          // undefined
          return std::nullopt;
        case IGB_MIDI_SYS_EX_START:
        case IGB_MIDI_SYS_EX_END:
        case IGB_MIDI_TUNE:
          // 1 byte message
          event.data1 = MidiEvent::noData;
          event.data2 = MidiEvent::noData;
          return MidiEvent { .status = recv_byte };
        default:
          event.status = recv_byte;
          event.data1 = MidiEvent::noData;
          event.data2 = MidiEvent::noData;
          return std::nullopt;
      }
    }
    // data byte
    if (!(event.status & 0x80)) {
      // No valid status seen yet (power-on mid-stream / hot-plug): dropping
      // the byte beats emitting a status-0 garbage event, which would even be
      // forwarded on a chain wire as raw data bytes.
      return std::nullopt;
    }
    if (event.data1 == MidiEvent::noData) {
      event.data1 = recv_byte;
    } else if (event.data2 == MidiEvent::noData) {
      event.data2 = recv_byte;
    }
    if (event.data2 != MidiEvent::noData) {
      // Complete 3-byte message. Clear the data bytes (keep the status) so
      // a running-status continuation starts a fresh message instead of
      // re-emitting this one's stale data.
      MidiEvent out = event;
      event.data1 = MidiEvent::noData;
      event.data2 = MidiEvent::noData;
      return out;
    }
    // 2 byte message? Channel-voice statuses carry the channel in the
    // low nibble, so mask it off before matching (0xC0..0xCF are all
    // Program Change); system common (>= 0xF0) must stay unmasked.
    uint8_t status_type = (event.status < 0xF0) ? (event.status & 0xF0)
                                                : event.status;
    switch(status_type) {
      case IGB_MIDI_PROG_CHG:
      case IGB_MIDI_CH_PRESSURE:
      case IGB_MIDI_TIME_CODE:
      case IGB_MIDI_SONG_SELECT: {
        // Complete 2-byte message — same running-status reset as above.
        MidiEvent out = event;
        event.data1 = MidiEvent::noData;
        return out;
      }
      default:
        return std::nullopt;
    }
  }

  std::optional<MidiEvent> getEvent() {
    std::optional<uint8_t> tmp = std::nullopt;
    while ((tmp = rx_buffer.get())) {
      if (auto e = feedByte(tmp.value())) {
        return e;
      }
    }
    return std::nullopt;
  }
};

}
}

