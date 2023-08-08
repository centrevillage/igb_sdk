#pragma once

#include <igb_sdk/base.hpp>
#include <igb_util/ring_buf.hpp>

namespace igb {
namespace sdk {

template<typename BtnIdType, size_t history_size = 8>
struct ButtonHistory {
  struct Info {
    uint32_t tick;
    BtnIdType id;
    uint8_t state; // 0 = leave, 1 = press, other = inactive
  };
  RingBuf<Info, history_size> history;

  void init() {
    for (size_t i = 0; i < history_size; ++i) {
      auto& info = history._data[i];
      info.state = 255;
    }
  }

  void process(uint32_t current_tick) {
    for (size_t i = 0; i < history_size; ++i) {
      auto& info = history._data[i];
      if (info.state != 255) {
        if (current_tick - info.tick > 0x7FFFFFFFUL) {
          // timeout
          info.state = 255;
        }
      }
    }
  }

  void add(const Info& info) {
    history.add(info);
  }

  uint32_t findTapInterval(BtnIdType id, uint32_t long_press_limit_tick, uint32_t min_interval_tick, uint32_t max_interval_tick) const {
    const auto& info0 = history.peekLast(0);
    const auto& info1 = history.peekLast(1);
    const auto& info2 = history.peekLast(2);
    const auto& info3 = history.peekLast(3);
    const uint32_t interval = info1.tick - info3.tick;
    if (
        info0.id == id && info0.state == 0 &&
        info1.id == id && info1.state == 1 &&
        info2.id == id && info2.state == 0 &&
        info3.id == id && info3.state == 1 &&
        min_interval_tick < interval && interval < max_interval_tick &&
        (info2.tick - info3.tick) < (uint32_t)long_press_limit_tick &&
        (info0.tick - info1.tick) < (uint32_t)long_press_limit_tick
        ) {
      return interval;
    }

    return 0; // not found
  }

  char findMorseCode(BtnIdType id, uint32_t long_press_tick, uint32_t max_interval_tick) const {

    char c = (char)0;

    if (history.peekLast(0).state != 0 || history.peekLast(0).id != id) {
      return c;
    }

    uint8_t morse_codes = 0UL;
    for (uint8_t peek_idx = 0; peek_idx < history_size; peek_idx += 2) {
      auto& info1 = history.peekLast(peek_idx);
      auto& info2 = history.peekLast(peek_idx+1);
      if (info1.id == id && info2.id == id && info1.state == 0 && info2.state == 1) {
        uint32_t press_time_tick = info1.tick - info2.tick;
        if (press_time_tick > max_interval_tick) {
          break;
        }
        if (press_time_tick >= long_press_tick) {
          morse_codes |= (3 << peek_idx); // long tone = 2
        } else {
          morse_codes |= (1 << peek_idx); // short tone = 1
        }
      } else {
        break;
      }

      if (peek_idx + (uint8_t)2 < (uint8_t)history_size) {
        auto& info3 = history.peekLast(peek_idx+2);
        if (info3.id != id
            || (info2.tick - info3.tick > max_interval_tick)) {
          break;
        }
      }
    }

    switch(morse_codes) {
      case 0b00000111: // ._
        c = 'A';
        break;
      case 0b11010101: // _...
        c = 'B';
        break;
      case 0b11011101: // _._.
        c = 'C';
        break;
      case 0b00110101: // _..
        c = 'D';
        break;
      case 0b00000001: // .
        c = 'E';
        break;
      case 0b01011101: // .._.
        c = 'F';
        break;
      case 0b00111101: // __.
        c = 'G';
        break;
      case 0b01010101: // ....
        c = 'H';
        break;
      case 0b00000101: // ..
        c = 'I';
        break;
      case 0b01111111: // .___
        c = 'J';
        break;
      case 0b00110111: // _._
        c = 'K';
        break;
      case 0b01110101: // ._..
        c = 'L';
        break;
      case 0b00001111: // __
        c = 'M';
        break;
      case 0b00001101: // _.
        c = 'N';
        break;
      case 0b00111111: // ___
        c = 'O';
        break;
      case 0b01111101: // .__.
        c = 'P';
        break;
      case 0b11110111: // __._
        c = 'Q';
        break;
      case 0b00011101: // ._.
        c = 'R';
        break;
      case 0b00010101: // ...
        c = 'S';
        break;
      case 0b00000011: // _
        c = 'T';
        break;
      case 0b00010111: // .._
        c = 'U';
        break;
      case 0b01010111: // ..._
        c = 'V';
        break;
      case 0b00011111: // .__
        c = 'W';
        break;
      case 0b11010111: // _.._
        c = 'X';
        break;
      case 0b11011111: // _.__
        c = 'Y';
        break;
      case 0b11110101: // __..
        c = 'Z';
        break;
      default:
        break;
    }

    return c;
  }
};

}
}

