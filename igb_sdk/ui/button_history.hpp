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
};

}
}

