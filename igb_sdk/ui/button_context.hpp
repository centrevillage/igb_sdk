#pragma once

#include <functional>
#include <utility>
#include <igb_sdk/base.hpp>
#include <igb_sdk/ui/button.hpp>
#include <igb_sdk/ui/button_history.hpp>
#include <igb_util/conversion.hpp>

namespace igb {
namespace sdk {

struct ButtonMethod {
  constexpr static uint32_t makeStateBits(auto&& id, auto&&... rest) {
    return ((uint32_t)1 << static_cast<uint32_t>(id)) | makeStateBits(rest...);
  }
  constexpr static uint32_t makeStateBits() {
    return 0;
  }

  constexpr static bool isAllOff(uint32_t state_bits) {
    return !state_bits;
  }

  constexpr static bool exactMatch(uint32_t state_bits, auto&& id, auto&&... rest) {
    return state_bits == makeStateBits(id, rest...);
  }

  constexpr static bool contains(uint32_t state_bits, auto&& id, auto&&... rest) {
    return state_bits & makeStateBits(id, rest...);
  }

  constexpr static bool match(uint32_t state_bits, auto&& state, auto&&... rest) {
    return (
      state.second == !!(state_bits & ((uint32_t)1 << static_cast<uint32_t>(state.first)))
        && match(rest...)
    );
  }

  constexpr static bool match() {
    return true;
  }
};


template<typename AppBtnId, typename BtnCollection>
struct ButtonContext {
  std::function<void(AppBtnId, bool)> on_change = [](AppBtnId, bool){};
  BtnCollection buttons;

  typedef ButtonHistory<AppBtnId> AppBtnHistory;
  typedef typename AppBtnHistory::Info AppBtnHistoryInfo;

  AppBtnHistory history;
  uint32_t state_bits = 0;
  uint32_t prev_state_bits = 0;

  void init() {
    history.init();
    buttons.init();
  }

  void process(uint32_t current_tick) {
    history.process(current_tick);
    buttons.process();

    uint32_t new_bits = create_bits(
      buttons, [](auto& btn) { return btn.isOn(); }
    );

    if (state_bits != new_bits) {
      uint32_t diff_bits = state_bits ^ new_bits;
      state_bits = new_bits;
      for (size_t i = 0; i < buttons.size(); ++i) {
        if (diff_bits & (1UL << i)) {
          bool is_on = !!(new_bits & (1UL << i));
          history.add(
            AppBtnHistoryInfo {
              .tick = current_tick,
              .id = static_cast<AppBtnId>(i),
              .state = (is_on ? (uint8_t)1 : (uint8_t)0)
            }
          );
          if (on_change) {
            on_change(static_cast<AppBtnId>(i), is_on);
          }
        }
      }
      prev_state_bits = state_bits;
    }
  }
};

}
}

