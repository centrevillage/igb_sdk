#ifndef IGB_SDK_UTIL_HARD_TIMER_STM32_H
#define IGB_SDK_UTIL_HARD_TIMER_STM32_H

#include <cmath>
#include <array>
#include <algorithm>
#include <type_traits>
#include <igb_stm32/periph/tim.hpp>
#include <igb_util/tbl.hpp>

namespace igb {
namespace sdk {

template<typename TimCls, typename count_t /* uint16_t or uint32_t */, uint8_t priority, uint32_t tim_base_clock>
struct HardCcTimerStm32 {
  static_assert(std::is_same<count_t, uint16_t>::value || std::is_same<count_t, uint32_t>::value, "count_t must be uint16_t or uint32_t");

  constexpr static uint32_t sub_timer_count = 3;
  constexpr static uint32_t cc_ch_count = 4;

  TimCls _tim;
  struct CcState {
    count_t interval_tick = 0;
    float interval_tick_f = 0.0f;
    count_t interval_tick_adj = 0;
    count_t cc_value = 0;
    uint32_t update_count = 0;
    bool active = false;
    std::function<void(void)> on_capture = [](){}; // set by application

    IGB_FAST_INLINE count_t getAdjestValue() {
      return (euclid_tbl_32[interval_tick_adj] & (1UL << (update_count & 0x1F))) ? 1 : 0;
    }
    IGB_FAST_INLINE void nextCc() {
      cc_value += interval_tick + getAdjestValue();
      update_count++;
    }
  };
  std::array<CcState, cc_ch_count> _cc_states;

  constexpr IGB_FAST_INLINE static float secToTick(float interval_sec) {
    return interval_sec * (float)tim_base_clock;
  }
  constexpr IGB_FAST_INLINE static float tickToSec(float tick) {
    return tick / (float)tim_base_clock;
  }

  void init() { // general timer api
    _tim.count(0);
    const count_t period = (count_t)0xFFFFFFFF;
    _tim.init(
      igb::stm32::TimConf {
        .prescale = 0,
        .period = period,
        .interrupt_priority = priority,
        .enable_it = true
      }
    );
  }

  void enable() {
    if (!_tim.enable()) {
      _tim.start();
    }
  }

  void disable() {
    _tim.stop();
  }

  void setupTimer(uint8_t cc_idx, float interval_sec, auto&& update_func) { // general timer api
    auto& state = _cc_states[cc_idx];
    setIntervalSec(cc_idx, interval_sec);
    state.on_capture = update_func;
    state.active = false;
    enableCh(cc_idx);
  }

  // DEPRECATED
  void init(float cc1_interval_sec, auto&& update_func) { // general timer api
    _tim.count(0);
    setIntervalSec(cc1_interval_sec);
    _cc_states[0].on_capture = update_func;

    const count_t period = (count_t)0xFFFFFFFF;
    _tim.init(
      igb::stm32::TimConf {
        .prescale = 0,
        .period = period,
        .enable_it_cc1 = true,
        .interrupt_priority = priority
      }
    );
    if (!_tim.enable()) {
      _tim.start();
    }
  }

  // DEPRECATED
  void initSubTimer(uint8_t sub_timer_idx, float interval_sec, auto&& update_func) { // specialized timer api
    if (sub_timer_idx < sub_timer_count) {
      const auto cc_idx = sub_timer_idx + 1;
      auto& state = _cc_states[cc_idx];
      setIntervalSec(cc_idx, interval_sec);
      state.on_capture = update_func;
      state.active = false;
      enableCh(cc_idx);
    }
  }

  IGB_FAST_INLINE void setIntervalSec(float interval_sec) { // general timer api
    const float interval_tick_f = secToTick(interval_sec);
    setIntervalTick(interval_tick_f);
  }
  IGB_FAST_INLINE void setIntervalSec(uint8_t cc_idx, float interval_sec) { // specialized timer api
    const float interval_tick_f = secToTick(interval_sec);
    setIntervalTick(cc_idx, interval_tick_f);
  }
  // DEPRECATED
  IGB_FAST_INLINE void setSubTimerIntervalSec(uint8_t sub_timer_idx, float interval_sec) { // specialized timer api
    if (sub_timer_idx < sub_timer_count) {
      setIntervalSec(sub_timer_idx+1, interval_sec);
    }
  }
  IGB_FAST_INLINE void setIntervalTick(count_t interval_tick) { // general timer api
    setIntervalTick(0, interval_tick);
  }
  IGB_FAST_INLINE void setIntervalTick(float interval_tick_f) { // general timer api
    setIntervalTick(0, interval_tick_f);
  }
  IGB_FAST_INLINE float getIntervalSec(uint8_t cc_idx = 0) const { // general timer api
    auto& state = _cc_states[cc_idx];
    return std::round(tickToSec((float)state.interval_tick));
  }
  IGB_FAST_INLINE float getIntervalTick(uint8_t cc_idx = 0) const { // general timer api
    auto& state = _cc_states[cc_idx];
    return state.interval_tick_f;
  }
  // DEPRECATED
  IGB_FAST_INLINE float getSubTimerIntervalSec(uint8_t sub_timer_idx) const { // general timer api
    if (sub_timer_idx < sub_timer_count) {
      return getIntervalSec(sub_timer_idx+1);
    }
    return 0.0f;
  }
  // DEPRECATED
  IGB_FAST_INLINE float getSubTimerIntervalTick(uint8_t sub_timer_idx) const { // general timer api
    if (sub_timer_idx < sub_timer_count) {
      return getIntervalTick(sub_timer_idx+1);
    }
    return 0.0f;
  }

  IGB_FAST_INLINE void start(uint8_t cc_idx = 0) { // general timer api
    _startTimer(cc_idx);
  }

  // DEPRECATED:
  IGB_FAST_INLINE void startSubTimer(uint8_t sub_timer_idx) { // specialized timer api
    if (sub_timer_idx < sub_timer_count) {
      const auto cc_idx = sub_timer_idx + 1;
      _startTimer(cc_idx);
    }
  }

  // DEPRECATED:
  IGB_FAST_INLINE void stopSubTimer(uint8_t sub_timer_idx) { // specialized timer api
    if (sub_timer_idx < sub_timer_count) {
      const auto cc_idx = sub_timer_idx + 1;
      _stopTimer(cc_idx);
    }
  }

  IGB_FAST_INLINE void _startTimer(uint8_t cc_idx) {
    auto& state = _cc_states[cc_idx];
    count_t cc_value = _tim.count() + state.interval_tick;
    _setCcValue(cc_idx, cc_value);
    state.active = true;
    state.cc_value = cc_value;
  }


  IGB_FAST_INLINE void stop(uint8_t cc_idx = 0) { // general timer api
    _stopTimer(cc_idx);
  }

  IGB_FAST_INLINE void _stopTimer(uint8_t cc_idx) { // general timer api
    auto& state = _cc_states[cc_idx];
    state.active = false;
  }

  IGB_FAST_INLINE void enableCh(uint8_t cc_idx) { // specialized api
    switch (cc_idx) {
      case 0:
        _tim.enableItCc1(true);
        break;
      case 1:
        _tim.enableItCc2(true);
        break;
      case 2:
        _tim.enableItCc3(true);
        break;
      case 3:
        _tim.enableItCc4(true);
        break;
    }
  }

  IGB_FAST_INLINE void disableCh(uint8_t cc_idx) { // specialized api
    switch (cc_idx) {
      case 0:
        _tim.enableItCc1(false);
        break;
      case 1:
        _tim.enableItCc2(false);
        break;
      case 2:
        _tim.enableItCc3(false);
        break;
      case 3:
        _tim.enableItCc4(false);
        break;
    }
  }

  IGB_FAST_INLINE void setCallback(uint8_t cc_idx, auto&& func) { // specialized api
    if (cc_idx < 4) {
      _cc_states[cc_idx].on_capture = func;
    }
  }

  IGB_FAST_INLINE void setIntervalTick(uint8_t cc_idx, count_t interval_tick) { // specialized api
    auto& state = _cc_states[cc_idx];
    state.interval_tick = interval_tick;
    state.interval_tick_f = (float)interval_tick;
    state.interval_tick_adj = 0;
  }

  IGB_FAST_INLINE void setIntervalTick(uint8_t cc_idx, float interval_tick_f) { // specialized api
    auto& state = _cc_states[cc_idx];
    if (interval_tick_f > 0.0f) {
      const auto tick_u32 = (count_t)interval_tick_f;
      state.interval_tick_f = interval_tick_f;
      state.interval_tick = tick_u32;
      const auto mod = interval_tick_f - (float)tick_u32;
      state.interval_tick_adj = std::clamp(
          (count_t)(mod * 32.0f),
          (count_t)0, (count_t) 31);
    }
  }

  // DEPRECATED
  IGB_FAST_INLINE void setSubTimerIntervalTick(uint8_t sub_timer_idx, float interval_tick_f) { // specialized timer api
    setIntervalTick(sub_timer_idx+1, interval_tick_f);
  }
  // DEPRECATED
  IGB_FAST_INLINE void setSubTimerIntervalTick(uint8_t sub_timer_idx, count_t interval_tick) { // specialized timer api
    setIntervalTick(sub_timer_idx+1, interval_tick);
  }

  IGB_FAST_INLINE count_t tick() { // specialized api
    return _tim.count();
  }

  IGB_FAST_INLINE void _setCcValue(uint8_t cc_idx, count_t value) {
    switch (cc_idx) {
      case 0:
        _tim.cc1Value(value);
        break;
      case 1:
        _tim.cc2Value(value);
        break;
      case 2:
        _tim.cc3Value(value);
        break;
      case 3:
        _tim.cc4Value(value);
        break;
    }
  }

  IGB_FAST_INLINE void cc1Handler() { // specialized api
    if (_tim.is(igb::stm32::TimState::cc1)) {
      auto& state = _cc_states[0];
      if (state.active) {
        state.on_capture();
        state.nextCc();
        _tim.cc1Value(state.cc_value);
      }

      _tim.clear(igb::stm32::TimState::cc1);
    }
  }
  IGB_FAST_INLINE void cc2Handler() { // specialized api
    if (_tim.is(igb::stm32::TimState::cc2)) {
      auto& state = _cc_states[1];
      if (state.active) {
        state.on_capture();
        state.nextCc();
        _tim.cc2Value(state.cc_value);
      }

      _tim.clear(igb::stm32::TimState::cc2);
    }
  }
  IGB_FAST_INLINE void cc3Handler() { // specialized api
    if (_tim.is(igb::stm32::TimState::cc3)) {
      auto& state = _cc_states[2];
      if (state.active) {
        state.on_capture();
        state.nextCc();
        _tim.cc3Value(state.cc_value);
      }

      _tim.clear(igb::stm32::TimState::cc3);
    }
  }
  IGB_FAST_INLINE void cc4Handler() { // specialized api
    if (_tim.is(igb::stm32::TimState::cc4)) {
      auto& state = _cc_states[3];
      if (state.active) {
        state.on_capture();
        state.nextCc();
        _tim.cc4Value(state.cc_value);
      }

      _tim.clear(igb::stm32::TimState::cc4);
    }
  }

  // call me on TIMx_Handler
  IGB_FAST_INLINE void irqHandler() { // specialized api
    cc1Handler();
    cc2Handler();
    cc3Handler();
    cc4Handler();
  }
};

}
}

#endif /* IGB_SDK_UTIL_HARD_TIMER_STM32_H */
