#pragma once

#include <cmath>
#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>
#include <functional>
#include <type_traits>
#include <igb_util/tbl.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace sdk {

template<typename TimCls, typename count_t /* uint16_t or uint32_t */, size_t cc_ch, uint32_t tim_base_clock, typename float_t = float>
struct SoftCcTimer {
  static_assert(std::is_same<count_t, uint16_t>::value || std::is_same<count_t, uint32_t>::value, "count_t must be uint16_t or uint32_t");
  static_assert(cc_ch >= 1, "cc ch size must be greater equal than 1.");

  constexpr static size_t sub_timer_count = cc_ch - 1;
  constexpr static size_t cc_ch_count = cc_ch;

  TimCls _tim;
  struct CcState {
    count_t interval_tick = 0;
    float_t interval_tick_f = 0.0f;
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
    IGB_FAST_INLINE void process(count_t current_tick) {
      if (active) {
        if ((int32_t)(current_tick - cc_value) >= 0) {
          on_capture();
          nextCc();
        }
      }
    }
    IGB_FAST_INLINE count_t getCcValue() const {
      return cc_value;
    }
  };
  std::array<CcState, cc_ch_count> _cc_states;

  constexpr IGB_FAST_INLINE static float_t secToTick(float_t interval_sec) {
    return interval_sec * (float_t)tim_base_clock;
  }
  constexpr IGB_FAST_INLINE static float_t tickToSec(float_t tick) {
    return tick / (float_t)tim_base_clock;
  }

  void init() { // general timer api
    _tim.count(0);
    //const count_t period = (count_t)0xFFFFFFFF;
    //_tim.init(
    //  igb::stm32::TimConf {
    //    .prescale = 0,
    //    .period = period,
    //    .interrupt_priority = priority,
    //    .enable_it = true
    //  }
    //);
  }

  void enable() {
    if (!_tim.enable()) {
      _tim.start();
    }
  }

  void disable() {
    _tim.stop();
  }

  void setupTimer(uint8_t cc_idx, float_t interval_sec, auto&& update_func) { // general timer api
    auto& state = _cc_states[cc_idx];
    setIntervalSec(cc_idx, interval_sec);
    state.on_capture = update_func;
    state.active = false;
  }

  IGB_FAST_INLINE void setIntervalSec(float_t interval_sec) { // general timer api
    const float_t interval_tick_f = secToTick(interval_sec);
    setIntervalTick(interval_tick_f);
  }
  IGB_FAST_INLINE void setIntervalSec(uint8_t cc_idx, float_t interval_sec) { // specialized timer api
    const float_t interval_tick_f = secToTick(interval_sec);
    setIntervalTick(cc_idx, interval_tick_f);
  }
  IGB_FAST_INLINE void setIntervalTick(count_t interval_tick) { // general timer api
    setIntervalTick(0, interval_tick);
  }
  IGB_FAST_INLINE void setIntervalTick(float_t interval_tick_f) { // general timer api
    setIntervalTick(0, interval_tick_f);
  }
  IGB_FAST_INLINE float_t getIntervalSec(uint8_t cc_idx = 0) const { // general timer api
    auto& state = _cc_states[cc_idx];
    return std::round(tickToSec((float_t)state.interval_tick));
  }
  IGB_FAST_INLINE float_t getIntervalTick(uint8_t cc_idx = 0) const { // general timer api
    auto& state = _cc_states[cc_idx];
    return state.interval_tick_f;
  }
  IGB_FAST_INLINE void start(uint8_t cc_idx = 0) { // general timer api
    _startTimer(cc_idx);
  }
  IGB_FAST_INLINE void start(uint8_t cc_idx, count_t t) { // general timer api
    _startTimer(cc_idx, t);
  }
  IGB_FAST_INLINE void _startTimer(uint8_t cc_idx) {
    _startTimer(cc_idx, _tim.count());
  }
  IGB_FAST_INLINE void _startTimer(uint8_t cc_idx, count_t t) {
    auto& state = _cc_states[cc_idx];
    count_t cc_value = t + state.interval_tick;
    state.active = true;
    state.cc_value = cc_value;
  }
  IGB_FAST_INLINE void stop(uint8_t cc_idx = 0) { // general timer api
    _stopTimer(cc_idx);
  }
  IGB_FAST_INLINE bool isStart(uint8_t cc_idx) const {
    const auto& state = _cc_states[cc_idx];
    return state.active;
  }
  IGB_FAST_INLINE void _stopTimer(uint8_t cc_idx) { // general timer api
    auto& state = _cc_states[cc_idx];
    state.active = false;
  }
  IGB_FAST_INLINE void setCallback(uint8_t cc_idx, auto&& func) { // specialized api
    if (cc_idx < cc_ch_count) {
      _cc_states[cc_idx].on_capture = func;
    }
  }
  IGB_FAST_INLINE void setIntervalTick(uint8_t cc_idx, count_t interval_tick) { // specialized api
    auto& state = _cc_states[cc_idx];
    state.interval_tick = interval_tick;
    state.interval_tick_f = (float_t)interval_tick;
    state.interval_tick_adj = 0;
  }
  IGB_FAST_INLINE void setIntervalTick(uint8_t cc_idx, float_t interval_tick_f) { // specialized api
    auto& state = _cc_states[cc_idx];
    if (interval_tick_f > 0.0f) {
      const auto tick_u32 = (count_t)interval_tick_f;
      state.interval_tick_f = interval_tick_f;
      state.interval_tick = tick_u32;
      const auto mod = interval_tick_f - (float_t)tick_u32;
      state.interval_tick_adj = std::clamp(
          (count_t)(mod * 32.0f),
          (count_t)0, (count_t) 31);
    }
  }

  IGB_FAST_INLINE count_t tick() { // specialized api
    return _tim.count();
  }

  IGB_FAST_INLINE void process() {
    for (auto& cc_state : _cc_states) {
      cc_state.process(tick());
    }
  }
  IGB_FAST_INLINE void process(uint8_t cc_idx) {
    _cc_states[cc_idx].process(tick());
  }

  IGB_FAST_INLINE count_t getCcTick(uint8_t cc_idx) { 
    auto& state = _cc_states[cc_idx];
    return state.getCcValue();
  }

  IGB_FAST_INLINE void irqHandler() { /* no irq handler */ }
};

} // sdk
} // igb
