#ifndef IGB_SDK_UTIL_HARD_TIMER_STM32_H
#define IGB_SDK_UTIL_HARD_TIMER_STM32_H

#include <cmath>
#include <array>
#include <algorithm>
#include <igb_stm32/periph/tim.hpp>

namespace igb {
namespace sdk {

template<typename Tim32Cls /* 32bit tim class (TIM2 etc) */, uint8_t priority, uint32_t tim_base_clock>
struct HardCcTimerStm32 {
  constexpr static uint32_t sub_timer_count = 3;
  constexpr static uint32_t adjust_tbl[32] = {
// #!ruby
//def gen_basic_euclid(k, n)
//  if k > n
//    k = n
//  end
//  pattern = [[1]] * k + [[0]] * (n - k)
//  while k > 0
//    cut = [k, pattern.length - k].min
//    pattern = (0 ... cut).map {|i| pattern[i] + pattern[k + i]} + pattern[cut ... k] + pattern[(k+cut)...]
//    k = cut
//  end
//  pattern.flatten
//end
//
//(0..31).each do |fill|
//  print <<-EOS
//    0b#{gen_basic_euclid(fill, 32).join('')},
//  EOS
//end
    0b00000000000000000000000000000000,
    0b10000000000000000000000000000000,
    0b10000000000000001000000000000000,
    0b10000000000100000000010000000000,
    0b10000000100000001000000010000000,
    0b10000001000001000001000000100000,
    0b10000010000100001000001000010000,
    0b10000100010000100001000100001000,
    0b10001000100010001000100010001000,
    0b10001001000100010010001001000100,
    0b10001001001001001000100100100100,
    0b10010100100100100100100100100100,
    0b10010100100101001001010010010100,
    0b10010101001010010100101001010010,
    0b10010101010010101001010101001010,
    0b10010101010101010100101010101010,
    0b10101010101010101010101010101010,
    0b10110101010101010101101010101010,
    0b10110101010110101011010101011010,
    0b10110101011010110101101011010110,
    0b10110101101101011011010110110101,
    0b10110101101101101101101101101101,
    0b10111011011011011011101101101101,
    0b10111011011101110110111011011101,
    0b10111011101110111011101110111011,
    0b10111101110111101111011101111011,
    0b10111110111101111011111011110111,
    0b10111111011111011111011111101111,
    0b10111111101111111011111110111111,
    0b10111111111101111111110111111111,
    0b10111111111111111011111111111111,
    0b10111111111111111111111111111111,
  };

  Tim32Cls _tim;
  struct CcState {
    uint32_t interval_tick = 0;
    float interval_tick_f = 0.0f;
    uint32_t interval_tick_adj = 0;
    uint32_t cc_value = 0;
    uint32_t update_count = 0;
    bool active = false;
    std::function<void(void)> on_capture = [](){}; // set by application

    IGB_FAST_INLINE uint32_t getAdjestValue() {
      return (adjust_tbl[interval_tick_adj] & (1UL << (update_count & 0x1F))) ? 1 : 0;
    }
    IGB_FAST_INLINE void nextCc() {
      cc_value += interval_tick + getAdjestValue();
      update_count++;
    }
  };
  std::array<CcState, 4> _cc_states;

  void init(float cc1_interval_sec, auto&& update_func) { // general timer api
    _tim.count(0);
    setIntervalSec(cc1_interval_sec);
    _cc_states[0].on_capture = update_func;
    _tim.init(
      igb::stm32::TimConf {
        .prescale = 0,
        .period = 0xFFFFFFFF,
        .enable_it_cc1 = true,
        .interrupt_priority = priority
      }
    );
    if (!_tim.enable()) {
      _tim.start();
    }
  }

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
    const float interval_tick_f = interval_sec * (float)tim_base_clock;
    setIntervalTick(interval_tick_f);
  }
  IGB_FAST_INLINE void setIntervalSec(uint8_t cc_idx, float interval_sec) { // specialized timer api
    const float interval_tick_f = interval_sec * (float)tim_base_clock;
    setIntervalTick(cc_idx, interval_tick_f);
  }
  IGB_FAST_INLINE void setSubTimerIntervalSec(uint8_t sub_timer_idx, float interval_sec) { // specialized timer api
    if (sub_timer_idx < sub_timer_count) {
      setIntervalSec(sub_timer_idx+1, interval_sec);
    }
  }
  IGB_FAST_INLINE void setIntervalTick(uint32_t interval_tick) { // general timer api
    setIntervalTick(0, interval_tick);
  }
  IGB_FAST_INLINE void setIntervalTick(float interval_tick_f) { // general timer api
    setIntervalTick(0, interval_tick_f);
  }
  IGB_FAST_INLINE float getIntervalSec(uint8_t cc_idx = 0) const { // general timer api
    auto& state = _cc_states[cc_idx];
    return std::round((float)state.interval_tick / (float)tim_base_clock);
  }
  IGB_FAST_INLINE float getIntervalTick(uint8_t cc_idx = 0) const { // general timer api
    auto& state = _cc_states[cc_idx];
    return state.interval_tick_f;
  }
  IGB_FAST_INLINE float getSubTimerIntervalSec(uint8_t sub_timer_idx) const { // general timer api
    if (sub_timer_idx < sub_timer_count) {
      return getIntervalSec(sub_timer_idx+1);
    }
    return 0.0f;
  }
  IGB_FAST_INLINE float getSubTimerIntervalTick(uint8_t sub_timer_idx) const { // general timer api
    if (sub_timer_idx < sub_timer_count) {
      return getIntervalTick(sub_timer_idx+1);
    }
    return 0.0f;
  }

  IGB_FAST_INLINE void start() { // general timer api
    _startTimer(0);
  }

  IGB_FAST_INLINE void startSubTimer(uint8_t sub_timer_idx) { // specialized timer api
    if (sub_timer_idx < sub_timer_count) {
      const auto cc_idx = sub_timer_idx + 1;
      _startTimer(cc_idx);
    }
  }

  IGB_FAST_INLINE void stopSubTimer(uint8_t sub_timer_idx) { // specialized timer api
    if (sub_timer_idx < sub_timer_count) {
      const auto cc_idx = sub_timer_idx + 1;
      _stopTimer(cc_idx);
    }
  }

  IGB_FAST_INLINE void _startTimer(uint8_t cc_idx) {
    auto& state = _cc_states[cc_idx];
    uint32_t cc_value = _tim.count() + state.interval_tick;
    _setCcValue(cc_idx, cc_value);
    state.active = true;
    state.cc_value = cc_value;
  }


  IGB_FAST_INLINE void stop() { // general timer api
    _stopTimer(0);
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

  IGB_FAST_INLINE void setIntervalTick(uint8_t cc_idx, uint32_t interval_tick) { // specialized api
    auto& state = _cc_states[cc_idx];
    if (interval_tick > 0) {
      auto& state = _cc_states[0];
      state.interval_tick = interval_tick;
      state.interval_tick_f = (float)interval_tick;
      state.interval_tick_adj = 0;
    }
  }

  IGB_FAST_INLINE void setIntervalTick(uint8_t cc_idx, float interval_tick_f) { // specialized api
    auto& state = _cc_states[cc_idx];
    if (interval_tick_f > 0.0f) {
      const auto tick_u32 = (uint32_t)interval_tick_f;
      state.interval_tick_f = interval_tick_f;
      state.interval_tick = tick_u32;
      const auto mod = interval_tick_f - (float)tick_u32;
      state.interval_tick_adj = std::clamp(
          (uint32_t)(mod * 32.0f),
          (uint32_t)0, (uint32_t) 31);
    }
  }

  IGB_FAST_INLINE void setSubTimerIntervalTick(uint8_t sub_timer_idx, float interval_tick_f) { // specialized timer api
    setIntervalTick(sub_timer_idx+1, interval_tick_f);
  }
  IGB_FAST_INLINE void setSubTimerIntervalTick(uint8_t sub_timer_idx, uint32_t interval_tick) { // specialized timer api
    setIntervalTick(sub_timer_idx+1, interval_tick);
  }

  IGB_FAST_INLINE uint32_t tick() { // specialized api
    return _tim.count();
  }

  IGB_FAST_INLINE void _setCcValue(uint8_t cc_idx, uint32_t value) {
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
        state.nextCc();
        _tim.cc1Value(state.cc_value);
        state.on_capture();
      }

      _tim.clear(igb::stm32::TimState::cc1);
    }
  }
  IGB_FAST_INLINE void cc2Handler() { // specialized api
    if (_tim.is(igb::stm32::TimState::cc2)) {
      auto& state = _cc_states[1];
      if (state.active) {
        state.nextCc();
        _tim.cc2Value(state.cc_value);
        state.on_capture();
      }

      _tim.clear(igb::stm32::TimState::cc2);
    }
  }
  IGB_FAST_INLINE void cc3Handler() { // specialized api
    if (_tim.is(igb::stm32::TimState::cc3)) {
      auto& state = _cc_states[2];
      if (state.active) {
        state.nextCc();
        _tim.cc3Value(state.cc_value);
        state.on_capture();
      }

      _tim.clear(igb::stm32::TimState::cc3);
    }
  }
  IGB_FAST_INLINE void cc4Handler() { // specialized api
    if (_tim.is(igb::stm32::TimState::cc4)) {
      auto& state = _cc_states[3];
      if (state.active) {
        state.nextCc();
        _tim.cc4Value(state.cc_value);
        state.on_capture();
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
