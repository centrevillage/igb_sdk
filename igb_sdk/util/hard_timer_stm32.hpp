#ifndef IGB_SDK_UTIL_HARD_TIMER_STM32_H
#define IGB_SDK_UTIL_HARD_TIMER_STM32_H

#include <cmath>
#include <array>
#include <igb_stm32/periph/tim.hpp>

namespace igb {
namespace sdk {

template<typename Tim32Cls /* 32bit tim class (TIM2 etc) */, uint8_t priority, uint32_t tim_base_clock>
struct HardCcTimerStm32 {
  Tim32Cls _tim;
  struct CcState {
    uint32_t interval_tick = 0;
    uint32_t cc_value = 0;
    bool active = false;
    std::function<void(void)> on_capture = [](){}; // set by application
  };
  std::array<CcState, 4> _cc_states;

  void init(float cc1_interval_sec, auto&& update_func) { // general timer api
    _tim.count(0);
    setPeriodSec(cc1_interval_sec);
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

  IGB_FAST_INLINE void setPeriodSec(float interval_sec) { // general timer api
    const float interval_tick_f = interval_sec * (float)tim_base_clock;
    const uint32_t interval_tick = (uint32_t)std::round(interval_tick_f);
    setPeriodTick(interval_tick);
  }
  IGB_FAST_INLINE void setPeriodTick(uint32_t interval_tick) { // general timer api
    if (interval_tick > 0) {
      auto& state = _cc_states[0];
      state.interval_tick = interval_tick;
    }
  }
  IGB_FAST_INLINE float getPeriodSec() const { // general timer api
    auto& state = _cc_states[0];
    return std::round((float)state.interval_tick / (float)tim_base_clock);
  }
  IGB_FAST_INLINE uint32_t getPeriodTick() const { // general timer api
    auto& state = _cc_states[0];
    return state.interval_tick;
  }

  IGB_FAST_INLINE void start() { // general timer api
    auto& state = _cc_states[0];
    uint32_t cc_value = _tim.count() + state.interval_tick;
    _tim.cc1Value(cc_value);
    state.active = true;
    state.cc_value = cc_value;
  }

  IGB_FAST_INLINE void stop() { // general timer api
    auto& state = _cc_states[0];
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
    if (cc_idx < 4) {
      _cc_states[cc_idx].interval_tick = interval_tick;
    }
  }
  IGB_FAST_INLINE uint32_t getIntervalTick(uint8_t cc_idx) const noexcept { // specialized api
    if (cc_idx < 4) {
      return _cc_states[cc_idx].interval_tick;
    }
    return 0;
  }

  IGB_FAST_INLINE uint32_t tick() { // specialized api
    return _tim.count();
  }

  IGB_FAST_INLINE void cc1Handler() { // specialized api
    if (_tim.is(igb::stm32::TimState::cc1)) {
      auto& state = _cc_states[0];
      if (state.active) {
        state.cc_value += state.interval_tick;
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
        state.cc_value += state.interval_tick;
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
        state.cc_value += state.interval_tick;
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
        state.cc_value += state.interval_tick;
        _tim.cc4Value(state.cc_value);
        state.on_capture();
      }

      _tim.clear(igb::stm32::TimState::cc4);
    }
  }

  // call me on TIMx_Handler
  // if you are concerned for the performance, just call ccxHandler you need.
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
