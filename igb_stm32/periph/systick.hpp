#ifndef IGB_STM32_PERIPH_SYSTICK_H
#define IGB_STM32_PERIPH_SYSTICK_H

#include <igb_stm32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>

extern "C" {
struct SystickState {
  volatile uint32_t tick = 0;
  volatile uint32_t interval = 0;
  volatile uint32_t _msec_scaling = 1;
  volatile uint32_t _usec_scaling = 1000;
};

extern volatile SystickState _systick_state;
}

namespace igb {
namespace stm32 {

enum class SystickTimerInterval : uint32_t {
  msec = 1000UL,
  _100usec = 10000UL,
  _10usec = 100000UL,
  usec = 1000000UL,
};

struct SystickCtrl {
  static IGB_FAST_INLINE void setTimerInterval(uint32_t interval_) {
    _systick_state.interval = interval_;
    _systick_state._msec_scaling = (uint32_t)(((float)(SystemCoreClock) / 1000.0f) / (float)_systick_state.interval);
    _systick_state._usec_scaling = (uint32_t)((float)_systick_state.interval / ((float)(SystemCoreClock) / 1000000.0f));
    if (SysTick_Config(_systick_state.interval)) {
      /* Capture error */ 
      while (1);
    }
  }
  static IGB_FAST_INLINE void setTimerInterval(SystickTimerInterval interval_) {
    setTimerInterval(SystemCoreClock / static_cast<uint32_t>(interval_));
  }
  static IGB_FAST_INLINE uint32_t getCurrentTick() {
    return _systick_state.tick;
  }
  static IGB_FAST_INLINE uint32_t getCurrentMilliSec() {
    return _systick_state.tick / _systick_state._msec_scaling;
  }
  static IGB_FAST_INLINE uint32_t getCurrentMicroSec() {
    return _systick_state.tick * _systick_state._usec_scaling;
  }
  static IGB_FAST_INLINE void receiveTick() {
    //return _systick_state.tick++;
    _systick_state.tick = _systick_state.tick + 1;
    //return _systick_state.tick;
  }
};
//volatile SystickCtrl::State SystickCtrl::state;

}
}

extern "C" {

// if you use systick callback, please define USER_SYSTICK_HANDLER_CALLBACK macro in "igb_stm32_user_conf.h"
#ifndef USER_SYSTICK_HANDLER_CALLBACK
#define USER_SYSTICK_HANDLER_CALLBACK while(0){}
#endif

#ifdef USE_ARDUINO

#include "clock.h"
static IGB_FAST_INLINE uint32_t current_msec() {
  return getCurrentMillis();
}

static IGB_FAST_INLINE uint32_t current_usec() {
  return getCurrentMicros();
}

#elif USE_DAISY

#include "sys/system.h"
static IGB_FAST_INLINE uint32_t current_msec() {
  return daisy::System::GetNow();
}

static IGB_FAST_INLINE uint32_t current_usec() {
  return daisy::System::GetUs();
}

#else

void SysTick_Handler(void);

static IGB_FAST_INLINE uint32_t current_msec() {
  return igb::stm32::SystickCtrl::getCurrentMilliSec();
}

static IGB_FAST_INLINE uint32_t current_usec() {
  return igb::stm32::SystickCtrl::getCurrentMicroSec();
}

#endif /* USE_ARDUINO */

static IGB_FAST_INLINE void delay_msec(uint32_t msec) {
  __IO uint32_t base = current_msec();
  while((current_msec() - base) < msec) {
  }
}

}

#endif /* IGB_STM32_PERIPH_SYSTICK_H */
