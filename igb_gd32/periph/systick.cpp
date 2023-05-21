#include <igb_gd32/periph/systick.hpp>

volatile SystickState _systick_state;

#if !defined(USE_ARDUINO)

void SysTick_Handler(void) {
  igb::gd32::SystickCtrl::receiveTick();
  USER_SYSTICK_HANDLER_CALLBACK;
}

#endif
