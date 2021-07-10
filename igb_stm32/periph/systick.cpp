#include <igb_stm32/periph/systick.hpp>

volatile SystickState _systick_state;

#if !defined(USE_ARDUINO) && !defined(USE_DAISY)

void SysTick_Handler(void) {
  igb::stm32::SystickCtrl::receiveTick();
  USER_SYSTICK_HANDLER_CALLBACK;
}

#endif
