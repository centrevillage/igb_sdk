#pragma once

#include <igb_gd32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>

namespace igb {
namespace gd32 {

struct SysCfg {
  constexpr static auto info = GD32_PERIPH_INFO.syscfg;
  constexpr static auto addr = GD32_PERIPH_INFO.syscfg.addr;
  constexpr static auto addr_CFG0    = addr;
  constexpr static auto addr_EXTICR1 = addr + 0x08;
  constexpr static auto addr_EXTICR2 = addr + 0x0C;
  constexpr static auto addr_EXTICR3 = addr + 0x10;
  constexpr static auto addr_EXTICR4 = addr + 0x14;

  static RegEnum<addr_EXTICR1, IGB_BIT_MASK(4, 0), GpioType, 0> exti0GpioPort;
  static RegEnum<addr_EXTICR1, IGB_BIT_MASK(4, 4), GpioType, 4> exti1GpioPort;
  static RegEnum<addr_EXTICR1, IGB_BIT_MASK(4, 8), GpioType, 8> exti2GpioPort;
  static RegEnum<addr_EXTICR1, IGB_BIT_MASK(4, 12), GpioType, 12> exti3GpioPort;

  static RegEnum<addr_EXTICR2, IGB_BIT_MASK(4, 0), GpioType, 0> exti4GpioPort;
  static RegEnum<addr_EXTICR2, IGB_BIT_MASK(4, 4), GpioType, 4> exti5GpioPort;
  static RegEnum<addr_EXTICR2, IGB_BIT_MASK(4, 8), GpioType, 8> exti6GpioPort;
  static RegEnum<addr_EXTICR2, IGB_BIT_MASK(4, 12), GpioType, 12> exti7GpioPort;

  static RegEnum<addr_EXTICR3, IGB_BIT_MASK(4, 0), GpioType, 0> exti8GpioPort;
  static RegEnum<addr_EXTICR3, IGB_BIT_MASK(4, 4), GpioType, 4> exti9GpioPort;
  static RegEnum<addr_EXTICR3, IGB_BIT_MASK(4, 8), GpioType, 8> exti10GpioPort;
  static RegEnum<addr_EXTICR3, IGB_BIT_MASK(4, 12), GpioType, 12> exti11GpioPort;

  static RegEnum<addr_EXTICR4, IGB_BIT_MASK(4, 0), GpioType, 0> exti12GpioPort;
  static RegEnum<addr_EXTICR4, IGB_BIT_MASK(4, 4), GpioType, 4> exti13GpioPort;
  static RegEnum<addr_EXTICR4, IGB_BIT_MASK(4, 8), GpioType, 8> exti14GpioPort;
  static RegEnum<addr_EXTICR4, IGB_BIT_MASK(4, 12), GpioType, 12> exti15GpioPort;

  IGB_FAST_INLINE static void enableBusClock() { 
    info.bus.enableBusClock();
  }

  IGB_FAST_INLINE static void disableBusClock() { 
    info.bus.disableBusClock();
  }
};

}
}


