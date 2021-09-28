#ifndef IGB_STM32_PERIPH_SYSCFG_H
#define IGB_STM32_PERIPH_SYSCFG_H

#include <igb_stm32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>

namespace igb {
namespace stm32 {

#define IGB_SYSCFG ((SYSCFG_TypeDef*)addr)
#define IGB_SYSCFG_REG_ADDR(member) (addr + offsetof(SYSCFG_TypeDef, member))
#define IGB_SYSCFG_REG(member) ((SYSCFG_TypeDef*)IGB_SYSCFG_REG_ADDR(member))

struct SysCfg {
  constexpr static auto addr = SYSCFG_BASE;
// TODO: other series
#if defined(STM32F3)
  constexpr static auto addr_CFGR1 = IGB_SYSCFG_REG_ADDR(CFGR1);
#endif
  //constexpr static auto addr_RCR = IGB_SYSCFG_REG_ADDR(RCR);
  constexpr static auto addr_EXTICR1 = IGB_SYSCFG_REG_ADDR(EXTICR[0]);
  constexpr static auto addr_EXTICR2 = IGB_SYSCFG_REG_ADDR(EXTICR[1]);
  constexpr static auto addr_EXTICR3 = IGB_SYSCFG_REG_ADDR(EXTICR[2]);
  constexpr static auto addr_EXTICR4 = IGB_SYSCFG_REG_ADDR(EXTICR[3]);
  //constexpr static auto addr_CFGR2 = IGB_SYSCFG_REG_ADDR(CFGR2);
  //constexpr static auto addr_CFGR3 = IGB_SYSCFG_REG_ADDR(CFGR3);

// TODO: other series
#if defined(STM32F3)
  static RegFlag<addr_CFGR1, SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP> tim6Dac1Ch1DmaRmp;
  static RegFlag<addr_CFGR1, SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP> tim7Dac1Ch2DmaRmp;
  static RegFlag<addr_CFGR1, SYSCFG_CFGR1_DAC1_TRIG1_RMP> dac1Trig1Rmp;
#endif /* STM32F3 */

  static RegEnum<addr_EXTICR1, SYSCFG_EXTICR1_EXTI0_Msk, GpioType, SYSCFG_EXTICR1_EXTI0_Pos> exti0GpioPort;
  static RegEnum<addr_EXTICR1, SYSCFG_EXTICR1_EXTI1_Msk, GpioType, SYSCFG_EXTICR1_EXTI1_Pos> exti1GpioPort;
  static RegEnum<addr_EXTICR1, SYSCFG_EXTICR1_EXTI2_Msk, GpioType, SYSCFG_EXTICR1_EXTI2_Pos> exti2GpioPort;
  static RegEnum<addr_EXTICR1, SYSCFG_EXTICR1_EXTI3_Msk, GpioType, SYSCFG_EXTICR1_EXTI3_Pos> exti3GpioPort;

  static RegEnum<addr_EXTICR2, SYSCFG_EXTICR2_EXTI4_Msk, GpioType, SYSCFG_EXTICR2_EXTI4_Pos> exti4GpioPort;
  static RegEnum<addr_EXTICR2, SYSCFG_EXTICR2_EXTI5_Msk, GpioType, SYSCFG_EXTICR2_EXTI5_Pos> exti5GpioPort;
  static RegEnum<addr_EXTICR2, SYSCFG_EXTICR2_EXTI6_Msk, GpioType, SYSCFG_EXTICR2_EXTI6_Pos> exti6GpioPort;
  static RegEnum<addr_EXTICR2, SYSCFG_EXTICR2_EXTI7_Msk, GpioType, SYSCFG_EXTICR2_EXTI7_Pos> exti7GpioPort;

  static RegEnum<addr_EXTICR3, SYSCFG_EXTICR3_EXTI8_Msk, GpioType, SYSCFG_EXTICR3_EXTI8_Pos> exti8GpioPort;
  static RegEnum<addr_EXTICR3, SYSCFG_EXTICR3_EXTI9_Msk, GpioType, SYSCFG_EXTICR3_EXTI9_Pos> exti9GpioPort;
  static RegEnum<addr_EXTICR3, SYSCFG_EXTICR3_EXTI10_Msk, GpioType, SYSCFG_EXTICR3_EXTI10_Pos> exti10GpioPort;
  static RegEnum<addr_EXTICR3, SYSCFG_EXTICR3_EXTI11_Msk, GpioType, SYSCFG_EXTICR3_EXTI11_Pos> exti11GpioPort;

  static RegEnum<addr_EXTICR4, SYSCFG_EXTICR4_EXTI12_Msk, GpioType, SYSCFG_EXTICR4_EXTI12_Pos> exti12GpioPort;
  static RegEnum<addr_EXTICR4, SYSCFG_EXTICR4_EXTI13_Msk, GpioType, SYSCFG_EXTICR4_EXTI13_Pos> exti13GpioPort;
  static RegEnum<addr_EXTICR4, SYSCFG_EXTICR4_EXTI14_Msk, GpioType, SYSCFG_EXTICR4_EXTI14_Pos> exti14GpioPort;
  static RegEnum<addr_EXTICR4, SYSCFG_EXTICR4_EXTI15_Msk, GpioType, SYSCFG_EXTICR4_EXTI15_Pos> exti15GpioPort;
};

}
}


#endif /* IGB_STM32_PERIPH_SYSCFG_H */
