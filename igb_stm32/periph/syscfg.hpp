#pragma once

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
  constexpr static auto info = STM32_PERIPH_INFO.syscfg;
  constexpr static auto addr = SYSCFG_BASE;
// TODO: other series
#if defined(STM32F3)
  constexpr static auto addr_CFGR1 = IGB_SYSCFG_REG_ADDR(CFGR1);
#endif
  //constexpr static auto addr_RCR = IGB_SYSCFG_REG_ADDR(RCR);
#if defined(STM32G031xx)
  constexpr static auto addr_ITLINE0 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[0]);
  constexpr static auto addr_ITLINE1 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[1]);
  constexpr static auto addr_ITLINE2 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[2]);
  constexpr static auto addr_ITLINE3 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[3]);
  constexpr static auto addr_ITLINE4 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[4]);
  constexpr static auto addr_ITLINE5 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[5]);
  constexpr static auto addr_ITLINE6 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[6]);
  constexpr static auto addr_ITLINE7 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[7]);
  constexpr static auto addr_ITLINE8 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[8]);
  constexpr static auto addr_ITLINE9 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[9]);
  constexpr static auto addr_ITLINE10 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[10]);
  constexpr static auto addr_ITLINE11 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[11]);
  constexpr static auto addr_ITLINE12 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[12]);
  constexpr static auto addr_ITLINE13 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[13]);
  constexpr static auto addr_ITLINE14 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[14]);
  constexpr static auto addr_ITLINE15 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[15]);
  constexpr static auto addr_ITLINE16 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[16]);
  constexpr static auto addr_ITLINE17 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[17]);
  constexpr static auto addr_ITLINE18 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[18]);
  constexpr static auto addr_ITLINE19 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[19]);
  constexpr static auto addr_ITLINE20 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[20]);
  constexpr static auto addr_ITLINE21 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[21]);
  constexpr static auto addr_ITLINE22 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[22]);
  constexpr static auto addr_ITLINE23 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[23]);
  constexpr static auto addr_ITLINE24 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[24]);
  constexpr static auto addr_ITLINE25 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[25]);
  constexpr static auto addr_ITLINE26 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[26]);
  constexpr static auto addr_ITLINE27 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[27]);
  constexpr static auto addr_ITLINE28 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[28]);
  constexpr static auto addr_ITLINE29 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[29]);
  constexpr static auto addr_ITLINE30 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[30]);
  constexpr static auto addr_ITLINE31 = IGB_SYSCFG_REG_ADDR(IT_LINE_SR[31]);
#else
  constexpr static auto addr_EXTICR1 = IGB_SYSCFG_REG_ADDR(EXTICR[0]);
  constexpr static auto addr_EXTICR2 = IGB_SYSCFG_REG_ADDR(EXTICR[1]);
  constexpr static auto addr_EXTICR3 = IGB_SYSCFG_REG_ADDR(EXTICR[2]);
  constexpr static auto addr_EXTICR4 = IGB_SYSCFG_REG_ADDR(EXTICR[3]);
  //constexpr static auto addr_CFGR2 = IGB_SYSCFG_REG_ADDR(CFGR2);
  //constexpr static auto addr_CFGR3 = IGB_SYSCFG_REG_ADDR(CFGR3);
#endif

// TODO: other series
#if defined(STM32F3)
  static RegFlag<addr_CFGR1, SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP> tim6Dac1Ch1DmaRmp;
  static RegFlag<addr_CFGR1, SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP> tim7Dac1Ch2DmaRmp;
  static RegFlag<addr_CFGR1, SYSCFG_CFGR1_DAC1_TRIG1_RMP> dac1Trig1Rmp;
#endif /* STM32F3 */

#if defined(STM32G031xx)
  // TODO:
#else
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
#endif

  IGB_FAST_INLINE static void enableBusClock() { 
    info.bus.enableBusClock();
  }

  IGB_FAST_INLINE static void disableBusClock() { 
    info.bus.disableBusClock();
  }
};

}
}


