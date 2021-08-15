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

// TODO: other series
#if defined(STM32F3)

struct SysCfg {
  constexpr static auto addr = SYSCFG_BASE;
  constexpr static auto addr_CFGR1 = IGB_SYSCFG_REG_ADDR(CFGR1);
  constexpr static auto addr_RCR = IGB_SYSCFG_REG_ADDR(RCR);
  constexpr static auto addr_EXTICR1 = IGB_SYSCFG_REG_ADDR(EXTICR[0]);
  constexpr static auto addr_EXTICR2 = IGB_SYSCFG_REG_ADDR(EXTICR[1]);
  constexpr static auto addr_EXTICR3 = IGB_SYSCFG_REG_ADDR(EXTICR[2]);
  constexpr static auto addr_EXTICR4 = IGB_SYSCFG_REG_ADDR(EXTICR[3]);
  constexpr static auto addr_CFGR2 = IGB_SYSCFG_REG_ADDR(CFGR2);
  //constexpr static auto addr_CFGR3 = IGB_SYSCFG_REG_ADDR(CFGR3);

  RegFlag<addr_CFGR1, SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP> tim6Dac1Ch1DmaRmp;
  RegFlag<addr_CFGR1, SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP> tim7Dac1Ch2DmaRmp;
  RegFlag<addr_CFGR1, SYSCFG_CFGR1_DAC1_TRIG1_RMP> dac1Trig1Rmp;
  //TODO:
};

#endif /* STM32F3 */

}
}


#endif /* IGB_STM32_PERIPH_SYSCFG_H */
