#pragma once

#include <igb_stm32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>

namespace igb {
namespace stm32 {

#if defined(STM32F3)

#include "f3/_rcc_enums.hpp"

#elif defined(STM32F0)

#include "f0/_rcc_enums.hpp"

#elif defined(STM32H7)

#include "h7/_rcc_enums.hpp"

#elif defined(STM32G431xx)

#include "g4/_rcc_enums.hpp"

#endif

struct RccCtrl {
  static IGB_FAST_INLINE void enableBusClock(const auto& periph_bus_info) {
    periph_bus_info.enableBusClock();
  }

  static IGB_FAST_INLINE void disableBusClock(const auto& periph_bus_info) {
    periph_bus_info.disableBusClock();
  }

  static IGB_FAST_INLINE void enableHSE() {
    IGB_SET_BIT(RCC->CR, RCC_CR_HSEON);
  }

  static IGB_FAST_INLINE void disableHSE() {
    IGB_CLEAR_BIT(RCC->CR, RCC_CR_HSEON);
  }
  
  static IGB_FAST_INLINE bool isReadyHSE() {
    return (READ_BIT(RCC->CR, RCC_CR_HSERDY) == (RCC_CR_HSERDY));
  }

  static IGB_FAST_INLINE void enableHSI() {
    IGB_SET_BIT(RCC->CR, RCC_CR_HSION);
  }

  static IGB_FAST_INLINE void disableHSI() {
    IGB_CLEAR_BIT(RCC->CR, RCC_CR_HSION);
  }

  static IGB_FAST_INLINE bool isReadyHSI() {
    return (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == (RCC_CR_HSIRDY));
  }

#if defined(STM32F3) && defined(RCC_HSI48_SUPPORT)
  static IGB_FAST_INLINE void enableHSI48() {
    IGB_SET_BIT(RCC->CR2, RCC_CR2_HSI48ON);
  }

  static IGB_FAST_INLINE void disableHSI48() {
    IGB_CLEAR_BIT(RCC->CR2, RCC_CR2_HSI48ON);
  }
#endif

#if defined(RCC_CR2_HSI14ON)
  static IGB_FAST_INLINE void enableHSI14() {
    IGB_SET_BIT(RCC->CR2, RCC_CR2_HSI14ON);
  }

  static IGB_FAST_INLINE void disableHSI14() {
    IGB_CLEAR_BIT(RCC->CR2, RCC_CR2_HSI14ON);
  }
#endif

#if defined(RCC_BDCR_LSEON)
  static IGB_FAST_INLINE void enableLSE() {
    IGB_SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
  }

  static IGB_FAST_INLINE void disableLSE() {
    IGB_CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);
  }
#endif

#if defined(RCC_BDCR_LSEBYP)
  static IGB_FAST_INLINE void enableLseBypass() {
    IGB_SET_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
  }

  static IGB_FAST_INLINE void disableLseBypass() {
    IGB_CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
  }
#endif

#if defined(RCC_CSR_LSION)
  static IGB_FAST_INLINE void enableLSI() {
    IGB_SET_BIT(RCC->CSR, RCC_CSR_LSION);
  }

  static IGB_FAST_INLINE void disableLSI() {
    IGB_CLEAR_BIT(RCC->CSR, RCC_CSR_LSION);
  }
#endif

  static IGB_FAST_INLINE void setSystemClockSrc(RccClockSrc clock_src) {
    IGB_MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, static_cast<uint32_t>(clock_src));
  }

  static IGB_FAST_INLINE RccClockSrcStatus getSystemClockSrcStatus() {
    return static_cast<RccClockSrcStatus>(READ_BIT(RCC->CFGR, RCC_CFGR_SWS));
  }

#if defined(RCC_CFGR_HPRE)
  static IGB_FAST_INLINE void setPrescalerAHB(RccClockPrescalerAHB prescaler)  {
    IGB_MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, static_cast<uint32_t>(prescaler));
  }
#endif

#if defined(RCC_CFGR_PPRE)
  static IGB_FAST_INLINE void setPrescalerAPB1(RccClockPrescalerAPB1 prescaler) {
    IGB_MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, static_cast<uint32_t>(prescaler));
  }
#elif defined(RCC_CFGR_PPRE1)
  static IGB_FAST_INLINE void setPrescalerAPB1(RccClockPrescalerAPB1 prescaler) {
    IGB_MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, static_cast<uint32_t>(prescaler));
  }
#endif
#if defined(RCC_CFGR_PPRE2)
  static IGB_FAST_INLINE void setPrescalerAPB2(RccClockPrescalerAPB2 prescaler) {
    IGB_MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, static_cast<uint32_t>(prescaler));
  }
#endif

#if defined(RCC_CR_PLLON)
  static IGB_FAST_INLINE void enablePLL() {
    IGB_SET_BIT(RCC->CR, RCC_CR_PLLON);
  }

  static IGB_FAST_INLINE void disablePLL() {
    IGB_CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
  }

  static IGB_FAST_INLINE bool isReadyPLL() {
    return (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == (RCC_CR_PLLRDY));
  }
#endif

#if defined(STM32F3)

#include "f3/_rcc_methods.hpp"

#elif defined(STM32F0)

#include "f0/_rcc_methods.hpp"

#elif defined(STM32H7)

#include "h7/_rcc_methods.hpp"

#elif defined(STM32G431xx)

#include "g4/_rcc_methods.hpp"

#endif

};

}
}

