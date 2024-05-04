#pragma once

#include <igb_stm32/base.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace stm32 {

enum class FlashLatency {
  zero = 0,
#ifdef FLASH_ACR_LATENCY_0
  one = FLASH_ACR_LATENCY_0,
#else
  one = FLASH_ACR_LATENCY,
#endif
#ifdef FLASH_ACR_LATENCY_1
  two = FLASH_ACR_LATENCY_1,
#endif
#ifdef FLASH_ACR_LATENCY_2
  three = FLASH_ACR_LATENCY_2,
#endif
};

#if defined(STM32G431xx)
enum class FlashBootMode {
  hardwarePin = 0,
  flash,
  sram,
  system
};
#endif

struct FlashCtrl {
  static IGB_FAST_INLINE void setLatency(FlashLatency latency) {
    IGB_MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, static_cast<uint32_t>(latency));
  }

  static IGB_FAST_INLINE FlashLatency getLatency() {
    return static_cast<FlashLatency>(IGB_READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY));
  }

  static IGB_FAST_INLINE bool isBusy() {
#if defined(STM32H7)
    return IGB_READ_BIT(FLASH->SR1, FLASH_SR_BSY);
#else
    return IGB_READ_BIT(FLASH->SR, FLASH_SR_BSY);
#endif
  }

#if defined(STM32F0) || defined(STM32F3)
  static IGB_FAST_INLINE void enablePrefetch() {
    IGB_SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
  }

  static IGB_FAST_INLINE void disablePrefetch() {
    IGB_CLEAR_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
  }

  static IGB_FAST_INLINE bool isPrefetchEnabled() {
    return (IGB_READ_BIT(FLASH->ACR, FLASH_ACR_PRFTBS) == (FLASH_ACR_PRFTBS));
  }
#endif

#if defined(STM32F0) || defined(STM32F3) || defined(STM32G431xx)
  static IGB_FAST_INLINE bool isLock() {
    return IGB_READ_BIT(FLASH->CR, FLASH_CR_LOCK);
  }

  static IGB_FAST_INLINE void unlock() {
    if (isLock()) {
      IGB_WRITE_REG(FLASH->KEYR, FLASH_KEY1);
      IGB_WRITE_REG(FLASH->KEYR, FLASH_KEY2);
    }
  }

  static IGB_FAST_INLINE void lock() {
    IGB_SET_BIT(FLASH->CR, FLASH_CR_LOCK);
  }
#endif

#if defined(STM32G431xx)
  static IGB_FAST_INLINE void unlockOpt() {
    IGB_WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY1);
    IGB_WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY2);
  }

  static IGB_FAST_INLINE void setBootMode(FlashBootMode bootMode) {
    while (isBusy()) {}

    unlock();
    unlockOpt();

    switch (bootMode) {
      case FlashBootMode::hardwarePin:
        {
          IGB_SET_BIT(FLASH->OPTR, FLASH_OPTR_nSWBOOT0);
          IGB_CLEAR_BIT(FLASH->OPTR, FLASH_OPTR_nBOOT1);
        }
        break;
      case FlashBootMode::flash:
        {
          IGB_CLEAR_BIT(FLASH->OPTR, FLASH_OPTR_nSWBOOT0);
          IGB_SET_BIT(FLASH->OPTR, FLASH_OPTR_nBOOT0);
          IGB_CLEAR_BIT(FLASH->OPTR, FLASH_OPTR_nBOOT1);
        }
        break;
      case FlashBootMode::sram:
        {
          IGB_CLEAR_BIT(FLASH->OPTR, FLASH_OPTR_nSWBOOT0);
          IGB_CLEAR_BIT(FLASH->OPTR, FLASH_OPTR_nBOOT0);
          IGB_CLEAR_BIT(FLASH->OPTR, FLASH_OPTR_nBOOT1);
        }
        break;
      case FlashBootMode::system:
        {
          IGB_CLEAR_BIT(FLASH->OPTR, FLASH_OPTR_nSWBOOT0);
          IGB_CLEAR_BIT(FLASH->OPTR, FLASH_OPTR_nBOOT0);
          IGB_SET_BIT(FLASH->OPTR, FLASH_OPTR_nBOOT1);
        }
        break;
    }

    IGB_SET_BIT(FLASH->CR, FLASH_CR_OPTSTRT);

    while (isBusy()) {}

    lock(); // lock & optlock

    while (isBusy()) {}
  }

  static FlashBootMode getBootMode() {
    if (IGB_READ_BIT(FLASH->OPTR, FLASH_OPTR_nBOOT1))  {
      return FlashBootMode::system;
    }
    if (!IGB_READ_BIT(FLASH->OPTR, FLASH_OPTR_nSWBOOT0)) {
      if (IGB_READ_BIT(FLASH->OPTR, FLASH_OPTR_nSWBOOT0)) {
        return FlashBootMode::flash;
      }
      return FlashBootMode::sram;
    }
    return FlashBootMode::hardwarePin;
  }
#endif

#if defined(STM32F0) || defined(STM32F3)
  static IGB_FAST_INLINE void erasePage(uint32_t addr) {
    while (isBusy()) {}

    FLASH->CR = FLASH->CR | FLASH_CR_PER;
    FLASH->AR = addr; 
    FLASH->CR = FLASH->CR | FLASH_CR_STRT;

    while (isBusy()) {}

    FLASH->CR = FLASH->CR & ~FLASH_CR_PER;
  }

  static IGB_FAST_INLINE void programU16(uint32_t addr, uint16_t data) {
    while (isBusy()) {}

    FLASH->CR = FLASH->CR | FLASH_CR_PG;

    *(__IO uint16_t*)addr = data;

    while (isBusy()) {}

    FLASH->CR = FLASH->CR & ~FLASH_CR_PG;
  }
#endif
};

}
}

