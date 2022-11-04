#ifndef IGB_STM32_PERIPH_FLASH_H
#define IGB_STM32_PERIPH_FLASH_H

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

struct FlashCtrl {
  static IGB_FAST_INLINE void setLatency(FlashLatency latency) {
    IGB_MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, static_cast<uint32_t>(latency));
  }

  static IGB_FAST_INLINE FlashLatency getLatency() {
    return static_cast<FlashLatency>(IGB_READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY));
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
};

}
}

#endif /* IGB_STM32_PERIPH_FLASH_H */
