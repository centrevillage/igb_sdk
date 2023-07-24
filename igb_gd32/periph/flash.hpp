#pragma once

#include <igb_gd32/base.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace gd32 {

enum class FlashStatus {
  busy = 0,
  programError = 2,
  writeProtectError = 4,
  operationEnd = 5
};

enum class FlashLatency {
  zero = 0,
  one,
  two,
  //three,
};

struct FlashCtrl {
  constexpr static auto addr = FLASH_BASE;
  constexpr static auto addr_ACR = addr;
  constexpr static auto addr_KEYR = addr + 0x04;
  constexpr static auto addr_OPTKEYR = addr + 0x08;
  constexpr static auto addr_SR = addr + 0x0C;
  constexpr static auto addr_CR = addr + 0x10;
  constexpr static auto addr_AR = addr + 0x14;
  constexpr static auto addr_OBR = addr + 0x1C;
  constexpr static auto addr_WRPR = addr + 0x20;

  static RegEnum<addr_ACR, IGB_BIT_MASK(3, 0), FlashLatency, 0> latency;
  static Reg<addr_KEYR> unlockKey;
  static Reg<addr_OPTKEYR> unlockKeyOpt;
  static Reg<addr_SR> status;
  static RegFlag<addr_SR, IGB_BIT(0)> isBusy;
  static RegFlag<addr_SR, IGB_BIT(2)> isProgramError;
  static RegFlag<addr_SR, IGB_BIT(4)> isWriteProtectError;
  static RegFlag<addr_SR, IGB_BIT(5)> isOperationEnd;
  static Reg<addr_CR> control;
  static RegFlag<addr_CR, IGB_BIT(0)> cmdProgram;
  static RegFlag<addr_CR, IGB_BIT(1)> cmdErasePage;
  static RegFlag<addr_CR, IGB_BIT(2)> cmdEraseAllPage;
  static RegFlag<addr_CR, IGB_BIT(4)> cmdProgramOptByte;
  static RegFlag<addr_CR, IGB_BIT(5)> cmdEraseOptByte;
  static RegFlag<addr_CR, IGB_BIT(6)> cmdStart;
  static RegFlag<addr_CR, IGB_BIT(7)> lockFlag;
  static RegFlag<addr_CR, IGB_BIT(9)> enableOptByteWrite;
  static RegFlag<addr_CR, IGB_BIT(10)> enableErrorInterrupt;
  static RegFlag<addr_CR, IGB_BIT(12)> enableOperationEndInterrupt;
  static RegFlag<addr_CR, IGB_BIT(13)> forceOptByteReload;;

  static Reg<addr_AR> address;

  static RegFlag<addr_OBR, IGB_BIT(0)> optByteLoadError;
  static RegValue<addr_OBR, IGB_BIT_MASK(2, 1), 1> optByteReadProtectionLevel;
  static RegValue<addr_OBR, IGB_BIT_MASK(8, 8), 8> optByte;
  static RegValue<addr_OBR, IGB_BIT_MASK(8, 16), 16> optByteData0;
  static RegValue<addr_OBR, IGB_BIT_MASK(8, 24), 24> optByteData1;

  static RegRO<addr_WRPR> writeProtectionByte;

  static IGB_FAST_INLINE void setLatency(FlashLatency l) {
    latency(l);
  }

  static IGB_FAST_INLINE FlashLatency getLatency() {
    return latency();
  }

  static IGB_FAST_INLINE bool isLock() {
    return lockFlag();
  }

  static IGB_FAST_INLINE bool is(FlashStatus s) {
    volatile bool result = status() & (1UL << static_cast<uint32_t>(s));
    return result;
  }

  static IGB_FAST_INLINE void unlock() {
    if (isLock()) {
      unlockKey(FLASH_KEY1);
      unlockKey(FLASH_KEY2);
    }
  }

  static IGB_FAST_INLINE void lock() {
    lockFlag(true);
  }

  static IGB_FAST_INLINE void erasePage(uint32_t addr) {
    while (isBusy()) {}

    cmdErasePage(true); 
    address(addr);
    cmdStart(true);

    while (isBusy()) {}

    cmdErasePage(false);
  }

  static IGB_FAST_INLINE void programU16(uint32_t addr, uint16_t data) {
    while (isBusy()) {}

    cmdProgram(true);

    *(volatile uint16_t*)addr = data;

    while (isBusy()) {}

    cmdProgram(false);
  }
};

}
}

