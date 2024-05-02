#pragma once

#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/nvic.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/container.hpp>
#include <igb_util/bitmagic.hpp>
#include <igb_util/reg.hpp>


namespace igb {
namespace stm32 {

#define IGB_EXTI ((EXTI_TypeDef*)addr)
#define IGB_EXTI_REG_ADDR(member) (addr + offsetof(EXTI_TypeDef, member))
#define IGB_EXTI_REG(member) ((EXTI_TypeDef*)IGB_EXTI_REG_ADDR(member))

enum class ExtiLine : uint32_t {
  line0 = 0,
  line1,
  line2,
  line3,
  line4,
  line5,
  line6,
  line7,
  line8,
  line9,
  line10,
  line11,
  line12,
  line13,
  line14,
  line15,
  line16,
  line17,
  line18,
  line19,
  line20,
  line21,
  line22,
  line23,
  line24,
  line25,
  line26,
  line27,
  line28,
  line29,
  line30,
  line31
};
constexpr uint32_t to_idx(ExtiLine line) {
  switch (line) {
    case ExtiLine::line0:
      return 0;
      break;
    case ExtiLine::line1:
      return 1;
      break;
    case ExtiLine::line2:
      return 2;
      break;
    case ExtiLine::line3:
      return 3;
      break;
    case ExtiLine::line4:
      return 4;
      break;
    case ExtiLine::line5:
      return 5;
      break;
    case ExtiLine::line6:
      return 6;
      break;
    case ExtiLine::line7:
      return 7;
      break;
    case ExtiLine::line8:
      return 8;
      break;
    case ExtiLine::line9:
      return 9;
      break;
    case ExtiLine::line10:
      return 10;
      break;
    case ExtiLine::line11:
      return 11;
      break;
    case ExtiLine::line12:
      return 12;
      break;
    case ExtiLine::line13:
      return 13;
      break;
    case ExtiLine::line14:
      return 14;
      break;
    case ExtiLine::line15:
      return 15;
      break;
    case ExtiLine::line16:
      return 16;
      break;
    case ExtiLine::line17:
      return 17;
      break;
    case ExtiLine::line18:
      return 18;
      break;
    case ExtiLine::line19:
      return 19;
      break;
    case ExtiLine::line20:
      return 20;
      break;
    case ExtiLine::line21:
      return 21;
      break;
    case ExtiLine::line22:
      return 22;
      break;
    case ExtiLine::line23:
      return 23;
      break;
    case ExtiLine::line24:
      return 24;
      break;
    case ExtiLine::line25:
      return 25;
      break;
    case ExtiLine::line26:
      return 26;
      break;
    case ExtiLine::line27:
      return 27;
      break;
    case ExtiLine::line28:
      return 28;
      break;
    case ExtiLine::line29:
      return 29;
      break;
    case ExtiLine::line30:
      return 30;
      break;
    case ExtiLine::line31:
      return 31;
      break;
    default:
      break;
  }
  return 0;
}
constexpr uint32_t to_bits(ExtiLine line) {
  return (1UL << to_idx(line));
}

enum class ExtiTrigType : uint8_t {
  falling = 0,
  rising,
  falling_and_rising
};

enum class ExtiMode : uint8_t {
  interrupt = 0,
  event,
  interrupt_and_event
};

struct ExtiConf {
  ExtiTrigType trig_type = ExtiTrigType::rising;
};

struct ExtiCtrl {
  constexpr static auto info = STM32_PERIPH_INFO.exti;
  constexpr static auto addr = STM32_PERIPH_INFO.exti.addr;
#if defined(STM32H7) || defined(STM32G431xx)
  // TODO: define IMR2,3
  constexpr static auto addr_IMR = IGB_EXTI_REG_ADDR(IMR1);
  constexpr static auto addr_EMR = IGB_EXTI_REG_ADDR(EMR1);
  constexpr static auto addr_RTSR = IGB_EXTI_REG_ADDR(RTSR1);
  constexpr static auto addr_FTSR = IGB_EXTI_REG_ADDR(FTSR1);
  constexpr static auto addr_SWIER = IGB_EXTI_REG_ADDR(SWIER1);
  constexpr static auto addr_PR = IGB_EXTI_REG_ADDR(PR1);
#else
  constexpr static auto addr_IMR = IGB_EXTI_REG_ADDR(IMR);
  constexpr static auto addr_EMR = IGB_EXTI_REG_ADDR(EMR);
  constexpr static auto addr_RTSR = IGB_EXTI_REG_ADDR(RTSR);
  constexpr static auto addr_FTSR = IGB_EXTI_REG_ADDR(FTSR);
  constexpr static auto addr_SWIER = IGB_EXTI_REG_ADDR(SWIER);
  constexpr static auto addr_PR = IGB_EXTI_REG_ADDR(PR);
#endif

  static Reg<addr_IMR> reg_IMR;
  static Reg<addr_EMR> reg_EMR;
  static Reg<addr_RTSR> reg_RTSR;
  static Reg<addr_FTSR> reg_FTSR;
  static Reg<addr_SWIER> reg_SWIER;
  static Reg<addr_PR> reg_PR;

  IGB_FAST_INLINE static void enableIt(ExtiLine line) {
    reg_IMR(reg_IMR() | to_bits(line));
  }
  IGB_FAST_INLINE static void disableIt(ExtiLine line) {
    reg_IMR(reg_IMR() & ~to_bits(line));
  }

  IGB_FAST_INLINE static void enableIrqn(ExtiLine line, uint8_t priority) {
    const auto irqn = info.line_irqns[to_idx(line)];
    NvicCtrl::setPriority(irqn, priority);
    NvicCtrl::enable(irqn);
  }

  IGB_FAST_INLINE static void disableIrqn(ExtiLine line) {
    const auto irqn = info.line_irqns[to_idx(line)];
    NvicCtrl::disable(irqn);
  }

  IGB_FAST_INLINE static void enableEvent(ExtiLine line) {
    reg_EMR(reg_EMR() | to_bits(line));
  }
  IGB_FAST_INLINE static void disableEvent(ExtiLine line) {
    reg_EMR(reg_EMR() & ~to_bits(line));
  }

  IGB_FAST_INLINE static void enableRisingTrig(ExtiLine line) {
    reg_RTSR(reg_RTSR() | to_bits(line));
  }
  IGB_FAST_INLINE static void disableRisingTrig(ExtiLine line) {
    reg_RTSR(reg_RTSR() & ~to_bits(line));
  }

  IGB_FAST_INLINE static void enableFallingTrig(ExtiLine line) {
    reg_FTSR(reg_FTSR() | to_bits(line));
  }
  IGB_FAST_INLINE static void disableFallingTrig(ExtiLine line) {
    reg_FTSR(reg_FTSR() & ~to_bits(line));
  }

  IGB_FAST_INLINE static void softwareTrigger(ExtiLine line) {
    reg_SWIER(reg_SWIER() | to_bits(line));
  }

  IGB_FAST_INLINE static void clearItState(ExtiLine line) {
    reg_PR(reg_PR() | to_bits(line));
  }

  IGB_FAST_INLINE static bool isIt(ExtiLine line) {
    return reg_PR() & to_bits(line);
  }

  IGB_FAST_INLINE static void enableLine(ExtiLine line, ExtiTrigType trig_type, ExtiMode mode, uint8_t priority) {
    switch (trig_type) {
      case ExtiTrigType::falling:
        disableRisingTrig(line);
        enableFallingTrig(line);
        break;
      case ExtiTrigType::rising:
        disableFallingTrig(line);
        enableRisingTrig(line);
        break;
      case ExtiTrigType::falling_and_rising:
        enableFallingTrig(line);
        enableRisingTrig(line);
        break;
      default:
        break;
    }

    switch (mode) {
      case ExtiMode::interrupt:
        disableEvent(line);
        enableIt(line);
        break;
      case ExtiMode::event:
        disableIt(line);
        enableEvent(line);
        break;
      case ExtiMode::interrupt_and_event:
        enableEvent(line);
        enableIt(line);
        break;
      default:
        break;
    }

    enableIrqn(line, priority);
  }

  IGB_FAST_INLINE static void disableLine(ExtiLine line) {
    disableIrqn(line);
    disableIt(line);
    disableEvent(line);
  }
};

#undef IGB_EXTI_REG
#undef IGB_EXTI_REG_ADDR
#undef IGB_EXTI

}
}

