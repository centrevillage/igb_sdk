#ifndef IGB_GD32_PERIPH_DAC_H
#define IGB_GD32_PERIPH_DAC_H

#include <stddef.h>

#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>
#include <igb_gd32/base.hpp>
#include <igb_gd32/periph/gpio.hpp>
#include <igb_gd32/periph/rcc.hpp>
#include <igb_gd32/periph/systick.hpp>
#include <igb_gd32/periph/nvic.hpp>

namespace igb {
namespace gd32 {

#define IGB_DAC ((DAC_TypeDef*)addr)
#define IGB_DAC_REG_ADDR(member) (addr + offsetof(DAC_TypeDef, member))
#define IGB_DAC_REG(member) ((DAC_TypeDef*)IGB_DAC_REG_ADDR(member))

enum class DacTriggerSelect {
  timer5_trgo = 0,
  timer2_trgo = 1,
  timer14_trgo = 3,
  timer1_trgo = 4,
  exti_line9 = 6,
  software = 7,
};

enum class DacWaveOut {
  disable = 0,
  noise = 1,
  triangle = 2,
};

struct DacConf {
  GpioPinType pin_type;
  bool enable_buffer = true;
  bool enable_trigger = false;
  DacTriggerSelect trigger_select = DacTriggerSelect::software;
  DacWaveOut wave_out = DacWaveOut::disable;
  uint8_t wave_bit_width = 12-1;
  bool dma = false;
  bool enable_underrun_it = false;
  uint16_t interrupt_priority = 1;
};

template<DacType DAC_TYPE>
struct Dac {
  constexpr static auto type = DAC_TYPE;
  constexpr static auto info = GD32_PERIPH_INFO.dac[to_idx(type)];
  constexpr static auto addr = GD32_PERIPH_INFO.dac[to_idx(type)].addr;
  constexpr static auto addr_CTL = IGB_DAC_REG_ADDR(CTL);
  constexpr static auto addr_SWT = IGB_DAC_REG_ADDR(SWT);
  constexpr static auto addr_R12DH = IGB_DAC_REG_ADDR(R12DH);
  constexpr static auto addr_L12DH = IGB_DAC_REG_ADDR(L12DH);
  constexpr static auto addr_R8DH = IGB_DAC_REG_ADDR(R8DH);
  constexpr static auto addr_DO = IGB_DAC_REG_ADDR(DO);
  constexpr static auto addr_STAT = IGB_DAC_REG_ADDR(STAT);

  RegFlag<addr_CTL, IGB_BIT(0)> enableDac;
  RegFlag<addr_CTL, IGB_BIT(1)> disableBuffer;
  RegFlag<addr_CTL, IGB_BIT(2)> enableTrigger;
  RegEnum<addr_CTL, IGB_BIT_MASK(3, 3), DacTriggerSelect, 3> triggerSelect;
  RegEnum<addr_CTL, IGB_BIT_MASK(2, 6), DacWaveOut, 6> waveOut;
  RegValue<addr_CTL, IGB_BIT_MASK(4, 8), 8> waveBitWidth;
  RegFlag<addr_CTL, IGB_BIT(12)> dma;
  RegFlag<addr_CTL, IGB_BIT(13)> enableItUnderrun;

  RegFlag<addr_SWT, IGB_BIT(0)> softwareTrigger;

  RegValue<addr_R12DH, IGB_BIT_MASK(12, 0), 0> data12bitR;
  RegValue<addr_L12DH, IGB_BIT_MASK(12, 4), 4> data12bitL;

  RegValue<addr_R8DH, IGB_BIT_MASK(8, 0), 0> data8bitR;

  RegValueRO<addr_DO, IGB_BIT_MASK(12, 0), 0> outputData;

  RegFlag<addr_STAT, IGB_BIT(13)> dmaUnderrunFlag;

  static IGB_FAST_INLINE void enableBusClock() {
    GD32_PERIPH_INFO.dac[to_idx(type)].bus.enableBusClock();
  }

  static IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    GpioPin pin = GpioPin::newPin(pin_type);
    pin.enable();
    pin.setMode(GpioMode::analog);
    pin.setPullMode(GpioPullMode::no);
  }

  IGB_FAST_INLINE void init(auto&& conf) {
    enableBusClock();
    prepareGpio(conf.pin_type);
    disableBuffer(!conf.enable_buffer);
    triggerSelect(conf.trigger_select);
    waveOut(conf.wave_out);
    waveBitWidth(conf.wave_bit_width);

    enableDac(true);

    if (conf.dma) {
      dma(true);
    }
    if (conf.enable_trigger) {
      enableTrigger(true);
    }
    if (conf.enable_underrun_it) {
      NvicCtrl::setPriority(info.irqn, conf.interrupt_priority);
      NvicCtrl::enable(info.irqn);
      enableItUnderrun(true);
    }
  }
};

#undef IGB_DAC_REG
#undef IGB_DAC_REG_ADDR
#undef IGB_DAC

}
}

#endif /* IGB_GD32_PERIPH_DAC_H */
