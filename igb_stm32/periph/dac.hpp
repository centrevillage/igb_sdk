#ifndef IGB_STM32_PERIPH_DAC_H
#define IGB_STM32_PERIPH_DAC_H

#include <stddef.h>

#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/rcc.hpp>
#include <igb_stm32/periph/systick.hpp>
#include <igb_stm32/periph/nvic.hpp>

namespace igb {
namespace stm32 {

#define IGB_DAC ((DAC_TypeDef*)addr)
#define IGB_DAC_REG_ADDR(member) (addr + offsetof(DAC_TypeDef, member))
#define IGB_DAC_REG(member) ((DAC_TypeDef*)IGB_DAC_REG_ADDR(member))

// TODO: other series
#if defined(STM32F3)

enum class DacWaveOut : uint32_t {
  disable = 0,
  noise = 1,
  triangle = 2,
};

struct DacConf {
  GpioPinType pin_type;
  bool enable_buffer = true;
  bool enable_trigger = false;
  uint8_t trigger_select = 0;
  DacWaveOut wave_out = DacWaveOut::disable;
  uint8_t wave_param = 0;
  bool dma = false;
  bool enable_underrun_it = false;
  uint16_t interrupt_priority = 1;
};

template<DacType DAC_TYPE>
struct Dac {
  constexpr static auto type = DAC_TYPE;
  constexpr static auto info = STM32_PERIPH_INFO.dac[to_idx(type)];
  constexpr static auto addr = STM32_PERIPH_INFO.dac[to_idx(type)].addr;
  constexpr static auto addr_CR = IGB_DAC_REG_ADDR(CR);
  constexpr static auto addr_SWTRIGR = IGB_DAC_REG_ADDR(SWTRIGR);
  constexpr static auto addr_DHR12R1 = IGB_DAC_REG_ADDR(DHR12R1);
  constexpr static auto addr_DHR12L1 = IGB_DAC_REG_ADDR(DHR12L1);
  constexpr static auto addr_DHR8R1 = IGB_DAC_REG_ADDR(DHR8R1);
  constexpr static auto addr_DHR12R2 = IGB_DAC_REG_ADDR(DHR12R2);
  constexpr static auto addr_DHR12L2 = IGB_DAC_REG_ADDR(DHR12L2);
  constexpr static auto addr_DHR8R2 = IGB_DAC_REG_ADDR(DHR8R2);
  constexpr static auto addr_DHR12RD = IGB_DAC_REG_ADDR(DHR12RD);
  constexpr static auto addr_DHR12LD = IGB_DAC_REG_ADDR(DHR12LD);
  constexpr static auto addr_DHR8RD = IGB_DAC_REG_ADDR(DHR8RD);
  constexpr static auto addr_DOR1 = IGB_DAC_REG_ADDR(DOR1);
  constexpr static auto addr_DOR2 = IGB_DAC_REG_ADDR(DOR2);
  constexpr static auto addr_SR = IGB_DAC_REG_ADDR(SR);

  RegFlag<addr_CR, DAC_CR_EN1> enable1;
  RegFlag<addr_CR, DAC_CR_BOFF1, false> enableBuffer1;
#if defined(DAC_CR_OUTEN1)
  RegFlag<addr_CR, DAC_CR_OUTEN1> enableOutput1;
#endif
  RegFlag<addr_CR, DAC_CR_TEN1> enableTrigger1;
  RegValue<addr_CR, DAC_CR_TSEL1_Msk, DAC_CR_TSEL1_Pos> triggerSelect1;
  RegEnum<addr_CR, DAC_CR_WAVE1_Msk, DacWaveOut, DAC_CR_WAVE1_Pos> waveOut1;
  RegValue<addr_CR, DAC_CR_MAMP1_Msk, DAC_CR_MAMP1_Pos> waveParam1;
  RegFlag<addr_CR, DAC_CR_DMAEN1> dma1;
  RegFlag<addr_CR, DAC_CR_DMAUDRIE1> enableUnderrunIt1;
  RegFlag<addr_SWTRIGR, DAC_SWTRIGR_SWTRIG1> enableSoftwareTrigger1;
  RegValue<addr_DHR12R1, DAC_DHR12R1_DACC1DHR_Msk, DAC_DHR12R1_DACC1DHR_Pos> data12bitR1;
  RegValue<addr_DHR12L1, DAC_DHR12L1_DACC1DHR_Msk, DAC_DHR12L1_DACC1DHR_Pos> data12bitL1;
  RegValue<addr_DHR8R1, DAC_DHR8R1_DACC1DHR_Msk, DAC_DHR8R1_DACC1DHR_Pos> data8bitR1;
  RegValue<addr_DHR12RD, DAC_DHR12RD_DACC1DHR_Msk, DAC_DHR12RD_DACC1DHR_Pos> data12bitRD1;
  RegValue<addr_DHR12LD, DAC_DHR12LD_DACC1DHR_Msk, DAC_DHR12LD_DACC1DHR_Pos> data12bitLD1;
  RegValue<addr_DHR8RD, DAC_DHR8RD_DACC1DHR_Msk, DAC_DHR8RD_DACC1DHR_Pos> data8bitRD1;
  RegValueRO<addr_DOR1, DAC_DOR1_DACC1DOR_Msk, DAC_DOR1_DACC1DOR_Pos> dataOut1;
  RegFlag<addr_SR, DAC_SR_DMAUDR1> isUnderrunError1;

  RegFlag<addr_CR, DAC_CR_EN2> enable2;
  RegFlag<addr_CR, DAC_CR_BOFF2, false> enableBuffer2;
#if defined(DAC_CR_OUTEN2)
  RegFlag<addr_CR, DAC_CR_OUTEN2> enableOutput2;
#endif
  RegFlag<addr_CR, DAC_CR_TEN2> enableTrigger2;
  RegValue<addr_CR, DAC_CR_TSEL2_Msk, DAC_CR_TSEL2_Pos> triggerSelect2;
  RegEnum<addr_CR, DAC_CR_WAVE2_Msk, DacWaveOut, DAC_CR_WAVE2_Pos> waveOut2;
  RegValue<addr_CR, DAC_CR_MAMP2_Msk, DAC_CR_MAMP2_Pos> waveParam2;
  RegFlag<addr_CR, DAC_CR_DMAEN2> dma2;
  RegFlag<addr_CR, DAC_CR_DMAUDRIE2> enableUnderrunIt2;
  RegFlag<addr_SWTRIGR, DAC_SWTRIGR_SWTRIG2> enableSoftwareTrigger2;
  RegValue<addr_DHR12R2, DAC_DHR12R2_DACC2DHR_Msk, DAC_DHR12R2_DACC2DHR_Pos> data12bitR2;
  RegValue<addr_DHR12L2, DAC_DHR12L2_DACC2DHR_Msk, DAC_DHR12L2_DACC2DHR_Pos> data12bitL2;
  RegValue<addr_DHR8R2, DAC_DHR8R2_DACC2DHR_Msk, DAC_DHR8R2_DACC2DHR_Pos> data8bitR2;
  RegValue<addr_DHR12RD, DAC_DHR12RD_DACC2DHR_Msk, DAC_DHR12RD_DACC2DHR_Pos> data12bitRD2;
  RegValue<addr_DHR12LD, DAC_DHR12LD_DACC2DHR_Msk, DAC_DHR12LD_DACC2DHR_Pos> data12bitLD2;
  RegValue<addr_DHR8RD, DAC_DHR8RD_DACC2DHR_Msk, DAC_DHR8RD_DACC2DHR_Pos> data8bitRD12;
  RegValueRO<addr_DOR2, DAC_DOR2_DACC2DOR_Msk, DAC_DOR2_DACC2DOR_Pos> dataOut2;
  RegFlag<addr_SR, DAC_SR_DMAUDR2> isUnderrunError2;

  static IGB_FAST_INLINE void enableBusClock() {
    STM32_PERIPH_INFO.dac[to_idx(type)].bus.enableBusClock();
  }

  static IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    GpioPin pin = GpioPin::newPin(pin_type);
    pin.enable();
    pin.setMode(GpioMode::analog);
    pin.setPullMode(GpioPullMode::no);
  }

  IGB_FAST_INLINE void _setParams1(auto&& conf) {
    (
      enableBuffer1.val(conf.enable_buffer) |
      triggerSelect1.val(conf.trigger_select) |
      waveOut1.val(conf.wave_out) |
      waveParam1.val(conf.wave_param)
    ).update();
  }

  IGB_FAST_INLINE void _setParams2(auto&& conf) {
    (
      enableBuffer2.val(conf.enable_buffer) |
      triggerSelect2.val(conf.trigger_select) |
      waveOut2.val(conf.wave_out) |
      waveParam2.val(conf.wave_param)
    ).update();
  }

  IGB_FAST_INLINE void init1(auto&& conf) {
    enableBusClock();
    prepareGpio(conf.pin_type);
    _setParams1(conf);
    enable1(true);
    if (conf.dma) {
      dma1(true);
    }
    if (conf.enable_trigger) {
      enableTrigger1(conf.enable_trigger);
    }
    if (conf.enable_underrun_it) {
      NvicCtrl::setPriority(info.irqn, conf.interrupt_priority);
      NvicCtrl::enable(info.irqn);
      enableUnderrunIt1(true);
    }
  }

  IGB_FAST_INLINE void init2(auto&& conf) {
    enableBusClock();
    prepareGpio(conf.pin_type);
    _setParams2(conf);
    enable2(true);
    if (conf.dma) {
      dma2(true);
    }
    if (conf.enable_trigger) {
      enableTrigger2(conf.enable_trigger);
    }
    if (conf.enable_underrun_it) {
      NvicCtrl::setPriority(info.irqn, conf.interrupt_priority);
      NvicCtrl::enable(info.irqn);
      enableUnderrunIt2(true);
    }
  }

  IGB_FAST_INLINE void init(auto&& conf1, auto&& conf2) {
    enableBusClock();
    prepareGpio(conf1.pin_type);
    prepareGpio(conf2.pin_type);
    _setParams1(conf1);
    _setParams2(conf2);
    enable1(true);
    enable2(true);
    if (conf1.dma) {
      dma1(true);
    }
    if (conf1.enable_trigger) {
      enableTrigger1(conf1.enable_trigger);
    }
    if (conf1.enable_underrun_it) {
      NvicCtrl::setPriority(info.irqn, conf1.interrupt_priority);
      NvicCtrl::enable(info.irqn);
      enableUnderrunIt1(true);
    }
  }
};

#endif /* STM32F3 */

#undef IGB_DAC_REG
#undef IGB_DAC_REG_ADDR
#undef IGB_DAC

}
}

#endif /* IGB_STM32_PERIPH_DAC_H */
