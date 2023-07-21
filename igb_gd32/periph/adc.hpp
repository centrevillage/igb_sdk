#ifndef IGB_GD32_PERIPH_ADC_H
#define IGB_GD32_PERIPH_ADC_H

#include <stddef.h>

#include <igb_gd32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_gd32/periph/gpio.hpp>
#include <igb_gd32/periph/rcc.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>
#include <igb_gd32/periph/systick.hpp>
#include <igb_gd32/periph/nvic.hpp>

namespace igb {
namespace gd32 {

#define IGB_ADC ((ADC_TypeDef*)addr)
#define IGB_ADC_REG_ADDR(member) (addr + offsetof(ADC_TypeDef, member))
#define IGB_ADC_REG(member) ((ADC_TypeDef*)IGB_ADC_REG_ADDR(member))

enum class AdcResolution : uint32_t {
  _12bit = 0,
  _10bit,
  _8bit,
  _6bit
};

enum class AdcAlignment : uint32_t {
  right = 0,
  left = 1
};

enum class AdcExternalTrigger : uint32_t {
  timer0_ch0 = 0,
  timer0_ch1,
  timer0_ch2,
  timer1_ch1,
  timer2_trgo,
  timer14_ch0,
  exti_line11,
  software
};

enum class AdcSamplingTime : uint32_t {
  clock1 = 0,
  clock7,
  clock13,
  clock28,
  clock41,
  clock55,
  clock71,
  clock239
};

enum class AdcOversamplingRatio : uint32_t {
  _x2 = 0,
  _x4,
  _x8,
  _x16,
  _x32,
  _x64,
  _x128,
  _x256
};

enum class AdcStatus : uint32_t {
  watchDog = IGB_BIT(0),
  endOfConversion = IGB_BIT(1),
  conversionStarted = IGB_BIT(4),
};

enum class AdcInterruptType : uint32_t {
  endOfConversion = IGB_BIT(5),
  watchDog = IGB_BIT(6),
};

enum class AdcChannel : uint32_t {
  ch0 = 0,
  ch1,
  ch2,
  ch3,
  ch4,
  ch5,
  ch6,
  ch7,
  ch8,
  ch9,
  ch10,
  ch11,
  ch12,
  ch13,
  ch14,
  ch15,
  ch16,
  ch17,
  ch18,
};

struct AdcPinConf {
  AdcChannel ch;
  GpioPinType pin_type;
  AdcSamplingTime sampling_time = AdcSamplingTime::clock28;
};

struct AdcConf {
  AdcResolution resolution = AdcResolution::_12bit;
  AdcAlignment data_align = AdcAlignment::right;
  bool enable_external_trigger = true;
  AdcExternalTrigger external_trigger_select = AdcExternalTrigger::software;
  bool discontinuous_conv_mode = false;
  uint8_t discontinuous_conv_channel_count = 0;
  bool continuous_conv_mode = false;
  bool scanMode = false;
  bool watchDogRoutine = false;
  bool watchDogSingle = false;
  bool dma = false;
  bool vbat = false;
  bool tsvr = false;
  bool enable_interrupt = false;
  uint32_t interrupt_bits = static_cast<uint32_t>(AdcInterruptType::endOfConversion);
  bool interrupt_priority = 1;
};

template<AdcType ADC_TYPE>
struct Adc {
  constexpr static auto type = ADC_TYPE;
  constexpr static auto info = GD32_PERIPH_INFO.adc[to_idx(type)];
  constexpr static auto addr = GD32_PERIPH_INFO.adc[to_idx(type)].addr;
  constexpr static auto addr_STAT = IGB_ADC_REG_ADDR(STAT);
  constexpr static auto addr_CTL0 = IGB_ADC_REG_ADDR(CTL0);
  constexpr static auto addr_CTL1 = IGB_ADC_REG_ADDR(CTL1);
  constexpr static auto addr_SAMPT0 = IGB_ADC_REG_ADDR(SAMPT0);
  constexpr static auto addr_SAMPT1 = IGB_ADC_REG_ADDR(SAMPT1);
  constexpr static auto addr_WDHT = IGB_ADC_REG_ADDR(WDHT);
  constexpr static auto addr_WDLT = IGB_ADC_REG_ADDR(WDLT);
  constexpr static auto addr_RSQ0 = IGB_ADC_REG_ADDR(RSQ0);
  constexpr static auto addr_RSQ1 = IGB_ADC_REG_ADDR(RSQ1);
  constexpr static auto addr_RSQ2 = IGB_ADC_REG_ADDR(RSQ2);
  constexpr static auto addr_RDATA = IGB_ADC_REG_ADDR(RDATA);
  constexpr static auto addr_OVSAMPCTL = IGB_ADC_REG_ADDR(OVSAMPCTL);

  RegFlag<addr_STAT, IGB_BIT(0)> watchDogEvent; // WDE; Analog watchdog event flag
  RegFlag<addr_STAT, IGB_BIT(1)> endOfConversion; // EOC; End flag of routine sequence conversion
  RegFlag<addr_STAT, IGB_BIT(4)> conversionStarted; // STRC; Start flag of routine sequence conversion

  RegValue<addr_CTL0, IGB_BIT_MASK(5, 0), 0> watchDogChSelect; // WDCHSEL; Analog watchdog channel select
  RegFlag<addr_CTL0, IGB_BIT(5)> enableItEndOfConversion; // EOCIE; Interrupt enable for EOC
  RegFlag<addr_CTL0, IGB_BIT(6)> enableItWatchDogEvent; // WDEIE; Interrupt enable for WDE
  RegFlag<addr_CTL0, IGB_BIT(8)> scanMode; // SM; Scan mode
  RegFlag<addr_CTL0, IGB_BIT(9)> watchDogSingleCh; // WDSC; When in scan mode, analog watchdog is effective on a single channel
  RegFlag<addr_CTL0, IGB_BIT(11)> discontinuousConvMode; // DISRC; Discontinuous mode on routine sequence
  RegValue<addr_CTL0, IGB_BIT_MASK(3, 13), 13> discontinuousConvChannelCount; // DISNUM; Number of conversions in discontinuous mode
  RegFlag<addr_CTL0, IGB_BIT(23)> enableRoutineChWatchDog; // RWDEN; Routine channel analog watchdog enable
  RegEnum<addr_CTL0, IGB_BIT_MASK(2, 24), AdcResolution, 24> resolution; // DRES; ADC resolution

  RegFlag<addr_CTL1, IGB_BIT(0)> enableAdc; // ADCON; ADC ON
  RegFlag<addr_CTL1, IGB_BIT(1)> continuousConvMode; // CTN; Continuous mode
  RegFlag<addr_CTL1, IGB_BIT(2)> calibrationStart; // CLB; ADC calibration
  RegFlag<addr_CTL1, IGB_BIT(3)> resetCalibrationStart; // RSTCLB; Reset calibration. This bit is set by software and cleared by hardware after the calibration registers are initialized.
  RegFlag<addr_CTL1, IGB_BIT(8)> dma; // DMA; DMA request enable.
  RegEnum<addr_CTL1, IGB_BIT_MASK(1, 11), AdcAlignment, 11> dataAlign; // DAL; Data alignment
  RegEnum<addr_CTL1, IGB_BIT_MASK(3, 17), AdcExternalTrigger, 17> externalTrigSelect; // ETSRC; External trigger select for routine sequence
  RegFlag<addr_CTL1, IGB_BIT(20)> enableExternalTrigger; // ETERC; External trigger enable for routine sequence
  RegFlag<addr_CTL1, IGB_BIT(22)> softStartConversion; // SWRCST; Software start conversion of routine sequence.
  RegFlag<addr_CTL1, IGB_BIT(23)> enableTsvr; // TSVREN; Channel 16 and 17 enable of ADC.
  RegFlag<addr_CTL1, IGB_BIT(24)> enableVbat; // VBATEN; This bit is set and cleared by software to enable/disable the VBAT channel.

  RegEnum<addr_SAMPT0, IGB_BIT_MASK(3, 0), AdcSamplingTime, 0> ch10SamplingTime; // SPT10; Channel sampling time
  RegEnum<addr_SAMPT0, IGB_BIT_MASK(3, 3), AdcSamplingTime, 3> ch11SamplingTime; // SPT11
  RegEnum<addr_SAMPT0, IGB_BIT_MASK(3, 6), AdcSamplingTime, 6> ch12SamplingTime; // SPT12
  RegEnum<addr_SAMPT0, IGB_BIT_MASK(3, 9), AdcSamplingTime, 9> ch13SamplingTime; // SPT13
  RegEnum<addr_SAMPT0, IGB_BIT_MASK(3, 12), AdcSamplingTime, 12> ch14SamplingTime; // SPT14
  RegEnum<addr_SAMPT0, IGB_BIT_MASK(3, 15), AdcSamplingTime, 15> ch15SamplingTime; // SPT15
  RegEnum<addr_SAMPT0, IGB_BIT_MASK(3, 18), AdcSamplingTime, 18> ch16SamplingTime; // SPT16
  RegEnum<addr_SAMPT0, IGB_BIT_MASK(3, 21), AdcSamplingTime, 21> ch17SamplingTime; // SPT17
  RegEnum<addr_SAMPT0, IGB_BIT_MASK(3, 24), AdcSamplingTime, 24> ch18SamplingTime; // SPT18

  RegEnum<addr_SAMPT1, IGB_BIT_MASK(3, 0), AdcSamplingTime, 0> ch0SamplingTime; // SPT0
  RegEnum<addr_SAMPT1, IGB_BIT_MASK(3, 3), AdcSamplingTime, 3> ch1SamplingTime; // SPT1
  RegEnum<addr_SAMPT1, IGB_BIT_MASK(3, 6), AdcSamplingTime, 6> ch2SamplingTime; // SPT2
  RegEnum<addr_SAMPT1, IGB_BIT_MASK(3, 9), AdcSamplingTime, 9> ch3SamplingTime; // SPT3
  RegEnum<addr_SAMPT1, IGB_BIT_MASK(3, 12), AdcSamplingTime, 12> ch4SamplingTime; // SPT4
  RegEnum<addr_SAMPT1, IGB_BIT_MASK(3, 15), AdcSamplingTime, 15> ch5SamplingTime; // SPT5
  RegEnum<addr_SAMPT1, IGB_BIT_MASK(3, 18), AdcSamplingTime, 18> ch6SamplingTime; // SPT6
  RegEnum<addr_SAMPT1, IGB_BIT_MASK(3, 21), AdcSamplingTime, 21> ch7SamplingTime; // SPT7
  RegEnum<addr_SAMPT1, IGB_BIT_MASK(3, 24), AdcSamplingTime, 24> ch8SamplingTime; // SPT8
  RegEnum<addr_SAMPT1, IGB_BIT_MASK(3, 27), AdcSamplingTime, 27> ch9SamplingTime; // SPT9

  RegValue<addr_WDHT, IGB_BIT_MASK(12, 0), 0> watchdogHighThreshold; // WDHT; High threshold for analog watchdog 
  RegValue<addr_WDLT, IGB_BIT_MASK(12, 0), 0> watchdogLowThreshold; // WDLT; Low threshold for analog watchdog 

  RegValue<addr_RSQ0, IGB_BIT_MASK(4, 20), 20> seqChLength; // this value is the actual length - 1

  RegValue<addr_RSQ0, IGB_BIT_MASK(5, 0), 0> seqOrder12Ch;
  RegValue<addr_RSQ0, IGB_BIT_MASK(5, 5), 5> seqOrder13Ch;
  RegValue<addr_RSQ0, IGB_BIT_MASK(5, 10), 10> seqOrder14Ch;
  RegValue<addr_RSQ0, IGB_BIT_MASK(5, 15), 15> seqOrder15Ch;

  RegValue<addr_RSQ1, IGB_BIT_MASK(5, 0), 0> seqOrder6Ch;
  RegValue<addr_RSQ1, IGB_BIT_MASK(5, 5), 5> seqOrder7Ch;
  RegValue<addr_RSQ1, IGB_BIT_MASK(5, 10), 10> seqOrder8Ch;
  RegValue<addr_RSQ1, IGB_BIT_MASK(5, 15), 15> seqOrder9Ch;
  RegValue<addr_RSQ1, IGB_BIT_MASK(5, 20), 20> seqOrder10Ch;
  RegValue<addr_RSQ1, IGB_BIT_MASK(5, 25), 25> seqOrder11Ch;

  RegValue<addr_RSQ2, IGB_BIT_MASK(5, 0), 0> seqOrder0Ch;
  RegValue<addr_RSQ2, IGB_BIT_MASK(5, 5), 5> seqOrder1Ch;
  RegValue<addr_RSQ2, IGB_BIT_MASK(5, 10), 10> seqOrder2Ch;
  RegValue<addr_RSQ2, IGB_BIT_MASK(5, 15), 15> seqOrder3Ch;
  RegValue<addr_RSQ2, IGB_BIT_MASK(5, 20), 20> seqOrder4Ch;
  RegValue<addr_RSQ2, IGB_BIT_MASK(5, 25), 25> seqOrder5Ch;

  RegValueRO<addr_RDATA, IGB_BIT_MASK(16, 0), 0> data; // read-only

  RegFlag<addr_OVSAMPCTL, IGB_BIT(0)> enableOverSampling; // OVSEN; Oversampling Enable
  RegEnum<addr_OVSAMPCTL, IGB_BIT_MASK(3, 2), AdcOversamplingRatio, 2> oversamplingRatio; // OVSR; Oversampling ratio
  RegValue<addr_OVSAMPCTL, IGB_BIT_MASK(4, 5), 5> oversamplingShift; // OVSS
  RegFlag<addr_OVSAMPCTL, IGB_BIT(9)> triggeredOversampling; // TOVS

  IGB_FAST_INLINE void enable() {
    enableAdc(true);
  }

  IGB_FAST_INLINE void disable() {
    enableAdc(false);
  }

  IGB_FAST_INLINE bool is(AdcStatus status) {
    volatile bool result = IGB_ADC->STAT & static_cast<uint32_t>(status);
    return result;
  }

  IGB_FAST_INLINE void clear(AdcStatus status) {
    IGB_CLEAR_BIT(IGB_ADC->STAT, static_cast<uint32_t>(status));
  }

  IGB_FAST_INLINE void enableIt(AdcInterruptType interrupt) {
    IGB_SET_BIT(IGB_ADC->CTL0, static_cast<uint32_t>(interrupt));
  }

  IGB_FAST_INLINE void disableIt(AdcInterruptType interrupt) {
    IGB_CLEAR_BIT(IGB_ADC->CTL0, static_cast<uint32_t>(interrupt));
  }

  IGB_FAST_INLINE void startCalibration() {
    resetCalibrationStart(true);
    while (resetCalibrationStart()) {}
    calibrationStart(true);
    while (calibrationStart()) {}
  }

  IGB_FAST_INLINE void startConversion() {
    softStartConversion(true);
  }

  IGB_FAST_INLINE bool checkReady() {
    return !is(AdcStatus::conversionStarted);
  }

  IGB_FAST_INLINE bool checkEndOfConversion() {
    if (is(AdcStatus::endOfConversion)) {
      clear(AdcStatus::endOfConversion);
      return true;
    }
    return false;
  }

  IGB_FAST_INLINE uint16_t readData() { return data(); }

  static IGB_FAST_INLINE void enableBusClock() {
    GD32_PERIPH_INFO.adc[to_idx(type)].bus.enableBusClock();
  }

  static IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    GpioPin pin = GpioPin::newPin(pin_type);
    pin.enable();
    pin.setMode(GpioMode::analog);
    pin.setPullMode(GpioPullMode::no);
  }

  static IGB_FAST_INLINE void prepareGpios(auto&& first, auto&&... rest) {
    prepareGpio(first.pin_type);
    prepareGpios(rest...);
  }
  static IGB_FAST_INLINE void prepareGpios() { }

  IGB_FAST_INLINE void _initCh(AdcPinConf pin_conf) {
    auto smp = pin_conf.sampling_time;
    switch (pin_conf.ch) {
      case AdcChannel::ch0:
        ch0SamplingTime(smp);
        break;
      case AdcChannel::ch1:
        ch1SamplingTime(smp);
        break;
      case AdcChannel::ch2:
        ch2SamplingTime(smp);
        break;
      case AdcChannel::ch3:
        ch3SamplingTime(smp);
        break;
      case AdcChannel::ch4:
        ch4SamplingTime(smp);
        break;
      case AdcChannel::ch5:
        ch5SamplingTime(smp);
        break;
      case AdcChannel::ch6:
        ch6SamplingTime(smp);
        break;
      case AdcChannel::ch7:
        ch7SamplingTime(smp);
        break;
      case AdcChannel::ch8:
        ch8SamplingTime(smp);
        break;
      case AdcChannel::ch9:
        ch9SamplingTime(smp);
        break;
      case AdcChannel::ch10:
        ch10SamplingTime(smp);
        break;
      case AdcChannel::ch11:
        ch11SamplingTime(smp);
        break;
      case AdcChannel::ch12:
        ch12SamplingTime(smp);
        break;
      case AdcChannel::ch13:
        ch13SamplingTime(smp);
        break;
      case AdcChannel::ch14:
        ch14SamplingTime(smp);
        break;
      case AdcChannel::ch15:
        ch15SamplingTime(smp);
        break;
      case AdcChannel::ch16:
        ch16SamplingTime(smp);
        break;
      case AdcChannel::ch17:
        ch17SamplingTime(smp);
        break;
      case AdcChannel::ch18:
        ch18SamplingTime(smp);
        break;
      default:
        break;
    }
  }

  IGB_FAST_INLINE void _initChannels(auto&& first, auto&&... rest) {
    _initCh(first);
    _initChannels(rest...);
  }
  IGB_FAST_INLINE void _initChannels() { }

  IGB_FAST_INLINE void _setSeqOrder(uint8_t idx, auto&& pin_conf) {
    uint32_t ch = static_cast<uint32_t>(pin_conf.ch);
    switch(idx) {
      case 0:
        seqOrder0Ch(ch);
        break;
      case 1:
        seqOrder1Ch(ch);
        break;
      case 2:
        seqOrder2Ch(ch);
        break;
      case 3:
        seqOrder3Ch(ch);
        break;
      case 4:
        seqOrder4Ch(ch);
        break;
      case 5:
        seqOrder5Ch(ch);
        break;
      case 6:
        seqOrder6Ch(ch);
        break;
      case 7:
        seqOrder7Ch(ch);
        break;
      case 8:
        seqOrder8Ch(ch);
        break;
      case 9:
        seqOrder9Ch(ch);
        break;
      case 10:
        seqOrder10Ch(ch);
        break;
      case 11:
        seqOrder11Ch(ch);
        break;
      case 12:
        seqOrder12Ch(ch);
        break;
      case 13:
        seqOrder13Ch(ch);
        break;
      case 14:
        seqOrder14Ch(ch);
        break;
      case 15:
        seqOrder15Ch(ch);
        break;
      default:
        break;
    }
  }

  IGB_FAST_INLINE void _setSeqOrders(uint8_t idx, auto&& first, auto&&... rest) {
    _setSeqOrder(idx, first);
    _setSeqOrders(idx+1, rest...);
  }
  IGB_FAST_INLINE void _setSeqOrders(uint8_t idx) { }

  IGB_FAST_INLINE void init(auto&& conf, auto&&... pin_confs) {
    enableBusClock();
    prepareGpios(pin_confs...);
    
    if (conf.enable_interrupt) {
      NvicCtrl::setPriority(info.irqn, conf.interrupt_priority);
      NvicCtrl::enable(info.irqn);
      IGB_MODIFY_REG(IGB_ADC->CTL0, (IGB_BIT(5) | IGB_BIT(6)), conf.interrupt_bits);
    }

    resolution(conf.resolution);
    dataAlign(conf.data_align);
    externalTrigSelect(conf.external_trigger_select);
    enableExternalTrigger(conf.enable_external_trigger);
    discontinuousConvMode(conf.discontinuous_conv_mode);
    discontinuousConvChannelCount(conf.discontinuous_conv_channel_count > 0UL ? (conf.discontinuous_conv_channel_count - 1UL) : 0UL);
    continuousConvMode(conf.continuous_conv_mode);
    dma(conf.dma);
    enableVbat(conf.vbat);
    enableTsvr(conf.tsvr);
    scanMode(conf.scanMode);
    enableRoutineChWatchDog(conf.watchDogRoutine);
    watchDogSingleCh(conf.watchDogSingle);

    delay_msec(2);

    seqChLength((sizeof...(pin_confs)) - 1UL);
    _setSeqOrders(0, pin_confs...);

    _initChannels(pin_confs...);

    startCalibration();

    delay_msec(2);

    enable();

    delay_msec(2);
  }
};

#undef IGB_ADC_REG
#undef IGB_ADC_REG_ADDR
#undef IGB_ADC

}
}

#endif /* IGB_GD32_PERIPH_ADC_H */
