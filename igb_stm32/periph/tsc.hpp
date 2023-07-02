#pragma once

#include <functional>
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

#define IGB_TSC ((TSC_TypeDef*)addr)
#define IGB_TSC_REG_ADDR(member) (addr + offsetof(TSC_TypeDef, member))
#define IGB_TSC_REG(member) ((TSC_TypeDef*)IGB_TSC_REG_ADDR(member))

enum class TscAcquisitionMode : uint8_t {
  normal       = 0,
  synchronized = 1
};

enum class TscSyncPolarity : uint8_t {
  fallingEdgeOnly = 0,
  risingEdgeAndHighLevel = 1
};

enum class TscIoDefaultMode : uint8_t {
  pushpullLow = 0,
  inputFloating = 1
};

enum class TscMaxCount : uint8_t {
  _255 = 0,
  _511,
  _1023,
  _2047,
  _4095,
  _8191,
  _16383
};

enum class TscPulsePrescaler : uint8_t {
  div1 = 0,
  div2,
  div4,
  div8,
  div16,
  div32,
  div64,
  div128,
};

enum class TscSpreadSpectrumPrescaler : uint8_t {
  div1 = 0,
  div2,
};

enum class TscGroup : uint8_t {
  g1 = 0,
  g2,
  g3,
  g4,
  g5,
  g6,
  g7,
  g8,
};

enum class TscChannel : uint8_t {
  g1i1 = 0,
  g1i2,
  g1i3,
  g1i4,
  g2i1,
  g2i2,
  g2i3,
  g2i4,
  g3i1,
  g3i2,
  g3i3,
  g3i4,
  g4i1,
  g4i2,
  g4i3,
  g4i4,
  g5i1,
  g5i2,
  g5i3,
  g5i4,
  g6i1,
  g6i2,
  g6i3,
  g6i4,
  g7i1,
  g7i2,
  g7i3,
  g7i4,
  g8i1,
  g8i2,
  g8i3,
  g8i4,
};

using TscGroupBit = BitStruct<TscGroup, uint32_t>;
using TscChannelBit = BitStruct<TscChannel, uint32_t>;

struct TscCtrl {
  constexpr static auto info = STM32_PERIPH_INFO.tsc;
  constexpr static auto addr = STM32_PERIPH_INFO.tsc.addr;
  constexpr static auto addr_CR = IGB_TSC_REG_ADDR(CR);
  constexpr static auto addr_IER = IGB_TSC_REG_ADDR(IER);
  constexpr static auto addr_ICR = IGB_TSC_REG_ADDR(IER);
  constexpr static auto addr_ISR = IGB_TSC_REG_ADDR(ISR);
  constexpr static auto addr_IOHCR = IGB_TSC_REG_ADDR(IOHCR);
  constexpr static auto addr_IOASCR = IGB_TSC_REG_ADDR(IOASCR);
  constexpr static auto addr_IOSCR = IGB_TSC_REG_ADDR(IOSCR);
  constexpr static auto addr_IOCCR = IGB_TSC_REG_ADDR(IOCCR);
  constexpr static auto addr_IOGCSR = IGB_TSC_REG_ADDR(IOGCSR);
  constexpr static auto addr_IOGXCR = IGB_TSC_REG_ADDR(IOGXCR);
  
  static RegFlag<addr_CR, TSC_CR_TSCE> enable;
  //static RegFlag<addr_CR, TSC_CR_START> start;
  static RegEnum<addr_CR, TSC_CR_AM_Msk, TscAcquisitionMode, TSC_CR_AM_Pos> acquisitionMode;
  static RegEnum<addr_CR, TSC_CR_SYNCPOL_Msk, TscSyncPolarity, TSC_CR_SYNCPOL_Pos> syncPolarity;
  static RegEnum<addr_CR, TSC_CR_IODEF_Msk, TscIoDefaultMode, TSC_CR_IODEF_Pos> ioDefaultMode;
  static RegEnum<addr_CR, TSC_CR_MCV_Msk, TscMaxCount, TSC_CR_MCV_Pos> maxCount;
  static RegEnum<addr_CR, TSC_CR_PGPSC_Msk, TscPulsePrescaler, TSC_CR_PGPSC_Pos> pulsePrescaler;
  static RegEnum<addr_CR, TSC_CR_SSPSC_Msk, TscSpreadSpectrumPrescaler, TSC_CR_SSPSC_Pos> spreadSpectrumPrescaler;
  static RegFlag<addr_CR, TSC_CR_SSE> enableSpreadSpectrum;
  static RegValue<addr_CR, TSC_CR_SSD_Msk, TSC_CR_SSD_Pos> spreadSpectrumDeviation;
  static RegValue<addr_CR, TSC_CR_CTPL_Msk, TSC_CR_CTPL_Pos> chargePulseLow;
  static RegValue<addr_CR, TSC_CR_CTPH_Msk, TSC_CR_CTPH_Pos> chargePulseHigh;

  static RegFlag<addr_IER, TSC_IER_EOAIE> enableItEndOfAcquisition;
  static RegFlag<addr_IER, TSC_IER_MCEIE> enableItMaxCountError;

  static RegFlag<addr_ICR, TSC_ICR_EOAIC> clearItEndOfAcquisition;
  static RegFlag<addr_ICR, TSC_ICR_MCEIC> clearItMaxCountError;

  static RegFlagRO<addr_ISR, TSC_ISR_EOAF> isEndOfAcquisition;
  static RegFlagRO<addr_ISR, TSC_ISR_MCEF> isMaxCountError;

  static IGB_FAST_INLINE void enableBusClock() {
    info.bus.enableBusClock();
  }

  static IGB_FAST_INLINE void start() {
    info.p_tsc->ICR = info.p_tsc->ICR | TSC_ICR_EOAIC | TSC_ICR_MCEIC; // request clearing MCEF and EOAF flags 
    info.p_tsc->CR = info.p_tsc->CR | TSC_CR_START;
  }

  static IGB_FAST_INLINE void analogSwitch(TscChannelBit bit) {
    info.p_tsc->IOASCR = bit.get();
  }

  static IGB_FAST_INLINE void samplingCap(TscChannel io) {
    info.p_tsc->IOSCR = info.p_tsc->IOSCR | (1UL << static_cast<uint32_t>(io));
  }

  static IGB_FAST_INLINE void samplingCap(TscChannelBit bit) {
    info.p_tsc->IOSCR = bit.get();
  }

  static IGB_FAST_INLINE uint32_t samplingCap() {
    return info.p_tsc->IOSCR;
  }

  static IGB_FAST_INLINE void clearSamplingCap() {
    info.p_tsc->IOSCR = 0;
  }

  static IGB_FAST_INLINE void enableHysteresisCtrl(TscChannel io) {
    info.p_tsc->IOHCR = info.p_tsc->IOHCR | (1UL << static_cast<uint32_t>(io));
  }

  static IGB_FAST_INLINE void clearHisteresisCtrl() {
    info.p_tsc->IOHCR = 0;
  }

  static IGB_FAST_INLINE void hysteresisCtrlBit(TscChannelBit bit) {
    info.p_tsc->IOHCR = bit.get();
  }

  static IGB_FAST_INLINE void enableGroup(TscGroup group) {
    info.p_tsc->IOGCSR = info.p_tsc->IOGCSR | (1UL << static_cast<uint32_t>(group));
  }

  static IGB_FAST_INLINE void disableGroup(TscGroup group) {
    info.p_tsc->IOGCSR = info.p_tsc->IOGCSR & ~(1UL << static_cast<uint32_t>(group));
  }

  static IGB_FAST_INLINE void groupCtrl(TscGroupBit bit) {
    info.p_tsc->IOGCSR = bit.get();
  }

  static IGB_FAST_INLINE void enableChannel(TscChannel ch) {
    info.p_tsc->IOCCR = info.p_tsc->IOCCR | (1UL << static_cast<uint32_t>(ch));
  }

  static IGB_FAST_INLINE void channelCtrl(TscChannelBit bit) {
    info.p_tsc->IOCCR = bit.get();
  }

  static IGB_FAST_INLINE uint16_t getValue(TscGroup group) {
    return info.p_tsc->IOGXCR[static_cast<uint8_t>(group)];
  }
};


constexpr TscGroup tsc_pin_to_group(TscChannel ch) {
  switch (ch) {
    case TscChannel::g1i1:
    case TscChannel::g1i2:
    case TscChannel::g1i3:
    case TscChannel::g1i4:
      return TscGroup::g1;
      [[fallthrough]];
    case TscChannel::g2i1:
    case TscChannel::g2i2:
    case TscChannel::g2i3:
    case TscChannel::g2i4:
      return TscGroup::g2;
      [[fallthrough]];
    case TscChannel::g3i1:
    case TscChannel::g3i2:
    case TscChannel::g3i3:
    case TscChannel::g3i4:
      return TscGroup::g3;
      [[fallthrough]];
    case TscChannel::g4i1:
    case TscChannel::g4i2:
    case TscChannel::g4i3:
    case TscChannel::g4i4:
      return TscGroup::g4;
      [[fallthrough]];
    case TscChannel::g5i1:
    case TscChannel::g5i2:
    case TscChannel::g5i3:
    case TscChannel::g5i4:
      return TscGroup::g5;
      [[fallthrough]];
    case TscChannel::g6i1:
    case TscChannel::g6i2:
    case TscChannel::g6i3:
    case TscChannel::g6i4:
      return TscGroup::g6;
      [[fallthrough]];
    case TscChannel::g7i1:
    case TscChannel::g7i2:
    case TscChannel::g7i3:
    case TscChannel::g7i4:
      return TscGroup::g7;
      [[fallthrough]];
    case TscChannel::g8i1:
    case TscChannel::g8i2:
    case TscChannel::g8i3:
    case TscChannel::g8i4:
      return TscGroup::g8;
      [[fallthrough]];
    default:
      break;
  }
  return TscGroup::g1;
}

enum class TscIoType {
  none = 0,
  input,
  samplingCap
};

struct TscPinConf {
  TscChannel channel;
  GpioPinType pin_type;
  TscIoType io_type;
  bool enable_hysteresis = false;
  bool enable_analog_switch = false;
};

enum class TscProcessState : uint8_t {
  ready = 0,
  start
};

enum class TscTouchState : uint8_t {
  inactive = 0,
  released,
  pressed
};

struct TscChannelState {
  float threshold = 900.0f; // touch / untouch threshold value
  float base_value = 1000.0f; // value when not touched
  float value = 1000.0f; // current value
  TscTouchState state;
};

struct TscConf {
  TscMaxCount max_count = TscMaxCount::_16383;
  TscPulsePrescaler pulse_prescaler = TscPulsePrescaler::div4;
  uint8_t charge_pulse_low = 1;
  uint8_t charge_pulse_high = 1;
  bool enable_spread_spectrum = false;
  uint8_t spread_spectrum_deviation = 0;
  TscSpreadSpectrumPrescaler spread_spectrum_prescaler = TscSpreadSpectrumPrescaler::div1;
  TscIoDefaultMode io_default_mode = TscIoDefaultMode::pushpullLow;
  TscSyncPolarity sync_polarity = TscSyncPolarity::fallingEdgeOnly;
  TscAcquisitionMode acquisition_mode = TscAcquisitionMode::normal;
  
  bool enable_it_end_of_acquisition = false;
  bool enable_it_max_count_error = false;
  uint8_t interrupt_priority = 1;

  float value_filter_coeff = 0.1f;
};

struct Tsc {
  struct Channel {
    const TscChannelState& channel;

    IGB_FAST_INLINE bool read() const noexcept {
      return channel.state == TscTouchState::pressed;
    }

    IGB_FAST_INLINE float getBaseValue() const noexcept {
      return channel.base_value;
    }

    IGB_FAST_INLINE float getValue() const noexcept {
      return channel.value;
    }

    IGB_FAST_INLINE float getThreshold() const noexcept {
      return channel.threshold;
    }
  };

  static const uint16_t MAX_COUNT = 16383;
  static const uint8_t CHANNEL_MAX = 32;
  static const uint8_t CHANNEL_SIZE_OF_GROUP = 4;
  static const uint8_t GROUP_MAX = 8;
  constexpr static const uint32_t CH_OF_GRP_MASK[8] = {
    (0b0001UL << 0) | (0b0001UL << 4) | (0b0001UL << 8) | (0b0001UL << 12) | (0b0001UL << 16) | (0b0001UL << 20) | (0b0001UL << 24) | (0b0001UL << 28),
    (0b0010UL << 0) | (0b0010UL << 4) | (0b0010UL << 8) | (0b0010UL << 12) | (0b0010UL << 16) | (0b0010UL << 20) | (0b0010UL << 24) | (0b0010UL << 28),
    (0b0100UL << 0) | (0b0100UL << 4) | (0b0100UL << 8) | (0b0100UL << 12) | (0b0100UL << 16) | (0b0100UL << 20) | (0b0100UL << 24) | (0b0100UL << 28),
    (0b1000UL << 0) | (0b1000UL << 4) | (0b1000UL << 8) | (0b1000UL << 12) | (0b1000UL << 16) | (0b1000UL << 20) | (0b1000UL << 24) | (0b1000UL << 28),
  };

  TscGroupBit group_bit;
  TscChannelBit channel_bit;
  TscChannelBit sampling_cap_bit;
  TscChannelBit input_ch_bit;
  TscChannelBit hysteresis_bit;
  TscChannelBit analog_sw_bit;
  TscChannelState channels[CHANNEL_MAX];
  uint32_t touch_bit;
  std::function<void(uint32_t)> on_acquisition_end;
  std::function<void()> on_error;

  TscProcessState _state = TscProcessState::ready;
  uint8_t _process_ch_of_grp_idx = 0;
  uint32_t _touch_bits = 0;
  float _value_filter_coeff = 0.f;

  // auto = TscPinConf
  static IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type, TscIoType io_type) {
    auto result = get_af_idx(PeriphType::tsc, pin_type);
    if (!(result.has_value())) { return; }
    if (io_type == TscIoType::none) { return; }

    GpioPin pin = GpioPin::newPin(pin_type);
    pin.enable();
    pin.setMode(GpioMode::alternate);
    pin.setPullMode(GpioPullMode::no);
    pin.setSpeedMode(GpioSpeedMode::medium);
    if (io_type == TscIoType::input) {
      pin.setOutputMode(GpioOutputMode::pushpull);
    } else { // sampling cap
      pin.setOutputMode(GpioOutputMode::opendrain);
    }
    pin.setAlternateFunc(result.value());
  }

  static IGB_FAST_INLINE void prepareGpios(auto&& first, auto&&... rest) {
    prepareGpio(first.pin_type, first.io_type);
    prepareGpios(rest...);
  }
  static IGB_FAST_INLINE void prepareGpios() { }

  static IGB_FAST_INLINE TscGroupBit constructGroupBit(auto&& first, auto&&... rest) {
    return TscGroupBit(tsc_pin_to_group(first.channel)) | constructGroupBit(rest...);
  }
  static IGB_FAST_INLINE TscGroupBit constructGroupBit() { return TscGroupBit {}; }

  static IGB_FAST_INLINE TscChannelBit constructChannelBit(auto&& first, auto&&... rest) {
    return TscChannelBit(first.channel) | constructChannelBit(rest...);
  }
  static IGB_FAST_INLINE TscChannelBit constructChannelBit() { return TscChannelBit {}; }

  static IGB_FAST_INLINE TscChannelBit constructSamplingCapBit(auto&& first, auto&&... rest) {
    return (first.io_type == TscIoType::samplingCap ? TscChannelBit(first.channel) : (TscChannelBit {})) | constructSamplingCapBit(rest...);
  }
  static IGB_FAST_INLINE TscChannelBit constructSamplingCapBit() { return TscChannelBit {}; }

  static IGB_FAST_INLINE TscChannelBit constructInputChBit(auto&& first, auto&&... rest) {
    return (first.io_type == TscIoType::samplingCap ? (TscChannelBit {}) : TscChannelBit(first.channel)) | constructInputChBit(rest...);
  }
  static IGB_FAST_INLINE TscChannelBit constructInputChBit() { return TscChannelBit {}; }

  static IGB_FAST_INLINE TscChannelBit constructHysteresisBit(auto&& first, auto&&... rest) {
    return (first.enable_hysteresis ? TscChannelBit(first.channel) : (TscChannelBit {})) | constructHysteresisBit(rest...);
  }
  static IGB_FAST_INLINE TscChannelBit constructHysteresisBit() { return TscChannelBit {}; }

  static IGB_FAST_INLINE TscChannelBit constructAnalogSwitchBit(auto&& first, auto&&... rest) {
    return (first.enable_analog_switch ? TscChannelBit(first.channel) : (TscChannelBit {})) | constructAnalogSwitchBit(rest...);
  }
  static IGB_FAST_INLINE TscChannelBit constructAnalogSwitchBit() { return TscChannelBit {}; }

  IGB_FAST_INLINE void setThreshold(TscChannel channel, float threshold) {
    channels[static_cast<uint8_t>(channel)].threshold = threshold;
  }

  IGB_FAST_INLINE uint16_t getValue(TscChannel channel) {
    return channels[static_cast<uint8_t>(channel)].value;
  }

  IGB_FAST_INLINE void initConfig(auto&& tsc_conf /* TscConf */) {
    TscCtrl::enableBusClock();

    (
       TscCtrl::maxCount.val(tsc_conf.max_count) |
       TscCtrl::pulsePrescaler.val(tsc_conf.pulse_prescaler) |
       TscCtrl::chargePulseLow.val(tsc_conf.charge_pulse_low) |
       TscCtrl::chargePulseHigh.val(tsc_conf.charge_pulse_high) |
       TscCtrl::acquisitionMode.val(tsc_conf.acquisition_mode) |
       TscCtrl::syncPolarity.val(tsc_conf.sync_polarity) |
       TscCtrl::ioDefaultMode.val(tsc_conf.io_default_mode) |
       TscCtrl::enableSpreadSpectrum.val(tsc_conf.enable_spread_spectrum) |
       TscCtrl::spreadSpectrumPrescaler.val(tsc_conf.spread_spectrum_prescaler) |
       TscCtrl::spreadSpectrumDeviation.val(tsc_conf.spread_spectrum_deviation)
    ).update();

    if (tsc_conf.enable_it_end_of_acquisition || tsc_conf.enable_it_max_count_error) {
      NvicCtrl::setPriority(TscCtrl::info.irqn, tsc_conf.interrupt_priority);
      NvicCtrl::enable(TscCtrl::info.irqn);
    }

    (
      TscCtrl::enableItEndOfAcquisition.val(tsc_conf.enable_it_end_of_acquisition) |
      TscCtrl::enableItMaxCountError.val(tsc_conf.enable_it_max_count_error)
    ).update();

    TscCtrl::enable(true);
    TscCtrl::samplingCap(sampling_cap_bit);
    TscCtrl::hysteresisCtrlBit(hysteresis_bit);
    TscCtrl::analogSwitch(analog_sw_bit);
    TscCtrl::groupCtrl(group_bit);
    
    for (uint8_t i = 0; i < CHANNEL_MAX; ++i) {
      if (input_ch_bit & (1UL << i)) {
        channels[i].state = TscTouchState::released;
      } else {
        channels[i].state = TscTouchState::inactive;
      }
    }

    _value_filter_coeff = tsc_conf.value_filter_coeff;
  }

  IGB_FAST_INLINE void initConfigDefault() {
    TscCtrl::enableBusClock();
    TscCtrl::maxCount(TscMaxCount::_16383);
    TscCtrl::pulsePrescaler(TscPulsePrescaler::div4);
    TscCtrl::chargePulseLow(1);
    TscCtrl::chargePulseHigh(1);
    TscCtrl::enable(true);
    TscCtrl::samplingCap(sampling_cap_bit);
    TscCtrl::clearHisteresisCtrl();
    TscCtrl::groupCtrl(group_bit);
    
    for (uint8_t i = 0; i < CHANNEL_MAX; ++i) {
      if (input_ch_bit & (1UL << i)) {
        channels[i].state = TscTouchState::released;
      } else {
        channels[i].state = TscTouchState::inactive;
      }
    }
  }

  IGB_FAST_INLINE void init(auto&& tsc_conf /* TscConf */, auto&&... pin_confs /* TscPinConf */) {
    prepareGpios(pin_confs...);
    group_bit = constructGroupBit(pin_confs...);
    channel_bit = constructChannelBit(pin_confs...);
    sampling_cap_bit = constructSamplingCapBit(pin_confs...);
    input_ch_bit = constructInputChBit(pin_confs...);
    hysteresis_bit = constructHysteresisBit(pin_confs...);
    analog_sw_bit = constructAnalogSwitchBit(pin_confs...);
    initConfig(tsc_conf);
    _state = TscProcessState::ready;
    _process_ch_of_grp_idx = 0;
    _touch_bits = 0;
  }

  // deprecated:
  IGB_FAST_INLINE void initDefault(auto&&... confs) {
    prepareGpios(confs...);
    group_bit = constructGroupBit(confs...);
    channel_bit = constructChannelBit(confs...);
    sampling_cap_bit = constructSamplingCapBit(confs...);
    input_ch_bit = constructInputChBit(confs...);
    hysteresis_bit = constructHysteresisBit(confs...);
    analog_sw_bit = constructAnalogSwitchBit(confs...);
    initConfigDefault();
    _state = TscProcessState::ready;
    _process_ch_of_grp_idx = 0;
    _touch_bits = 0;
  }

  IGB_FAST_INLINE void startAcquisition() {
    TscCtrl::channelCtrl(input_ch_bit & CH_OF_GRP_MASK[_process_ch_of_grp_idx]);
    TscCtrl::start();
  }

  IGB_FAST_INLINE void execAcquisition() {
    for (uint8_t i = _process_ch_of_grp_idx; i < CHANNEL_MAX; i += CHANNEL_SIZE_OF_GROUP) {
      TscChannelState& ch = channels[i];
      if (ch.state != TscTouchState::inactive) {
        uint16_t v = TscCtrl::getValue(static_cast<TscGroup>(i / CHANNEL_SIZE_OF_GROUP));
        if (v == MAX_COUNT) {
          continue;
        }
        float new_value = (float)v;
        ch.value = (ch.value * (1.0f - _value_filter_coeff)) + (new_value * _value_filter_coeff);
      }
    }
  }

  IGB_FAST_INLINE uint8_t _calcNextIdx(uint8_t base_idx) {
    while (!(input_ch_bit & CH_OF_GRP_MASK[base_idx]) && base_idx < CHANNEL_SIZE_OF_GROUP) { ++base_idx; }
    return base_idx;
  }

  IGB_FAST_INLINE bool isStart() {
    return _state == TscProcessState::start;
  }

  IGB_FAST_INLINE bool isReady() {
    return _state == TscProcessState::ready;
  }

  IGB_FAST_INLINE void start() {
    startAcquisition();
    _state = TscProcessState::start;
  }

  IGB_FAST_INLINE void updateTouchBits() {
    for (uint8_t i = 0; i < CHANNEL_MAX; ++i) {
      if (input_ch_bit & (1UL << i)) {
        TscChannelState& ch = channels[i];
        if (ch.value < ch.threshold) {
          if (ch.state == TscTouchState::released) {
            ch.state = TscTouchState::pressed;
            _touch_bits |= (1UL << i);
          }
        } else {
          if (ch.state == TscTouchState::pressed) {
            ch.state = TscTouchState::released;
            _touch_bits &= ~(1UL << i);
          }
        }
      }
    }
  }

  // return true if acuisition end
  IGB_FAST_INLINE bool processAcquisition() {
    bool retval = false;
    if (TscCtrl::isEndOfAcquisition()) {
      execAcquisition();
      uint8_t next_idx = _calcNextIdx(_process_ch_of_grp_idx + 1);
      if (next_idx >= CHANNEL_SIZE_OF_GROUP) {
        retval = true;
        if (on_acquisition_end) {
          updateTouchBits();
          on_acquisition_end(_touch_bits);
        }
        next_idx = _calcNextIdx(0);
      }
      _process_ch_of_grp_idx = next_idx;
      _state = TscProcessState::ready;
    }
    return retval;
  }

  IGB_FAST_INLINE bool process() {
    if (!input_ch_bit) { return false; }
    if (isReady()) {
      start();
    } else {
      return processAcquisition();
    }
    return false;
  }

  IGB_FAST_INLINE void irqHandler() {
    if (TscCtrl::isEndOfAcquisition()) {
      processAcquisition();
      if (isReady()) {
        start();
      }
      TscCtrl::clearItEndOfAcquisition(true);
    }
    if (TscCtrl::isMaxCountError()) {
      if (on_error) {
        on_error();
      }
      TscCtrl::clearItMaxCountError(true);
    }
  }

  IGB_FAST_INLINE Channel newChannel(TscChannel ch) {
    return Channel {
      .channel = channels[static_cast<uint8_t>(ch)]
    };
  }
};

#undef IGB_TSC_REG
#undef IGB_TSC_REG_ADDR
#undef IGB_TSC

}
}

