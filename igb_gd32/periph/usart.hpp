#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

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

#define IGB_USART ((USART_TypeDef*)addr)
#define IGB_USART_REG_ADDR(member) (addr + offsetof(USART_TypeDef, member))
#define IGB_USART_REG(member) ((USART_TypeDef*)IGB_USART_REG_ADDR(member))

enum class UsartState : uint32_t {
  parityError = IGB_BIT(0),
  frameError = IGB_BIT(1),
  noiseDetected = IGB_BIT(2),
  overrunError = IGB_BIT(3),
  idleLineDetected = IGB_BIT(4),
  rxNotEmpty = IGB_BIT(5),
  txComplete = IGB_BIT(6),
  txEmpty = IGB_BIT(7),
  linBreakDetected = IGB_BIT(8),
  ctsInterrupt = IGB_BIT(9),
  cts = IGB_BIT(10),
  rxTimeout = IGB_BIT(11),
  endOfBlock = IGB_BIT(12),
  busy = IGB_BIT(16),
  characterMatch = IGB_BIT(17),
  breakCharacterRequested = IGB_BIT(18),
  rxMuted = IGB_BIT(19),
  wakeup = IGB_BIT(20),
  txEnableAck = IGB_BIT(21),
  rxEnableAck = IGB_BIT(22)
};


enum class UsartParity : uint32_t {
  even = 0,
  odd = 1
};

enum class UsartWakeupType : uint32_t {
  idleLine = 0,
  addressMark = 1
};

enum class UsartOverSampling : uint32_t {
  x16 = 0,
  x8 = 1
};

enum class UsartAddressSize : uint32_t {
  _4bit = 0,
  _7bit = 1
};

enum class UsartLinBreakSize : uint32_t {
  _10bit = 0,
  _11bit = 1
};

enum class UsartClockPhase : uint32_t {
  firstClock = 0,
  secondClock = 1
};

enum class UsartClockPolarity : uint32_t {
  low = 0,
  hight = 1
};

enum class UsartStopBit : uint32_t {
  one = 0,
  half,
  two,
  oneHalf
};

enum class UsartBitOrder : uint32_t {
  lsbFirst = 0,
  msbFirst
};

enum class UsartSampleBitMode : uint32_t {
  threeSampleBit = 0,
  oneSampleBit
};

enum class UsartDriverEnablePolarity : uint32_t {
  high = 0,
  low
};

enum class UsartWakeupItFlagType : uint32_t {
  wakeupOnAddress = 0,
  wakeupOnStartbit = 2,
  wakeupOnRxne = 3
};

enum class UsartDataWidth : uint32_t {
  _8bit = 0,
  _9bit = 1
};

struct UsartConf {
  std::optional<GpioPinType> rx_pin_type = std::nullopt; // only use enableRx = true
  std::optional<GpioPinType> tx_pin_type = std::nullopt; // only use enableTx = true
  std::optional<GpioPinType> ck_pin_type = std::nullopt; // only use enableClock = true

  bool enable_wakeup = false;
  bool enable_rx = false;
  bool enable_tx = false;

  bool enable_parity = false;
  UsartParity parity_select = UsartParity::even;
  UsartWakeupType wakeup_type = UsartWakeupType::idleLine;
  UsartDataWidth data_width = UsartDataWidth::_8bit;
  bool mute_mode = false;
  UsartOverSampling over_sampling = UsartOverSampling::x16;
  uint8_t driver_enable_de_assertion_time;
  uint8_t driver_enable_assertion_time;

  UsartAddressSize address_size = UsartAddressSize::_4bit;
  UsartLinBreakSize lin_break_size = UsartLinBreakSize::_10bit;
  bool output_lb_clock = false;
  UsartClockPhase clock_phase = UsartClockPhase::firstClock;
  UsartClockPolarity clock_polarity = UsartClockPolarity::low;
  bool enable_clock = false;
  UsartStopBit stop_bit = UsartStopBit::one;
  bool enable_lin = false;
  bool swap_tx_rx = false;
  bool invert_rx_polarity = false;
  bool invert_tx_polarity = false;
  bool invert_data_polarity = false;
  UsartBitOrder bit_order = UsartBitOrder::lsbFirst;
  bool enable_rx_timeout = false;
  uint8_t address = 0;

  bool enable_irda = false;
  bool lowpower_irda = false;
  bool enable_half_duplex = false;
  bool enable_smartcard_nack = false;
  bool enable_smartcard_mode = false;
  bool dma_rx = false;
  bool dma_tx = false;
  bool enable_rts = false;
  bool enable_cts = false;
  UsartSampleBitMode sample_bit_mode = UsartSampleBitMode::threeSampleBit;
  bool enable_overrun = true;
  bool dma_deact_on_rx_error = false;
  bool driver_enable_mode = false;
  UsartDriverEnablePolarity driver_enable_polarity = UsartDriverEnablePolarity::high;
  uint8_t smartcard_auto_retry_count = 0;
  UsartWakeupItFlagType wakeup_it_flag_type = UsartWakeupItFlagType::wakeupOnAddress;

  //uint16_t baudrate = 0;
  //uint16_t brr = 0;
  uint32_t base_clock_freq = 8000000UL;
  uint32_t baudrate = 31250UL;
  uint8_t irda_prescaler = 0;
  uint8_t smartcard_guard_time = 0;

  uint32_t rx_timeout = 0;
  uint8_t block_length = 0;

  bool request_break_sending = false;
  bool request_enter_mute_mode = false;
  bool request_rx_data_flush = false;
  bool request_tx_data_flush = false;

  bool enable_it_idle = false;
  bool enable_it_rx_not_empty = false;
  bool enable_it_tx_complete = false;
  bool enable_it_tx_empty = false;
  bool enable_it_parity_error = false;
  bool enable_it_character_match = false;
  bool enable_it_rx_timeout = false;
  bool enable_it_end_of_block = false;
  bool enable_it_line_break = false;
  bool enable_it_error = false;
  bool enable_it_cts = false;
  bool enable_it_wakeup = false;

  uint8_t interrupt_priority = 1;
};

template<UsartType USART_TYPE>
struct Usart {
  constexpr static auto type = USART_TYPE;
  constexpr static auto info = GD32_PERIPH_INFO.usart[to_idx(type)];
  constexpr static auto addr = GD32_PERIPH_INFO.usart[to_idx(type)].addr;
  constexpr static auto addr_CTL0 = IGB_USART_REG_ADDR(CTL0);
  constexpr static auto addr_CTL1 = IGB_USART_REG_ADDR(CTL1);
  constexpr static auto addr_CTL2 = IGB_USART_REG_ADDR(CTL2);
  constexpr static auto addr_BAUD = IGB_USART_REG_ADDR(BAUD);
  constexpr static auto addr_GP = IGB_USART_REG_ADDR(GP);
  constexpr static auto addr_RT = IGB_USART_REG_ADDR(RT);
  constexpr static auto addr_CMD = IGB_USART_REG_ADDR(CMD);
  constexpr static auto addr_STAT = IGB_USART_REG_ADDR(STAT);
  constexpr static auto addr_INTC = IGB_USART_REG_ADDR(INTC);
  constexpr static auto addr_RDATA = IGB_USART_REG_ADDR(RDATA);
  constexpr static auto addr_TDATA = IGB_USART_REG_ADDR(TDATA);

  RegFlag<addr_CTL0, IGB_BIT(0)> enable;
  RegFlag<addr_CTL0, IGB_BIT(1)> enableWakeup;
  RegFlag<addr_CTL0, IGB_BIT(2)> enableRx;
  RegFlag<addr_CTL0, IGB_BIT(3)> enableTx;

  RegFlag<addr_CTL0, IGB_BIT(4)> enableItIdle;
  RegFlag<addr_CTL0, IGB_BIT(5)> enableItRxNotEmpty;
  RegFlag<addr_CTL0, IGB_BIT(6)> enableItTxComplete;
  RegFlag<addr_CTL0, IGB_BIT(7)> enableItTxEmpty;
  RegFlag<addr_CTL0, IGB_BIT(8)> enableItParityError;
  RegFlag<addr_CTL0, IGB_BIT(14)> enableItCharacterMatch;
  RegFlag<addr_CTL0, IGB_BIT(26)> enableItRxTimeout;
  RegFlag<addr_CTL0, IGB_BIT(27)> enableItEndOfBlock;
  RegFlag<addr_CTL0, IGB_BIT(10)> enableParity;
  RegEnum<addr_CTL0, IGB_BIT(9), UsartParity, 9> paritySelect;
  RegEnum<addr_CTL0, IGB_BIT(11), UsartWakeupType, 11> wakeupType;

  RegEnum<addr_CTL0, IGB_BIT(12), UsartDataWidth, 12> dataWidth;
  RegFlag<addr_CTL0, IGB_BIT(13)> muteMode;
  RegEnum<addr_CTL0, IGB_BIT(15), UsartOverSampling, 15> overSampling;
  RegValue<addr_CTL0, IGB_BIT_MASK(5, 16), 16> driverEnableDeAssertionTime;
  RegValue<addr_CTL0, IGB_BIT_MASK(5, 21), 21> driverEnableAssertionTime;

  RegEnum<addr_CTL1, IGB_BIT(4), UsartAddressSize, 4> addressSize;
  RegEnum<addr_CTL1, IGB_BIT(5), UsartLinBreakSize, 5> linBreakSize;
  RegFlag<addr_CTL1, IGB_BIT(8)> outputLbClock;
  RegEnum<addr_CTL1, IGB_BIT(9), UsartClockPhase, 9> clockPhase;
  RegEnum<addr_CTL1, IGB_BIT(10), UsartClockPolarity, 10> clockPolarity;
  RegFlag<addr_CTL1, IGB_BIT(11)> enableClock;
  RegEnum<addr_CTL1, IGB_BIT_MASK(2, 12), UsartStopBit, 12> stopBit;
  RegFlag<addr_CTL1, IGB_BIT(14)> enableLin;
  RegFlag<addr_CTL1, IGB_BIT(15)> swapTxRx;
  RegFlag<addr_CTL1, IGB_BIT(16)> invertRxPolarity;
  RegFlag<addr_CTL1, IGB_BIT(17)> invertTxPolarity;
  RegFlag<addr_CTL1, IGB_BIT(18)> invertDataPolarity;
  RegEnum<addr_CTL1, IGB_BIT(19), UsartBitOrder, 19> bitOrder;
  RegFlag<addr_CTL1, IGB_BIT(23)> enableRxTimeout;
  RegValue<addr_CTL1, IGB_BIT_MASK(8, 24), 24> address;
  RegFlag<addr_CTL1, IGB_BIT(6)> enableItLineBreak;

  RegFlag<addr_CTL2, IGB_BIT(1)> enableIrDA;
  RegFlag<addr_CTL2, IGB_BIT(2)> lowpowerIrDA;
  RegFlag<addr_CTL2, IGB_BIT(3)> enableHalfDuplex;
  RegFlag<addr_CTL2, IGB_BIT(4)> enableSmartcardNack;
  RegFlag<addr_CTL2, IGB_BIT(5)> enableSmartcardMode;
  RegFlag<addr_CTL2, IGB_BIT(6)> dmaRx;
  RegFlag<addr_CTL2, IGB_BIT(7)> dmaTx;
  RegFlag<addr_CTL2, IGB_BIT(8)> enableRts;
  RegFlag<addr_CTL2, IGB_BIT(9)> enableCts;
  RegEnum<addr_CTL2, IGB_BIT(11), UsartSampleBitMode, 11> sampleBitMode;
  RegFlag<addr_CTL2, IGB_BIT(12), false> enableOverrun;
  RegFlag<addr_CTL2, IGB_BIT(13)> dmaDeactOnRxError;
  RegFlag<addr_CTL2, IGB_BIT(14)> driverEnableMode;
  RegEnum<addr_CTL2, IGB_BIT(15), UsartDriverEnablePolarity, 15> driverEnablePolarity;
  RegValue<addr_CTL2, IGB_BIT_MASK(3, 17), 17> smartcardAutoRetryCount;
  RegEnum<addr_CTL2, IGB_BIT_MASK(2, 20), UsartWakeupItFlagType, 20> wakeupItFlagType;
  RegFlag<addr_CTL2, IGB_BIT(0)> enableItError;
  RegFlag<addr_CTL2, IGB_BIT(10)> enableItCts;
  RegFlag<addr_CTL2, IGB_BIT(22)> enableItWakeup;

  Reg<addr_BAUD> reg_BAUD;

  IGB_FAST_INLINE void setBaudrate(uint32_t base_freq, uint32_t baudrate, UsartOverSampling over_sampling) {
    switch (over_sampling) {
      case UsartOverSampling::x16:
        reg_BAUD((uint16_t)((base_freq + (baudrate / 2)) / baudrate));
        break;
      case UsartOverSampling::x8:
        {
          const uint16_t div = ((base_freq * 2) + (baudrate / 2)) / baudrate;
          reg_BAUD((div & 0xFFF0) | ((div & 0x000F) >> 1));
        }
        break;
      default:
        break;
    }
  }

  RegValue<addr_GP, IGB_BIT_MASK(8, 0), 0> irdaPrescaler;
  RegValue<addr_GP, IGB_BIT_MASK(8, 8), 8> smartcardGuardTime;

  RegValue<addr_RT, IGB_BIT_MASK(24, 0), 0> rxTimeout;
  RegValue<addr_RT, IGB_BIT_MASK(8, 24), 24> blockLength;

  RegFlag<addr_CMD, IGB_BIT(1)> requestBreakSending;
  RegFlag<addr_CMD, IGB_BIT(2)> requestEnterMuteMode;
  RegFlag<addr_CMD, IGB_BIT(3)> requestRxDataFlush;
  RegFlag<addr_CMD, IGB_BIT(4)> requestTxDataFlush;

  IGB_FAST_INLINE static bool is(UsartState state) {
    return IGB_USART->STAT & static_cast<uint32_t>(state);
  }

  IGB_FAST_INLINE static void clear(UsartState state) {
    IGB_USART->INTC = IGB_USART->INTC | static_cast<uint32_t>(state);
  }

  Reg<addr_RDATA> rxData;
  Reg<addr_TDATA> txData;

  IGB_FAST_INLINE void enableBusClock() {
    GD32_PERIPH_INFO.usart[to_idx(type)].bus.enableBusClock();
  }

  IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    auto periph_type = as_periph_type(type);
    if (!periph_type) { return; }
    auto result = get_af_idx(periph_type.value(), pin_type);
    if (!result) { return; }

    GpioPin pin = GpioPin::newPin(pin_type);

    pin.enable();
    pin.setMode(GpioMode::alternate);
    pin.setPullMode(GpioPullMode::no);
    pin.setSpeedMode(GpioSpeedMode::high);
    pin.setOutputMode(GpioOutputMode::pushpull);
    pin.setAlternateFunc(result.value());
  }

  IGB_FAST_INLINE void init(auto&& conf) {
    enableBusClock();
    enable(false);

    if (conf.enable_tx && conf.tx_pin_type) {
      prepareGpio(conf.tx_pin_type.value());
    }
    if (conf.enable_rx && conf.rx_pin_type) {
      prepareGpio(conf.rx_pin_type.value());
    }
    if (conf.enable_clock && conf.ck_pin_type) {
      prepareGpio(conf.ck_pin_type.value());
    }

    if (
      conf.enable_it_idle || 
      conf.enable_it_rx_not_empty ||
      conf.enable_it_tx_complete ||
      conf.enable_it_tx_empty ||
      conf.enable_it_parity_error ||
      conf.enable_it_character_match ||
      conf.enable_it_rx_timeout ||
      conf.enable_it_end_of_block ||
      conf.enable_it_line_break ||
      conf.enable_it_error ||
      conf.enable_it_cts ||
      conf.enable_it_wakeup
    ) {
      NvicCtrl::setPriority(info.irqn, conf.interrupt_priority);
      NvicCtrl::enable(info.irqn);
    }

    setBaudrate(conf.base_clock_freq, conf.baudrate, conf.over_sampling);

    (
     enableWakeup.val(conf.enable_wakeup) |
     enableRx.val(conf.enable_rx) |
     enableTx.val(conf.enable_tx) |
     enableItIdle.val(conf.enable_it_idle) |
     enableItRxNotEmpty.val(conf.enable_it_rx_not_empty) |
     enableItTxComplete.val(conf.enable_it_tx_complete) |
     enableItTxEmpty.val(conf.enable_it_tx_empty) |
     enableItParityError.val(conf.enable_it_parity_error) |
     enableItCharacterMatch.val(conf.enable_it_character_match) |
     enableItRxTimeout.val(conf.enable_it_rx_timeout) |
     enableItEndOfBlock.val(conf.enable_it_end_of_block) |
     enableParity.val(conf.enable_parity) |
     paritySelect.val(conf.parity_select) |
     wakeupType.val(conf.wakeup_type) |
     dataWidth.val(conf.data_width) |
     muteMode.val(conf.mute_mode) |
     overSampling.val(conf.over_sampling) |
     driverEnableDeAssertionTime.val(conf.driver_enable_de_assertion_time) |
     driverEnableAssertionTime.val(conf.driver_enable_assertion_time)
    ).update();

    (
     addressSize.val(conf.address_size) |
     linBreakSize.val(conf.lin_break_size) |
     outputLbClock.val(conf.output_lb_clock) |
     clockPhase.val(conf.clock_phase) |
     clockPolarity.val(conf.clock_polarity) |
     enableClock.val(conf.enable_clock) |
     stopBit.val(conf.stop_bit) |
     enableLin.val(conf.enable_lin) |
     swapTxRx.val(conf.swap_tx_rx) |
     invertRxPolarity.val(conf.invert_rx_polarity) |
     invertTxPolarity.val(conf.invert_tx_polarity) |
     invertDataPolarity.val(conf.invert_data_polarity) |
     bitOrder.val(conf.bit_order) |
     enableRxTimeout.val(conf.enable_rx_timeout) |
     address.val(conf.address) |
     enableItLineBreak.val(conf.enable_it_line_break)
    ).update();

    (
     enableIrDA.val(conf.enable_irda) |
     lowpowerIrDA.val(conf.lowpower_irda) |
     enableHalfDuplex.val(conf.enable_half_duplex) |
     enableSmartcardNack.val(conf.enable_smartcard_nack) |
     enableSmartcardMode.val(conf.enable_smartcard_mode) |
     dmaRx.val(conf.dma_rx) |
     dmaTx.val(conf.dma_tx) |
     enableRts.val(conf.enable_rts) |
     enableCts.val(conf.enable_cts) |
     sampleBitMode.val(conf.sample_bit_mode) |
     enableOverrun.val(conf.enable_overrun) |
     dmaDeactOnRxError.val(conf.dma_deact_on_rx_error) |
     driverEnableMode.val(conf.driver_enable_mode) |
     driverEnablePolarity.val(conf.driver_enable_polarity) |
     smartcardAutoRetryCount.val(conf.smartcard_auto_retry_count) |
     wakeupItFlagType.val(conf.wakeup_it_flag_type) |
     enableItError.val(conf.enable_it_error) |
     enableItCts.val(conf.enable_it_cts) |
     enableItWakeup.val(conf.enable_it_wakeup)
    ).update();

    (
     irdaPrescaler.val(conf.irda_prescaler) | 
     irdaPrescaler.val(conf.smartcard_guard_time)
    ).update();

    (
     rxTimeout.val(conf.rx_timeout) |
     blockLength.val(conf.block_length)
    ).update();

    (
     requestBreakSending.val(conf.request_break_sending) |
     requestEnterMuteMode.val(conf.request_enter_mute_mode) |
     requestRxDataFlush.val(conf.request_rx_data_flush) |
     requestTxDataFlush.val(conf.request_tx_data_flush)
    ).update();

    enable(true);
  }
};

#undef IGB_USART_REG
#undef IGB_USART_REG_ADDR
#undef IGB_USART

}
}

