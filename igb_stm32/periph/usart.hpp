#ifndef IGB_STM32_PERIPH_USART_H
#define IGB_STM32_PERIPH_USART_H

#include <stddef.h>
#include <optional>

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

#define IGB_USART ((USART_TypeDef*)addr)
#define IGB_USART_REG_ADDR(member) (addr + offsetof(USART_TypeDef, member))
#define IGB_USART_REG(member) ((USART_TypeDef*)IGB_USART_REG_ADDR(member))

// TODO: other series
#if defined(STM32F3)

enum class UsartInterruptType1 : uint32_t {
  idle = USART_CR1_IDLEIE,
  rxne = USART_CR1_RXNEIE,
  txComplete = USART_CR1_TCIE,
  txe = USART_CR1_TXEIE,
  pe = USART_CR1_PEIE,
  characterMatch = USART_CR1_CMIE,
  receiverTimeout = USART_CR1_RTOIE,
  endOfBlock = USART_CR1_EOBIE
};

enum class UsartState : uint32_t {
  parityError = USART_ISR_PE,
  frameError = USART_ISR_FE,
  noiseDetected = USART_ISR_NE,
  overrunError = USART_ISR_ORE,
  idleLineDetected = USART_ISR_IDLE,
  rxNotEmpty = USART_ISR_RXNE,
  txComplete = USART_ISR_TC,
  txEmpty = USART_ISR_TXE,
  linBreakDetected = USART_ISR_LBDF,
  ctsInterrupt = USART_ISR_CTSIF,
  cts = USART_ISR_CTS,
  rxTimeout = USART_ISR_RTOF,
  endOfBlock = USART_ISR_EOBF,
  autoBaudrateError = USART_ISR_ABRE,
  autoBaudrate = USART_ISR_ABRF,
  busy = USART_ISR_BUSY,
  characterMatch = USART_ISR_CMF,
  breakCharacterRequested = USART_ISR_SBKF,
  rxMuted = USART_ISR_RWU,
  wakeup = USART_ISR_WUF,
  txEnableAck = USART_ISR_TEACK,
  rxEnableAck = USART_ISR_REACK
};


enum class UsartParity : uint32_t {
  even = 0,
  odd = USART_CR1_PS
};

enum class UsartWakeupType : uint32_t {
  idleLine = 0,
  addressMark = USART_CR1_WAKE
};

enum class UsartOverSampling : uint32_t {
  x16 = 0,
  x8 = USART_CR1_OVER8
};

enum class UsartAddressSize : uint32_t {
  _4bit = 0,
  _7bit = USART_CR2_ADDM7
};

enum class UsartLinBreakSize : uint32_t {
  _10bit = 0,
  _11bit = USART_CR2_LBDL
};

enum class UsartClockPhase : uint32_t {
  firstClock = 0,
  secondClock = USART_CR2_CPHA
};

enum class UsartClockPolarity : uint32_t {
  low = 0,
  hight = USART_CR2_CPOL
};

enum class UsartStopBit : uint32_t {
  one = 0,
  half,
  two,
  oneHalf
};

enum class UsartBitOrder : uint32_t {
  lsbFirst = 0,
  msbFirst = USART_CR2_MSBFIRST
};

enum class UsartAutoBaudrateMode : uint32_t {
  detectOnStartBit = 0,
  detectOnFallingEdge,
  detectOn7fFrame,
  detectOn55Frame
};

enum class UsartSampleBitMode : uint32_t {
  threeSampleBit = 0,
  oneSampleBit = USART_CR3_ONEBIT
};

enum class UsartDriverEnablePolarity : uint32_t {
  high = 0,
  low = USART_CR3_DEP
};

enum class UsartWakeupItFlagType : uint32_t {
  wakeupOnAddress = 0,
  wakeupOnStartbit = 2,
  wakeupOnRxne = 3
};

enum class UsartDataWidth : uint32_t {
  _8bit = 0,
#ifdef USART_CR1_M1
  _7bit = USART_CR1_M1,
#endif
  _9bit = USART_CR1_M0
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
  bool enable_auto_baudrate = false;
  UsartAutoBaudrateMode auto_baudrate_mode = UsartAutoBaudrateMode::detectOnStartBit;
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

  bool request_auto_baudrate = false;
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
  constexpr static auto info = STM32_PERIPH_INFO.usart[to_idx(type)];
  constexpr static auto addr = STM32_PERIPH_INFO.usart[to_idx(type)].addr;
  constexpr static auto addr_CR1 = IGB_USART_REG_ADDR(CR1);
  constexpr static auto addr_CR2 = IGB_USART_REG_ADDR(CR2);
  constexpr static auto addr_CR3 = IGB_USART_REG_ADDR(CR3);
  constexpr static auto addr_BRR = IGB_USART_REG_ADDR(BRR);
  constexpr static auto addr_GTPR = IGB_USART_REG_ADDR(GTPR);
  constexpr static auto addr_RTOR = IGB_USART_REG_ADDR(RTOR);
  constexpr static auto addr_RQR = IGB_USART_REG_ADDR(RQR);
  constexpr static auto addr_ISR = IGB_USART_REG_ADDR(ISR);
  constexpr static auto addr_ICR = IGB_USART_REG_ADDR(ICR);
  constexpr static auto addr_RDR = IGB_USART_REG_ADDR(RDR);
  constexpr static auto addr_TDR = IGB_USART_REG_ADDR(TDR);

  RegFlag<addr_CR1, USART_CR1_UE> enable;
  RegFlag<addr_CR1, USART_CR1_UESM> enableWakeup;
  RegFlag<addr_CR1, USART_CR1_RE> enableRx;
  RegFlag<addr_CR1, USART_CR1_TE> enableTx;

  RegFlag<addr_CR1, USART_CR1_IDLEIE> enableItIdle;
  RegFlag<addr_CR1, USART_CR1_RXNEIE> enableItRxNotEmpty;
  RegFlag<addr_CR1, USART_CR1_TCIE> enableItTxComplete;
  RegFlag<addr_CR1, USART_CR1_TXEIE> enableItTxEmpty;
  RegFlag<addr_CR1, USART_CR1_PEIE> enableItParityError;
  RegFlag<addr_CR1, USART_CR1_CMIE> enableItCharacterMatch;
  RegFlag<addr_CR1, USART_CR1_RTOIE> enableItRxTimeout;
  RegFlag<addr_CR1, USART_CR1_EOBIE> enableItEndOfBlock;
  RegFlag<addr_CR1, USART_CR1_PCE> enableParity;
  RegEnum<addr_CR1, USART_CR1_PS_Msk, UsartParity> paritySelect;
  RegEnum<addr_CR1, USART_CR1_WAKE_Msk, UsartWakeupType> wakeupType;

#ifdef USART_CR1_M1
  RegEnum<addr_CR1, USART_CR1_M0_Msk | USART_CR1_M1_Msk, UsartDataWidth> dataWidth;
#else
  RegEnum<addr_CR1, USART_CR1_M0_Msk, UsartDataWidth> dataWidth;
#endif
  RegFlag<addr_CR1, USART_CR1_MME> muteMode;
  RegEnum<addr_CR1, USART_CR1_OVER8_Msk, UsartOverSampling> overSampling;
  RegValue<addr_CR1, USART_CR1_DEDT_Msk, USART_CR1_DEDT_Pos> driverEnableDeAssertionTime;
  RegValue<addr_CR1, USART_CR1_DEAT_Msk, USART_CR1_DEAT_Pos> driverEnableAssertionTime;

  RegEnum<addr_CR2, USART_CR2_ADDM7_Msk, UsartAddressSize> addressSize;
  RegEnum<addr_CR2, USART_CR2_LBDL_Msk, UsartLinBreakSize> linBreakSize;
  RegFlag<addr_CR2, USART_CR2_LBCL> outputLbClock;
  RegEnum<addr_CR2, USART_CR2_CPHA_Msk, UsartClockPhase> clockPhase;
  RegEnum<addr_CR2, USART_CR2_CPOL_Msk, UsartClockPolarity> clockPolarity;
  RegFlag<addr_CR2, USART_CR2_CLKEN> enableClock;
  RegEnum<addr_CR2, USART_CR2_STOP_Msk, UsartStopBit, USART_CR2_STOP_Pos> stopBit;
  RegFlag<addr_CR2, USART_CR2_LINEN> enableLin;
  RegFlag<addr_CR2, USART_CR2_SWAP> swapTxRx;
  RegFlag<addr_CR2, USART_CR2_RXINV> invertRxPolarity;
  RegFlag<addr_CR2, USART_CR2_TXINV> invertTxPolarity;
  RegFlag<addr_CR2, USART_CR2_DATAINV> invertDataPolarity;
  RegEnum<addr_CR2, USART_CR2_MSBFIRST, UsartBitOrder> bitOrder;
  RegFlag<addr_CR2, USART_CR2_ABREN> enableAutoBaudrate;
  RegEnum<addr_CR2, USART_CR2_ABRMODE_Msk, UsartAutoBaudrateMode, USART_CR2_ABRMODE_Pos> autoBaudrateMode;
  RegFlag<addr_CR2, USART_CR2_RTOEN> enableRxTimeout;
  RegValue<addr_CR2, USART_CR2_ADD_Msk, USART_CR2_ADD_Pos> address;
  RegFlag<addr_CR2, USART_CR2_LBDIE> enableItLineBreak;


  RegFlag<addr_CR3, USART_CR3_IREN> enableIrDA;
  RegFlag<addr_CR3, USART_CR3_IRLP> lowpowerIrDA;
  RegFlag<addr_CR3, USART_CR3_HDSEL> enableHalfDuplex;
  RegFlag<addr_CR3, USART_CR3_NACK> enableSmartcardNack;
  RegFlag<addr_CR3, USART_CR3_SCEN> enableSmartcardMode;
  RegFlag<addr_CR3, USART_CR3_DMAR> dmaRx;
  RegFlag<addr_CR3, USART_CR3_DMAT> dmaTx;
  RegFlag<addr_CR3, USART_CR3_RTSE> enableRts;
  RegFlag<addr_CR3, USART_CR3_CTSE> enableCts;
  RegEnum<addr_CR3, USART_CR3_ONEBIT_Msk, UsartSampleBitMode> sampleBitMode;
  RegFlag<addr_CR3, USART_CR3_OVRDIS, false> enableOverrun;
  RegFlag<addr_CR3, USART_CR3_DDRE> dmaDeactOnRxError;
  RegFlag<addr_CR3, USART_CR3_DEM> driverEnableMode;
  RegEnum<addr_CR3, USART_CR3_DEP_Msk, UsartDriverEnablePolarity> driverEnablePolarity;
  RegValue<addr_CR3, USART_CR3_SCARCNT_Msk, USART_CR3_SCARCNT_Pos> smartcardAutoRetryCount;
  RegEnum<addr_CR3, USART_CR3_WUS_Msk, UsartWakeupItFlagType, USART_CR3_WUS_Pos> wakeupItFlagType;
  RegFlag<addr_CR3, USART_CR3_EIE> enableItError;
  RegFlag<addr_CR3, USART_CR3_CTSIE> enableItCts;
  RegFlag<addr_CR3, USART_CR3_WUFIE> enableItWakeup;

  Reg<addr_BRR> reg_BRR;

  IGB_FAST_INLINE void setBaudrate(uint32_t base_freq, uint32_t baudrate, UsartOverSampling over_sampling) {
    switch (over_sampling) {
      case UsartOverSampling::x16:
        reg_BRR((uint16_t)((base_freq + (baudrate / 2)) / baudrate));
        break;
      case UsartOverSampling::x8:
        {
          const uint16_t div = ((base_freq * 2) + (baudrate / 2)) / baudrate;
          reg_BRR((div & 0xFFF0) | ((div & 0x000F) >> 1));
        }
        break;
      default:
        break;
    }
  }

  RegValue<addr_GTPR, USART_GTPR_PSC_Msk, USART_GTPR_PSC_Pos> irdaPrescaler;
  RegValue<addr_GTPR, USART_GTPR_GT_Msk, USART_GTPR_GT_Pos> smartcardGuardTime;

  RegValue<addr_RTOR, USART_RTOR_RTO_Msk, USART_RTOR_RTO_Pos> rxTimeout;
  RegValue<addr_RTOR, USART_RTOR_BLEN_Msk, USART_RTOR_BLEN_Pos> blockLength;

  RegFlag<addr_RQR, USART_RQR_ABRRQ> requestAutoBaudrate;
  RegFlag<addr_RQR, USART_RQR_SBKRQ> requestBreakSending;
  RegFlag<addr_RQR, USART_RQR_MMRQ> requestEnterMuteMode;
  RegFlag<addr_RQR, USART_RQR_RXFRQ> requestRxDataFlush;
  RegFlag<addr_RQR, USART_RQR_TXFRQ> requestTxDataFlush;

  IGB_FAST_INLINE static bool is(UsartState state) {
    return IGB_USART->ISR & static_cast<uint32_t>(state);
  }

  IGB_FAST_INLINE static void clear(UsartState state) {
    IGB_USART->ICR = IGB_USART->ICR | static_cast<uint32_t>(state);
  }

  Reg<addr_RDR> rxData;
  Reg<addr_TDR> txData;

  IGB_FAST_INLINE void enableBusClock() {
    STM32_PERIPH_INFO.usart[to_idx(type)].bus.enableBusClock();
  }

  IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    auto periph_type = as_periph_type(type);
    if (!periph_type) { return; }
    auto result = get_af_idx(periph_type.value(), pin_type);
    if (!result) { return; }

    GpioType gpio_type = extract_gpio_type(pin_type);
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
     autoBaudrateMode.val(conf.auto_baudrate_mode) |
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

    enableAutoBaudrate(conf.enable_auto_baudrate);

    (
     requestAutoBaudrate.val(conf.request_auto_baudrate) |
     requestBreakSending.val(conf.request_break_sending) |
     requestEnterMuteMode.val(conf.request_enter_mute_mode) |
     requestRxDataFlush.val(conf.request_rx_data_flush) |
     requestTxDataFlush.val(conf.request_tx_data_flush)
    ).update();

    enable(true);
  }
};

#endif /* STM32F3 */

#undef IGB_USART_REG
#undef IGB_USART_REG_ADDR
#undef IGB_USART

}
}

#endif /* IGB_STM32_PERIPH_USART_H */
