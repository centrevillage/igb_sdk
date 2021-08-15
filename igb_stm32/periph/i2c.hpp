#ifndef IGB_STM32_PERIPH_I2C_H
#define IGB_STM32_PERIPH_I2C_H

#include <stddef.h>
#include <optional>

#include <igb_stm32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/rcc.hpp>
#include <igb_stm32/periph/systick.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>

namespace igb {
namespace stm32 {

#define IGB_I2C ((I2C_TypeDef*)addr)
#define IGB_I2C_REG_ADDR(member) (addr + offsetof(I2C_TypeDef, member))
#define IGB_I2C_REG(member) ((I2C_TypeDef*)IGB_I2C_REG_ADDR(member))

enum class I2cAddressingMode : uint32_t {
  _7bit = 0,
  _10bit = I2C_CR2_ADD10
};

enum class I2cOwnAddress1Size : uint32_t {
  _7bit = 0,
  _10bit = I2C_OAR1_OA1MODE
};

enum class I2cOwnAddress2Mask: uint32_t {
  nomask = I2C_OAR2_OA2NOMASK,
  mask01 = I2C_OAR2_OA2MASK01,
  mask02 = I2C_OAR2_OA2MASK02,
  mask03 = I2C_OAR2_OA2MASK03,
  mask04 = I2C_OAR2_OA2MASK04,
  mask05 = I2C_OAR2_OA2MASK05,
  mask06 = I2C_OAR2_OA2MASK06,
  mask07 = I2C_OAR2_OA2MASK07
};

enum class I2cSmbusTimeoutAMode : uint32_t {
  sclLow = 0,
  sdaSclHigh = I2C_TIMEOUTR_TIDLE
};

enum class I2cStatus : uint32_t {
  txDataEmpty = I2C_ISR_TXE,
  txInterrupt = I2C_ISR_TXIS,
  rxNotEmpty = I2C_ISR_RXNE,
  addressMatched = I2C_ISR_ADDR,
  nack = I2C_ISR_NACKF,
  stop = I2C_ISR_STOPF,
  txComplete = I2C_ISR_TC,
  txCompleteReload = I2C_ISR_TCR,
  busError = I2C_ISR_BERR,
  arbitrationLost = I2C_ISR_ARLO,
  overOrUnderrun = I2C_ISR_OVR,
  smbusPacketErrorCalkError = I2C_ISR_PECERR,
  smbusTimeout = I2C_ISR_TIMEOUT,
  smbusAlert = I2C_ISR_ALERT,
  busy = I2C_ISR_BUSY
};

enum class I2cInterruptType {
  addressMatched = I2C_ICR_ADDRCF,
  nack = I2C_ICR_NACKCF,
  stop = I2C_ICR_STOPCF,
  busError = I2C_ICR_BERRCF,
  arbitrationLost = I2C_ICR_ARLOCF,
  overOrUnderrun = I2C_ICR_OVRCF,
  smbusPacketErrorCalkError = I2C_ICR_PECCF,
  smbusTimeout = I2C_ICR_TIMOUTCF,
  smbusAlert = I2C_ICR_ALERTCF
};

enum class I2cReloadEndType {
  softEnd = 0,
  reload = I2C_CR2_RELOAD,
  autoEnd = I2C_CR2_AUTOEND
};

enum class I2cAckType {
  ack = 0,
  nack = I2C_CR2_NACK
};

enum class I2cTransferRequestType {
  write = 0,
  read = I2C_CR2_RD_WRN
};

// TODO: base_clockをRCCの設定から自動計算
template<I2cType I2C_TYPE, GpioPinType SCL_PIN, GpioPinType SDA_PIN>
struct I2c {
  constexpr static auto type = I2C_TYPE;
  constexpr static auto scl_pin = SCL_PIN;
  constexpr static auto sda_pin = SDA_PIN;
  constexpr static auto addr = STM32_PERIPH_INFO.i2c[static_cast<size_t>(type)].addr;

  IGB_FAST_INLINE void enable() { IGB_I2C->CR1 |= I2C_CR1_PE; }
  IGB_FAST_INLINE void disable() { IGB_I2C->CR1 &= ~I2C_CR1_PE; }
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_ANFOFF, true>          analogFilter;
  RegValue<IGB_I2C_REG_ADDR(CR1), I2C_CR1_DNF, I2C_CR1_DNF_Pos> digitalFilter;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_TXDMAEN>               dmaTx;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_RXDMAEN>               dmaRx;
  Reg<IGB_I2C_REG_ADDR(TXDR)>                                   txData;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_NOSTRETCH, true>       clockStretch;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_SBC>                   slaveByteControl;
#ifdef I2C_CR1_WUPEN
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_WUPEN>                 wakeup;
#endif
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_GCEN>                  generalCall;

  RegEnum<IGB_I2C_REG_ADDR(CR2), I2C_CR2_ADD10, I2cAddressingMode> masterAddressing;

  RegFlag<IGB_I2C_REG_ADDR(OAR1), I2C_OAR1_OA1EN>   ownAddress1;
  RegValue<IGB_I2C_REG_ADDR(OAR1), I2C_OAR1_OA1, 0> ownAddress1Value;
  RegEnum<IGB_I2C_REG_ADDR(OAR1), I2C_OAR1_OA1MODE, I2cOwnAddress1Size> ownAddress1Size;

  RegFlag<IGB_I2C_REG_ADDR(OAR2), I2C_OAR2_OA2EN>   ownAddress2;
  RegValue<IGB_I2C_REG_ADDR(OAR2), I2C_OAR2_OA2, 0> ownAddress2Value;
  RegEnum<IGB_I2C_REG_ADDR(OAR2), I2C_OAR2_OA2MSK, I2cOwnAddress2Mask> ownAddress2Mask;

  Reg<IGB_I2C_REG_ADDR(TIMINGR)> timing;
  RegValue<IGB_I2C_REG_ADDR(TIMINGR), I2C_TIMINGR_SCLL, I2C_TIMINGR_SCLL_Pos> timingScll;
  RegValue<IGB_I2C_REG_ADDR(TIMINGR), I2C_TIMINGR_SCLH, I2C_TIMINGR_SCLH_Pos> timingSclh;
  RegValue<IGB_I2C_REG_ADDR(TIMINGR), I2C_TIMINGR_SDADEL, I2C_TIMINGR_SDADEL_Pos> timingSdadel;
  RegValue<IGB_I2C_REG_ADDR(TIMINGR), I2C_TIMINGR_SCLDEL, I2C_TIMINGR_SCLDEL_Pos> timingScldel;
  RegValue<IGB_I2C_REG_ADDR(TIMINGR), I2C_TIMINGR_PRESC, I2C_TIMINGR_PRESC_Pos> timingPrescale;

  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_SMBHEN> smBusHost;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_SMBDEN> smBusDevice;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_ALERTEN> smBusAlert;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_PECEN> smBusPacketErrCalc;

  RegFlag<IGB_I2C_REG_ADDR(TIMEOUTR), I2C_TIMEOUTR_TIMOUTEN> smBusTimeoutA;
  RegFlag<IGB_I2C_REG_ADDR(TIMEOUTR), I2C_TIMEOUTR_TEXTEN>   smBusTimeoutB;
  RegValue<IGB_I2C_REG_ADDR(TIMEOUTR), I2C_TIMEOUTR_TIMEOUTA, 0> smBusTimeoutAValue;
  RegValue<IGB_I2C_REG_ADDR(TIMEOUTR), I2C_TIMEOUTR_TIMEOUTB, I2C_TIMEOUTR_TIMEOUTB_Pos> smBusTimeoutBValue;
  RegEnum<IGB_I2C_REG_ADDR(TIMEOUTR), I2C_TIMEOUTR_TIDLE, I2cSmbusTimeoutAMode> smBusTimeoutAMode;

  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_TXIE> interruptTx;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_RXIE> interruptRx;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_ADDRIE> interruptAddr;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_NACKIE> interruptNack;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_STOPIE> interruptStop;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_TCIE> interruptTransferComplete;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_ERRIE> interruptError;

  IGB_FAST_INLINE bool is(I2cStatus status) {
    return IGB_I2C->ISR & static_cast<uint32_t>(status);
  }

  IGB_FAST_INLINE void clear(I2cStatus status) {
    if (status == I2cStatus::txDataEmpty) {
      IGB_I2C->ISR |= I2C_ISR_TXE;
    } else {
      IGB_I2C->ICR = static_cast<uint32_t>(status);
    }
  }

  IGB_FAST_INLINE void clear(I2cInterruptType interrupt) {
    IGB_I2C->ICR |= static_cast<uint32_t>(interrupt);
  }

  RegEnum<IGB_I2C_REG_ADDR(CR2), I2C_CR2_RELOAD | I2C_CR2_AUTOEND, I2cReloadEndType>  reloadEndMode;
  RegValue<IGB_I2C_REG_ADDR(CR2), I2C_CR2_NBYTES, I2C_CR2_NBYTES_Pos>  transferSize;
  RegEnum<IGB_I2C_REG_ADDR(CR2), I2C_CR2_NACK, I2cAckType> ackNextData;
  RegFlag<IGB_I2C_REG_ADDR(CR2), I2C_CR2_START> startCondition;
  RegFlag<IGB_I2C_REG_ADDR(CR2), I2C_CR2_STOP> stopCondition;
  RegEnum<IGB_I2C_REG_ADDR(CR2), I2C_CR2_RD_WRN, I2cTransferRequestType> transferRequest;
  RegFlag<IGB_I2C_REG_ADDR(CR2), I2C_CR2_HEAD10R> auto10bitRead;
  RegValue<IGB_I2C_REG_ADDR(CR2), I2C_CR2_SADD, 0> slaveAddr;

  IGB_FAST_INLINE bool IsTransferDirectionWrite() {
    return !(IGB_I2C->ISR & I2C_ISR_DIR); 
  }
  IGB_FAST_INLINE bool IsTransferDirectionRead() {
    return (IGB_I2C->ISR & I2C_ISR_DIR); 
  }

  IGB_FAST_INLINE uint32_t addressMatchCode() {
    return (IGB_I2C->ISR & I2C_ISR_ADDCODE) >> I2C_ISR_ADDCODE_Pos << 1;
  }

  RegFlag<IGB_I2C_REG_ADDR(CR2), I2C_CR2_PECBYTE> smBusPacketErrCalcCompare;

  IGB_FAST_INLINE uint32_t smBusPacketErrorCalcValue() {
    return IGB_I2C->PECR & I2C_PECR_PEC;
  }

  IGB_FAST_INLINE uint8_t receiveU8() {
    return IGB_I2C->RXDR & I2C_RXDR_RXDATA;
  }

  IGB_FAST_INLINE std::optional<uint8_t> receiveU8sync(uint32_t timeout_msec = 1000) {
    uint32_t msec = current_msec();
    while(!is(I2cStatus::txInterrupt)) {
      if (current_msec() - msec > timeout_msec) {
        return std::nullopt;
      }
    }
    return receiveU8();
  }

  IGB_FAST_INLINE bool isReceivable() {
    return is(I2cStatus::txInterrupt);
  }

  IGB_FAST_INLINE void sendU8(uint8_t value) {
    IGB_I2C->TXDR = value;
  }

  IGB_FAST_INLINE bool sendU8sync(uint8_t value, uint32_t timeout_msec = 1000) {
    uint32_t msec = current_msec();
    while(!is(I2cStatus::txInterrupt)) {
      if (current_msec() - msec > timeout_msec) {
        return false;
      }
    }
    sendU8(value);
    return true;
  }

  IGB_FAST_INLINE bool isSendable() {
    return is(I2cStatus::txInterrupt);
  }

  IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    auto periph_type = as_periph_type(type);
    if (!periph_type) { return; }

    auto result = get_af_idx(periph_type.value(), pin_type);
    if (!result) { return; }

    GpioType gpio_type = extract_gpio_type(pin_type);
    GpioPin pin = GpioPin::newPin(pin_type);
    pin.setMode(GpioMode::alternate);
    pin.setPullMode(GpioPullMode::up);
    pin.setSpeedMode(GpioSpeedMode::high);
    pin.setOutputMode(GpioOutputMode::opendrain);
    pin.setAlternateFunc(result.value());
    pin.enable();
  }

  IGB_FAST_INLINE void initDefault(uint8_t address = 1) {
    const auto& i2c_info = STM32_PERIPH_INFO.i2c[static_cast<size_t>(type)];
    i2c_info.bus.enableBusClock();
    i2c_info.bus.forceResetBusClock();
    i2c_info.bus.releaseResetBusClock();

    prepareGpio(scl_pin);
    prepareGpio(sda_pin);

    reloadEndMode(I2cReloadEndType::autoEnd);
    ownAddress2.disable();
    generalCall.disable();
    clockStretch.enable();

    disable();
    analogFilter.enable();
    digitalFilter(0);

    timing(0x00702025); // 100kHz (clock src = HSI 8MHz), TODO: calculate timing from params
    enable();
    ownAddress1.disable();
    auto ownAddress1Reg = ownAddress1.val(true) | ownAddress1Value.val(address << 1);
    ownAddress1Reg.update();
    ownAddress1Size(I2cOwnAddress1Size::_7bit);
    smBusHost.disable(); smBusDevice.disable(); // i2c mode
    ackNextData(I2cAckType::ack);
    ownAddress2Mask(I2cOwnAddress2Mask::nomask);
    auto10bitRead.val(false);
  }

  IGB_FAST_INLINE void init(uint8_t address = 1) {
    initDefault(address);
  }

  IGB_FAST_INLINE void beginTransfer(uint8_t address, uint8_t transfer_size, I2cTransferRequestType request_type, I2cReloadEndType reload_end = I2cReloadEndType::autoEnd) {
    auto reg =
      slaveAddr.val((uint32_t)address)
      | ackNextData.val(I2cAckType::ack)
      | transferSize.val(transfer_size)
      | reloadEndMode.val(reload_end)
      | transferRequest.val(request_type)
      | stopCondition.val(false)
      | startCondition.val(true)
    ;
    reg.update();
  }

  IGB_FAST_INLINE bool isTransferEnd() {
    return is(I2cStatus::stop);
  }

  // TODO: autoEnd でない時も正常に停止できる様に
  IGB_FAST_INLINE bool endTransfer(uint32_t timeout_msec = 1000) {
    // wait for auto end
    uint32_t msec = current_msec();
    while(!is(I2cStatus::stop)) {
      if (current_msec() - msec > timeout_msec) {
        return false;
      }
    }
    return true;
  }

  // common api ==

  IGB_FAST_INLINE void beginSending(uint8_t address, uint8_t transfer_size) {
    beginTransfer(address << 1, transfer_size, I2cTransferRequestType::write, I2cReloadEndType::autoEnd);
  }

  IGB_FAST_INLINE bool endSending() {
    return endTransfer();
  }

  IGB_FAST_INLINE void beginReading(uint8_t address, uint8_t transfer_size) {
    beginTransfer((address << 1) | 1, transfer_size, I2cTransferRequestType::read, I2cReloadEndType::autoEnd);
  }

  IGB_FAST_INLINE bool endReading() {
    return endTransfer();
  }

  IGB_FAST_INLINE bool checkSlave(uint8_t address, uint32_t timeout_msec = 1000) {
    auto reg =
      slaveAddr.val((uint32_t)address << 1)
      | ackNextData.val(I2cAckType::ack)
      | transferSize.val(0)
      | reloadEndMode.val(I2cReloadEndType::autoEnd)
      | transferRequest.val(I2cTransferRequestType::write)
      | stopCondition.val(false)
      | startCondition.val(true)
    ;
    reg.update();

    uint32_t msec = current_msec();

    while (!is(I2cStatus::stop) && !is(I2cStatus::nack)) {
      if (current_msec() - msec > timeout_msec) {
        return false;
      }
    }

    if (is(I2cStatus::nack)) {

      while (!is(I2cStatus::stop)) {
        if (current_msec() - msec > timeout_msec) {
          return false;
        }
      }

      clear(I2cStatus::nack);
      clear(I2cStatus::stop);
    } else {
      while (!is(I2cStatus::stop)) {
        if (current_msec() - msec > timeout_msec) {
          return false;
        }
      }

      clear(I2cStatus::stop);
    }

    return true;
  }
};

#undef IGB_I2C_REG
#undef IGB_I2C_REG_ADDR
#undef IGB_I2C

} /* stm32 */
} /* igb */

#endif /* IGB_STM32_PERIPH_I2C_H */
