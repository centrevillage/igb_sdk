#ifndef IGB_STM32_PERIPH_I2C_H
#define IGB_STM32_PERIPH_I2C_H

#include <stddef.h>

#include <igb_stm32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/rcc.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>

namespace igb {
namespace stm32 {

#define IGB_I2C ((I2C_TypeDef*)addr)
#define IGB_I2C_REG_ADDR(member) (addr + offsetof(I2C_TypeDef, member))
#define IGB_I2C_REG(member) ((I2C_TypeDef*)IGB_I2C_REG_ADDR(member))

template<I2cType type>
struct I2c {
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
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_WUPEN>                 wakeup;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_GCEN>                  generalCall;

  enum class AddressingMode : uint32_t {
    _7bit = 0,
    _10_bit = I2C_CR2_ADD10
  };
  RegEnum<IGB_I2C_REG_ADDR(CR2), I2C_CR2_ADD10, AddressingMode> masterAddressing;

  RegFlag<IGB_I2C_REG_ADDR(OAR1), I2C_OAR1_OA1EN>   ownAddress1;
  RegValue<IGB_I2C_REG_ADDR(OAR1), I2C_OAR1_OA1, 0> ownAddress1Value;
  enum class OwnAddress1Size : uint32_t {
    _7bit = 0,
    _10_bit = I2C_OAR1_OA1MODE
  };
  RegEnum<IGB_I2C_REG_ADDR(OAR1), I2C_OAR1_OA1MODE, OwnAddress1Size> ownAddress1Size;

  RegFlag<IGB_I2C_REG_ADDR(OAR2), I2C_OAR2_OA2EN>   ownAddress2;
  RegValue<IGB_I2C_REG_ADDR(OAR2), I2C_OAR2_OA2, 0> ownAddress2Value;
  enum class OwnAddress2Mask: uint32_t {
    nomask = I2C_OAR2_OA2NOMASK,
    mask01 = I2C_OAR2_OA2MASK01,
    mask02 = I2C_OAR2_OA2MASK02,
    mask03 = I2C_OAR2_OA2MASK03,
    mask04 = I2C_OAR2_OA2MASK04,
    mask05 = I2C_OAR2_OA2MASK05,
    mask06 = I2C_OAR2_OA2MASK06,
    mask07 = I2C_OAR2_OA2MASK07
  };
  RegEnum<IGB_I2C_REG_ADDR(OAR2), I2C_OAR2_OA2MSK, OwnAddress2Mask> ownAddress2Mask;

  Reg<IGB_I2C_REG_ADDR(TIMINGR)> timing;

  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_SMBHEN> smBusHost;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_SMBDEN> smBusDevice;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_ALERTEN> smBusAlert;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_PECEN> smBusPacketErrCalc;

  RegFlag<IGB_I2C_REG_ADDR(TIMEOUTR), I2C_TIMEOUTR_TIMOUTEN> smBusTimeoutA;
  RegFlag<IGB_I2C_REG_ADDR(TIMEOUTR), I2C_TIMEOUTR_TEXTEN>   smBusTimeoutB;
  RegValue<IGB_I2C_REG_ADDR(TIMEOUTR), I2C_TIMEOUTR_TIMEOUTA, 0> smBusTimeoutAValue;
  RegValue<IGB_I2C_REG_ADDR(TIMEOUTR), I2C_TIMEOUTR_TIMEOUTB, I2C_TIMEOUTR_TIMEOUTB_Pos> smBusTimeoutBValue;
  enum class SmbusTimeoutAMode : uint32_t {
    scl_low = 0,
    sda_scl_high = I2C_TIMEOUTR_TIDLE
  };
  RegEnum<IGB_I2C_REG_ADDR(TIMEOUTR), I2C_TIMEOUTR_TIDLE, SmbusTimeoutAMode> smBusTimeoutAMode;

  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_TXIE> interruptTx;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_RXIE> interruptRx;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_ADDRIE> interruptAddr;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_NACKIE> interruptNack;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_STOPIE> interruptStop;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_TCIE> interruptTransferComplete;
  RegFlag<IGB_I2C_REG_ADDR(CR1), I2C_CR1_ERRIE> interruptError;

  enum class Status {
    txDataEmpty = I2C_ISR_TXE,
    txInterrupt = I2C_ISR_TXIS,
    rxNotEmpty = I2C_ISR_RXNE,
    addressMatched = I2C_ISR_ADDR,
    nack = I2C_ISR_NACKF,
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

  IGB_FAST_INLINE bool is(Status status) {
    return IGB_I2C->ISR & static_cast<uint32_t>(status);
  }

  IGB_FAST_INLINE void clearStatus() {
    IGB_I2C->ISR = I2C_ISR_TXE;
  }

  enum class IntteruptType {
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

  IGB_FAST_INLINE void clearInterrupt(IntteruptType interrupt) {
    IGB_I2C->ICR |= static_cast<uint32_t>(interrupt);
  }

  RegFlag<IGB_I2C_REG_ADDR(CR2), I2C_CR2_AUTOEND> autoEndMode;
  RegFlag<IGB_I2C_REG_ADDR(CR2), I2C_CR2_RELOAD>  reloadMode;
  RegValue<IGB_I2C_REG_ADDR(CR2), I2C_CR2_NBYTES, I2C_CR2_NBYTES_Pos>  transferSize;
  enum class AckType {
    ack = 0,
    nack = I2C_CR2_NACK
  };
  RegEnum<IGB_I2C_REG_ADDR(CR2), I2C_CR2_NACK, AckType> ackNextData;
  RegFlag<IGB_I2C_REG_ADDR(CR2), I2C_CR2_START> startCondition;
  RegFlag<IGB_I2C_REG_ADDR(CR2), I2C_CR2_STOP> stopCondition;
  RegFlag<IGB_I2C_REG_ADDR(CR2), I2C_CR2_HEAD10R> auto10bitRead;
  enum class TransferRequestType {
    write = 0,
    read = I2C_CR2_RD_WRN
  };
  RegEnum<IGB_I2C_REG_ADDR(CR2), I2C_CR2_RD_WRN, TransferRequestType> transferRequest;
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

  IGB_FAST_INLINE void transferU8(uint8_t value) {
    IGB_I2C->TXDR = value;
  }

  IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    auto periph_type = as_periph_type(type);
    if (!periph_type) { return; }

    auto result = get_af_idx(periph_type.value(), pin_type);
    if (!result) { return; }

    GpioType gpio_type = extract_gpio_type(pin_type);
    GpioPin pin = GpioPin::newPin(pin_type);
    pin.setMode(GpioMode::ALTERNATE);
    pin.setPullMode(GpioPullMode::UP);
    pin.setSpeedMode(GpioSpeedMode::HIGH);
    pin.setOutputMode(GpioOutputMode::OPENDRAIN);
    pin.setAlternateFunc(result.value());
    pin.enable();
  }

  IGB_FAST_INLINE void prepareI2c(GpioPinType scl_pin, GpioPinType sda_pin) {
    const auto& i2c_info = STM32_PERIPH_INFO.i2c[static_cast<size_t>(type)];
    i2c_info.bus.enableBusClock();
    prepareGpio(scl_pin);
    prepareGpio(sda_pin);

    autoEndMode.enable();
    ownAddress2.disable();
    generalCall.disable();
    clockStretch.enable();

    disable();
    analogFilter.enable();
    digitalFilter(0);
    timing(0x2000090E); // TODO: calculate timing from params
    enable();
    ownAddress1.disable();
    ownAddress1Value(0);
    ownAddress1Size(OwnAddress1Size::_7bit);
    smBusHost.disable(); smBusDevice.disable(); // i2c mode
    ackNextData(AckType::ack);
    ownAddress2Mask(OwnAddress2Mask::nomask);
  }
};

#undef IGB_I2C_REG
#undef IGB_I2C_REG_ADDR
#undef IGB_I2C

} /* stm32 */
} /* igb */

#endif /* IGB_STM32_PERIPH_I2C_H */
