#ifndef IGB_STM32_PERIPH_I2C_H
#define IGB_STM32_PERIPH_I2C_H

#include <igb_stm32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/rcc.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/accessor.hpp>

namespace igb {
namespace stm32 {

template<volatile I2C_TypeDef* const p_i2c>
struct I2c {
  IGB_FAST_INLINE void enable() {
    p_i2c->CR1 |= I2C_CR1_PE;
  }

  IGB_FAST_INLINE void disable() {
    p_i2c->CR1 &= ~I2C_CR1_PE;
  }

  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_ANFOFF, true>          analogFilter;
  RegValueAccessor<&(p_i2c->CR1), I2C_CR1_DNF, I2C_CR1_DNF_Pos> digitalFilter;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_TXDMAEN>               dmaTx;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_RXDMAEN>               dmaRx;
  RegAccessor<&(p_i2c->TXDR)>                                   txData;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_NOSTRETCH, true>       clockStretch;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_SBC>                   slaveByteControl;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_WUPEN>                 wakeup;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_GCEN>                  generalCall;

  enum class AddressingMode : uint32_t {
    _7bit = 0,
    _10_bit = I2C_CR2_ADD10
  };
  RegEnumAccessor<&(p_i2c->CR2), I2C_CR2_ADD10, AddressingMode> masterAddressing;

  RegFlagAccessor<&(p_i2c->OAR1), I2C_OAR1_OA1EN>   ownAddress1;
  RegValueAccessor<&(p_i2c->OAR1), I2C_OAR1_OA1, 0> ownAddress1Value;
  enum class OwnAddress1Size: uint32_t {
    _7bit = 0,
    _10_bit = I2C_OAR1_OA1MODE
  };
  RegEnumAccessor<&(p_i2c->OAR1), I2C_OAR1_OA1MODE, OwnAddress1Size> ownAddress1Size;

  RegFlagAccessor<&(p_i2c->OAR2), I2C_OAR2_OA2EN>   ownAddress2;
  RegValueAccessor<&(p_i2c->OAR2), I2C_OAR2_OA2, 0> ownAddress2Value;
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
  RegEnumAccessor<&(p_i2c->OAR2), I2C_OAR2_OA2MSK, OwnAddress2Mask> ownAddress2Mask;

  RegAccessor<&(p_i2c->TIMINGR)> timing;

  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_SMBHEN> smBusHost;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_SMBDEN> smBusDevice;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_ALERTEN> smBusAlert;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_PECEN> smBusPacketErrCalc;

  RegFlagAccessor<&(p_i2c->TIMEOUTR), I2C_TIMEOUTR_TIMOUTEN> smBusTimeoutA;
  RegFlagAccessor<&(p_i2c->TIMEOUTR), I2C_TIMEOUTR_TEXTEN>   smBusTimeoutB;
  RegValueAccessor<&(p_i2c->TIMEOUTR), I2C_TIMEOUTR_TIMEOUTA, 0> smBusTimeoutAValue;
  RegValueAccessor<&(p_i2c->TIMEOUTR), I2C_TIMEOUTR_TIMEOUTB, I2C_TIMEOUTR_TIMEOUTB_Pos> smBusTimeoutBValue;
  enum class SmbusTimeoutAMode : uint32_t {
    scl_low = 0,
    sda_scl_high = I2C_TIMEOUTR_TIDLE
  };
  RegEnumAccessor<&(p_i2c->TIMEOUTR), I2C_TIMEOUTR_TIDLE, SmbusTimeoutAMode> smBusTimeoutAMode;

  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_TXIE> interruptTx;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_RXIE> interruptRx;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_ADDRIE> interruptAddr;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_NACKIE> interruptNack;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_STOPIE> interruptStop;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_TCIE> interruptTransferComplete;
  RegFlagAccessor<&(p_i2c->CR1), I2C_CR1_ERRIE> interruptError;

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
    return p_i2c->ISR & static_cast<uint32_t>(status);
  }

  IGB_FAST_INLINE void clearStatus() {
    p_i2c->ISR = I2C_ISR_TXE;
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
    p_i2c->ICR |= static_cast<uint32_t>(interrupt);
  }

  RegFlagAccessor<&(p_i2c->CR2), I2C_CR2_AUTOEND> autoEndMode;
  RegFlagAccessor<&(p_i2c->CR2), I2C_CR2_RELOAD>  reloadMode;
  RegValueAccessor<&(p_i2c->CR2), I2C_CR2_NBYTES, I2C_CR2_NBYTES_Pos>  transferSize;
  enum class AckType {
    ack = 0,
    nack = I2C_CR2_NACK
  };
  RegEnumAccessor<&(p_i2c->CR2), I2C_CR2_NACK, AckType> ackNextData;
  RegFlagAccessor<&(p_i2c->CR2), I2C_CR2_START> startCondition;
  RegFlagAccessor<&(p_i2c->CR2), I2C_CR2_STOP> stopCondition;
  RegFlagAccessor<&(p_i2c->CR2), I2C_CR2_HEAD10R> auto10bitRead;
  enum class TransferRequestType {
    write = 0,
    read = I2C_CR2_RD_WRN
  };
  RegEnumAccessor<&(p_i2c->CR2), I2C_CR2_RD_WRN, TransferRequestType> transferRequest;
  RegValueAccessor<&(p_i2c->CR2), I2C_CR2_SADD, 0> slaveAddr;

  IGB_FAST_INLINE bool IsTransferDirectionWrite() {
    return !(p_i2c->ISR & I2C_ISR_DIR); 
  }
  IGB_FAST_INLINE bool IsTransferDirectionRead() {
    return (p_i2c->ISR & I2C_ISR_DIR); 
  }

  IGB_FAST_INLINE uint32_t addressMatchCode() {
    return (p_i2c->ISR & I2C_ISR_ADDCODE) >> I2C_ISR_ADDCODE_Pos << 1;
  }

  RegFlagAccessor<&(p_i2c->CR2), I2C_CR2_PECBYTE> smBusPacketErrCalcCompare;

  IGB_FAST_INLINE uint32_t smBusPacketErrorCalcValue() {
    return p_i2c->PECR & I2C_PECR_PEC;
  }

  IGB_FAST_INLINE uint8_t receiveU8() {
    return p_i2c->RXDR & I2C_RXDR_RXDATA;
  }

  IGB_FAST_INLINE void transferU8(uint8_t value) {
    p_i2c->TXDR = value;
  }
};

} /* stm32 */
} /* igb */

#endif /* IGB_STM32_PERIPH_I2C_H */
