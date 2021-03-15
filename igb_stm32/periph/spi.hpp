#ifndef IGB_STM32_PERIPH_SPI_H
#define IGB_STM32_PERIPH_SPI_H

#include <igb_stm32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/rcc.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace stm32 {

enum class SpiMode : uint32_t {
  slave  = 0,
#if defined(STM32H7)
  master = SPI_CFG2_MASTER,
#else
  master = (SPI_CR1_MSTR | SPI_CR1_SSI),
#endif
};

enum class SpiStandard : uint32_t {
  motorola = 0,
#if defined(STM32H7)
  ti = SPI_CFG2_SP_0,
#else
  ti = SPI_CR2_FRF,
#endif
};

enum class SpiClockPhase : uint32_t {
  oneEdge = 0,
#if defined(STM32H7)
  twoEdge = SPI_CFG2_CPHA,
#else
  twoEdge = SPI_CR1_CPHA,
#endif
};

enum class SpiClockPolarity : uint32_t {
  low = 0,
#if defined(STM32H7)
  high = SPI_CFG2_CPOL,
#else
  high = SPI_CR1_CPOL,
#endif
};

enum class SpiBaudratePrescaler : uint32_t {
  div2 = 0,
#if defined(STM32H7)
  div4 = SPI_CFG1_MBR_0,
  div8 = SPI_CFG1_MBR_1,
  div16 = SPI_CFG1_MBR_1 | SPI_CFG1_MBR_0,
  div32 = SPI_CFG1_MBR_2,
  div64 = SPI_CFG1_MBR_2 | SPI_CFG1_MBR_0,
  div128 = SPI_CFG1_MBR_2 | SPI_CFG1_MBR_1,
  div256 = SPI_CFG1_MBR_2 | SPI_CFG1_MBR_1 | SPI_CFG1_MBR_0,
#else
  div4 = SPI_CR1_BR_0,
  div8 = SPI_CR1_BR_1,
  div16 = SPI_CR1_BR_1 | SPI_CR1_BR_0,
  div32 = SPI_CR1_BR_2,
  div64 = SPI_CR1_BR_2 | SPI_CR1_BR_0,
  div128 = SPI_CR1_BR_2 | SPI_CR1_BR_1,
  div256 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,
#endif
};

enum class SpiBitOrder : uint32_t {
  msbFirst = 0,
#if defined(STM32H7)
  lsbFirst = SPI_CFG2_LSBFRST,
#else
  lsbFirst = SPI_CR1_LSBFIRST,
#endif
};

enum class SpiTransDir : uint32_t {
  fullDuplex = 0,
#if defined(STM32H7)
  simplexTx = SPI_CFG2_COMM_0,
  simplexRx = SPI_CFG2_COMM_1,
  halfDuplexRx = SPI_CFG2_COMM_0 | SPI_CFG2_COMM_1,
  halfDuplexTx = SPI_CFG2_COMM_0 | SPI_CFG2_COMM_1 | SPI_CR1_HDDIR,
#else
  simplexRx  = SPI_CR1_RXONLY,
  halfDuplexRx = SPI_CR1_BIDIMODE,
  halfDuplexTx = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE,
#endif
};

enum class SpiDataWidth : uint32_t {
#if defined(STM32H7)
  _4bit = SPI_CFG1_DSIZE_0 | SPI_CFG1_DSIZE_1,
  _5bit = SPI_CFG1_DSIZE_2,
  _6bit = SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_0,
  _7bit = SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1,
  _8bit = SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
  _9bit = SPI_CFG1_DSIZE_3,
  _10bit = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_0,
  _11bit = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_1,
  _12bit = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
  _13bit = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2,
  _14bit = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_0,
  _15bit = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1,
  _16bit = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
  _17bit = SPI_CFG1_DSIZE_4,
  _18bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_0,
  _19bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_1,
  _20bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_0 | SPI_CFG1_DSIZE_1,
  _21bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_2,
  _22bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_0,
  _23bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1,
  _24bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
  _25bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3,
  _26bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_0,
  _27bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_1,
  _28bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
  _29bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2,
  _30bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_0,
  _31bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1,
  _32bit = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
#elif defined(STM32F4)
  _8bit = 0,
  _16bit = SPI_CR1_DFF,
#else
  _4bit = SPI_CR2_DS_0 | SPI_CR2_DS_1,
  _5bit = SPI_CR2_DS_2,
  _6bit = SPI_CR2_DS_2 | SPI_CR2_DS_0,
  _7bit = SPI_CR2_DS_2 | SPI_CR2_DS_1,
  _8bit = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0,
  _9bit = SPI_CR2_DS_3,
  _10bit = SPI_CR2_DS_3 | SPI_CR2_DS_0,
  _11bit = SPI_CR2_DS_3 | SPI_CR2_DS_1,
  _12bit = SPI_CR2_DS_3 | SPI_CR2_DS_1 | SPI_CR2_DS_0,
  _13bit = SPI_CR2_DS_3 | SPI_CR2_DS_2,
  _14bit = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_0,
  _15bit = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1,
  _16bit = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0,
#endif
};

#if defined(STM32H7)
enum class SpiFifoThreshold : uint32_t {
  _1data = 0,
  _2data = SPI_CFG1_FTHLV_0,
  _3data = SPI_CFG1_FTHLV_1,
  _4data = SPI_CFG1_FTHLV_0 | SPI_CFG1_FTHLV_1,
  _5data = SPI_CFG1_FTHLV_2,
  _6data = SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_0,
  _7data = SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_1,
  _8data = SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_1 | SPI_CFG1_FTHLV_0,
  _9data = SPI_CFG1_FTHLV_3,
  _10data = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_0,
  _11data = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_1,
  _12data = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_1 | SPI_CFG1_FTHLV_0,
  _13data = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_2,
  _14data = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_0,
  _15data = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_1,
  _16data = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_1 | SPI_CFG1_FTHLV_0,
};
#elif defined(STM32F0) || defined(STM32F3)
enum class SpiFifoThreshold : uint32_t {
  half = 0,
  quarter = SPI_CR2_FRXTH,
};
#endif

#if defined(STM32H7)
enum class SpiCrcWidth : uint32_t {
  _4bit = SPI_CFG1_CRCSIZE_0 | SPI_CFG1_CRCSIZE_1,
  _5bit = SPI_CFG1_CRCSIZE_2,
  _6bit = SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_0,
  _7bit = SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1,
  _8bit = SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0,
  _9bit = SPI_CFG1_CRCSIZE_3,
  _10bit = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_0,
  _11bit = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_1,
  _12bit = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0,
  _13bit = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2,
  _14bit = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_0,
  _15bit = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1,
  _16bit = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0,
  _17bit = (SPI_CFG1_CRCSIZE_4),
  _18bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_0),
  _19bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_1),
  _20bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_0 | SPI_CFG1_CRCSIZE_1),
  _21bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_2),
  _22bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_0),
  _23bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1),
  _24bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0),
  _25bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3),
  _26bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_0),
  _27bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_1),
  _28bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0),
  _29bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2),
  _30bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_0),
  _31bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1),
  _32bit = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0),
};
#elif defined(STM32F0) || defined(STM32F3)
enum class SpiCrcWidth : uint32_t {
  _8bit = 0,
  _16bit = SPI_CR1_CRCL,
};
#endif

enum class SpiNssMode : uint32_t {
#if defined(STM32H7)
  hardInput  = 0,
  hardOutput = SPI_CFG2_SSOE,
  soft       = SPI_CFG2_SSM,
#else
  hardInput  = 0,
  hardOutput = SPI_CR1_SSM,
  soft       = SPI_CR1_SSM,
#endif
};

#if defined(STM32H7)
enum class SpiNssPolarity : uint32_t {
  low = 0,
  high = SPI_CFG2_SSIOP,
};
#endif

enum class SpiState : uint32_t {
#if defined(STM32H7)
  rxWordNotEmpty    = SPI_SR_RXWNE,
  rxPacketAbailable = SPI_SR_RXP,
  txPacketAbailable = SPI_SR_TXP,
  dxPacketAbailable = SPI_SR_DXP,
  endOfTransfer     = SPI_SR_EOT,
  txTransferFull    = SPI_SR_TXTF,
  underrun          = SPI_SR_UDR,
  crcError          = SPI_SR_CRCE,
  modeFault         = SPI_SR_MODF,
  overrun           = SPI_SR_OVR,
  frameFormatError  = SPI_SR_TIFRE,
  dataReloaded      = SPI_SR_TSERF,
  suspendDone       = SPI_SR_SUSP,
  txComplete        = SPI_SR_TXC,
#else
  rxBufNotEmpty    = SPI_SR_RXNE,
  txBufEmpty       = SPI_SR_TXE,
  busy             = SPI_SR_BSY,
  crcError         = SPI_SR_CRCERR,
  modeFault        = SPI_SR_MODF,
  overrun          = SPI_SR_OVR,
  frameFormatError = SPI_SR_FRE,
#endif
};

enum class SpiInterruptType : uint32_t {
#if defined(STM32H7)
  rxPacketAbailable = SPI_IER_RXPIE,
  txPacketAbailable = SPI_IER_TXPIE,
  dxPacketAbailable = SPI_IER_DXPIE,
  endOfTransfer     = SPI_IER_EOTIE,
  txTransferFull    = SPI_IER_TXTFIE,
  underrun          = SPI_IER_UDRIE,
  overrun           = SPI_IER_OVRIE,
  crcError          = SPI_IER_CRCEIE,
  frameFormatError  = SPI_IER_TIFREIE,
  modeFault         = SPI_IER_MODFIE,
  dataReloaded      = SPI_IER_TSERFIE,
#else
  rxBufNotEmpty     = SPI_CR2_RXNEIE,
  txBufEmpty        = SPI_CR2_TXEIE,
  error             = SPI_CR2_ERRIE,
#endif
};

enum class SpiDmaReqTarget : uint32_t {
#if defined(STM32H7)
  rx = SPI_CFG1_RXDMAEN,
  tx = SPI_CFG1_TXDMAEN,
#else
  rx = SPI_CR2_RXDMAEN,
  tx = SPI_CR2_TXDMAEN,
#endif
};

#if !defined(STM32H7)
enum class SpiDmaParityTarget : uint32_t {
  rx = SPI_CR2_RXDMAEN,
  tx = SPI_CR2_TXDMAEN,
};

enum class SpiDmaParityType : uint32_t {
  even = 0,
  odd  = 1,
};
#endif

struct Spi {
  const SpiType type;
  SPI_TypeDef* p_spi;

  IGB_FAST_INLINE void enable() {
    p_spi->CR1 |= SPI_CR1_SPE;
  }

  IGB_FAST_INLINE void disable() {
    p_spi->CR1 &= ~SPI_CR1_SPE;
  }

  IGB_FAST_INLINE void setMode(SpiMode mode) {
#if defined(STM32H7)
    MODIFY_REG(p_spi->CFG2, SPI_CFG2_MASTER, as<uint32_t>(mode));
#else
    MODIFY_REG(p_spi->CR1, SPI_CR1_MSTR | SPI_CR1_SSI, as<uint32_t>(mode));
#endif
  }

  IGB_FAST_INLINE void setStandard(SpiStandard standard) {
#if defined(STM32H7)
    MODIFY_REG(p_spi->CFG2, SPI_CFG2_SP, as<uint32_t>(standard));
#else
    MODIFY_REG(p_spi->CR2, SPI_CR2_FRF, as<uint32_t>(standard));
#endif
  }

  IGB_FAST_INLINE void setClockPhase(SpiClockPhase phase) {
#if defined(STM32H7)
    MODIFY_REG(p_spi->CFG2, SPI_CFG2_CPHA, as<uint32_t>(phase));
#else
    MODIFY_REG(p_spi->CR1, SPI_CR1_CPHA, as<uint32_t>(phase));
#endif
  }

  IGB_FAST_INLINE void setClockPolarity(SpiClockPolarity polarity) {
#if defined(STM32H7)
    MODIFY_REG(p_spi->CFG2, SPI_CFG2_CPOL, as<uint32_t>(polarity));
#else
    MODIFY_REG(p_spi->CR1, SPI_CR1_CPOL, as<uint32_t>(polarity));
#endif
  }

  IGB_FAST_INLINE void setBaudratePrescaler(SpiBaudratePrescaler prescaler) {
#if defined(STM32H7)
    MODIFY_REG(p_spi->CFG1, SPI_CFG1_MBR, as<uint32_t>(prescaler));
#else
    MODIFY_REG(p_spi->CR1, SPI_CR1_BR, as<uint32_t>(prescaler));
#endif
  }

  IGB_FAST_INLINE void setTransBitOrder(SpiBitOrder order) {
#if defined(STM32H7)
    MODIFY_REG(p_spi->CFG2, SPI_CFG2_LSBFRST, as<uint32_t>(order));
#else
    MODIFY_REG(p_spi->CR1, SPI_CR1_LSBFIRST, as<uint32_t>(order));
#endif
  }

  IGB_FAST_INLINE void setTransDir(SpiTransDir dir) {
#if defined(STM32H7)
    MODIFY_REG(p_spi->CR1, SPI_CR1_HDDIR,  as<uint32_t>(dir) & SPI_CR1_HDDIR);
    MODIFY_REG(p_spi->CFG2, SPI_CFG2_COMM, as<uint32_t>(dir) & SPI_CFG2_COMM);
#else
    MODIFY_REG(p_spi->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE, as<uint32_t>(dir));
#endif
  }

#if defined(STM32H7)
  IGB_FAST_INLINE void setDataWidth(SpiDataWidth data_width) {
    MODIFY_REG(p_spi->CFG1, SPI_CFG1_DSIZE, as<uint32_t>(data_width));
  }
#elif defined(STM32F0) || defined(STM32F3)
  IGB_FAST_INLINE void setDataWidth(SpiDataWidth data_width) {
    MODIFY_REG(p_spi->CR2, SPI_CR2_DS, as<uint32_t>(data_width));
  }
#elif defined(STM32F4)
  IGB_FAST_INLINE void setDataWidth(SpiDataWidth data_width) {
    MODIFY_REG(p_spi->CR1, SPI_CR1_DFF, as<uint32_t>(data_width));
  }
#endif

#if defined(STM32H7)
  IGB_FAST_INLINE void setFifoThreshold(SpiFifoThreshold threshold) {
    MODIFY_REG(p_spi->CFG1, SPI_CFG1_FTHLV, as<uint32_t>(threshold));
  }
#elif defined(STM32F0) || defined(STM32F3)
  IGB_FAST_INLINE void setRxFifoThreshold(SpiFifoThreshold threshold) {
    MODIFY_REG(p_spi->CR2, SPI_CR2_FRXTH, as<uint32_t>(threshold));
  }
#endif

  IGB_FAST_INLINE void setCrc(bool enable) {
#if defined(STM32H7)
    if (enable) {
      SET_BIT(p_spi->CFG1, SPI_CFG1_CRCEN);
    } else {
      CLEAR_BIT(p_spi->CFG1, SPI_CFG1_CRCEN);
    }
#else
    if (enable) {
      p_spi->CR1 |= SPI_CR1_CRCEN;
    } else {
      p_spi->CR1 &= ~SPI_CR1_CRCEN;
    }
#endif
  }

#if defined(STM32H7)
  IGB_FAST_INLINE void setCrcWidth(SpiCrcWidth width) {
    MODIFY_REG(p_spi->CFG1, SPI_CFG1_CRCSIZE, as<uint32_t>(width));
  }
#elif defined(STM32F0) || defined(STM32F3)
  IGB_FAST_INLINE void setCrcWidth(SpiCrcWidth width) {
    MODIFY_REG(p_spi->CR1, SPI_CR1_CRCL, as<uint32_t>(width));
  }
#endif

#if !defined(STM32H7)
  IGB_FAST_INLINE void setCrcNext() {
    p_spi->CR1 |= SPI_CR1_CRCNEXT;
  }
#endif

  IGB_FAST_INLINE void setCrcPolynomial(uint32_t poly) {
#if defined(STM32H7)
    WRITE_REG(p_spi->CRCPOLY, poly);
#else
    p_spi->CRCPR = (uint16_t)poly;
#endif
  }

  IGB_FAST_INLINE uint32_t getRxCrc() {
#if defined(STM32H7)
    return (uint32_t)(READ_REG(p_spi->RXCRC));
#else
    return (uint32_t)(p_spi->RXCRCR);
#endif
  }

  IGB_FAST_INLINE uint32_t getTxCrc() {
#if defined(STM32H7)
    return (uint32_t)(READ_REG(p_spi->TXCRC));
#else
    return (uint32_t)(p_spi->TXCRCR);
#endif
  }

  IGB_FAST_INLINE void setNssMode(SpiNssMode mode) {
#if defined(STM32H7)
    MODIFY_REG(p_spi->CFG2, SPI_CFG2_SSM | SPI_CFG2_SSOE, as<uint32_t>(mode));
#else
    MODIFY_REG(p_spi->CR1, SPI_CR1_SSM,  as<uint32_t>(mode));
    MODIFY_REG(p_spi->CR2, SPI_CR2_SSOE, ((uint32_t)(as<uint32_t>(mode) >> 16U)));
#endif
  }

#if defined(STM32H7)
  IGB_FAST_INLINE void setNssPulseMng(bool enable) {
    if (enable) {
      SET_BIT(p_spi->CFG2, SPI_CFG2_SSOM);
    } else {
      CLEAR_BIT(p_spi->CFG2, SPI_CFG2_SSOM);
    }
  }
#elif defined(STM32F0) || defined(STM32F3)
  IGB_FAST_INLINE void setNssPulseMng(bool enable) {
    if (enable) {
      p_spi->CR2 |= SPI_CR2_NSSP;
    } else {
      p_spi->CR2 &= ~SPI_CR2_NSSP;
    }
  }
#endif

#if defined(STM32H7)
  IGB_FAST_INLINE void setNssPolarity(SpiNssPolarity polarity) {
    MODIFY_REG(p_spi->CFG2, SPI_CFG2_SSIOP, as<uint32_t>(polarity));
  }
#endif

  IGB_FAST_INLINE bool isState(SpiState state) {
    return !!(p_spi->SR & as<uint32_t>(state));
  }
  
  IGB_FAST_INLINE void clearState(SpiState state) {
    CLEAR_BIT(p_spi->SR, as<uint32_t>(state));
  }

  IGB_FAST_INLINE void enableInterrupt(SpiInterruptType type) {
#if defined(STM32H7)
    SET_BIT(p_spi->IER, as<uint32_t>(type));
#else
    p_spi->CR2 |= as<uint32_t>(type);
#endif
  }

  IGB_FAST_INLINE void disableInterrupt(SpiInterruptType type) {
#if defined(STM32H7)
    CLEAR_BIT(p_spi->IER, as<uint32_t>(type));
#else
    p_spi->CR2 &= ~(as<uint32_t>(type));
#endif
  }

  IGB_FAST_INLINE void enableDmaReq(SpiDmaReqTarget type) {
#if defined(STM32H7)
    SET_BIT(p_spi->CFG1, as<uint32_t>(type));
#else
    p_spi->CR2 |= as<uint32_t>(type);
#endif
  }

  IGB_FAST_INLINE void disableDmaReq(SpiDmaReqTarget type) {
#if defined(STM32H7)
    CLEAR_BIT(p_spi->CFG1, as<uint32_t>(type));
#else
    p_spi->CR2 &= ~(as<uint32_t>(type));
#endif
  }

#if defined(STM32F0) || defined(STM32F3)
  IGB_FAST_INLINE void setDmaParity(SpiDmaParityTarget target, SpiDmaParityType type) {
    switch (target) {
      case SpiDmaParityTarget::tx:
        MODIFY_REG(p_spi->CR2, SPI_CR2_LDMATX, (as<uint32_t>(type) << SPI_CR2_LDMATX_Pos));
        break;
      case SpiDmaParityTarget::rx:
        MODIFY_REG(p_spi->CR2, SPI_CR2_LDMARX, (as<uint32_t>(type) << SPI_CR2_LDMARX_Pos));
        break;
    }
  }

  IGB_FAST_INLINE uint32_t getRegAddr() {
    return (uint32_t) &(p_spi->DR);
  }
#endif

#if defined(STM32H7)
  IGB_FAST_INLINE void setTransferSize(uint32_t count) {
    MODIFY_REG(p_spi->CR2, SPI_CR2_TSIZE, count);
  }

  IGB_FAST_INLINE void startMasterTransfer() {
    SET_BIT(p_spi->CR1, SPI_CR1_CSTART);
  }
#endif

  IGB_FAST_INLINE uint8_t receiveU8() {
#if defined(STM32H7)
    return (*((__IO uint8_t *)&p_spi->RXDR));
#else
    return (uint8_t)(p_spi->DR);
#endif
  }

  IGB_FAST_INLINE uint8_t receiveU8sync() {
#if defined(STM32H7)
    while (!isState(SpiState::rxPacketAbailable));
    return receiveU8();
#else
    while (!isState(SpiState::rxBufNotEmpty));
    return receiveU8();
#endif
  }

  IGB_FAST_INLINE uint16_t receiveU16() {
#if defined(STM32H7)
    return (uint16_t)(READ_REG(p_spi->RXDR));
#else
    return (uint16_t)(p_spi->DR);
#endif
  }

  IGB_FAST_INLINE uint8_t receiveU16sync() {
#if defined(STM32H7)
    while (!isState(SpiState::rxPacketAbailable));
    return receiveU16();
#else
    while (!isState(SpiState::rxBufNotEmpty));
    return receiveU16();
#endif
  }

  IGB_FAST_INLINE void sendU8(uint8_t data) {
#if defined(STM32H7)
    *((__IO uint8_t *)&p_spi->TXDR) = data;
#else
#if defined (__GNUC__)
    __IO uint8_t *spidr = ((__IO uint8_t *)&p_spi->DR);
    *spidr = data;
#else
    *((__IO uint8_t *)&p_spi->DR) = data;
#endif /* __GNUC__ */
#endif
  }

  IGB_FAST_INLINE void sendU8sync(uint8_t data) {
#if defined(STM32H7)
    disable();
    setTransferSize(1);
    enable();
    startMasterTransfer();
    while (!isState(SpiState::txPacketAbailable));
    sendU8(data);
    while (!isState(SpiState::endOfTransfer));
    SET_BIT(p_spi->IFCR, SPI_IFCR_EOTC);
    SET_BIT(p_spi->IFCR, SPI_IFCR_TXTFC);
    disable();
    p_spi->IER &= (~(SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));
    CLEAR_BIT(p_spi->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
#else
    __IO uint16_t tmp = transferU8sync(data);
#endif
  }

  IGB_FAST_INLINE void sendBufU8sync(uint8_t* buffer, size_t size) {
#if defined(STM32H7)
    disable();
    setTransferSize(size);
    enable();
    startMasterTransfer();
    for (size_t i = 0; i < size; ++i) {
      while (!isState(SpiState::txPacketAbailable));
      sendU8(buffer[i]);
    }
    while (!isState(SpiState::endOfTransfer));
    SET_BIT(p_spi->IFCR, SPI_IFCR_EOTC);
    SET_BIT(p_spi->IFCR, SPI_IFCR_TXTFC);
    disable();
    p_spi->IER &= (~(SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));
    CLEAR_BIT(p_spi->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
#else
    for (size_t i = 0; i < size; ++i) {
      sendU8sync(buffer[i]);
    }
#endif
  }

  IGB_FAST_INLINE void sendU16(uint16_t data) {
#if defined(STM32H7)
#if defined (__GNUC__)
  __IO uint16_t *spitxdr = ((__IO uint16_t *)&p_spi->TXDR);
  *spitxdr = data;
#else
  p_spi->TXDR = data;
#endif
#else
#if defined (__GNUC__)
  __IO uint16_t *spidr = ((__IO uint16_t *)&p_spi->DR);
  *spidr = data;
#else
  p_spi->DR = data;
#endif /* __GNUC__ */
#endif /* defined(STM32H7) */
  }

  IGB_FAST_INLINE uint8_t transferU8sync(uint8_t data) {
#if defined(STM32H7)
    disable();
    setTransferSize(1);
    enable();
    startMasterTransfer();
    while (!isState(SpiState::txPacketAbailable));
    sendU8(data);
    while (!isState(SpiState::rxPacketAbailable));
    uint8_t result = receiveU8();
    while (!isState(SpiState::endOfTransfer));
    SET_BIT(p_spi->IFCR, SPI_IFCR_EOTC);
    SET_BIT(p_spi->IFCR, SPI_IFCR_TXTFC);
    disable();
    p_spi->IER &= (~(SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));
    CLEAR_BIT(p_spi->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
    return result;
#else
    while (isState(SpiState::busy));
    while (!isState(SpiState::txBufEmpty));
    sendU8(data);
    while (!isState(SpiState::rxBufNotEmpty));
     return receiveU8();
#endif
  }

  static IGB_FAST_INLINE Spi newSpi(const SpiType spi_type) {
    return Spi {
      .type = spi_type,
      .p_spi = STM32_PERIPH_INFO.spi[as<uint8_t>(spi_type)].p_spi
    };
  }

  IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    auto periph_type = as_periph_type(type);
    if (!periph_type) { return; }

    auto result = get_af_idx(periph_type.value(), pin_type);
    if (!result) { return; }

    GpioType gpio_type = extract_gpio_type(pin_type);
    GpioPin pin = GpioPin::newPin(pin_type);
    pin.setMode(GpioMode::alternate);
    pin.setPullMode(GpioPullMode::no);
    pin.setSpeedMode(GpioSpeedMode::high);
    pin.setOutputMode(GpioOutputMode::pushpull);
    pin.setAlternateFunc(result.value());
    pin.enable();
  }

  IGB_FAST_INLINE void initDefault() {
#if defined(STM32H7)
    setNssPolarity(SpiNssPolarity::low);
    setStandard(SpiStandard::motorola);
    setFifoThreshold(SpiFifoThreshold::_1data);
    setNssPulseMng(false);

    setCrc(false);
    setDataWidth(SpiDataWidth::_8bit);

    SET_BIT(p_spi->CR1, SPI_CR1_SSI); // avoid a MODF Error

    setNssMode(SpiNssMode::soft);
    setClockPolarity(SpiClockPolarity::low);
    setClockPhase(SpiClockPhase::oneEdge);
    setTransBitOrder(SpiBitOrder::msbFirst);
    setMode(SpiMode::master);
    setTransDir(SpiTransDir::fullDuplex);
#elif defined(STM32F0) || defined(STM32F3)
    setTransDir(SpiTransDir::fullDuplex);
    setMode(SpiMode::master);
    setDataWidth(SpiDataWidth::_8bit);
    setClockPolarity(SpiClockPolarity::low);
    setClockPhase(SpiClockPhase::oneEdge);
    setNssMode(SpiNssMode::soft);
    setTransBitOrder(SpiBitOrder::msbFirst);
    setCrc(false);
    setCrcPolynomial(7);
    setStandard(SpiStandard::motorola);
    setRxFifoThreshold(SpiFifoThreshold::quarter);
    setNssPulseMng(true);
#endif
  }

  IGB_FAST_INLINE void prepareSpiMaster(GpioPinType mosi_pin, GpioPinType miso_pin, GpioPinType sck_pin, SpiBaudratePrescaler prescaler) {
    const auto& spi_info = STM32_PERIPH_INFO.spi[as<uint8_t>(type)];
    spi_info.bus.enableBusClock();
    prepareGpio(mosi_pin);
    prepareGpio(miso_pin);
    prepareGpio(sck_pin);

    setBaudratePrescaler(prescaler);
    initDefault();
    enable();
  }

  IGB_FAST_INLINE void prepareSpiMasterOutOnly(GpioPinType mosi_pin, GpioPinType sck_pin, SpiBaudratePrescaler prescaler) {
    const auto& spi_info = STM32_PERIPH_INFO.spi[as<uint8_t>(type)];
    spi_info.bus.enableBusClock();
    prepareGpio(mosi_pin);
    prepareGpio(sck_pin);

#if defined(STM32H7)
    setNssPolarity(SpiNssPolarity::low);
    setStandard(SpiStandard::motorola);
    setFifoThreshold(SpiFifoThreshold::_1data);
    setNssPulseMng(false);

    setBaudratePrescaler(prescaler);
    setCrc(false);
    setDataWidth(SpiDataWidth::_8bit);

    SET_BIT(p_spi->CR1, SPI_CR1_SSI); // avoid a MODF Error

    setNssMode(SpiNssMode::soft);
    setClockPolarity(SpiClockPolarity::low);
    setClockPhase(SpiClockPhase::oneEdge);
    setTransBitOrder(SpiBitOrder::msbFirst);
    setMode(SpiMode::master);
    setTransDir(SpiTransDir::simplexTx);
#if defined(SPI_I2SCFGR_I2SMOD)
    CLEAR_BIT(p_spi->I2SCFGR, SPI_I2SCFGR_I2SMOD);
#endif
    CLEAR_BIT(p_spi->CR1, SPI_CR1_TCRCINI);
    CLEAR_BIT(p_spi->CR1, SPI_CR1_RCRCINI);
    CLEAR_BIT(p_spi->CFG2, SPI_CFG2_AFCNTR);
    CLEAR_BIT(p_spi->CFG2, SPI_CFG2_IOSWP);
    MODIFY_REG(p_spi->CFG2, 0xFFUL, 0x00UL);
#else
    setBaudratePrescaler(prescaler);
    initDefault();
#endif
    enable();
  }

  IGB_FAST_INLINE void prepareSpiMasterOutOnlyHardSS(GpioPinType mosi_pin, GpioPinType sck_pin, GpioPinType cs_pin, SpiBaudratePrescaler prescaler) {
    const auto& spi_info = STM32_PERIPH_INFO.spi[as<uint8_t>(type)];
    spi_info.bus.enableBusClock();
    prepareGpio(mosi_pin);
    prepareGpio(sck_pin);
    prepareGpio(cs_pin);

#if defined(STM32H7)
    setNssPolarity(SpiNssPolarity::low);
    setStandard(SpiStandard::motorola);
    setFifoThreshold(SpiFifoThreshold::_1data);
    setNssPulseMng(true);

    setBaudratePrescaler(prescaler);
    setCrc(false);
    setDataWidth(SpiDataWidth::_8bit);

    SET_BIT(p_spi->CR1, SPI_CR1_SSI); // avoid a MODF Error

    setNssMode(SpiNssMode::hardOutput);
    setClockPolarity(SpiClockPolarity::low);
    setClockPhase(SpiClockPhase::oneEdge);
    setTransBitOrder(SpiBitOrder::msbFirst);
    setMode(SpiMode::master);
    setTransDir(SpiTransDir::simplexTx);
#if defined(SPI_I2SCFGR_I2SMOD)
    CLEAR_BIT(p_spi->I2SCFGR, SPI_I2SCFGR_I2SMOD);
#endif
    CLEAR_BIT(p_spi->CR1, SPI_CR1_TCRCINI);
    CLEAR_BIT(p_spi->CR1, SPI_CR1_RCRCINI);
    CLEAR_BIT(p_spi->CFG2, SPI_CFG2_AFCNTR);
    CLEAR_BIT(p_spi->CFG2, SPI_CFG2_IOSWP);
    MODIFY_REG(p_spi->CFG2, 0xFFUL, 0x00UL);
#else
    setBaudratePrescaler(prescaler);
    initDefault();
#endif
    enable();
  }

};

}
}

#endif /* IGB_STM32_PERIPH_SPI_H */
