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
  SLAVE  = 0,
#if defined(STM32H7)
  MASTER = SPI_CFG2_MASTER,
#else
  MASTER = (SPI_CR1_MSTR | SPI_CR1_SSI),
#endif
};

enum class SpiStandard : uint32_t {
  MOTOROLA = 0,
#if defined(STM32H7)
  TI = SPI_CFG2_SP_0,
#else
  TI = SPI_CR2_FRF,
#endif
};

enum class SpiClockPhase : uint32_t {
  ONE_EDGE = 0,
#if defined(STM32H7)
  TWO_EDGE = SPI_CFG2_CPHA,
#else
  TWO_EDGE = SPI_CR1_CPHA,
#endif
};

enum class SpiClockPolarity : uint32_t {
  LOW = 0,
#if defined(STM32H7)
  HIGH = SPI_CFG2_CPOL,
#else
  HIGH = SPI_CR1_CPOL,
#endif
};

enum class SpiBaudratePrescaler : uint32_t {
  DIV2 = 0,
#if defined(STM32H7)
  DIV4 = SPI_CFG1_MBR_0,
  DIV8 = SPI_CFG1_MBR_1,
  DIV16 = SPI_CFG1_MBR_1 | SPI_CFG1_MBR_0,
  DIV32 = SPI_CFG1_MBR_2,
  DIV64 = SPI_CFG1_MBR_2 | SPI_CFG1_MBR_0,
  DIV128 = SPI_CFG1_MBR_2 | SPI_CFG1_MBR_1,
  DIV256 = SPI_CFG1_MBR_2 | SPI_CFG1_MBR_1 | SPI_CFG1_MBR_0,
#else
  DIV4 = SPI_CR1_BR_0,
  DIV8 = SPI_CR1_BR_1,
  DIV16 = SPI_CR1_BR_1 | SPI_CR1_BR_0,
  DIV32 = SPI_CR1_BR_2,
  DIV64 = SPI_CR1_BR_2 | SPI_CR1_BR_0,
  DIV128 = SPI_CR1_BR_2 | SPI_CR1_BR_1,
  DIV256 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,
#endif
};

enum class SpiBitOrder : uint32_t {
  MSB_FIRST = 0,
#if defined(STM32H7)
  LSB_FIRST = SPI_CFG2_LSBFRST,
#else
  LSB_FIRST = SPI_CR1_LSBFIRST,
#endif
};

enum class SpiTransDir : uint32_t {
  FULL_DUPLEX = 0,
#if defined(STM32H7)
  SIMPLEX_TX = SPI_CFG2_COMM_0,
  SIMPLEX_RX = SPI_CFG2_COMM_1,
  HALF_DUPLEX_RX = SPI_CFG2_COMM_0 | SPI_CFG2_COMM_1,
  HALF_DUPLEX_TX = SPI_CFG2_COMM_0 | SPI_CFG2_COMM_1 | SPI_CR1_HDDIR,
#else
  SIMPLEX_RX  = SPI_CR1_RXONLY,
  HALF_DUPLEX_RX = SPI_CR1_BIDIMODE,
  HALF_DUPLEX_TX = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE,
#endif
};

enum class SpiDataWidth : uint32_t {
#if defined(STM32H7)
  _4BIT = SPI_CFG1_DSIZE_0 | SPI_CFG1_DSIZE_1,
  _5BIT = SPI_CFG1_DSIZE_2,
  _6BIT = SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_0,
  _7BIT = SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1,
  _8BIT = SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
  _9BIT = SPI_CFG1_DSIZE_3,
  _10BIT = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_0,
  _11BIT = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_1,
  _12BIT = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
  _13BIT = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2,
  _14BIT = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_0,
  _15BIT = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1,
  _16BIT = SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
  _17BIT = SPI_CFG1_DSIZE_4,
  _18BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_0,
  _19BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_1,
  _20BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_0 | SPI_CFG1_DSIZE_1,
  _21BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_2,
  _22BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_0,
  _23BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1,
  _24BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
  _25BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3,
  _26BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_0,
  _27BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_1,
  _28BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
  _29BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2,
  _30BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_0,
  _31BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1,
  _32BIT = SPI_CFG1_DSIZE_4 | SPI_CFG1_DSIZE_3 | SPI_CFG1_DSIZE_2 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_0,
#elif defined(STM32F4)
  _8BIT = 0,
  _16BIT = SPI_CR1_DFF,
#else
  _4BIT = SPI_CR2_DS_0 | SPI_CR2_DS_1,
  _5BIT = SPI_CR2_DS_2,
  _6BIT = SPI_CR2_DS_2 | SPI_CR2_DS_0,
  _7BIT = SPI_CR2_DS_2 | SPI_CR2_DS_1,
  _8BIT = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0,
  _9BIT = SPI_CR2_DS_3,
  _10BIT = SPI_CR2_DS_3 | SPI_CR2_DS_0,
  _11BIT = SPI_CR2_DS_3 | SPI_CR2_DS_1,
  _12BIT = SPI_CR2_DS_3 | SPI_CR2_DS_1 | SPI_CR2_DS_0,
  _13BIT = SPI_CR2_DS_3 | SPI_CR2_DS_2,
  _14BIT = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_0,
  _15BIT = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1,
  _16BIT = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0,
#endif
};

#if defined(STM32H7)
enum class SpiFifoThreshold : uint32_t {
  _1DATA = 0,
  _2DATA = SPI_CFG1_FTHLV_0,
  _3DATA = SPI_CFG1_FTHLV_1,
  _4DATA = SPI_CFG1_FTHLV_0 | SPI_CFG1_FTHLV_1,
  _5DATA = SPI_CFG1_FTHLV_2,
  _6DATA = SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_0,
  _7DATA = SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_1,
  _8DATA = SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_1 | SPI_CFG1_FTHLV_0,
  _9DATA = SPI_CFG1_FTHLV_3,
  _10DATA = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_0,
  _11DATA = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_1,
  _12DATA = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_1 | SPI_CFG1_FTHLV_0,
  _13DATA = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_2,
  _14DATA = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_0,
  _15DATA = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_1,
  _16DATA = SPI_CFG1_FTHLV_3 | SPI_CFG1_FTHLV_2 | SPI_CFG1_FTHLV_1 | SPI_CFG1_FTHLV_0,
};
#elif defined(STM32F0) || defined(STM32F3)
enum class SpiFifoThreshold : uint32_t {
  HALF = 0,
  QUARTER = SPI_CR2_FRXTH,
};
#endif

#if defined(STM32H7)
enum class SpiCrcWidth : uint32_t {
  _4BIT = SPI_CFG1_CRCSIZE_0 | SPI_CFG1_CRCSIZE_1,
  _5BIT = SPI_CFG1_CRCSIZE_2,
  _6BIT = SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_0,
  _7BIT = SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1,
  _8BIT = SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0,
  _9BIT = SPI_CFG1_CRCSIZE_3,
  _10BIT = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_0,
  _11BIT = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_1,
  _12BIT = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0,
  _13BIT = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2,
  _14BIT = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_0,
  _15BIT = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1,
  _16BIT = SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0,
  _17BIT = (SPI_CFG1_CRCSIZE_4),
  _18BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_0),
  _19BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_1),
  _20BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_0 | SPI_CFG1_CRCSIZE_1),
  _21BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_2),
  _22BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_0),
  _23BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1),
  _24BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0),
  _25BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3),
  _26BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_0),
  _27BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_1),
  _28BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0),
  _29BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2),
  _30BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_0),
  _31BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1),
  _32BIT = (SPI_CFG1_CRCSIZE_4 | SPI_CFG1_CRCSIZE_3 | SPI_CFG1_CRCSIZE_2 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_0),
};
#elif defined(STM32F0) || defined(STM32F3)
enum class SpiCrcWidth : uint32_t {
  _8BIT = 0,
  _16BIT = SPI_CR1_CRCL,
};
#endif

enum class SpiNssMode : uint32_t {
#if defined(STM32H7)
  HARD_INPUT  = 0,
  HARD_OUTPUT = SPI_CFG2_SSOE,
  SOFT        = SPI_CFG2_SSM,
#else
  HARD_INPUT  = 0,
  HARD_OUTPUT = SPI_CR1_SSM,
  SOFT        = SPI_CR1_SSM,
#endif
};

enum class SpiState : uint32_t {
#if defined(STM32H7)
  RX_WORD_NOT_EMPTY   = SPI_SR_RXWNE,
  RX_PACKET_ABAILABLE = SPI_SR_RXP,
  TX_PACKET_ABAILABLE = SPI_SR_TXP,
  DX_PACKET_ABAILABLE = SPI_SR_DXP,
  END_OF_TRANSFER     = SPI_SR_EOT,
  TX_TRANSFER_FULL    = SPI_SR_TXTF,
  UNDERRUN            = SPI_SR_UDR,
  CRC_ERROR           = SPI_SR_CRCE,
  MODE_FAULT          = SPI_SR_MODF,
  OVERRUN             = SPI_SR_OVR,
  FRAME_FORMAT_ERROR  = SPI_SR_TIFRE,
  DATA_RELOADED       = SPI_SR_TSERF,
  SUSPEND_DONE        = SPI_SR_SUSP,
#else
  RX_BUF_NOT_EMPTY   = SPI_SR_RXNE,
  TX_BUF_EMPTY       = SPI_SR_TXE,
  BUSY               = SPI_SR_BSY,
  CRC_ERROR          = SPI_SR_CRCERR,
  MODE_FAULT         = SPI_SR_MODF,
  OVERRUN            = SPI_SR_OVR,
  FRAME_FORMAT_ERROR = SPI_SR_FRE,
#endif
};

enum class SpiInterruptType : uint32_t {
#if defined(STM32H7)
  RX_PACKET_ABAILABLE = SPI_IER_RXPIE,
  TX_PACKET_ABAILABLE = SPI_IER_TXPIE,
  DX_PACKET_ABAILABLE = SPI_IER_DXPIE,
  END_OF_TRANSFER     = SPI_IER_EOTIE,
  TX_TRANSFER_FULL    = SPI_IER_TXTFIE,
  UNDERRUN            = SPI_IER_UDRIE,
  OVERRUN             = SPI_IER_OVRIE,
  CRC_ERROR           = SPI_IER_CRCEIE,
  FRAME_FORMAT_ERROR  = SPI_IER_TIFREIE,
  MODE_FAULT          = SPI_IER_MODFIE,
  DATA_RELOADED       = SPI_IER_TSERFIE,
#else
  RX_BUF_NOT_EMPTY = SPI_CR2_RXNEIE,
  TX_BUF_EMPTY     = SPI_CR2_TXEIE,
  ERROR            = SPI_CR2_ERRIE,
#endif
};

enum class SpiDmaReqTarget : uint32_t {
#if defined(STM32H7)
  RX = SPI_CFG1_RXDMAEN,
  TX = SPI_CFG1_TXDMAEN,
#else
  RX = SPI_CR2_RXDMAEN,
  TX = SPI_CR2_TXDMAEN,
#endif
};

#if !defined(STM32H7)
enum class SpiDmaParityTarget : uint32_t {
  RX = SPI_CR2_RXDMAEN,
  TX = SPI_CR2_TXDMAEN,
};

enum class SpiDmaParityType : uint32_t {
  EVEN = 0,
  ODD  = 1,
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
      case SpiDmaParityTarget::TX:
        MODIFY_REG(p_spi->CR2, SPI_CR2_LDMATX, (as<uint32_t>(type) << SPI_CR2_LDMATX_Pos));
        break;
      case SpiDmaParityTarget::RX:
        MODIFY_REG(p_spi->CR2, SPI_CR2_LDMARX, (as<uint32_t>(type) << SPI_CR2_LDMARX_Pos));
        break;
    }
  }

  IGB_FAST_INLINE uint32_t getRegAddr() {
    return (uint32_t) &(p_spi->DR);
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
    while (!isState(SpiState::RX_PACKET_ABAILABLE));
    return receiveU8();
#else
    while (!isState(SpiState::RX_BUF_NOT_EMPTY));
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
    while (!isState(SpiState::RX_PACKET_ABAILABLE));
    return receiveU16();
#else
    while (!isState(SpiState::RX_BUF_NOT_EMPTY));
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
    while (!isState(SpiState::TX_PACKET_ABAILABLE));
    sendU8(data);
#else
    while (isState(SpiState::BUSY));
    while (!isState(SpiState::TX_BUF_EMPTY));
    sendU8(data);
#endif /* __GNUC__ */
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

  IGB_FAST_INLINE void sendU16sync(uint8_t data) {
#if defined(STM32H7)
    while (!isState(SpiState::TX_PACKET_ABAILABLE));
    sendU16(data);
#else
    while (isState(SpiState::BUSY));
    while (!isState(SpiState::TX_BUF_EMPTY));
    sendU16(data);
#endif /* __GNUC__ */
  }

  IGB_FAST_INLINE uint8_t transferU8sync(uint8_t data) {
#if defined(STM32H7)
    // TODO: 要検証
    //while (isState(SpiState::TX_TRANSFER_FULL));
    while (!isState(SpiState::TX_PACKET_ABAILABLE));
    sendU8(data);
    while (!isState(SpiState::RX_PACKET_ABAILABLE));
    return receiveU8();
#else
    while (isState(SpiState::BUSY));
    while (!isState(SpiState::TX_BUF_EMPTY));
    sendU8(data);
    while (!isState(SpiState::RX_BUF_NOT_EMPTY));
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
    pin.setMode(GpioMode::ALTERNATE);
    pin.setPullMode(GpioPullMode::NO);
    pin.setSpeedMode(GpioSpeedMode::HIGH);
    pin.setOutputMode(GpioOutputMode::PUSHPULL);
    pin.setAlternateFunc(result.value());
    pin.enable();
  }

  IGB_FAST_INLINE void initDefault() {
    setTransDir(SpiTransDir::FULL_DUPLEX);
    setMode(SpiMode::MASTER);
    setDataWidth(SpiDataWidth::_8BIT);
    setClockPolarity(SpiClockPolarity::LOW);
    setClockPhase(SpiClockPhase::ONE_EDGE);
    setNssMode(SpiNssMode::SOFT);
    setTransBitOrder(SpiBitOrder::MSB_FIRST);
    setCrc(false);
    setCrcPolynomial(7);
    setStandard(SpiStandard::MOTOROLA);

#if defined(STM32H7)
    setFifoThreshold(SpiFifoThreshold::_1DATA);
    setNssPulseMng(true);
#elif defined(STM32F0) || defined(STM32F3)
    setRxFifoThreshold(SpiFifoThreshold::QUARTER);
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

    setBaudratePrescaler(prescaler);
#if defined(STM32H7)
    setTransDir(SpiTransDir::SIMPLEX_TX);
    setMode(SpiMode::MASTER);
    setDataWidth(SpiDataWidth::_8BIT);
    setClockPolarity(SpiClockPolarity::LOW);
    setClockPhase(SpiClockPhase::ONE_EDGE);
    setNssMode(SpiNssMode::SOFT);
    setTransBitOrder(SpiBitOrder::MSB_FIRST);
    setCrc(false);
    setCrcPolynomial(7);
    setStandard(SpiStandard::MOTOROLA);
    setFifoThreshold(SpiFifoThreshold::_1DATA);
    setNssPulseMng(true);
#else
    initDefault();
#endif
    enable();
  }

};

}
}

#endif /* IGB_STM32_PERIPH_SPI_H */
