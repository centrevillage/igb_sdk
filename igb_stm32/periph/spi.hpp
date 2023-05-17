#ifndef IGB_STM32_PERIPH_SPI_H
#define IGB_STM32_PERIPH_SPI_H

#include <igb_stm32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/rcc.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>

namespace igb {
namespace stm32 {

#define IGB_SPI ((SPI_TypeDef*)addr)
#define IGB_SPI_REG_ADDR(member) (addr + offsetof(SPI_TypeDef, member))
#define IGB_SPI_REG(member) ((SPI_TypeDef*)IGB_SPI_REG_ADDR(member))

#if defined(STM32H7)
# define SPI_IT_RXP    SPI_IER_RXPIE
# define SPI_IT_TXP    SPI_IER_TXPIE
# define SPI_IT_DXP    SPI_IER_DXPIE
# define SPI_IT_EOT    SPI_IER_EOTIE
# define SPI_IT_TXTF   SPI_IER_TXTFIE
# define SPI_IT_UDR    SPI_IER_UDRIE
# define SPI_IT_OVR    SPI_IER_OVRIE
# define SPI_IT_CRCERR SPI_IER_CRCEIE
# define SPI_IT_FRE    SPI_IER_TIFREIE
# define SPI_IT_MODF   SPI_IER_MODFIE
# define SPI_IT_TSERF  SPI_IER_TSERFIE
#endif

#if defined(STM32H7)
enum class SpiMode : uint32_t {
  slave  = 0,
  master = SPI_CFG2_MASTER,
};
enum class SpiStandard : uint32_t {
  motorola = 0,
  ti = SPI_CFG2_SP_0,
};
enum class SpiClockPhase : uint32_t {
  oneEdge = 0,
  twoEdge = SPI_CFG2_CPHA,
};
enum class SpiClockPolarity : uint32_t {
  low = 0,
  high = SPI_CFG2_CPOL,
};
enum class SpiBaudratePrescaler : uint32_t {
  div2 = 0,
  div4 = SPI_CFG1_MBR_0,
  div8 = SPI_CFG1_MBR_1,
  div16 = SPI_CFG1_MBR_1 | SPI_CFG1_MBR_0,
  div32 = SPI_CFG1_MBR_2,
  div64 = SPI_CFG1_MBR_2 | SPI_CFG1_MBR_0,
  div128 = SPI_CFG1_MBR_2 | SPI_CFG1_MBR_1,
  div256 = SPI_CFG1_MBR_2 | SPI_CFG1_MBR_1 | SPI_CFG1_MBR_0,
};
enum class SpiBitOrder : uint32_t {
  msbFirst = 0,
  lsbFirst = SPI_CFG2_LSBFRST,
};
enum class SpiTransDir : uint32_t {
  fullDuplex = 0,
  simplexTx = SPI_CFG2_COMM_0,
  simplexRx = SPI_CFG2_COMM_1,
  halfDuplexRx = SPI_CFG2_COMM_0 | SPI_CFG2_COMM_1,
  halfDuplexTx = SPI_CFG2_COMM_0 | SPI_CFG2_COMM_1 | SPI_CR1_HDDIR,
};
enum class SpiNssMode : uint32_t {
  hardInput  = 0,
  hardOutput = SPI_CFG2_SSOE,
  soft       = SPI_CFG2_SSM,
};
enum class SpiNssPolarity : uint32_t {
  low = 0,
  high = SPI_CFG2_SSIOP,
};
#else
enum class SpiMode : uint32_t {
  slave  = 0,
  master = (SPI_CR1_MSTR | SPI_CR1_SSI),
};
enum class SpiStandard : uint32_t {
  motorola = 0,
  ti = SPI_CR2_FRF,
};
enum class SpiClockPhase : uint32_t {
  oneEdge = 0,
  twoEdge = SPI_CR1_CPHA,
};
enum class SpiClockPolarity : uint32_t {
  low = 0,
  high = SPI_CR1_CPOL,
};
enum class SpiBaudratePrescaler : uint32_t {
  div2 = 0,
  div4 = SPI_CR1_BR_0,
  div8 = SPI_CR1_BR_1,
  div16 = SPI_CR1_BR_1 | SPI_CR1_BR_0,
  div32 = SPI_CR1_BR_2,
  div64 = SPI_CR1_BR_2 | SPI_CR1_BR_0,
  div128 = SPI_CR1_BR_2 | SPI_CR1_BR_1,
  div256 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,
};
enum class SpiBitOrder : uint32_t {
  msbFirst = 0,
  lsbFirst = SPI_CR1_LSBFIRST,
};
enum class SpiTransDir : uint32_t {
  fullDuplex = 0,
  simplexRx  = SPI_CR1_RXONLY,
  halfDuplexRx = SPI_CR1_BIDIMODE,
  halfDuplexTx = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE,
};
enum class SpiNssMode : uint32_t {
  hardInput  = 0,
  hardOutput = SPI_CR1_SSM,
  soft       = SPI_CR1_SSM,
};
#endif

#if defined(STM32H7)
enum class SpiDataWidth : uint32_t {
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
};
#elif defined(STM32F4)
enum class SpiDataWidth : uint32_t {
  _8bit = 0,
  _16bit = SPI_CR1_DFF,
};
#else
enum class SpiDataWidth : uint32_t {
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
};
#endif

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

template<SpiType SPI_TYPE, GpioPinType MOSI_PIN, GpioPinType MISO_PIN, GpioPinType SCK_PIN>
struct Spi {
  constexpr static auto type = SPI_TYPE;
  constexpr static auto mosi_pin = MOSI_PIN;
  constexpr static auto miso_pin = MISO_PIN;
  constexpr static auto sck_pin = SCK_PIN;
  constexpr static auto addr = STM32_PERIPH_INFO.spi[to_idx(type)].addr;

  IGB_FAST_INLINE void enable() { IGB_SPI->CR1 = IGB_SPI->CR1 | SPI_CR1_SPE; }
  IGB_FAST_INLINE void disable() { IGB_SPI->CR1 = IGB_SPI->CR1 & ~SPI_CR1_SPE; }

#if defined(STM32H7)
  RegEnum<IGB_SPI_REG_ADDR(CFG2), SPI_CFG2_MASTER, SpiMode> mode;
  RegEnum<IGB_SPI_REG_ADDR(CFG2), SPI_CFG2_SP, SpiStandard> standard;
  RegEnum<IGB_SPI_REG_ADDR(CFG2), SPI_CFG2_CPHA, SpiClockPhase> clockPhase;
  RegEnum<IGB_SPI_REG_ADDR(CFG2), SPI_CFG2_CPOL, SpiClockPolarity> clockPolarity;
  RegEnum<IGB_SPI_REG_ADDR(CFG1), SPI_CFG1_MBR, SpiBaudratePrescaler> baudratePrescaler;
  RegEnum<IGB_SPI_REG_ADDR(CFG2), SPI_CFG2_LSBFRST, SpiBitOrder> transBitOrder;
  struct SpiTransDirInfo {
    IGB_FAST_INLINE uint32_t operator()() {
      return (IGB_SPI->CR1 & SPI_CR1_HDDIR) | (IGB_SPI->CFG2 & SPI_CFG2_COMM);
    }
    IGB_FAST_INLINE void operator()(SpiTransDir dir) {
      IGB_MODIFY_REG(IGB_SPI->CR1, SPI_CR1_HDDIR,  as<uint32_t>(dir) & SPI_CR1_HDDIR);
      IGB_MODIFY_REG(IGB_SPI->CFG2, SPI_CFG2_COMM, as<uint32_t>(dir) & SPI_CFG2_COMM);
    }
  };
  SpiTransDirInfo transDir;
#else
  RegEnum<IGB_SPI_REG_ADDR(CR1), SPI_CR1_MSTR | SPI_CR1_SSI, SpiMode> mode;
  RegEnum<IGB_SPI_REG_ADDR(CR2), SPI_CR2_FRF, SpiStandard> standard;
  RegEnum<IGB_SPI_REG_ADDR(CR1), SPI_CR1_CPHA, SpiClockPhase> clockPhase;
  RegEnum<IGB_SPI_REG_ADDR(CR1), SPI_CR1_CPOL, SpiClockPolarity> clockPolarity;
  RegEnum<IGB_SPI_REG_ADDR(CR1), SPI_CR1_BR, SpiBaudratePrescaler> baudratePrescaler;
  RegEnum<IGB_SPI_REG_ADDR(CR1), SPI_CR1_LSBFIRST, SpiBitOrder> transBitOrder;
  RegEnum<IGB_SPI_REG_ADDR(CR1), SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE, SpiTransDir> transDir;
#endif

#if defined(STM32H7)
  RegEnum<IGB_SPI_REG_ADDR(CFG1), SPI_CFG1_DSIZE, SpiDataWidth> dataWidth;
#elif defined(STM32F4)
  RegEnum<IGB_SPI_REG_ADDR(CR1), SPI_CR1_DFF, SpiDataWidth> dataWidth;
#else
  RegEnum<IGB_SPI_REG_ADDR(CR2), SPI_CR2_DS, SpiDataWidth> dataWidth;
#endif

#if defined(STM32H7)
  RegEnum<IGB_SPI_REG_ADDR(CFG1), SPI_CFG1_FTHLV, SpiFifoThreshold> fifoThreshold;
#elif defined(STM32F0) || defined(STM32F3)
  RegEnum<IGB_SPI_REG_ADDR(CR2), SPI_CR2_FRXTH, SpiFifoThreshold> rxFifoThreshold;
#endif

#if defined(STM32H7)
  RegFlag<IGB_SPI_REG_ADDR(CFG1), SPI_CFG1_CRCEN> crc;
#else
  RegFlag<IGB_SPI_REG_ADDR(CR1), SPI_CR1_CRCEN> crc;
#endif

#if defined(STM32H7)
  RegEnum<IGB_SPI_REG_ADDR(CFG1), SPI_CFG1_CRCSIZE, SpiCrcWidth> crcWidth;
#elif defined(STM32F0) || defined(STM32F3)
  RegEnum<IGB_SPI_REG_ADDR(CR1), SPI_CR1_CRCL, SpiCrcWidth> crcWidth;
#endif

#if !defined(STM32H7)
  RegFlag<IGB_SPI_REG_ADDR(CR1), SPI_CR1_CRCNEXT> crcNext;
#endif

#if defined(STM32H7)
  Reg<IGB_SPI_REG_ADDR(CRCPOLY)> crcPolynomial;
#else
  Reg<IGB_SPI_REG_ADDR(CRCPR)> crcPolynomial;
#endif

#if defined(STM32H7)
  Reg<IGB_SPI_REG_ADDR(RXCRC)> rxCrc;
#else
  Reg<IGB_SPI_REG_ADDR(RXCRCR)> rxCrc;
#endif

#if defined(STM32H7)
  Reg<IGB_SPI_REG_ADDR(TXCRC)> txCrc;
#else
  Reg<IGB_SPI_REG_ADDR(TXCRCR)> txCrc;
#endif

#if defined(STM32H7)
  RegEnum<IGB_SPI_REG_ADDR(CFG2), SPI_CFG2_SSM | SPI_CFG2_SSOE, SpiNssMode> nssMode;
#else
  struct SpiNssModeInfo {
    IGB_FAST_INLINE uint32_t operator()() {
      return (IGB_SPI->CR1 & SPI_CR1_SSM) | ((IGB_SPI->CR2 & SPI_CR2_SSOE) << 16U);
    };
    IGB_FAST_INLINE void operator()(SpiNssMode mode) {
      IGB_MODIFY_REG(IGB_SPI->CR1, SPI_CR1_SSM,  as<uint32_t>(mode));
      IGB_MODIFY_REG(IGB_SPI->CR2, SPI_CR2_SSOE, ((uint32_t)(as<uint32_t>(mode) >> 16U)));
    };
  };
  SpiNssModeInfo nssMode;
#endif

#if defined(STM32H7)
  RegFlag<IGB_SPI_REG_ADDR(CFG2), SPI_CFG2_SSOM> nssPulseMng;
#elif defined(STM32F0) || defined(STM32F3)
  RegFlag<IGB_SPI_REG_ADDR(CR2), SPI_CR2_NSSP> nssPulseMng;
#endif

#if defined(STM32H7)
  RegEnum<IGB_SPI_REG_ADDR(CFG2), SPI_CFG2_SSIOP, SpiNssPolarity> nssPolarity;
#endif

  IGB_FAST_INLINE bool is(SpiState state) {
    return !!(IGB_SPI->SR & as<uint32_t>(state));
  }
  
  IGB_FAST_INLINE void clear(SpiState state) {
    IGB_CLEAR_BIT(IGB_SPI->SR, as<uint32_t>(state));
  }

#if defined(STM32H7)
  enum class InterruptType : uint32_t {
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
  };
#else
  enum class InterruptType : uint32_t {
    rxBufNotEmpty     = SPI_CR2_RXNEIE,
    txBufEmpty        = SPI_CR2_TXEIE,
    error             = SPI_CR2_ERRIE,
  };
#endif

  IGB_FAST_INLINE void enable(InterruptType type) {
#if defined(STM32H7)
    IGB_SET_BIT(IGB_SPI->IER, as<uint32_t>(type));
#else
    IGB_SPI->CR2 = IGB_SPI->CR2 | as<uint32_t>(type);
#endif
  }

  IGB_FAST_INLINE void disable(InterruptType type) {
#if defined(STM32H7)
    IGB_CLEAR_BIT(IGB_SPI->IER, as<uint32_t>(type));
#else
    IGB_SPI->CR2 = IGB_SPI->CR2 & ~(as<uint32_t>(type));
#endif
  }

  enum class DmaReqType : uint32_t {
#if defined(STM32H7)
    rx = SPI_CFG1_RXDMAEN,
    tx = SPI_CFG1_TXDMAEN,
#else
    rx = SPI_CR2_RXDMAEN,
    tx = SPI_CR2_TXDMAEN,
#endif
  };

  IGB_FAST_INLINE void enable(DmaReqType type) {
#if defined(STM32H7)
    IGB_SET_BIT(IGB_SPI->CFG1, as<uint32_t>(type));
#else
    IGB_SPI->CR2 = IGB_SPI->CR2 | as<uint32_t>(type);
#endif
  }

  IGB_FAST_INLINE void disable(DmaReqType type) {
#if defined(STM32H7)
    IGB_CLEAR_BIT(IGB_SPI->CFG1, as<uint32_t>(type));
#else
    IGB_SPI->CR2 = IGB_SPI->CR2 & ~(as<uint32_t>(type));
#endif
  }

#if defined(STM32F0) || defined(STM32F3)
  enum class DmaParityTarget : uint32_t {
    rx = SPI_CR2_RXDMAEN,
    tx = SPI_CR2_TXDMAEN,
  };
  enum class DmaParityType : uint32_t {
    even = 0,
    odd  = 1,
  };
  IGB_FAST_INLINE void dmaParity(DmaParityTarget target, DmaParityType type) {
    switch (target) {
      case DmaParityTarget::tx:
        IGB_MODIFY_REG(IGB_SPI->CR2, SPI_CR2_LDMATX, (as<uint32_t>(type) << SPI_CR2_LDMATX_Pos));
        break;
      case DmaParityTarget::rx:
        IGB_MODIFY_REG(IGB_SPI->CR2, SPI_CR2_LDMARX, (as<uint32_t>(type) << SPI_CR2_LDMARX_Pos));
        break;
    }
  }

  IGB_FAST_INLINE uint32_t getRegAddr() {
    return (uint32_t) &(IGB_SPI->DR);
  }
#endif

#if defined(STM32H7)
  RegValue<IGB_SPI_REG_ADDR(CR2), SPI_CR2_TSIZE, 0> transferSize;
  RegFlag<IGB_SPI_REG_ADDR(CR1), SPI_CR1_CSTART> startMasterTransfer;
#endif

  IGB_FAST_INLINE uint8_t receiveU8() {
#if defined(STM32H7)
    return (*((__IO uint8_t *)&IGB_SPI->RXDR));
#else
    return (uint8_t)(IGB_SPI->DR);
#endif
  }

  IGB_FAST_INLINE uint8_t receiveU8sync(auto&& wait_func) {
#if defined(STM32H7)
    while (!is(SpiState::rxPacketAbailable)) { wait_func(); }
    return receiveU8();
#else
    while (!is(SpiState::rxBufNotEmpty)) { wait_func(); }
    return receiveU8();
#endif
  }
  IGB_FAST_INLINE uint8_t receiveU8sync() {
    return receiveU8sync([](){});
  }

  IGB_FAST_INLINE uint16_t receiveU16() {
#if defined(STM32H7)
    return (uint16_t)(READ_REG(IGB_SPI->RXDR));
#else
    return (uint16_t)(IGB_SPI->DR);
#endif
  }

  IGB_FAST_INLINE uint8_t receiveU16sync(auto&& wait_func) {
#if defined(STM32H7)
    while (!is(SpiState::rxPacketAbailable)) { wait_func(); }
    return receiveU16();
#else
    while (!is(SpiState::rxBufNotEmpty)) { wait_func(); }
    return receiveU16();
#endif
  }
  IGB_FAST_INLINE uint8_t receiveU16sync() {
    return receiveU16sync([](){});
  }

  IGB_FAST_INLINE void sendU8(uint8_t data) {
#if defined(STM32H7)
    *((__IO uint8_t *)&IGB_SPI->TXDR) = data;
#else
#if defined (__GNUC__)
    __IO uint8_t *spidr = ((__IO uint8_t *)&IGB_SPI->DR);
    *spidr = data;
#else
    *((__IO uint8_t *)&IGB_SPI->DR) = data;
#endif /* __GNUC__ */
#endif
  }

  IGB_FAST_INLINE void sendU8sync(uint8_t data, auto&& wait_func) {
#if defined(STM32H7)
    disable();
    transferSize(1);
    enable();
    startMasterTransfer(true);
    while (!is(SpiState::txPacketAbailable)) { wait_func(); }
    sendU8(data);
    while (!is(SpiState::endOfTransfer)) { wait_func(); }
    IGB_SET_BIT(IGB_SPI->IFCR, SPI_IFCR_EOTC);
    IGB_SET_BIT(IGB_SPI->IFCR, SPI_IFCR_TXTFC);
    disable();
    IGB_SPI->IER = IGB_SPI->IER & (~(SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));
    IGB_CLEAR_BIT(IGB_SPI->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
#else
    __IO uint16_t tmp IGB_UNUSED = transferU8sync(data, wait_func);
#endif
  }
  IGB_FAST_INLINE void sendU8sync(uint8_t data) {
    sendU8sync(data, [](){});
  }

  IGB_FAST_INLINE void sendBufU8sync(uint8_t* buffer, size_t size, auto&& wait_func) {
#if defined(STM32H7)
    disable();
    transferSize(size);
    enable();
    startMasterTransfer(true);
    for (size_t i = 0; i < size; ++i) {
      while (!is(SpiState::txPacketAbailable)) { wait_func(); }
      sendU8(buffer[i]);
    }
    while (!is(SpiState::endOfTransfer)) { wait_func(); }
    IGB_SET_BIT(IGB_SPI->IFCR, SPI_IFCR_EOTC);
    IGB_SET_BIT(IGB_SPI->IFCR, SPI_IFCR_TXTFC);
    disable();
    IGB_SPI->IER = IGB_SPI->IER & (~(SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));
    IGB_CLEAR_BIT(IGB_SPI->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
#else
    for (size_t i = 0; i < size; ++i) {
      sendU8sync(buffer[i], wait_func);
    }
#endif
  }
  IGB_FAST_INLINE void sendBufU8sync(uint8_t* buffer, size_t size) {
    sendBufU8sync(buffer, size, [](){});
  }

  IGB_FAST_INLINE void sendU16(uint16_t data) {
#if defined(STM32H7)
#if defined (__GNUC__)
  __IO uint16_t *spitxdr = ((__IO uint16_t *)&IGB_SPI->TXDR);
  *spitxdr = data;
#else
  IGB_SPI->TXDR = data;
#endif
#else
#if defined (__GNUC__)
  __IO uint16_t *spidr = ((__IO uint16_t *)&IGB_SPI->DR);
  *spidr = data;
#else
  IGB_SPI->DR = data;
#endif /* __GNUC__ */
#endif /* defined(STM32H7) */
  }

  IGB_FAST_INLINE uint8_t transferU8sync(uint8_t data, auto&& wait_func) {
#if defined(STM32H7)
    disable();
    transferSize(1);
    enable();
    startMasterTransfer(true);
    while (!is(SpiState::txPacketAbailable)) { wait_func(); }
    sendU8(data);
    while (!is(SpiState::rxPacketAbailable)) { wait_func(); }
    uint8_t result = receiveU8();
    while (!is(SpiState::endOfTransfer)) { wait_func(); }
    IGB_SET_BIT(IGB_SPI->IFCR, SPI_IFCR_EOTC);
    IGB_SET_BIT(IGB_SPI->IFCR, SPI_IFCR_TXTFC);
    disable();
    IGB_SPI->IER = IGB_SPI->IER & (~(SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));
    IGB_CLEAR_BIT(IGB_SPI->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
    return result;
#else
    while (is(SpiState::busy)) { wait_func(); }
    while (!is(SpiState::txBufEmpty))  { wait_func(); }

    sendU8(data);

    while (!is(SpiState::rxBufNotEmpty)) { wait_func(); }

    return receiveU8();
#endif
  }
  IGB_FAST_INLINE uint8_t transferU8sync(uint8_t data) {
    return transferU8sync(data, [](){});
  }

  IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    auto periph_type = as_periph_type(type);
    if (!periph_type) { return; }

    auto result = get_af_idx(periph_type.value(), pin_type);
    if (!result) { return; }

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
    nssPolarity(SpiNssPolarity::low);
    standard(SpiStandard::motorola);
    fifoThreshold(SpiFifoThreshold::_1data);
    nssPulseMng(false);

    crc(false);
    dataWidth(SpiDataWidth::_8bit);

    IGB_SET_BIT(IGB_SPI->CR1, SPI_CR1_SSI); // avoid a MODF Error

    nssMode(SpiNssMode::soft);
    clockPolarity(SpiClockPolarity::low);
    clockPhase(SpiClockPhase::oneEdge);
    transBitOrder(SpiBitOrder::msbFirst);
    mode(SpiMode::master);
    transDir(SpiTransDir::fullDuplex);
#elif defined(STM32F0) || defined(STM32F3)
    transDir(SpiTransDir::fullDuplex);
    mode(SpiMode::master);
    dataWidth(SpiDataWidth::_8bit);
    clockPolarity(SpiClockPolarity::low);
    clockPhase(SpiClockPhase::oneEdge);
    nssMode(SpiNssMode::soft);
    transBitOrder(SpiBitOrder::msbFirst);
    crc(false);
    crcPolynomial(7);
    standard(SpiStandard::motorola);
    rxFifoThreshold(SpiFifoThreshold::quarter);
    nssPulseMng(true);
#endif
  }

  IGB_FAST_INLINE void initMaster(SpiBaudratePrescaler prescaler = SpiBaudratePrescaler::div16) {
    const auto& spi_info = STM32_PERIPH_INFO.spi[to_idx(type)];
    spi_info.bus.enableBusClock();
    prepareGpio(mosi_pin);
    prepareGpio(miso_pin);
    prepareGpio(sck_pin);

    baudratePrescaler(prescaler);
    initDefault();
    enable();
  }

  IGB_FAST_INLINE void initMasterOutOnly(SpiBaudratePrescaler prescaler) {
    const auto& spi_info = STM32_PERIPH_INFO.spi[to_idx(type)];
    spi_info.bus.enableBusClock();
    prepareGpio(mosi_pin);
    prepareGpio(sck_pin);

#if defined(STM32H7)
    nssPolarity(SpiNssPolarity::low);
    standard(SpiStandard::motorola);
    fifoThreshold(SpiFifoThreshold::_1data);
    nssPulseMng(false);

    baudratePrescaler(prescaler);
    crc(false);
    dataWidth(SpiDataWidth::_8bit);

    IGB_SET_BIT(IGB_SPI->CR1, SPI_CR1_SSI); // avoid a MODF Error

    nssMode(SpiNssMode::soft);
    clockPolarity(SpiClockPolarity::low);
    clockPhase(SpiClockPhase::oneEdge);
    transBitOrder(SpiBitOrder::msbFirst);
    mode(SpiMode::master);
    transDir(SpiTransDir::simplexTx);
#if defined(SPI_I2SCFGR_I2SMOD)
    IGB_CLEAR_BIT(IGB_SPI->I2SCFGR, SPI_I2SCFGR_I2SMOD);
#endif
    IGB_CLEAR_BIT(IGB_SPI->CR1, SPI_CR1_TCRCINI);
    IGB_CLEAR_BIT(IGB_SPI->CR1, SPI_CR1_RCRCINI);
    IGB_CLEAR_BIT(IGB_SPI->CFG2, SPI_CFG2_AFCNTR);
    IGB_CLEAR_BIT(IGB_SPI->CFG2, SPI_CFG2_IOSWP);
    IGB_MODIFY_REG(IGB_SPI->CFG2, 0xFFUL, 0x00UL);
#else
    baudratePrescaler(prescaler);
    initDefault();
#endif
    enable();
  }

  IGB_FAST_INLINE void initMasterOutOnlyHardSS(GpioPinType cs_pin, SpiBaudratePrescaler prescaler) {
    const auto& spi_info = STM32_PERIPH_INFO.spi[to_idx(type)];
    spi_info.bus.enableBusClock();
    prepareGpio(mosi_pin);
    prepareGpio(sck_pin);
    prepareGpio(cs_pin);

#if defined(STM32H7)
    nssPolarity(SpiNssPolarity::low);
    standard(SpiStandard::motorola);
    fifoThreshold(SpiFifoThreshold::_1data);
    nssPulseMng(true);

    baudratePrescaler(prescaler);
    crc(false);
    dataWidth(SpiDataWidth::_8bit);

    IGB_SET_BIT(IGB_SPI->CR1, SPI_CR1_SSI); // avoid a MODF Error

    nssMode(SpiNssMode::hardOutput);
    clockPolarity(SpiClockPolarity::low);
    clockPhase(SpiClockPhase::oneEdge);
    transBitOrder(SpiBitOrder::msbFirst);
    mode(SpiMode::master);
    transDir(SpiTransDir::simplexTx);
#if defined(SPI_I2SCFGR_I2SMOD)
    IGB_CLEAR_BIT(IGB_SPI->I2SCFGR, SPI_I2SCFGR_I2SMOD);
#endif
    IGB_CLEAR_BIT(IGB_SPI->CR1, SPI_CR1_TCRCINI);
    IGB_CLEAR_BIT(IGB_SPI->CR1, SPI_CR1_RCRCINI);
    IGB_CLEAR_BIT(IGB_SPI->CFG2, SPI_CFG2_AFCNTR);
    IGB_CLEAR_BIT(IGB_SPI->CFG2, SPI_CFG2_IOSWP);
    IGB_MODIFY_REG(IGB_SPI->CFG2, 0xFFUL, 0x00UL);
#else
    baudratePrescaler(prescaler);
    initDefault();
#endif
    enable();
  }

  IGB_FAST_INLINE void init(SpiBaudratePrescaler prescaler = SpiBaudratePrescaler::div16) {
    // TODO: config default prescaler
    initMaster(prescaler);
  }
};

#undef IGB_SPI_REG
#undef IGB_SPI_REG_ADDR
#undef IGB_SPI

}
}

#endif /* IGB_STM32_PERIPH_SPI_H */
