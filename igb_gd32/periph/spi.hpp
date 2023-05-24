#ifndef IGB_GD32_PERIPH_SPI_H
#define IGB_GD32_PERIPH_SPI_H

#include <igb_gd32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_gd32/periph/gpio.hpp>
#include <igb_gd32/periph/rcc.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>

namespace igb {
namespace gd32 {

#define IGB_SPI ((SPI_TypeDef*)addr)
#define IGB_SPI_REG_ADDR(member) (addr + offsetof(SPI_TypeDef, member))
#define IGB_SPI_REG(member) ((SPI_TypeDef*)IGB_SPI_REG_ADDR(member))

enum class SpiMode : uint32_t {
  slave  = 0,
  master = 1
};
enum class SpiStandard : uint32_t {
  motorola = 0,
  ti = 1,
};
enum class SpiClockPhase : uint32_t {
  oneEdge = 0,
  twoEdge = 1,
};
enum class SpiClockPolarity : uint32_t {
  low = 0,
  high = 1,
};
enum class SpiBaudratePrescaler : uint32_t {
  div2 = 0,
  div4,
  div8,
  div16,
  div32,
  div64,
  div128,
  div256,
};
enum class SpiBitOrder : uint32_t {
  msbFirst = 0,
  lsbFirst = 1,
};
enum class SpiTransDir : uint32_t {
  fullDuplex = 0,
  simplexRx  = IGB_BIT(10),
  halfDuplexRx = IGB_BIT(15),
  halfDuplexTx = IGB_BIT(15) | IGB_BIT(14),
};
enum class SpiNssMode : uint32_t {
  hardInput  = 0,
  hardOutput = IGB_BIT(2) << 16U,
  soft       = IGB_BIT(9),
};

enum class SpiDataWidth : uint32_t {
  _8bit = 0,
  _16bit,
};

enum class SpiState : uint32_t {
  rxBufNotEmpty    = IGB_BIT(0),
  txBufEmpty       = IGB_BIT(1),
  busy             = IGB_BIT(7),
  crcError         = IGB_BIT(4),
  modeFault        = IGB_BIT(5),
  overrun          = IGB_BIT(6),
  frameFormatError = IGB_BIT(8),
  i2sChannelSide   = IGB_BIT(2),
  i2sUnderrun      = IGB_BIT(3),
};

template<SpiType SPI_TYPE, GpioPinType MOSI_PIN, GpioPinType MISO_PIN, GpioPinType SCK_PIN>
struct Spi {
  constexpr static auto type = SPI_TYPE;
  constexpr static auto mosi_pin = MOSI_PIN;
  constexpr static auto miso_pin = MISO_PIN;
  constexpr static auto sck_pin = SCK_PIN;
  constexpr static auto addr = GD32_PERIPH_INFO.spi[to_idx(type)].addr;

  IGB_FAST_INLINE void enable() { IGB_SET_BIT(IGB_SPI->CR1, IGB_BIT(6)); }
  IGB_FAST_INLINE void disable() { IGB_CLEAR_BIT(IGB_SPI->CR1, IGB_BIT(6)); }

  RegEnum<IGB_SPI_REG_ADDR(CR1), IGB_BIT(2), SpiMode, 2> mode;
  RegEnum<IGB_SPI_REG_ADDR(CR2), IGB_BIT(4), SpiStandard, 4> standard;
  RegEnum<IGB_SPI_REG_ADDR(CR1), IGB_BIT(0), SpiClockPhase, 0> clockPhase;
  RegEnum<IGB_SPI_REG_ADDR(CR1), IGB_BIT(1), SpiClockPolarity, 1> clockPolarity;
  RegEnum<IGB_SPI_REG_ADDR(CR1), IGB_BIT_MASK(3, 3), SpiBaudratePrescaler, 3> baudratePrescaler;
  RegEnum<IGB_SPI_REG_ADDR(CR1), IGB_BIT(7), SpiBitOrder, 7> transBitOrder;
  RegEnum<IGB_SPI_REG_ADDR(CR1), IGB_BIT(10) | IGB_BIT(15) | IGB_BIT(14), SpiTransDir> transDir;

  RegEnum<IGB_SPI_REG_ADDR(CR1), IGB_BIT(11), SpiDataWidth> dataWidth;
  RegFlag<IGB_SPI_REG_ADDR(CR1), IGB_BIT(13)> crc;
  RegFlag<IGB_SPI_REG_ADDR(CR1), IGB_BIT(12)> crcNext;

  Reg<IGB_SPI_REG_ADDR(CRCPR)> crcPolynomial;

  Reg<IGB_SPI_REG_ADDR(RXCRCR)> rxCrc;

  Reg<IGB_SPI_REG_ADDR(TXCRCR)> txCrc;

  struct SpiNssModeInfo {
    IGB_FAST_INLINE uint32_t operator()() {
      return (IGB_SPI->CR1 & IGB_BIT(9)) | ((IGB_SPI->CR2 & IGB_BIT(2)) << 16U);
    };
    IGB_FAST_INLINE void operator()(SpiNssMode mode) {
      IGB_MODIFY_REG(IGB_SPI->CR1, IGB_BIT(9),  as<uint32_t>(mode));
      IGB_MODIFY_REG(IGB_SPI->CR2, IGB_BIT(2), ((uint32_t)(as<uint32_t>(mode) >> 16U)));
    };
  };
  SpiNssModeInfo nssMode;

  RegFlag<IGB_SPI_REG_ADDR(CR2), IGB_BIT(3)> nssPulseMng;

  IGB_FAST_INLINE bool is(SpiState state) {
    return !!(IGB_SPI->SR & as<uint32_t>(state));
  }
  
  IGB_FAST_INLINE void clear(SpiState state) {
    IGB_CLEAR_BIT(IGB_SPI->SR, as<uint32_t>(state));
  }

  enum class InterruptType : uint32_t {
    rxBufNotEmpty     = IGB_BIT(6),
    txBufEmpty        = IGB_BIT(7),
    error             = IGB_BIT(5),
  };

  IGB_FAST_INLINE void enable(InterruptType type) {
    IGB_SPI->CR2 = IGB_SPI->CR2 | as<uint32_t>(type);
  }

  IGB_FAST_INLINE void disable(InterruptType type) {
    IGB_SPI->CR2 = IGB_SPI->CR2 & ~(as<uint32_t>(type));
  }

  enum class DmaReqType : uint32_t {
    rx = IGB_BIT(0),
    tx = IGB_BIT(1),
  };

  IGB_FAST_INLINE void enable(DmaReqType type) {
    IGB_SPI->CR2 = IGB_SPI->CR2 | as<uint32_t>(type);
  }

  IGB_FAST_INLINE void disable(DmaReqType type) {
    IGB_SPI->CR2 = IGB_SPI->CR2 & ~(as<uint32_t>(type));
  }

  IGB_FAST_INLINE uint32_t getRegAddr() {
    return (uint32_t) &(IGB_SPI->DR);
  }

  IGB_FAST_INLINE uint8_t receiveU8() {
    return (uint8_t)(IGB_SPI->DR);
  }

  IGB_FAST_INLINE uint8_t receiveU8sync(auto&& wait_func) {
    while (!is(SpiState::rxBufNotEmpty)) { wait_func(); }
    return receiveU8();
  }
  IGB_FAST_INLINE uint8_t receiveU8sync() {
    return receiveU8sync([](){});
  }

  IGB_FAST_INLINE uint16_t receiveU16() {
    return (uint16_t)(IGB_SPI->DR);
  }

  IGB_FAST_INLINE uint8_t receiveU16sync(auto&& wait_func) {
    while (!is(SpiState::rxBufNotEmpty)) { wait_func(); }
    return receiveU16();
  }
  IGB_FAST_INLINE uint8_t receiveU16sync() {
    return receiveU16sync([](){});
  }

  IGB_FAST_INLINE void sendU8(uint8_t data) {
#if defined (__GNUC__)
    __IO uint8_t *spidr = ((__IO uint8_t *)&IGB_SPI->DR);
    *spidr = data;
#else
    *((__IO uint8_t *)&IGB_SPI->DR) = data;
#endif /* __GNUC__ */
  }

  IGB_FAST_INLINE void sendU8sync(uint8_t data, auto&& wait_func) {
    __IO uint16_t tmp IGB_UNUSED = transferU8sync(data, wait_func);
  }
  IGB_FAST_INLINE void sendU8sync(uint8_t data) {
    sendU8sync(data, [](){});
  }

  IGB_FAST_INLINE void sendBufU8sync(uint8_t* buffer, size_t size, auto&& wait_func) {
    for (size_t i = 0; i < size; ++i) {
      sendU8sync(buffer[i], wait_func);
    }
  }
  IGB_FAST_INLINE void sendBufU8sync(uint8_t* buffer, size_t size) {
    sendBufU8sync(buffer, size, [](){});
  }

  IGB_FAST_INLINE void sendU16(uint16_t data) {
#if defined (__GNUC__)
  __IO uint16_t *spidr = ((__IO uint16_t *)&IGB_SPI->DR);
  *spidr = data;
#else
  IGB_SPI->DR = data;
#endif /* __GNUC__ */
  }

  IGB_FAST_INLINE uint8_t transferU8sync(uint8_t data, auto&& wait_func) {
    while (is(SpiState::busy)) { wait_func(); }
    while (!is(SpiState::txBufEmpty))  { wait_func(); }

    sendU8(data);

    while (!is(SpiState::rxBufNotEmpty)) { wait_func(); }

    return receiveU8();
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
    nssPulseMng(true);
  }

  IGB_FAST_INLINE void initMaster(SpiBaudratePrescaler prescaler = SpiBaudratePrescaler::div16) {
    const auto& spi_info = GD32_PERIPH_INFO.spi[to_idx(type)];
    spi_info.bus.enableBusClock();
    prepareGpio(mosi_pin);
    prepareGpio(miso_pin);
    prepareGpio(sck_pin);

    baudratePrescaler(prescaler);
    initDefault();
    enable();
  }

  IGB_FAST_INLINE void initMasterOutOnly(SpiBaudratePrescaler prescaler) {
    const auto& spi_info = GD32_PERIPH_INFO.spi[to_idx(type)];
    spi_info.bus.enableBusClock();
    prepareGpio(mosi_pin);
    prepareGpio(sck_pin);

    baudratePrescaler(prescaler);
    initDefault();
    enable();
  }

  IGB_FAST_INLINE void initMasterOutOnlyHardSS(GpioPinType cs_pin, SpiBaudratePrescaler prescaler) {
    const auto& spi_info = GD32_PERIPH_INFO.spi[to_idx(type)];
    spi_info.bus.enableBusClock();
    prepareGpio(mosi_pin);
    prepareGpio(sck_pin);
    prepareGpio(cs_pin);

    baudratePrescaler(prescaler);
    initDefault();
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

#endif /* IGB_GD32_PERIPH_SPI_H */
