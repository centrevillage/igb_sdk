#ifndef IGB_GD32_PERIPH_I2C_H
#define IGB_GD32_PERIPH_I2C_H

#include <stddef.h>
#include <optional>

#include <igb_gd32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_gd32/periph/gpio.hpp>
#include <igb_gd32/periph/rcc.hpp>
#include <igb_gd32/periph/systick.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>

namespace igb {
namespace gd32 {

#define IGB_I2C ((I2C_TypeDef*)addr)
#define IGB_I2C_REG_ADDR(member) (addr + offsetof(I2C_TypeDef, member))
#define IGB_I2C_REG(member) ((I2C_TypeDef*)IGB_I2C_REG_ADDR(member))

enum class I2cAddressSize : uint32_t {
  _7bit = 0,
  _10bit,
};

enum class I2cStatus : uint32_t {
  startSent = IGB_BIT(0),
  addressMatched = IGB_BIT(1),
  txComplete = IGB_BIT(2),
  address10bitMatched = IGB_BIT(3),
  stop = IGB_BIT(4),
  rxNotEmpty = IGB_BIT(6),
  txDataEmpty = IGB_BIT(7),
  busError = IGB_BIT(8),
  arbitrationLost = IGB_BIT(9),
  ackError = IGB_BIT(10),
  overOrUnderrun = IGB_BIT(11),
  pecError = IGB_BIT(12)
  smbusTimeout = IGB_BIT(14),
  smbusAlert = IGB_BIT(15),
  master = IGB_BIT(0) << 16U, // STAT1
  busy = IGB_BIT(1) << 16U,
  transmitter = IGB_BIT(2) << 16U,
  generalCallAddressReceived = IGB_BIT(4) << 16U,
  smbusAddressReceived = IGB_BIT(5) << 16U,
  smbusHostHeaderDetected = IGB_BIT(6) << 16U,
  dualAddressMatched = IGB_BIT(7) << 16U,
};

enum class I2cInterruptType {
  error = IGB_BIT(8),
  event = IGB_BIT(9),
  buffer = IGB_BIT(10),
};

struct I2cConf {
  //uint32_t timing = 0x00702025; // 100kHz (clock src = HSI 8MHz)
  //uint8_t timingScll = 0;
  //uint8_t timingSclh = 0;
  //uint8_t timing_sdadel = 0;
  //uint8_t timing_scldel = 0;
  //uint8_t timing_prescale = 0;

  //bool general_call = false;
  //bool clock_stretch = true;
  //bool analog_filter = false;
  //uint8_t digital_filter = 0;
  //bool dma_tx = false;
  //bool dma_rx = false;
  //bool slave_byte_control = false;
  //I2cAddressingMode master_addressing = I2cAddressingMode::_7bit;
  //bool own_address1 = true;
  //uint16_t own_address1_value = 1;
  //I2cOwnAddress1Size own_address1_size = I2cOwnAddress1Size::_7bit;
  //bool own_address2 = false;
  //uint8_t own_address2_value = 0;
  //I2cOwnAddress2Mask own_address2_mask = I2cOwnAddress2Mask::nomask;

  //bool sm_bus_host = false;
  //bool sm_bus_device = false;
  //bool sm_bus_alert = false;
  //bool sm_bus_packet_err_calc = false;
  //bool sm_bus_timeout_a = false; 
  //uint16_t sm_bus_timeout_a_value = 0;
  //bool sm_bus_timeout_b = false; 
  //uint16_t sm_bus_timeout_b_value = 0;
  //I2cSmbusTimeoutAMode sm_bus_timeout_a_mode = I2cSmbusTimeoutAMode::sclLow;

  //bool enable_it_tx = false;
  //bool enable_it_rx = false;
  //bool enable_it_addr = false;
  //bool enable_it_nack = false;
  //bool enable_it_stop = false;
  //bool enable_it_transfer_complete = false;
  //bool enable_it_error = false;

  //uint8_t interrupt_priority = 1;
};

// TODO: base_clockをRCCの設定から自動計算
template<I2cType I2C_TYPE, GpioPinType SCL_PIN, GpioPinType SDA_PIN>
struct I2c {
  constexpr static auto type = I2C_TYPE;
  constexpr static auto scl_pin = SCL_PIN;
  constexpr static auto sda_pin = SDA_PIN;
  constexpr static auto addr = GD32_PERIPH_INFO.i2c[to_idx(type)].addr;

  IGB_FAST_INLINE void enable() { IGB_SET_BIT(IGB_I2C->CTL0, IGB_BIT(0)); }
  IGB_FAST_INLINE void disable() { IGB_CLEAR_BIT(IGB_I2C->CTL0, IGB_BIT(0)); }


  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(1)> enableSmb;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(3)> isSmbHost;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(4)> enableSmbArp;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(5)> enablePec;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(6)> enableGeneralCall;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(7), true> enableSclStretch;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(8)> start;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(9)> stop;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(10)> ack;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(11)> positionOfAckAndPec;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(12)> enablePecTransfer;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(13)> enableSmbAlert;
  RegFlag<IGB_I2C_REG_ADDR(CTL0), IGB_BIT(15)> softwareReset;

  RegValue<IGB_I2C_REG_ADDR(CTL1), IGB_BIT_MASK(6, 0), 0> i2cClock;
  RegFlag<IGB_I2C_REG_ADDR(CTL1), IGB_BIT(8)> enableItError;
  RegFlag<IGB_I2C_REG_ADDR(CTL1), IGB_BIT(9)> enableItEvent;
  RegFlag<IGB_I2C_REG_ADDR(CTL1), IGB_BIT(10)> enableItBuffer;
  RegFlag<IGB_I2C_REG_ADDR(CTL1), IGB_BIT(11)> dma;
  RegFlag<IGB_I2C_REG_ADDR(CTL1), IGB_BIT(12)> isLastDmaTx;

  IGB_FAST_INLINE void setSlaveAddress(I2cAddressSize address_size, uint16_t address) {
    if (address_size == I2cAddressSize::_10bit) {
      IGB_MODIFY_REG(IGB_I2C->SADDR0, IGB_BIT_MASK(16, 0), IGB_BIT(15) | address);
    } else { // 7bit
      IGB_MODIFY_REG(IGB_I2C->SADDR0, IGB_BIT_MASK(16, 0), address << 1U);
    }
  }

  Reg<IGB_I2C_REG_ADDR(DATA)> data;

  RegValue<IGB_I2C_REG_ADDR(STAT1), IGB_BIT_MASK(8, 8), 8> pecValue;

  RegValue<IGB_I2C_REG_ADDR(CKCFG), IGB_BIT_MASK(12, 0), 0> clockControl;
  RegFlag<IGB_I2C_REG_ADDR(CKCFG), IGB_BIT(14)> dutyCycleFlag;
  RegFlag<IGB_I2C_REG_ADDR(CKCFG), IGB_BIT(15)> isFastMode;

  Reg<IGB_I2C_REG_ADDR(RT)> riseTime;

  RegFlag<(addr + 0x90UL), IGB_BIT(0)> isFastModePlus;


  IGB_FAST_INLINE bool is(I2cStatus status) {
    return (IGB_I2C->STAT0 & (static_cast<uint32_t>(status) & 0xFFFF))
      || (IGB_I2C->STAT1 & (static_cast<uint32_t>(status) >> 16U));
  }

  IGB_FAST_INLINE void clear(I2cStatus status) {
    uint32_t v = static_cast<uint32_t>(status);
    if (v > 0xFFFF) {
      IGB_CLEAR_BIT(IGB_I2C->STAT1, v >> 16U);
    } else {
      IGB_CLEAR_BIT(IGB_I2C->STAT0, v & 0xFFFF);
    }
  }


  IGB_FAST_INLINE uint8_t receiveU8() {
    return data();
  }

  IGB_FAST_INLINE std::optional<uint8_t> receiveU8sync(uint32_t timeout_msec = 1000) {
    uint32_t msec = current_msec();
    while(!is(I2cStatus::rxNotEmpty)) {
      if (current_msec() - msec > timeout_msec) {
        return std::nullopt;
      }
    }
    return receiveU8();
  }

  IGB_FAST_INLINE bool isReceivable() {
    return is(I2cStatus::rxNotEmpty);
  }

  IGB_FAST_INLINE void sendU8(uint8_t value) {
    data(value);
  }

  IGB_FAST_INLINE bool sendU8sync(uint8_t value, uint32_t timeout_msec = 1000) {
    uint32_t msec = current_msec();
    while(!is(I2cStatus::txDataEmpty)) {
      if (current_msec() - msec > timeout_msec) {
        return false;
      }
    }
    sendU8(value);
    return true;
  }

  IGB_FAST_INLINE bool isSendable() {
    return is(I2cStatus::txDataEmpty);
  }

  IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    auto periph_type = as_periph_type(type);
    if (!periph_type) { return; }

    auto result = get_af_idx(periph_type.value(), pin_type);
    if (!result) { return; }

    GpioPin pin = GpioPin::newPin(pin_type);
    pin.setMode(GpioMode::alternate);
    pin.setPullMode(GpioPullMode::up);
    pin.setSpeedMode(GpioSpeedMode::high);
    pin.setOutputMode(GpioOutputMode::opendrain);
    pin.setAlternateFunc(result.value());
    pin.enable();
  }

//  IGB_FAST_INLINE void initDefault(uint8_t address = 1) {
//    const auto& i2c_info = GD32_PERIPH_INFO.i2c[to_idx(type)];
//    i2c_info.bus.enableBusClock();
//    i2c_info.bus.forceResetBusClock();
//    i2c_info.bus.releaseResetBusClock();
//
//    prepareGpio(scl_pin);
//    prepareGpio(sda_pin);
//
//    reloadEndMode(I2cReloadEndType::autoEnd);
//    ownAddress2.disable();
//    generalCall.disable();
//    clockStretch.enable();
//
//    disable();
//    analogFilter.enable();
//    digitalFilter(0);
//
//    timing(0x00702025); // 100kHz (clock src = HSI 8MHz), TODO: calculate timing from params
//    enable();
//    ownAddress1.disable();
//    auto ownAddress1Reg = ownAddress1.val(true) | ownAddress1Value.val(address << 1);
//    ownAddress1Reg.update();
//    ownAddress1Size(I2cOwnAddress1Size::_7bit);
//    smBusHost.disable(); smBusDevice.disable(); // i2c mode
//    ackNextData(I2cAckType::ack);
//    ownAddress2Mask(I2cOwnAddress2Mask::nomask);
//    auto10bitRead.val(false);
//  }
//
//  IGB_FAST_INLINE void init(uint8_t address = 1) {
//    initDefault(address);
//  }
//
//  IGB_FAST_INLINE void init(auto&& conf) {
//    const auto& i2c_info = GD32_PERIPH_INFO.i2c[to_idx(type)];
//    i2c_info.bus.enableBusClock();
//    i2c_info.bus.forceResetBusClock();
//    i2c_info.bus.releaseResetBusClock();
//    prepareGpio(scl_pin);
//    prepareGpio(sda_pin);
//
//    reloadEndMode(I2cReloadEndType::autoEnd);
//
//    if (conf.own_address2) {
//      (
//       ownAddress2.val(conf.own_address2) |
//       ownAddress2Value.val(conf.own_address2_value) |
//       ownAddress2Mask.val(conf.own_address2_mask)
//      ).update();
//    } else {
//      ownAddress2.disable();
//    }
//
//    (generalCall.val(conf.general_call) | clockStretch.val(conf.clock_stretch)).update();
//
//    disable();
//    (analogFilter.val(conf.analog_filter) | digitalFilter.val(conf.digital_filter)).update();
//
//    timing(conf.timing);
//    enable();
//
//    ownAddress1.disable();
//    if (conf.own_address1) {
//      (ownAddress1.val(conf.own_address1) | ownAddress1Value.val(conf.own_address1_value << 1)).update();
//    }
//    ownAddress1Size(conf.own_address1_size);
//
//    (
//     smBusHost.val(conf.sm_bus_host) |
//     smBusDevice.val(conf.sm_bus_device) |
//     smBusAlert.val(conf.sm_bus_alert) |
//     smBusPacketErrCalc.val(conf.sm_bus_packet_err_calc)
//    ).update();
//
//    if (conf.sm_bus_host || conf.sm_bus_device) {
//      (
//       smBusTimeoutA.val(conf.sm_bus_timeout_a) |
//       smBusTimeoutB.val(conf.sm_bus_timeout_b) |
//       smBusTimeoutAValue.val(conf.sm_bus_timeout_a_value) |
//       smBusTimeoutBValue.val(conf.sm_bus_timeout_b_value) |
//       smBusTimeoutAMode.val(conf.sm_bus_timeout_a_mode)
//      ).update();
//    }
//
//    if (
//      conf.enable_it_tx ||
//      conf.enable_it_rx ||
//      conf.enable_it_addr ||
//      conf.enable_it_nack ||
//      conf.enable_it_stop ||
//      conf.enable_it_transfer_complete ||
//      conf.enable_it_error
//        ) {
//      // TODO: enable nvic
//      //NvicCtrl::setPriority(i2c_info.irqn, conf.interrupt_priority);
//      //NvicCtrl::enable(i2c_info.irqn);
//    }
//
//    (
//     enableItTx.val(conf.enable_it_tx) |
//     enableItRx.val(conf.enable_it_rx) |
//     enableItAddr.val(conf.enable_it_addr) |
//     enableItNack.val(conf.enable_it_nack) |
//     enableItStop.val(conf.enable_it_stop) |
//     enableItTransferComplete.val(conf.enable_it_transfer_complete) |
//     enableItError.val(conf.enable_it_error)
//    ).update();
//  }
//
//  IGB_FAST_INLINE void beginTransfer(uint8_t address, uint8_t transfer_size, I2cTransferRequestType request_type, I2cReloadEndType reload_end = I2cReloadEndType::autoEnd) {
//    auto reg =
//      slaveAddr.val((uint32_t)address)
//      | ackNextData.val(I2cAckType::ack)
//      | transferSize.val(transfer_size)
//      | reloadEndMode.val(reload_end)
//      | transferRequest.val(request_type)
//      | stopCondition.val(false)
//      | startCondition.val(true)
//    ;
//    reg.update();
//  }
//
//  IGB_FAST_INLINE bool isTransferEnd() {
//    return is(I2cStatus::stop);
//  }
//
//  // TODO: autoEnd でない時も正常に停止できる様に
//  IGB_FAST_INLINE bool endTransfer(uint32_t timeout_msec = 1000) {
//    // wait for auto end
//    uint32_t msec = current_msec();
//    while(!is(I2cStatus::stop)) {
//      if (current_msec() - msec > timeout_msec) {
//        return false;
//      }
//    }
//    return true;
//  }
//
//  // common api ==
//
//  IGB_FAST_INLINE void beginSending(uint8_t address, uint8_t transfer_size) {
//    beginTransfer(address << 1, transfer_size, I2cTransferRequestType::write, I2cReloadEndType::autoEnd);
//  }
//
//  IGB_FAST_INLINE bool endSending() {
//    return endTransfer();
//  }
//
//  IGB_FAST_INLINE void beginReading(uint8_t address, uint8_t transfer_size) {
//    beginTransfer((address << 1) | 1, transfer_size, I2cTransferRequestType::read, I2cReloadEndType::autoEnd);
//  }
//
//  IGB_FAST_INLINE bool endReading() {
//    return endTransfer();
//  }
//
//  IGB_FAST_INLINE bool checkSlave(uint8_t address, uint32_t timeout_msec = 1000) {
//    auto reg =
//      slaveAddr.val((uint32_t)address << 1)
//      | ackNextData.val(I2cAckType::ack)
//      | transferSize.val(0)
//      | reloadEndMode.val(I2cReloadEndType::autoEnd)
//      | transferRequest.val(I2cTransferRequestType::write)
//      | stopCondition.val(false)
//      | startCondition.val(true)
//    ;
//    reg.update();
//
//    uint32_t msec = current_msec();
//
//    while (!is(I2cStatus::stop) && !is(I2cStatus::nack)) {
//      if (current_msec() - msec > timeout_msec) {
//        return false;
//      }
//    }
//
//    if (is(I2cStatus::nack)) {
//
//      while (!is(I2cStatus::stop)) {
//        if (current_msec() - msec > timeout_msec) {
//          return false;
//        }
//      }
//
//      clear(I2cStatus::nack);
//      clear(I2cStatus::stop);
//    } else {
//      while (!is(I2cStatus::stop)) {
//        if (current_msec() - msec > timeout_msec) {
//          return false;
//        }
//      }
//
//      clear(I2cStatus::stop);
//    }
//
//    return true;
//  }
};

#undef IGB_I2C_REG
#undef IGB_I2C_REG_ADDR
#undef IGB_I2C

} /* gd32 */
} /* igb */

#endif /* IGB_GD32_PERIPH_I2C_H */
