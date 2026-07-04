#pragma once

#include <igb_sdk/base.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/null_functor.hpp>

namespace igb {
namespace sdk {

// PCM3060 24-bit stereo audio codec - register definitions
namespace pcm3060 {

// Register addresses (I2C register address / SPI register index)
constexpr uint8_t reg_sys_ctrl  = 0x40; // Register 64: System Control
constexpr uint8_t reg_dac_att_l = 0x41; // Register 65: DAC Attenuation Left
constexpr uint8_t reg_dac_att_r = 0x42; // Register 66: DAC Attenuation Right
constexpr uint8_t reg_dac_ctrl1 = 0x43; // Register 67: DAC Control 1
constexpr uint8_t reg_dac_ctrl2 = 0x44; // Register 68: DAC Control 2
constexpr uint8_t reg_dig_ctrl  = 0x45; // Register 69: Digital Control
constexpr uint8_t reg_adc_att_l = 0x46; // Register 70: ADC Attenuation Left
constexpr uint8_t reg_adc_att_r = 0x47; // Register 71: ADC Attenuation Right
constexpr uint8_t reg_adc_ctrl1 = 0x48; // Register 72: ADC Control 1
constexpr uint8_t reg_adc_ctrl2 = 0x49; // Register 73: ADC Control 2
constexpr uint8_t reg_count     = 10;

// System Control (0x40) bits
constexpr uint8_t bit_mrst  = 0x80; // B7: Mode Control Register Reset (0=reset, auto-returns to 1)
constexpr uint8_t bit_srst  = 0x40; // B6: System Reset (0=resync, auto-returns to 1)
constexpr uint8_t bit_adpsv = 0x20; // B5: ADC Power Save (0=normal, 1=power save)
constexpr uint8_t bit_dapsv = 0x10; // B4: DAC Power Save (0=normal, 1=power save)
constexpr uint8_t bit_se    = 0x01; // B0: DAC Output (0=differential, 1=single-ended)

// DAC/ADC Control 1 (0x43/0x48) masks
constexpr uint8_t bit_csel  = 0x80; // B7: Clock Select
constexpr uint8_t mask_ms   = 0x70; // B6-B4: Master/Slave Mode
constexpr uint8_t shift_ms  = 4;
constexpr uint8_t mask_fmt  = 0x03; // B1-B0: Audio Format

// DAC Control 2 (0x44) bits
constexpr uint8_t bit_over  = 0x40; // B6: Oversampling Rate (0=default, 1=double)
constexpr uint8_t bit_drev2 = 0x04; // B2: DAC Output Phase (0=normal, 1=inverted)
constexpr uint8_t bit_mut22 = 0x02; // B1: DAC Soft Mute R-ch
constexpr uint8_t bit_mut21 = 0x01; // B0: DAC Soft Mute L-ch

// Digital Control (0x45) bits
constexpr uint8_t bit_flt   = 0x80; // B7: Filter Rolloff (0=sharp, 1=slow)
constexpr uint8_t mask_dmf  = 0x60; // B6-B5: De-Emphasis Frequency
constexpr uint8_t shift_dmf = 5;
constexpr uint8_t bit_dmc   = 0x10; // B4: De-Emphasis (0=off, 1=on)
constexpr uint8_t bit_zrev  = 0x02; // B1: Zero-Flag Polarity
constexpr uint8_t bit_azro  = 0x01; // B0: Zero-Flag Function Select

// ADC Control 2 (0x49) bits
constexpr uint8_t bit_zcdd  = 0x10; // B4: Zero-Cross Detection Disable
constexpr uint8_t bit_byp   = 0x08; // B3: HPF Bypass
constexpr uint8_t bit_drev1 = 0x04; // B2: ADC Input Phase (0=normal, 1=inverted)
constexpr uint8_t bit_mut12 = 0x02; // B1: ADC Soft Mute R-ch
constexpr uint8_t bit_mut11 = 0x01; // B0: ADC Soft Mute L-ch

// Default register values after power-on/MRST
constexpr uint8_t default_sys_ctrl  = 0xF0; // MRST=1, SRST=1, ADPSV=1, DAPSV=1
constexpr uint8_t default_dac_att   = 0xFF; // 0 dB, no attenuation
constexpr uint8_t default_dac_ctrl1 = 0x00;
constexpr uint8_t default_dac_ctrl2 = 0x00;
constexpr uint8_t default_dig_ctrl  = 0x00;
constexpr uint8_t default_adc_att   = 0xD7; // 0 dB (offset: 215 = 0 dB)
constexpr uint8_t default_adc_ctrl1 = 0x00;
constexpr uint8_t default_adc_ctrl2 = 0x00;

enum class AudioFormat : uint8_t {
  i2s              = 0b00, // 24-bit I2S (default)
  leftJustified    = 0b01, // 24-bit Left-Justified
  rightJustified24 = 0b10, // 24-bit Right-Justified
  rightJustified16 = 0b11, // 16-bit Right-Justified
};

enum class MasterSlaveMode : uint8_t {
  slave     = 0b000, // Slave mode (default)
  master768 = 0b001, // Master, 768 fS
  master512 = 0b010, // Master, 512 fS
  master384 = 0b011, // Master, 384 fS
  master256 = 0b100, // Master, 256 fS
  master192 = 0b101, // Master, 192 fS (DAC only)
  master128 = 0b110, // Master, 128 fS (DAC only)
};

enum class DeEmphasisFreq : uint8_t {
  _44_1kHz = 0b00,
  _48kHz   = 0b01,
  _32kHz   = 0b10,
};

} // namespace pcm3060


// PCM3060 with I2C control interface
template<typename I2C_TYPE, typename GPIO_PIN_TYPE>
struct Pcm3060I2c {
  I2C_TYPE i2c;
  GPIO_PIN_TYPE rst_pin;

  constexpr static uint8_t i2c_address_base = 0x46; // 7-bit: 0b1000110

  enum class Address : uint8_t {
    addr0 = i2c_address_base,     // ADR pin = GND
    addr1 = i2c_address_base | 1, // ADR pin = VDD
  };

  uint8_t _address_byte = i2c_address_base;

  // fmt: audio format for both DAC and ADC
  bool init(Address address = Address::addr0, pcm3060::AudioFormat fmt = pcm3060::AudioFormat::leftJustified) {
    _address_byte = static_cast<uint8_t>(address);

    // MRST: reset all mode control registers to defaults
    uint8_t sys_reg = 0;
    if (!readRegister(pcm3060::reg_sys_ctrl, sys_reg)) { return false; }
    sys_reg &= ~pcm3060::bit_mrst;
    if (!writeRegister(pcm3060::reg_sys_ctrl, sys_reg)) { return false; }
    delay_msec(4);

    // SRST: system reset (resynchronize clocks)
    if (!readRegister(pcm3060::reg_sys_ctrl, sys_reg)) { return false; }
    sys_reg &= ~pcm3060::bit_srst;
    if (!writeRegister(pcm3060::reg_sys_ctrl, sys_reg)) { return false; }
    delay_msec(4);

    // Set audio format for DAC and ADC
    if (!setDacFormat(fmt)) { return false; }
    if (!setAdcFormat(fmt)) { return false; }

    // Disable power save for ADC and DAC (enable both)
    if (!readRegister(pcm3060::reg_sys_ctrl, sys_reg)) { return false; }
    sys_reg &= ~(pcm3060::bit_adpsv | pcm3060::bit_dapsv);
    return writeRegister(pcm3060::reg_sys_ctrl, sys_reg);
  }

  // Hardware reset via RST pin (active low, hold for >= 2048/fS)
  void resetHardware() {
    rst_pin.low();
    delay_msec(1);
    rst_pin.high();
    delay_msec(1);
  }

  bool writeRegister(uint8_t reg, uint8_t data) {
    i2c.beginSending(_address_byte, 2);
    i2c.sendU8sync(reg);
    i2c.sendU8sync(data);
    i2c.endSending();
    return true;
  }

  bool readRegister(uint8_t reg, uint8_t& data) {
    i2c.beginSending(_address_byte, 1);
    i2c.sendU8sync(reg);
    i2c.endSending();
    i2c.beginReading(_address_byte, 1);
    data = i2c.receiveU8sync();
    i2c.endReading();
    return true;
  }

  bool setDacFormat(pcm3060::AudioFormat fmt) {
    uint8_t reg = 0;
    if (!readRegister(pcm3060::reg_dac_ctrl1, reg)) { return false; }
    reg = (reg & ~pcm3060::mask_fmt) | static_cast<uint8_t>(fmt);
    return writeRegister(pcm3060::reg_dac_ctrl1, reg);
  }

  bool setAdcFormat(pcm3060::AudioFormat fmt) {
    uint8_t reg = 0;
    if (!readRegister(pcm3060::reg_adc_ctrl1, reg)) { return false; }
    reg = (reg & ~pcm3060::mask_fmt) | static_cast<uint8_t>(fmt);
    return writeRegister(pcm3060::reg_adc_ctrl1, reg);
  }

  bool setDacMasterSlave(pcm3060::MasterSlaveMode mode) {
    uint8_t reg = 0;
    if (!readRegister(pcm3060::reg_dac_ctrl1, reg)) { return false; }
    reg = (reg & ~pcm3060::mask_ms) | (static_cast<uint8_t>(mode) << pcm3060::shift_ms);
    return writeRegister(pcm3060::reg_dac_ctrl1, reg);
  }

  bool setAdcMasterSlave(pcm3060::MasterSlaveMode mode) {
    uint8_t reg = 0;
    if (!readRegister(pcm3060::reg_adc_ctrl1, reg)) { return false; }
    reg = (reg & ~pcm3060::mask_ms) | (static_cast<uint8_t>(mode) << pcm3060::shift_ms);
    return writeRegister(pcm3060::reg_adc_ctrl1, reg);
  }

  // DAC attenuation: 0-255, 255=0dB(no attenuation), 54-0=mute
  // Formula: level(dB) = 0.5 * (value - 255)
  bool setDacAttenuation(uint8_t left, uint8_t right) {
    if (!writeRegister(pcm3060::reg_dac_att_l, left)) { return false; }
    return writeRegister(pcm3060::reg_dac_att_r, right);
  }

  // ADC attenuation: 0-255, 215=0dB, 255=+20dB, 14-0=mute
  // Formula: level(dB) = 0.5 * (value - 215)
  bool setAdcAttenuation(uint8_t left, uint8_t right) {
    if (!writeRegister(pcm3060::reg_adc_att_l, left)) { return false; }
    return writeRegister(pcm3060::reg_adc_att_r, right);
  }

  bool setDacMute(bool mute_left, bool mute_right) {
    uint8_t reg = 0;
    if (!readRegister(pcm3060::reg_dac_ctrl2, reg)) { return false; }
    reg = (reg & ~(pcm3060::bit_mut21 | pcm3060::bit_mut22))
        | (mute_left  ? pcm3060::bit_mut21 : 0)
        | (mute_right ? pcm3060::bit_mut22 : 0);
    return writeRegister(pcm3060::reg_dac_ctrl2, reg);
  }

  bool setAdcMute(bool mute_left, bool mute_right) {
    uint8_t reg = 0;
    if (!readRegister(pcm3060::reg_adc_ctrl2, reg)) { return false; }
    reg = (reg & ~(pcm3060::bit_mut11 | pcm3060::bit_mut12))
        | (mute_left  ? pcm3060::bit_mut11 : 0)
        | (mute_right ? pcm3060::bit_mut12 : 0);
    return writeRegister(pcm3060::reg_adc_ctrl2, reg);
  }

  bool enableDac(bool enable) {
    uint8_t reg = 0;
    if (!readRegister(pcm3060::reg_sys_ctrl, reg)) { return false; }
    if (enable) { reg &= ~pcm3060::bit_dapsv; }
    else        { reg |=  pcm3060::bit_dapsv; }
    return writeRegister(pcm3060::reg_sys_ctrl, reg);
  }

  bool enableAdc(bool enable) {
    uint8_t reg = 0;
    if (!readRegister(pcm3060::reg_sys_ctrl, reg)) { return false; }
    if (enable) { reg &= ~pcm3060::bit_adpsv; }
    else        { reg |=  pcm3060::bit_adpsv; }
    return writeRegister(pcm3060::reg_sys_ctrl, reg);
  }

  bool setSingleEnded(bool single_ended) {
    uint8_t reg = 0;
    if (!readRegister(pcm3060::reg_sys_ctrl, reg)) { return false; }
    if (single_ended) { reg |=  pcm3060::bit_se; }
    else              { reg &= ~pcm3060::bit_se; }
    return writeRegister(pcm3060::reg_sys_ctrl, reg);
  }
};


// PCM3060 with SPI control interface (write-only; uses shadow registers)
template<typename SPI_TYPE, typename GPIO_PIN_TYPE, typename WAIT_FUNC = NullFunctor>
struct Pcm3060Spi {
  SPI_TYPE spi;
  GPIO_PIN_TYPE cs_pin; // MS pin (active low)

  WAIT_FUNC _wait_func;
  uint8_t _regs[pcm3060::reg_count] = {};

  void init(pcm3060::AudioFormat fmt = pcm3060::AudioFormat::leftJustified) {
    cs_pin.enable();
    cs_pin.initOutputDefault();
    cs_pin.high();
    _resetShadowRegs();

    // MRST: reset mode control registers
    _regs[0] = pcm3060::default_sys_ctrl & ~pcm3060::bit_mrst;
    writeRegister(pcm3060::reg_sys_ctrl, _regs[0]);
    delay_msec(4);
    _resetShadowRegs(); // all registers back to defaults after MRST

    // SRST: system reset
    _regs[0] &= ~pcm3060::bit_srst;
    writeRegister(pcm3060::reg_sys_ctrl, _regs[0]);
    delay_msec(4);
    _regs[0] |= pcm3060::bit_srst; // SRST auto-returns to 1

    // Set audio format for DAC and ADC
    setDacFormat(fmt);
    setAdcFormat(fmt);

    // Disable power save for ADC and DAC
    _regs[0] &= ~(pcm3060::bit_adpsv | pcm3060::bit_dapsv);
    writeRegister(pcm3060::reg_sys_ctrl, _regs[0]);
  }

  IGB_FAST_INLINE void writeRegister(uint8_t reg, uint8_t data) {
    // SPI 16-bit word: MSB=0(write) + IDX[6:0] + D[7:0]
    cs_pin.low();
    _writeByte(reg & 0x7F);
    _writeByte(data);
    cs_pin.high();
  }

  void setDacFormat(pcm3060::AudioFormat fmt) {
    uint8_t& r = _reg(pcm3060::reg_dac_ctrl1);
    r = (r & ~pcm3060::mask_fmt) | static_cast<uint8_t>(fmt);
    writeRegister(pcm3060::reg_dac_ctrl1, r);
  }

  void setAdcFormat(pcm3060::AudioFormat fmt) {
    uint8_t& r = _reg(pcm3060::reg_adc_ctrl1);
    r = (r & ~pcm3060::mask_fmt) | static_cast<uint8_t>(fmt);
    writeRegister(pcm3060::reg_adc_ctrl1, r);
  }

  void setDacMasterSlave(pcm3060::MasterSlaveMode mode) {
    uint8_t& r = _reg(pcm3060::reg_dac_ctrl1);
    r = (r & ~pcm3060::mask_ms) | (static_cast<uint8_t>(mode) << pcm3060::shift_ms);
    writeRegister(pcm3060::reg_dac_ctrl1, r);
  }

  void setAdcMasterSlave(pcm3060::MasterSlaveMode mode) {
    uint8_t& r = _reg(pcm3060::reg_adc_ctrl1);
    r = (r & ~pcm3060::mask_ms) | (static_cast<uint8_t>(mode) << pcm3060::shift_ms);
    writeRegister(pcm3060::reg_adc_ctrl1, r);
  }

  void setDacAttenuation(uint8_t left, uint8_t right) {
    _reg(pcm3060::reg_dac_att_l) = left;
    _reg(pcm3060::reg_dac_att_r) = right;
    writeRegister(pcm3060::reg_dac_att_l, left);
    writeRegister(pcm3060::reg_dac_att_r, right);
  }

  void setAdcAttenuation(uint8_t left, uint8_t right) {
    _reg(pcm3060::reg_adc_att_l) = left;
    _reg(pcm3060::reg_adc_att_r) = right;
    writeRegister(pcm3060::reg_adc_att_l, left);
    writeRegister(pcm3060::reg_adc_att_r, right);
  }

  void setDacMute(bool mute_left, bool mute_right) {
    uint8_t& r = _reg(pcm3060::reg_dac_ctrl2);
    r = (r & ~(pcm3060::bit_mut21 | pcm3060::bit_mut22))
      | (mute_left  ? pcm3060::bit_mut21 : 0)
      | (mute_right ? pcm3060::bit_mut22 : 0);
    writeRegister(pcm3060::reg_dac_ctrl2, r);
  }

  void setAdcMute(bool mute_left, bool mute_right) {
    uint8_t& r = _reg(pcm3060::reg_adc_ctrl2);
    r = (r & ~(pcm3060::bit_mut11 | pcm3060::bit_mut12))
      | (mute_left  ? pcm3060::bit_mut11 : 0)
      | (mute_right ? pcm3060::bit_mut12 : 0);
    writeRegister(pcm3060::reg_adc_ctrl2, r);
  }

  void enableDac(bool enable) {
    uint8_t& r = _reg(pcm3060::reg_sys_ctrl);
    if (enable) { r &= ~pcm3060::bit_dapsv; }
    else        { r |=  pcm3060::bit_dapsv; }
    writeRegister(pcm3060::reg_sys_ctrl, r);
  }

  void enableAdc(bool enable) {
    uint8_t& r = _reg(pcm3060::reg_sys_ctrl);
    if (enable) { r &= ~pcm3060::bit_adpsv; }
    else        { r |=  pcm3060::bit_adpsv; }
    writeRegister(pcm3060::reg_sys_ctrl, r);
  }

  void setSingleEnded(bool single_ended) {
    uint8_t& r = _reg(pcm3060::reg_sys_ctrl);
    if (single_ended) { r |=  pcm3060::bit_se; }
    else              { r &= ~pcm3060::bit_se; }
    writeRegister(pcm3060::reg_sys_ctrl, r);
  }

  IGB_FAST_INLINE void _writeByte(uint8_t byte) {
    volatile uint8_t tmp IGB_UNUSED = spi.transferU8sync(byte, _wait_func);
  }

  uint8_t& _reg(uint8_t addr) {
    return _regs[addr - pcm3060::reg_sys_ctrl];
  }

  void _resetShadowRegs() {
    _regs[0] = pcm3060::default_sys_ctrl;
    _regs[1] = pcm3060::default_dac_att;
    _regs[2] = pcm3060::default_dac_att;
    _regs[3] = pcm3060::default_dac_ctrl1;
    _regs[4] = pcm3060::default_dac_ctrl2;
    _regs[5] = pcm3060::default_dig_ctrl;
    _regs[6] = pcm3060::default_adc_att;
    _regs[7] = pcm3060::default_adc_att;
    _regs[8] = pcm3060::default_adc_ctrl1;
    _regs[9] = pcm3060::default_adc_ctrl2;
  }
};

} // namespace sdk
} // namespace igb
