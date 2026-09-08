#pragma once

#include <cstdint>
#include <igb_util/macro.hpp>
#include <igb_stm32/periph/systick.hpp>

namespace igb::sdk {

// WM8731 codec control over I2C (2-wire mode; CSB low -> address 0x1A).
//
// Register values and their order replicate libDaisy v5.3
// dev/codec_wm8731.cpp for the Daisy Seed 1.1: the MCU is the I2S master,
// 24-bit MSB-justified (left-justified) data, line inputs at 0 dB, headphone
// outputs muted, DAC selected, no de-emphasis, MIC / oscillator / CLKOUT
// powered down, sample-rate register left at the 48 kHz code (the SAI's MCLK
// decides the real rate; libDaisy ran 96 kHz that way). Every write is
// followed by the same 10 ms wait libDaisy used.
//
// The I2C peripheral (`i2c`) has to be initialised by the caller with the
// timing that matches its kernel clock, e.g.
//   Wm8731I2c<I2c<I2cType::i2c2, GpioPinType::ph4, GpioPinType::pb11>> codec;
//   codec.i2c.init(I2cConf { .timing = 0x30B00F2D /* 400 kHz @ 120 MHz */ });
//   codec.init();
template<typename I2C_TYPE>
struct Wm8731I2c {
  I2C_TYPE i2c;

  constexpr static uint8_t address_csb_low  = 0x1A;
  constexpr static uint8_t address_csb_high = 0x1B;

  enum class Reg : uint8_t {
    left_line_in     = 0x00,
    right_line_in    = 0x01,
    left_hp_out      = 0x02,
    right_hp_out     = 0x03,
    analogue_routing = 0x04,
    digital_routing  = 0x05,
    power            = 0x06,
    digital_format   = 0x07,
    sample_rate      = 0x08,
    active           = 0x09,
    reset            = 0x0F,
  };

  // register values (libDaisy names in comments)
  constexpr static uint16_t line_in_0db        = 0x17;               // CODEC_INPUT_0_DB
  constexpr static uint16_t hp_mute            = 0x00;               // CODEC_HEADPHONES_MUTE
  constexpr static uint16_t routing_line_dac   = 0x02 | 0x00 | 0x10; // MIC_MUTE | ADC_LINE | OUTPUT_DAC_ENABLE
  constexpr static uint16_t deemphasis_none    = 0x00;
  constexpr static uint16_t power_down_mcu_master = 0x02 | 0x40 | 0x20; // MIC | CLOCK_OUTPUT | OSCILLATOR
  constexpr static uint16_t format_lj_24bit_slave = 0x01 | (0x02 << 2) | 0x00; // MSB_FIRST_LJ | 24 BIT | SLAVE
  constexpr static uint16_t rate_48k           = 0x00 << 2;          // CODEC_RATE_48K_48K
  constexpr static uint32_t write_wait_msec    = 10;

  uint8_t _address = address_csb_low;

  // 7-bit register address followed by 9-bit data, MSB first (two bytes)
  bool writeReg(Reg reg, uint16_t data) {
    const uint8_t byte1 = (uint8_t)((((uint8_t)reg << 1) & 0xFE) | ((data >> 8) & 0x01));
    const uint8_t byte2 = (uint8_t)(data & 0xFF);
    i2c.beginSending(_address, 2);
    bool ok = i2c.sendU8sync(byte1);
    ok = i2c.sendU8sync(byte2) && ok;
    ok = i2c.endSending() && ok;
    delay_msec(write_wait_msec);
    return ok;
  }

  // ret: true = every register write was acknowledged
  bool init(uint8_t address = address_csb_low) {
    _address = address;
    bool ok = true;
    ok = writeReg(Reg::reset, 0x0000) && ok;
    ok = writeReg(Reg::left_line_in, line_in_0db) && ok;
    ok = writeReg(Reg::right_line_in, line_in_0db) && ok;
    ok = writeReg(Reg::left_hp_out, hp_mute) && ok;
    ok = writeReg(Reg::right_hp_out, hp_mute) && ok;
    ok = writeReg(Reg::analogue_routing, routing_line_dac) && ok;
    ok = writeReg(Reg::digital_routing, deemphasis_none) && ok;
    ok = writeReg(Reg::power, power_down_mcu_master) && ok;
    ok = writeReg(Reg::digital_format, format_lj_24bit_slave) && ok;
    ok = writeReg(Reg::sample_rate, rate_48k) && ok;
    ok = writeReg(Reg::active, 0x0000) && ok;
    ok = writeReg(Reg::active, 0x0001) && ok;
    return ok;
  }
};

}  // namespace igb::sdk
