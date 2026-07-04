#pragma once

// SDRAM init for Daisy Seed2 DFM (and similar Daisy boards using
// AS4C16M32MSA-6BIN: Alliance Memory, 64 MB, 32-bit).
//
// FMC Bank 1 (SDRAM), SDCLK = FMC_CLK / 2 = PLL2R / 2 = 100 MHz.
// Timing values empirically tuned (matches libDaisy sdram.cpp).

#include <cstdint>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/fmc.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/systick.hpp>

namespace igb::daisy {

struct Sdram {
  igb::stm32::FmcSdram<0> sdram;  // Bank 1 (index 0)

  void initGpio() {
    // All FMC pins: AF12, push-pull, very high speed, no pull
    constexpr igb::stm32::GpioPinType fmc_pins[] = {
      // Port C: SDNWE
      igb::stm32::GpioPinType::pc0,
      // Port D: D2, D3, D1, D0, D15, D14, D13
      igb::stm32::GpioPinType::pd0,  igb::stm32::GpioPinType::pd1,
      igb::stm32::GpioPinType::pd8,  igb::stm32::GpioPinType::pd9,
      igb::stm32::GpioPinType::pd10, igb::stm32::GpioPinType::pd14,
      igb::stm32::GpioPinType::pd15,
      // Port E: NBL0, NBL1, D4-D12
      igb::stm32::GpioPinType::pe0,  igb::stm32::GpioPinType::pe1,
      igb::stm32::GpioPinType::pe7,  igb::stm32::GpioPinType::pe8,
      igb::stm32::GpioPinType::pe9,  igb::stm32::GpioPinType::pe10,
      igb::stm32::GpioPinType::pe11, igb::stm32::GpioPinType::pe12,
      igb::stm32::GpioPinType::pe13, igb::stm32::GpioPinType::pe14,
      igb::stm32::GpioPinType::pe15,
      // Port F: A0-A9, SDNRAS
      igb::stm32::GpioPinType::pf0,  igb::stm32::GpioPinType::pf1,
      igb::stm32::GpioPinType::pf2,  igb::stm32::GpioPinType::pf3,
      igb::stm32::GpioPinType::pf4,  igb::stm32::GpioPinType::pf5,
      igb::stm32::GpioPinType::pf11, igb::stm32::GpioPinType::pf12,
      igb::stm32::GpioPinType::pf13, igb::stm32::GpioPinType::pf14,
      igb::stm32::GpioPinType::pf15,
      // Port G: A10, A11, A12, BA0, BA1, SDCLK, SDNCAS
      igb::stm32::GpioPinType::pg0,  igb::stm32::GpioPinType::pg1,
      igb::stm32::GpioPinType::pg2,  igb::stm32::GpioPinType::pg4,
      igb::stm32::GpioPinType::pg5,  igb::stm32::GpioPinType::pg8,
      igb::stm32::GpioPinType::pg15,
      // Port H: SDCKE0, SDNE0, D5(PH5), D16-D23
      igb::stm32::GpioPinType::ph2,  igb::stm32::GpioPinType::ph3,
      igb::stm32::GpioPinType::ph5,
      igb::stm32::GpioPinType::ph8,  igb::stm32::GpioPinType::ph9,
      igb::stm32::GpioPinType::ph10, igb::stm32::GpioPinType::ph11,
      igb::stm32::GpioPinType::ph12, igb::stm32::GpioPinType::ph13,
      igb::stm32::GpioPinType::ph14, igb::stm32::GpioPinType::ph15,
      // Port I: D24-D31, NBL2, NBL3
      igb::stm32::GpioPinType::pi0,  igb::stm32::GpioPinType::pi1,
      igb::stm32::GpioPinType::pi2,  igb::stm32::GpioPinType::pi3,
      igb::stm32::GpioPinType::pi4,  igb::stm32::GpioPinType::pi5,
      igb::stm32::GpioPinType::pi6,  igb::stm32::GpioPinType::pi7,
      igb::stm32::GpioPinType::pi9,  igb::stm32::GpioPinType::pi10,
    };

    for (auto pin_type : fmc_pins) {
      auto pin = igb::stm32::GpioPin::newPin(pin_type);
      pin.enable();
      pin.setMode(igb::stm32::GpioMode::alternate);
      pin.setPullMode(igb::stm32::GpioPullMode::no);
      pin.setSpeedMode(igb::stm32::GpioSpeedMode::veryHigh);
      pin.setOutputMode(igb::stm32::GpioOutputMode::pushpull);
      pin.setAlternateFunc(igb::stm32::GpioAf::af12); // AF12 = FMC
    }
  }

  void init() {
    igb::stm32::Fmc::enableBusClock();
    initGpio();

    // SDRAM control: Bank1, 9-col, 13-row, 32-bit, 4 internal banks, CAS=3
    sdram.initControl({
      .columnBits     = igb::stm32::FmcSdramColumnBits::_9bit,
      .rowBits        = igb::stm32::FmcSdramRowBits::_13bit,
      .dataWidth      = igb::stm32::FmcSdramDataWidth::_32bit,
      .internalBanks  = igb::stm32::FmcSdramInternalBanks::_4banks,
      .casLatency     = igb::stm32::FmcSdramCasLatency::_3clk,
      .clockPeriod    = igb::stm32::FmcSdramClockPeriod::_2clk,  // SDCLK = FMC_CLK / 2 = 100 MHz
      .enableReadBurst = true,
      .readPipeDelay  = igb::stm32::FmcSdramReadPipeDelay::_0clk,
    });

    // Timing — empirically tuned values from libDaisy sdram.cpp
    sdram.initTiming({
      .loadToActive    = 2,
      .exitSelfRefresh = 7,
      .selfRefreshTime = 4,
      .rowCycleDelay   = 8,   // started at 7, tuned up
      .writeRecovery   = 3,
      .rpDelay         = 16,
      .rcdDelay        = 10,  // started at 2, tuned up (critical)
    });

    igb::stm32::Fmc::enable();

    // --- Device initialization sequence (RM0433 §22.7.4) ---

    // Step 1: Clock Configuration Enable
    sdram.sendCommand(igb::stm32::FmcSdramCommandMode::clockEnable);
    // Step 2: Wait >= 100 us
    delay_msec(1);

    // Step 3: PALL (Precharge All)
    sdram.sendCommand(igb::stm32::FmcSdramCommandMode::pall);

    // Step 4: Auto-Refresh (x4)
    sdram.sendCommand(igb::stm32::FmcSdramCommandMode::autoRefresh, 4);

    // Step 5: Load Mode Register
    // Burst Length=4, Sequential, CAS Latency=3, Write Burst=Single
    constexpr uint32_t mode_reg =
      (1 << 1)               // Burst Length = 4
      | (0 << 3)             // Burst Type = Sequential
      | ((1 << 4) | (1 << 5)) // CAS Latency = 3
      | (1 << 9);            // Write Burst Mode = Single
    sdram.sendCommand(igb::stm32::FmcSdramCommandMode::loadModeReg, 1, mode_reg);

    // Step 6: Set refresh rate (libDaisy: 0x81A - 20 = 2054)
    sdram.refreshCount(2054);

    // Wait for SDRAM to be ready
    while (sdram.getModeStatus() != igb::stm32::FmcSdramModeStatus::normal) {}
  }
};

}  // namespace igb::daisy
