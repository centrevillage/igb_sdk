#pragma once

#include <array>
#include <cstdint>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_util/macro.hpp>

namespace igb::sdk {

enum class MuxIOType : uint8_t {
  input = 0,
  output
};

struct MuxIOConfig {
  igb::stm32::GpioPinType pin_type;
  MuxIOType io_type;
};

template <igb::stm32::GpioPinType... pins>
struct AddrPinList {};

template <MuxIOConfig... cfgs>
struct IOConfigList {};

// NOTE*
// 同一のアドレスで複数のマルチプレキサ(4051等)を制御する。
// 各マルチプレキサはそれぞれのGroupとして扱う。
template <uint32_t wait_tick, typename AddrPinListT, typename IOConfigListT>
struct MuxIO;

template <uint32_t wait_tick, igb::stm32::GpioPinType... addr_pin_types, MuxIOConfig... configs>
struct MuxIO<wait_tick, AddrPinList<addr_pin_types...>, IOConfigList<configs...>> {
  constexpr static std::size_t addr_pins_size = sizeof...(addr_pin_types);
  constexpr static std::size_t mux_size = (1 << addr_pins_size);
  constexpr static std::size_t configs_size = sizeof...(configs);

  static_assert(addr_pins_size > 0, "addr_pin_types is blank!");
  static_assert(configs_size > 0, "configs is blank!");

  struct GroupState {
    MuxIOConfig config;
    igb::stm32::GpioPin pin;
    std::array<bool, mux_size> state;
  };

  std::array<igb::stm32::GpioPin, addr_pins_size> addr_pins = {
    igb::stm32::GpioPin::newPin(addr_pin_types)...
  };
  std::array<GroupState, configs_size> group_states = {
    GroupState{ configs, igb::stm32::GpioPin::newPin(configs.pin_type), {} }...
  };

  uint8_t address_idx = 0;
  // Issue #104: last tick at which the address was stepped. Replaces the soft
  // timer so process() can de-bunch (see process()).
  uint32_t _last_step_tick = 0;

  void init(uint32_t tick) {
    address_idx = 0;
    _last_step_tick = tick;
    for (auto& pin : addr_pins) {
      pin.enable();
      pin.initOutputDefault();
      pin.off();
    }
    for (auto& gs : group_states) {
      gs.pin.enable();
      if (gs.config.io_type == MuxIOType::input) {
        gs.pin.initInput(igb::stm32::GpioPullMode::up, igb::stm32::GpioSpeedMode::veryHigh);
      } else {
        gs.pin.initOutputDefault();
        gs.pin.off();
      }
    }

  }

  // アドレス変更処理。
  void updateAddress() {
    for (auto& gs : group_states) {
      if (gs.config.io_type == MuxIOType::input) {
        gs.state[address_idx] = gs.pin.read();
      }
    }

    // address_idxをインクリメントしてaddr_pinsに反映
    address_idx = (address_idx + 1) % mux_size;

    for (auto& gs : group_states) {
      if (gs.config.io_type == MuxIOType::output && !gs.state[address_idx]) {
        gs.pin.off();
      }
    }

    for (std::size_t i = 0; i < addr_pins_size; ++i) {
      addr_pins[i].write((address_idx >> i) & 1);
    }

    // 状態がtrueのoutputはアドレス変更の直後に行う
    for (auto& gs : group_states) {
      if (gs.config.io_type == MuxIOType::output && gs.state[address_idx]) {
        gs.pin.on();
      }
    }
  }

  const std::array<bool, mux_size>& getState(std::size_t group_idx) const {
    return group_states[group_idx].state;
  }

  bool getState(std::size_t group_idx, std::size_t addr_idx) const {
    return group_states[group_idx].state[addr_idx];
  }

  void setState(std::size_t group_idx, std::size_t addr_idx, bool value) {
    group_states[group_idx].state[addr_idx] = value;
  }

  // Issue #104: step at most one address per call, gated by wait_tick. A long
  // interrupt (the ~1ms audio DMA IRQ runs at NVIC priority 0 and cannot be
  // preempted) blocks the main loop and leaves us several intervals behind.
  // The old soft timer caught up by firing once per (fast) main-loop iteration
  // afterwards, bunching many steps into a burst that under-lit those
  // addresses and made the LED brightness shimmer. Here a gap longer than one
  // interval is treated as an overrun: take a single step and drop the
  // accumulated lag (resume a full interval from now) instead of bursting.
  // Sub-interval lateness still advances by exactly wait_tick to hold cadence.
  // (The address frozen *during* the blocking IRQ is still held longer than a
  // normal slot — that residual needs stepping inside the audio IRQ, out of
  // scope here; see docs/104.)
  void process(uint32_t tick) {
    uint32_t elapsed = tick - _last_step_tick;
    if (elapsed < wait_tick) {
      return;
    }
    updateAddress();
    _last_step_tick =
      (elapsed >= 2u * wait_tick) ? tick : (_last_step_tick + wait_tick);
  }
};

}  // namespace igb::sdk
