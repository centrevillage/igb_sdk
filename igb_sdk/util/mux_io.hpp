#pragma once

#include <array>
#include <cstdint>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_util/macro.hpp>
#include <igb_sdk/util/soft_timer.hpp>

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
  SoftTimerSingle timer;

  void init(uint32_t tick) {
    address_idx = 0;
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

    timer.intervalCallback(wait_tick, tick, [this](){
      updateAddress();
    });
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

  void process(uint32_t tick) {
    timer.process(tick);
  }
};

}  // namespace igb::sdk
