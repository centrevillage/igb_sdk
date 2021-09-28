#ifndef IGB_STM32_PERIPH_FAST_GPIO_H
#define IGB_STM32_PERIPH_FAST_GPIO_H

#include <tuple>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/syscfg.hpp>
#include <igb_stm32/periph/exti.hpp>
#include <igb_util/reg.hpp>
#include <igb_util/macro.hpp>

// template version GPIO api

namespace igb {
namespace stm32 {

#define IGB_GPIO ((GPIO_TypeDef*)addr)
#define IGB_GPIO_REG_ADDR(member) (addr + offsetof(GPIO_TypeDef, member))
#define IGB_GPIO_REG(member) ((GPIO_TypeDef*)IGB_GPIO_REG_ADDR(member))

template<GpioType gpio_type>
struct FastGpioPort {
  constexpr static auto type = gpio_type;
  constexpr static auto info = STM32_PERIPH_INFO.gpio[to_idx(type)];
  constexpr static auto addr = STM32_PERIPH_INFO.gpio[to_idx(type)].addr;
  constexpr static auto addr_MODER = IGB_GPIO_REG_ADDR(MODER);
  constexpr static auto addr_OTYPER = IGB_GPIO_REG_ADDR(OTYPER);
  constexpr static auto addr_OSPEEDR = IGB_GPIO_REG_ADDR(OSPEEDR);
  constexpr static auto addr_PUPDR = IGB_GPIO_REG_ADDR(PUPDR);
  constexpr static auto addr_IDR = IGB_GPIO_REG_ADDR(IDR);
  constexpr static auto addr_ODR = IGB_GPIO_REG_ADDR(ODR);
  constexpr static auto addr_BSRR = IGB_GPIO_REG_ADDR(BSRR);
  constexpr static auto addr_LCKR = IGB_GPIO_REG_ADDR(LCKR);
  constexpr static auto addr_AFR0 = IGB_GPIO_REG_ADDR(AFR[0]);
  constexpr static auto addr_AFR1 = IGB_GPIO_REG_ADDR(AFR[1]);
#ifdef STM32_PERIPH_GPIO_REG_BRR_EXISTS
  constexpr static auto addr_BRR = IGB_GPIO_REG_ADDR(BRR);
#endif

  std::tuple<
// #!ruby
//0.step(30, 2) do |i|
//print <<-EOS
//    RegEnum<addr_MODER, (0x00000003UL << #{i}), GpioMode, #{i}>,
//EOS
//end
    RegEnum<addr_MODER, (0x00000003UL << 0), GpioMode, 0>,
    RegEnum<addr_MODER, (0x00000003UL << 2), GpioMode, 2>,
    RegEnum<addr_MODER, (0x00000003UL << 4), GpioMode, 4>,
    RegEnum<addr_MODER, (0x00000003UL << 6), GpioMode, 6>,
    RegEnum<addr_MODER, (0x00000003UL << 8), GpioMode, 8>,
    RegEnum<addr_MODER, (0x00000003UL << 10), GpioMode, 10>,
    RegEnum<addr_MODER, (0x00000003UL << 12), GpioMode, 12>,
    RegEnum<addr_MODER, (0x00000003UL << 14), GpioMode, 14>,
    RegEnum<addr_MODER, (0x00000003UL << 16), GpioMode, 16>,
    RegEnum<addr_MODER, (0x00000003UL << 18), GpioMode, 18>,
    RegEnum<addr_MODER, (0x00000003UL << 20), GpioMode, 20>,
    RegEnum<addr_MODER, (0x00000003UL << 22), GpioMode, 22>,
    RegEnum<addr_MODER, (0x00000003UL << 24), GpioMode, 24>,
    RegEnum<addr_MODER, (0x00000003UL << 26), GpioMode, 26>,
    RegEnum<addr_MODER, (0x00000003UL << 28), GpioMode, 28>,
    RegEnum<addr_MODER, (0x00000003UL << 30), GpioMode, 30>
  > modes;

  std::tuple<
    RegEnum<addr_OTYPER, 0, GpioOutputMode, 0>,
    RegEnum<addr_OTYPER, 1, GpioOutputMode, 1>,
    RegEnum<addr_OTYPER, 2, GpioOutputMode, 2>,
    RegEnum<addr_OTYPER, 3, GpioOutputMode, 3>,
    RegEnum<addr_OTYPER, 4, GpioOutputMode, 4>,
    RegEnum<addr_OTYPER, 5, GpioOutputMode, 5>,
    RegEnum<addr_OTYPER, 6, GpioOutputMode, 6>,
    RegEnum<addr_OTYPER, 7, GpioOutputMode, 7>,
    RegEnum<addr_OTYPER, 8, GpioOutputMode, 8>,
    RegEnum<addr_OTYPER, 9, GpioOutputMode, 9>,
    RegEnum<addr_OTYPER, 10, GpioOutputMode, 10>,
    RegEnum<addr_OTYPER, 11, GpioOutputMode, 11>,
    RegEnum<addr_OTYPER, 12, GpioOutputMode, 12>,
    RegEnum<addr_OTYPER, 13, GpioOutputMode, 13>,
    RegEnum<addr_OTYPER, 14, GpioOutputMode, 14>,
    RegEnum<addr_OTYPER, 15, GpioOutputMode, 15>
  > outputModes;

  std::tuple<
// #!ruby
//0.step(30, 2) do |i|
//print <<-EOS
//    RegEnum<addr_PUPDR, (0x00000003UL << #{i}), GpioPullMode, #{i}>,
//EOS
//end
    RegEnum<addr_PUPDR, (0x00000003UL << 0), GpioPullMode, 0>,
    RegEnum<addr_PUPDR, (0x00000003UL << 2), GpioPullMode, 2>,
    RegEnum<addr_PUPDR, (0x00000003UL << 4), GpioPullMode, 4>,
    RegEnum<addr_PUPDR, (0x00000003UL << 6), GpioPullMode, 6>,
    RegEnum<addr_PUPDR, (0x00000003UL << 8), GpioPullMode, 8>,
    RegEnum<addr_PUPDR, (0x00000003UL << 10), GpioPullMode, 10>,
    RegEnum<addr_PUPDR, (0x00000003UL << 12), GpioPullMode, 12>,
    RegEnum<addr_PUPDR, (0x00000003UL << 14), GpioPullMode, 14>,
    RegEnum<addr_PUPDR, (0x00000003UL << 16), GpioPullMode, 16>,
    RegEnum<addr_PUPDR, (0x00000003UL << 18), GpioPullMode, 18>,
    RegEnum<addr_PUPDR, (0x00000003UL << 20), GpioPullMode, 20>,
    RegEnum<addr_PUPDR, (0x00000003UL << 22), GpioPullMode, 22>,
    RegEnum<addr_PUPDR, (0x00000003UL << 24), GpioPullMode, 24>,
    RegEnum<addr_PUPDR, (0x00000003UL << 26), GpioPullMode, 26>,
    RegEnum<addr_PUPDR, (0x00000003UL << 28), GpioPullMode, 28>,
    RegEnum<addr_PUPDR, (0x00000003UL << 30), GpioPullMode, 30>
  > pullModes;

  std::tuple<
// #!ruby
//0.step(30, 2) do |i|
//print <<-EOS
//    RegEnum<addr_OSPEEDR, (0x00000003UL << #{i}), GpioSpeedMode, #{i}>,
//EOS
//end
    RegEnum<addr_OSPEEDR, (0x00000003UL << 0), GpioSpeedMode, 0>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 2), GpioSpeedMode, 2>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 4), GpioSpeedMode, 4>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 6), GpioSpeedMode, 6>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 8), GpioSpeedMode, 8>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 10), GpioSpeedMode, 10>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 12), GpioSpeedMode, 12>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 14), GpioSpeedMode, 14>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 16), GpioSpeedMode, 16>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 18), GpioSpeedMode, 18>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 20), GpioSpeedMode, 20>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 22), GpioSpeedMode, 22>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 24), GpioSpeedMode, 24>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 26), GpioSpeedMode, 26>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 28), GpioSpeedMode, 28>,
    RegEnum<addr_OSPEEDR, (0x00000003UL << 30), GpioSpeedMode, 30>
  > speedModes;

// #!ruby
//0.step(32-4, 4) do |i|
//print <<-EOS
//    RegEnum<addr_AFR0, (0x0000000FUL << #{i}), GpioAf, #{i}>,
//EOS
//end
//0.step(32-4, 4) do |i|
//print <<-EOS
//    RegEnum<addr_AFR1, (0x0000000FUL << #{i}), GpioAf, #{i}>,
//EOS
//end
  std::tuple<
    RegEnum<addr_AFR0, (0x0000000FUL << 0), GpioAf, 0>,
    RegEnum<addr_AFR0, (0x0000000FUL << 4), GpioAf, 4>,
    RegEnum<addr_AFR0, (0x0000000FUL << 8), GpioAf, 8>,
    RegEnum<addr_AFR0, (0x0000000FUL << 12), GpioAf, 12>,
    RegEnum<addr_AFR0, (0x0000000FUL << 16), GpioAf, 16>,
    RegEnum<addr_AFR0, (0x0000000FUL << 20), GpioAf, 20>,
    RegEnum<addr_AFR0, (0x0000000FUL << 24), GpioAf, 24>,
    RegEnum<addr_AFR0, (0x0000000FUL << 28), GpioAf, 28>,
    RegEnum<addr_AFR1, (0x0000000FUL << 0), GpioAf, 0>,
    RegEnum<addr_AFR1, (0x0000000FUL << 4), GpioAf, 4>,
    RegEnum<addr_AFR1, (0x0000000FUL << 8), GpioAf, 8>,
    RegEnum<addr_AFR1, (0x0000000FUL << 12), GpioAf, 12>,
    RegEnum<addr_AFR1, (0x0000000FUL << 16), GpioAf, 16>,
    RegEnum<addr_AFR1, (0x0000000FUL << 20), GpioAf, 20>,
    RegEnum<addr_AFR1, (0x0000000FUL << 24), GpioAf, 24>,
    RegEnum<addr_AFR1, (0x0000000FUL << 28), GpioAf, 28>
  > alternateFuncs;

  RegFlag<addr_LCKR, GPIO_LCKR_LCKK> lockKey;
  std::tuple<
// #!ruby
//(0..15).each do |i|
//print <<-EOS
//    RegFlag<addr_LCKR, (1UL << #{i})>,
//EOS
//end
    RegFlag<addr_LCKR, (1UL << 0)>,
    RegFlag<addr_LCKR, (1UL << 1)>,
    RegFlag<addr_LCKR, (1UL << 2)>,
    RegFlag<addr_LCKR, (1UL << 3)>,
    RegFlag<addr_LCKR, (1UL << 4)>,
    RegFlag<addr_LCKR, (1UL << 5)>,
    RegFlag<addr_LCKR, (1UL << 6)>,
    RegFlag<addr_LCKR, (1UL << 7)>,
    RegFlag<addr_LCKR, (1UL << 8)>,
    RegFlag<addr_LCKR, (1UL << 9)>,
    RegFlag<addr_LCKR, (1UL << 10)>,
    RegFlag<addr_LCKR, (1UL << 11)>,
    RegFlag<addr_LCKR, (1UL << 12)>,
    RegFlag<addr_LCKR, (1UL << 13)>,
    RegFlag<addr_LCKR, (1UL << 14)>,
    RegFlag<addr_LCKR, (1UL << 15)>
  > locks;

  Reg<addr_LCKR> lckr;

  Reg<addr_BSRR> bsrr;
  IGB_FAST_INLINE void on(uint32_t bits) {
    bsrr(bits);
  }
  IGB_FAST_INLINE void high(uint32_t bits) {
    bsrr(bits);
  }

#ifdef STM32_PERIPH_GPIO_REG_BRR_EXISTS
  Reg<addr_BRR> brr;
#endif
  IGB_FAST_INLINE void off(uint32_t bits) {
#ifdef STM32_PERIPH_GPIO_REG_BRR_EXISTS
    brr(bits);
#else
    bsrr(bits << 16);
#endif
  }
  IGB_FAST_INLINE void low(uint32_t bits) {
    off(bits);
  }

  RegRO<addr_IDR> idr;
  IGB_FAST_INLINE uint32_t read() {
    return idr();
  }
  RegRO<addr_ODR> odr;
  IGB_FAST_INLINE uint32_t readOutput() {
    return odr();
  }

  IGB_FAST_INLINE void enable() {
    STM32_PERIPH_INFO.gpio[to_idx(type)].bus.enableBusClock();
  }
};

template<GpioPinType pin_type>
struct FastGpioPin {
  constexpr static auto type = pin_type;
  constexpr static auto pin_idx = extract_pin_idx(pin_type);
  constexpr static uint32_t pin_bit = 1UL << pin_idx;
  constexpr static auto port_type = extract_gpio_type(pin_type);
  auto static port = FastGpioPort<port_type> {};

  IGB_FAST_INLINE void setMode(GpioMode mode) {
    auto& v = std::get<pin_idx>(port.modes);
    v(mode);
  }

  IGB_FAST_INLINE void setOutputMode(GpioOutputMode mode) {
    auto& v = std::get<pin_idx>(port.outputModes);
    v(mode);
  }

  IGB_FAST_INLINE void setPullMode(GpioPullMode mode) {
    auto& v = std::get<pin_idx>(port.pullModes);
    v(mode);
  }

  IGB_FAST_INLINE void setSpeedMode(GpioSpeedMode mode) {
    auto& v = std::get<pin_idx>(port.speedModes);
    v(mode);
  }

  IGB_FAST_INLINE void setAlternateFunc(GpioAf af) {
    auto& v = std::get<pin_idx>(port.alternateFuncs);
    v(af);
  }

  IGB_FAST_INLINE void lock() {
    auto& lk = std::get<pin_idx>(port.locks);
    (port.lockKey.val(true)  | lk.val(true)).update();
    (port.lockKey.val(false) | lk.val(true)).update();
    (port.lockKey.val(true)  | lk.val(true)).update();
    // avoid optimization
    volatile uint32_t temp IGB_UNUSED = port.lckr();
  }

  IGB_FAST_INLINE void on() {
    port.on(pin_bit);
  }

  IGB_FAST_INLINE void high() {
    on();
  }

  IGB_FAST_INLINE void off() {
    port.off(pin_bit);
  }

  IGB_FAST_INLINE void low() {
    off();
  }

  IGB_FAST_INLINE void write(bool flag) {
    if (flag) {
      on();
    } else {
      off();
    }
  }

  IGB_FAST_INLINE bool read() {
    return port.read() & pin_bit;
  }

  IGB_FAST_INLINE bool readOutput() {
    return port.readOutput() & (pin_bit);
  }

  IGB_FAST_INLINE void enable() {
    port.enable();
  }

  IGB_FAST_INLINE void enableExti(ExtiTrigType trig_type, ExtiMode mode, uint8_t priority) {
    switch (pin_idx) {
      case 0:
        SysCfg::exti0GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line0, trig_type, mode, priority);
        break;
      case 1:
        SysCfg::exti1GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line1, trig_type, mode, priority);
        break;
      case 2:
        SysCfg::exti2GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line2, trig_type, mode, priority);
        break;
      case 3:
        SysCfg::exti3GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line3, trig_type, mode, priority);
        break;
      case 4:
        SysCfg::exti4GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line4, trig_type, mode, priority);
        break;
      case 5:
        SysCfg::exti5GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line5, trig_type, mode, priority);
        break;
      case 6:
        SysCfg::exti6GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line6, trig_type, mode, priority);
        break;
      case 7:
        SysCfg::exti7GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line7, trig_type, mode, priority);
        break;
      case 8:
        SysCfg::exti8GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line8, trig_type, mode, priority);
        break;
      case 9:
        SysCfg::exti9GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line9, trig_type, mode, priority);
        break;
      case 10:
        SysCfg::exti10GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line10, trig_type, mode, priority);
        break;
      case 11:
        SysCfg::exti11GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line11, trig_type, mode, priority);
        break;
      case 12:
        SysCfg::exti12GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line12, trig_type, mode, priority);
        break;
      case 13:
        SysCfg::exti13GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line13, trig_type, mode, priority);
        break;
      case 14:
        SysCfg::exti14GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line14, trig_type, mode, priority);
        break;
      case 15:
        SysCfg::exti15GpioPort(port_type);
        ExtiCtrl::enableLine(ExtiLine::line15, trig_type, mode, priority);
        break;
      default:
        break;
    }
  }

  IGB_FAST_INLINE void initInput(GpioPullMode pull, GpioSpeedMode speed) {
    setMode(GpioMode::input);
    setPullMode(pull);
    setSpeedMode(speed);
  }

  IGB_FAST_INLINE void initInputDefault() {
    initInput(GpioPullMode::no, GpioSpeedMode::high);
  }

  IGB_FAST_INLINE void initOutput(GpioOutputMode output_mode, GpioSpeedMode speed) {
    setMode(GpioMode::output);
    setOutputMode(output_mode);
    setPullMode(GpioPullMode::no);
    setSpeedMode(speed);
  }

  IGB_FAST_INLINE void initOutputDefault() {
    initOutput(GpioOutputMode::pushpull, GpioSpeedMode::high);
  }
};

#undef IGB_GPIO_REG
#undef IGB_GPIO_REG_ADDR
#undef IGB_GPIO

}
}


#endif /* IGB_STM32_PERIPH_FAST_GPIO_H */
