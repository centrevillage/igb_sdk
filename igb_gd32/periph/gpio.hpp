#ifndef IGB_GD32_PERIPH_GPIO_H
#define IGB_GD32_PERIPH_GPIO_H

#include <igb_gd32/base.hpp>
//#include <igb_gd32/periph/syscfg.hpp>
//#include <igb_gd32/periph/exti.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace gd32 {

enum class GpioMode : uint32_t {
  input     = 0UL,
  output    = 0x00000001UL,
  alternate = 0x00000002UL,
  analog    = 0x00000003UL
};
constexpr uint32_t GPIO_MODE_MASK = 0x00000003UL;

enum class GpioSpeedMode : uint32_t {
  low    = 0UL,
  medium = 0x00000001UL,
  high   = 0x00000003UL
};
constexpr uint32_t GPIO_SPEED_MODE_MASK = 0x00000003UL;

enum class GpioPullMode : uint32_t {
  no   = 0UL,
  up   = 0x00000001UL,
  down = 0x00000002UL,
};
constexpr uint32_t GPIO_PULL_MODE_MASK = 0x00000003UL;

enum class GpioOutputMode : uint32_t {
  pushpull = 0UL,
  opendrain = 0x00000001UL
};

struct GpioPort {
  const GpioType type;
  GPIO_TypeDef* const p_gpio = GD32_PERIPH_INFO.gpio[to_idx(type)].p_gpio;

  IGB_FAST_INLINE void setMode(uint32_t pin_bit, GpioMode mode) {
    IGB_MODIFY_REG(p_gpio->MODER, ((pin_bit * pin_bit) * GPIO_MODE_MASK), ((pin_bit * pin_bit) * static_cast<uint32_t>(mode)));
  }

  IGB_FAST_INLINE void setOutputMode(uint32_t pin_bit, GpioOutputMode mode) {
    IGB_MODIFY_REG(p_gpio->OTYPER, pin_bit, (pin_bit * static_cast<uint32_t>(mode)));
  }

  IGB_FAST_INLINE void setPullMode(uint32_t pin_bit, GpioPullMode mode) {
    IGB_MODIFY_REG(p_gpio->PUPDR, ((pin_bit * pin_bit) * GPIO_PULL_MODE_MASK), ((pin_bit * pin_bit) * static_cast<uint32_t>(mode)));
  }

  IGB_FAST_INLINE void setSpeedMode(uint32_t pin_bit, GpioSpeedMode mode) {
    IGB_MODIFY_REG(p_gpio->OSPEEDR, ((pin_bit * pin_bit) * GPIO_SPEED_MODE_MASK), ((pin_bit * pin_bit) * static_cast<uint32_t>(mode)));
  }

//  IGB_FAST_INLINE void setAlternateFunc(uint32_t pin_bit, GpioAf af) {
//    uint32_t lsb = pin_bit & 0x00FF;
//    uint32_t msb = (pin_bit >> 8) & 0x00FF;
//#if defined(GPIO_AFRL_AFSEL0)
//    if (lsb) {
//      IGB_MODIFY_REG(p_gpio->AFR[0], ((((lsb * lsb) * lsb) * lsb) * GPIO_AFRL_AFSEL0), ((((lsb * lsb) * lsb) * lsb) * as<uint32_t>(af)));
//    }
//    if (msb) {
//      IGB_MODIFY_REG(p_gpio->AFR[1], ((((msb * msb) * msb) * msb) * GPIO_AFRH_AFSEL8), ((((msb * msb) * msb) * msb) * as<uint32_t>(af)));
//    }
//#elif defined(GPIO_AFRL_AFRL0)
//    if (lsb) {
//      IGB_MODIFY_REG(p_gpio->AFR[0], (GPIO_AFRL_AFRL0 << (POSITION_VAL(lsb) * 4U)), (as<uint32_t>(af) << (POSITION_VAL(lsb) * 4U)));
//    }
//    if (msb) {
//      IGB_MODIFY_REG(p_gpio->AFR[1], (GPIO_AFRH_AFRH0 << (POSITION_VAL(msb) * 4U)), (as<uint32_t>(af) << (POSITION_VAL(msb) * 4U)));
//    }
//#else
//  #error Unsupported MCU
//#endif
//  }

  //IGB_FAST_INLINE void lock(uint32_t pin_bit) {
  //  IGB_WRITE_REG(p_gpio->LCKR, GPIO_LCKR_LCKK | pin_bit);
  //  IGB_WRITE_REG(p_gpio->LCKR, pin_bit);
  //  IGB_WRITE_REG(p_gpio->LCKR, GPIO_LCKR_LCKK | pin_bit);
  //  __IO uint32_t temp IGB_UNUSED = IGB_READ_REG(p_gpio->LCKR);
  //}

  IGB_FAST_INLINE void on(uint32_t bits) {
    p_gpio->BSRR = bits;
  }

  IGB_FAST_INLINE void high(uint32_t bits) {
    on(bits);
  }

  IGB_FAST_INLINE void off(uint32_t bits) {
#ifdef GD32_PERIPH_GPIO_REG_BRR_EXISTS
    p_gpio->BRR = bits;
#else
    p_gpio->BSRR = bits << 16U;
#endif
  }

  IGB_FAST_INLINE void low(uint32_t bits) {
    off(bits);
  }

  IGB_FAST_INLINE uint32_t read() {
    return p_gpio->IDR;
  }

  IGB_FAST_INLINE uint32_t readOutput() {
    return p_gpio->ODR;
  }

  IGB_FAST_INLINE void toggle(uint32_t bits) {
    p_gpio->TG = bits;
  }

  IGB_FAST_INLINE void enable() {
    GD32_PERIPH_INFO.gpio[to_idx(type)].bus.enableBusClock();
  }

  IGB_FAST_INLINE void disable() {
    // TODO: clock の無効化
  }

};

struct GpioPin {
  GpioPort port;
  const uint32_t pin_bit = 0;

  IGB_FAST_INLINE void setMode(GpioMode mode) {
    port.setMode(pin_bit, mode);
  }

  IGB_FAST_INLINE void setOutputMode(GpioOutputMode mode) {
    port.setOutputMode(pin_bit, mode);
  }

  IGB_FAST_INLINE void setPullMode(GpioPullMode mode) {
    port.setPullMode(pin_bit, mode);
  }

  IGB_FAST_INLINE void setSpeedMode(GpioSpeedMode mode) {
    port.setSpeedMode(pin_bit, mode);
  }

  //IGB_FAST_INLINE void setAlternateFunc(GpioAf af) {
  //  port.setAlternateFunc(pin_bit, af);
  //}

  //IGB_FAST_INLINE void lock() {
  //  port.lock(pin_bit);
  //}

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

  IGB_FAST_INLINE void toggle() {
    port.toggle(pin_bit);
  }

  IGB_FAST_INLINE void enable() {
    port.enable();
  }

  IGB_FAST_INLINE void disable() {
    // TODO: clock の無効化
  }

  // TODO:
  //IGB_FAST_INLINE void enableExti(ExtiTrigType trig_type, ExtiMode mode, uint8_t priority) {
  //  SysCfg::enableBusClock();

  //  switch (pin_bit) {
  //    case (1UL << 0):
  //      SysCfg::exti0GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line0, trig_type, mode, priority);
  //      break;
  //    case (1UL << 1):
  //      SysCfg::exti1GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line1, trig_type, mode, priority);
  //      break;
  //    case (1UL << 2):
  //      SysCfg::exti2GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line2, trig_type, mode, priority);
  //      break;
  //    case (1UL << 3):
  //      SysCfg::exti3GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line3, trig_type, mode, priority);
  //      break;
  //    case (1UL << 4):
  //      SysCfg::exti4GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line4, trig_type, mode, priority);
  //      break;
  //    case (1UL << 5):
  //      SysCfg::exti5GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line5, trig_type, mode, priority);
  //      break;
  //    case (1UL << 6):
  //      SysCfg::exti6GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line6, trig_type, mode, priority);
  //      break;
  //    case (1UL << 7):
  //      SysCfg::exti7GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line7, trig_type, mode, priority);
  //      break;
  //    case (1UL << 8):
  //      SysCfg::exti8GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line8, trig_type, mode, priority);
  //      break;
  //    case (1UL << 9):
  //      SysCfg::exti9GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line9, trig_type, mode, priority);
  //      break;
  //    case (1UL << 10):
  //      SysCfg::exti10GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line10, trig_type, mode, priority);
  //      break;
  //    case (1UL << 11):
  //      SysCfg::exti11GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line11, trig_type, mode, priority);
  //      break;
  //    case (1UL << 12):
  //      SysCfg::exti12GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line12, trig_type, mode, priority);
  //      break;
  //    case (1UL << 13):
  //      SysCfg::exti13GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line13, trig_type, mode, priority);
  //      break;
  //    case (1UL << 14):
  //      SysCfg::exti14GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line14, trig_type, mode, priority);
  //      break;
  //    case (1UL << 15):
  //      SysCfg::exti15GpioPort(port.type);
  //      ExtiCtrl::enableLine(ExtiLine::line15, trig_type, mode, priority);
  //      break;
  //    default:
  //      break;
  //  }
  //}

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

  static IGB_FAST_INLINE GpioPin newPin(const GpioPinType pin_type) {
    GpioType gpio_type = extract_gpio_type(pin_type);
    uint8_t pin_idx = extract_pin_idx(pin_type);
    return GpioPin {
      .port = { gpio_type },
      .pin_bit = 1UL << pin_idx
    };
  }
};

} // namespace gd32
} // namespace igb

#endif /* IGB_GD32_PERIPH_GPIO_H */
