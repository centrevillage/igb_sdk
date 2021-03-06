#ifndef IGB_STM32_PERIPH_RCC_H
#define IGB_STM32_PERIPH_RCC_H

#include <igb_stm32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace stm32 {

enum class RccClockSrc {
  internal = RCC_CFGR_SW_HSI,
  external = RCC_CFGR_SW_HSE,
#if defined(RCC_CFGR_SW_CSI)
  csi = RCC_CFGR_SW_CSI,
#endif
#if defined(RCC_CFGR_SW_PLL)
  pll = RCC_CFGR_SW_PLL,
#elif defined(RCC_CFGR_SW_PLL1)
  pll = RCC_CFGR_SW_PLL1,
#endif
#if defined(RCC_HSI48_SUPPORT)
  hsi48 = RCC_CFGR_SW_HSI48
#endif
};

enum class RccClockSrcStatus {
  internal = RCC_CFGR_SWS_HSI,
  external = RCC_CFGR_SWS_HSE,
#if defined(RCC_CFGR_SW_CSI)
  csi = RCC_CFGR_SW_CSI,
#endif
#if defined(RCC_CFGR_SWS_PLL)
  pll = RCC_CFGR_SWS_PLL,
#elif defined(RCC_CFGR_SW_PLL1)
  pll = RCC_CFGR_SW_PLL1,
#endif
#if defined(RCC_HSI48_SUPPORT)
  hsi48 = RCC_CFGR_SWS_HSI48
#endif
};

#if defined(RCC_CFGR_HPRE)
enum class RccClockPrescalerAHB {
  div1 = RCC_CFGR_HPRE_DIV1,
  div2 = RCC_CFGR_HPRE_DIV2,
  div4 = RCC_CFGR_HPRE_DIV4,
  div8 = RCC_CFGR_HPRE_DIV8,
  div16 = RCC_CFGR_HPRE_DIV16,
  div64 = RCC_CFGR_HPRE_DIV64,
  div128 = RCC_CFGR_HPRE_DIV128,
  div256 = RCC_CFGR_HPRE_DIV256,
  div512 = RCC_CFGR_HPRE_DIV512,
};
#endif

#if defined(RCC_CFGR_PPRE)
enum class RccClockPrescalerAPB1 {
  div1 = RCC_CFGR_PPRE_DIV1,
  div2 = RCC_CFGR_PPRE_DIV2,
  div4 = RCC_CFGR_PPRE_DIV4,
  div8 = RCC_CFGR_PPRE_DIV8,
  div16 = RCC_CFGR_PPRE_DIV16
};
#elif defined(RCC_CFGR_PPRE1)
enum class RccClockPrescalerAPB1 {
  div1 = RCC_CFGR_PPRE1_DIV1,
  div2 = RCC_CFGR_PPRE1_DIV2,
  div4 = RCC_CFGR_PPRE1_DIV4,
  div8 = RCC_CFGR_PPRE1_DIV8,
  div16 = RCC_CFGR_PPRE1_DIV16
};
#endif

#if defined(RCC_CFGR_PPRE2)
enum class RccClockPrescalerAPB2 {
  div1 = RCC_CFGR_PPRE2_DIV1,
  div2 = RCC_CFGR_PPRE2_DIV2,
  div4 = RCC_CFGR_PPRE2_DIV4,
  div8 = RCC_CFGR_PPRE2_DIV8,
  div16 = RCC_CFGR_PPRE2_DIV16
};
#endif

#if defined(STM32F0) || defined(STM32F3)
#if defined(RCC_PLLSRC_PREDIV1_SUPPORT)
enum class RccPllClockSrc : uint32_t {
#if defined(RCC_CFGR_PLLSRC_HSI_PREDIV)
  internal = RCC_CFGR_PLLSRC_HSI_PREDIV,
#endif
#if defined(RCC_CFGR_PLLSRC_HSE_PREDIV)
  external = RCC_CFGR_PLLSRC_HSE_PREDIV,
#endif
#if defined(RCC_CFGR_PLLSRC_HSI48_PREDIV)
  hsi48 = RCC_CFGR_PLLSRC_HSI48_PREDIV
#endif
};
#else
enum class RccPllClockSrc : uint32_t {
  internal = 0,
#if defined(RCC_CFGR_PLLSRC_HSE_PREDIV)
  external = RCC_CFGR_PLLSRC_HSE_PREDIV 
#endif
};
#endif

enum class RccPllMul : uint32_t {
  mul2 = RCC_CFGR_PLLMUL2,
  mul3 = RCC_CFGR_PLLMUL3,
  mul4 = RCC_CFGR_PLLMUL4,
  mul5 = RCC_CFGR_PLLMUL5,
  mul6 = RCC_CFGR_PLLMUL6,
  mul7 = RCC_CFGR_PLLMUL7,
  mul8 = RCC_CFGR_PLLMUL8,
  mul9 = RCC_CFGR_PLLMUL9,
  mul10 = RCC_CFGR_PLLMUL10,
  mul11 = RCC_CFGR_PLLMUL11,
  mul12 = RCC_CFGR_PLLMUL12,
  mul13 = RCC_CFGR_PLLMUL13,
  mul14 = RCC_CFGR_PLLMUL14,
  mul15 = RCC_CFGR_PLLMUL15,
  mul16 = RCC_CFGR_PLLMUL16,
};

#if defined(RCC_PLLSRC_PREDIV1_SUPPORT)
enum class RccPllDiv {
#if defined(RCC_CFGR_PLLSRC_HSI_PREDIV)
  internal = RCC_CFGR_PLLSRC_HSI_PREDIV,
#endif
  div1 = RCC_CFGR2_PREDIV_DIV1,
  div2 = RCC_CFGR2_PREDIV_DIV2,
  div3 = RCC_CFGR2_PREDIV_DIV3,
  div4 = RCC_CFGR2_PREDIV_DIV4,
  div5 = RCC_CFGR2_PREDIV_DIV5,
  div6 = RCC_CFGR2_PREDIV_DIV6,
  div7 = RCC_CFGR2_PREDIV_DIV7,
  div8 = RCC_CFGR2_PREDIV_DIV8,
  div9 = RCC_CFGR2_PREDIV_DIV9,
  div10 = RCC_CFGR2_PREDIV_DIV10,
  div11 = RCC_CFGR2_PREDIV_DIV11,
  div12 = RCC_CFGR2_PREDIV_DIV12,
  div13 = RCC_CFGR2_PREDIV_DIV13,
  div14 = RCC_CFGR2_PREDIV_DIV14,
  div15 = RCC_CFGR2_PREDIV_DIV15,
  div16 = RCC_CFGR2_PREDIV_DIV16,
};
#else
enum class RccPllDiv : uint32_t {
  div1 = 0,
  div2,
  div3,
  div4,
  div5,
  div6,
  div7,
  div8,
  div9,
  div10,
  div11,
  div12,
  div13,
  div14,
  div15,
  div16,
};
#endif // RCC_PLLSRC_PREDIV1_SUPPORT
#endif // STM32_SERIES

#if defined(RCC_CFGR3_I2C1SW)
enum class I2cClockSrc : uint32_t {
  internal = 0,
  system,
};
#endif

#if defined(STM32F3)
#if defined(RCC_CFGR_ADCPRE)
enum class AdcClockDiv : uint32_t {
  div2 = RCC_CFGR_ADCPRE_DIV2,
  div4 = RCC_CFGR_ADCPRE_DIV4,
  div6 = RCC_CFGR_ADCPRE_DIV6,
  div8 = RCC_CFGR_ADCPRE_DIV8,
};
#elif defined(RCC_CFGR2_ADC1PRES)
enum class AdcClockSrc : uint8_t {
  ahb = 0,
  pll,
};
enum class AdcClockDiv : uint32_t {
  div1 = RCC_CFGR2_ADC1PRES_DIV1,
  div2 = RCC_CFGR2_ADC1PRES_DIV2,
  div4 = RCC_CFGR2_ADC1PRES_DIV4,
  div6 = RCC_CFGR2_ADC1PRES_DIV6,
  div8 = RCC_CFGR2_ADC1PRES_DIV8,
  div10 = RCC_CFGR2_ADC1PRES_DIV10,
  div12 = RCC_CFGR2_ADC1PRES_DIV12,
  div16 = RCC_CFGR2_ADC1PRES_DIV16,
  div32 = RCC_CFGR2_ADC1PRES_DIV32,
  div64 = RCC_CFGR2_ADC1PRES_DIV64,
  div128 = RCC_CFGR2_ADC1PRES_DIV128,
  div256 = RCC_CFGR2_ADC1PRES_DIV256,
};
#elif defined(RCC_CFGR2_ADCPRE12) || defined(RCC_CFGR2_ADCPRE34)
enum class AdcClockSrc : uint8_t {
  ahb = 0,
  pll,
};
enum class AdcClockDiv : uint32_t {
  div1 = RCC_CFGR2_ADCPRE12_DIV1,
  div2 = RCC_CFGR2_ADCPRE12_DIV2,
  div4 = RCC_CFGR2_ADCPRE12_DIV4,
  div6 = RCC_CFGR2_ADCPRE12_DIV6,
  div8 = RCC_CFGR2_ADCPRE12_DIV8,
  div10 = RCC_CFGR2_ADCPRE12_DIV10,
  div12 = RCC_CFGR2_ADCPRE12_DIV12,
  div16 = RCC_CFGR2_ADCPRE12_DIV16,
  div32 = RCC_CFGR2_ADCPRE12_DIV32,
  div64 = RCC_CFGR2_ADCPRE12_DIV64,
  div128 = RCC_CFGR2_ADCPRE12_DIV128,
  div256 = RCC_CFGR2_ADCPRE12_DIV256,
};
#endif /* RCC_CFGR_ADCPRE */
#endif /* STM32F3 */

struct RccCtrl {
  static IGB_FAST_INLINE void enableBusClock(const auto& periph_bus_info) {
    periph_bus_info.enableBusClock();
  }

  static IGB_FAST_INLINE void enableHSE() {
    SET_BIT(RCC->CR, RCC_CR_HSEON);
  }

  static IGB_FAST_INLINE void disableHSE() {
    CLEAR_BIT(RCC->CR, RCC_CR_HSEON);
  }
  
  static IGB_FAST_INLINE bool isReadyHSE() {
    return (READ_BIT(RCC->CR, RCC_CR_HSERDY) == (RCC_CR_HSERDY));
  }

  static IGB_FAST_INLINE void enableHSI() {
    SET_BIT(RCC->CR, RCC_CR_HSION);
  }

  static IGB_FAST_INLINE void disableHSI() {
    CLEAR_BIT(RCC->CR, RCC_CR_HSION);
  }

  static IGB_FAST_INLINE bool isReadyHSI() {
    return (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == (RCC_CR_HSIRDY));
  }

#if defined(RCC_HSI48_SUPPORT)
  static IGB_FAST_INLINE void enableHSI48() {
    SET_BIT(RCC->CR2, RCC_CR2_HSI48ON);
  }

  static IGB_FAST_INLINE void disableHSI48() {
    CLEAR_BIT(RCC->CR2, RCC_CR2_HSI48ON);
  }
#endif

#if defined(RCC_CR2_HSI14ON)
  static IGB_FAST_INLINE void enableHSI14() {
    SET_BIT(RCC->CR2, RCC_CR2_HSI14ON);
  }

  static IGB_FAST_INLINE void disableHSI14() {
    CLEAR_BIT(RCC->CR2, RCC_CR2_HSI14ON);
  }
#endif

#if defined(RCC_BDCR_LSEON)
  static IGB_FAST_INLINE void enableLSE() {
    SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
  }

  static IGB_FAST_INLINE void disableLSE() {
    CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);
  }
#endif

#if defined(RCC_BDCR_LSEBYP)
  static IGB_FAST_INLINE void enableLseBypass() {
    SET_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
  }

  static IGB_FAST_INLINE void disableLseBypass() {
    CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
  }
#endif

#if defined(RCC_CSR_LSION)
  static IGB_FAST_INLINE void enableLSI() {
    SET_BIT(RCC->CSR, RCC_CSR_LSION);
  }

  static IGB_FAST_INLINE void disableLSI() {
    CLEAR_BIT(RCC->CSR, RCC_CSR_LSION);
  }
#endif

  static IGB_FAST_INLINE void setSystemClockSrc(RccClockSrc clock_src) {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, static_cast<uint32_t>(clock_src));
  }

  static IGB_FAST_INLINE RccClockSrcStatus getSystemClockSrcStatus() {
    return static_cast<RccClockSrcStatus>(READ_BIT(RCC->CFGR, RCC_CFGR_SWS));
  }

#if defined(RCC_CFGR_HPRE)
  static IGB_FAST_INLINE void setPrescalerAHB(RccClockPrescalerAHB prescaler)  {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, static_cast<uint32_t>(prescaler));
  }
#endif

#if defined(RCC_CFGR_PPRE)
  static IGB_FAST_INLINE void setPrescalerAPB1(RccClockPrescalerAPB1 prescaler) {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, static_cast<uint32_t>(prescaler));
  }
#elif defined(RCC_CFGR_PPRE1)
  static IGB_FAST_INLINE void setPrescalerAPB1(RccClockPrescalerAPB1 prescaler) {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, static_cast<uint32_t>(prescaler));
  }
#endif
#if defined(RCC_CFGR_PPRE2)
  static IGB_FAST_INLINE void setPrescalerAPB2(RccClockPrescalerAPB2 prescaler) {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, static_cast<uint32_t>(prescaler));
  }
#endif

#if defined(RCC_CFGR3_I2C1SW) && STM32_PERIPH_I2C1_EXISTS
  static IGB_FAST_INLINE void setI2cClockSrc(I2cType type, I2cClockSrc src) {
    uint32_t i2c_src = 0;
    uint32_t clock_src = 0;
    switch(type) {
      case I2cType::i2c1:
        i2c_src = RCC_CFGR3_I2C1SW;
        switch(src) {
          case I2cClockSrc::internal:
            clock_src = RCC_CFGR3_I2C1SW_HSI;
            break;
          case I2cClockSrc::system:
            clock_src = RCC_CFGR3_I2C1SW_SYSCLK;
            break;
          default:
            break;
        }
        break;
#if defined(RCC_CFGR3_I2C2SW) && STM32_PERIPH_I2C2_EXISTS
      case I2cType::i2c2:
        i2c_src = RCC_CFGR3_I2C2SW;
        switch(src) {
          case I2cClockSrc::internal:
            clock_src = RCC_CFGR3_I2C2SW_HSI;
            break;
          case I2cClockSrc::system:
            clock_src = RCC_CFGR3_I2C2SW_SYSCLK;
            break;
          default:
            break;
        }
        break;
#endif
#if defined(RCC_CFGR3_I2C3SW) && STM32_PERIPH_I2C3_EXISTS
      case I2cType::i2c3:
        i2c_src = RCC_CFGR3_I2C3SW;
        switch(src) {
          case I2cClockSrc::internal:
            clock_src = RCC_CFGR3_I2C3SW_HSI;
            break;
          case I2cClockSrc::system:
            clock_src = RCC_CFGR3_I2C3SW_SYSCLK;
            break;
          default:
            break;
        }
        break;
#endif
      default:
        break;
    }

    MODIFY_REG(RCC->CFGR3, i2c_src, clock_src);
  }
#endif

#if defined(STM32F3)
#if defined(RCC_CFGR_ADCPRE)
  static IGB_FAST_INLINE void setAdcClockSrc(AdcType type, AdcClockDiv div = AdcClockDiv::div2) {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, static_cast<uint32_t>(div));
  }
#elif defined(RCC_CFGR2_ADC1PRES)
  static IGB_FAST_INLINE void setAdcClockSrc(AdcType type, AdcClockSrc src = AdcClockSrc::ahb, AdcClockDiv div = AdcClockDiv::div1) {
    if (src == AdcClockSrc::ahb) {
      MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADC1PRES, RCC_CFGR2_ADC1PRES_NO);
    } else if (src == AdcClockSrc::pll) {
      MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADC1PRES, static_cast<uint32_t>(div));
    }
  }
#elif defined(RCC_CFGR2_ADCPRE12) || defined(RCC_CFGR2_ADCPRE34)
  static IGB_FAST_INLINE void setAdcClockSrc(AdcType type, AdcClockSrc src = AdcClockSrc::ahb, AdcClockDiv div = AdcClockDiv::div1) {
#if defined(RCC_CFGR2_ADCPRE34)
    if (type == AdcType::adc1 || type == AdcType::adc2) {
      if (src == AdcClockSrc::ahb) {
        MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12, 0);
      } else {
        MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12, static_cast<uint32_t>(div));
      }
    } else {
      if (src == AdcClockSrc::ahb) {
        MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE34, 0);
      } else {
        uint32_t _div = 0;
        switch (div) {
          case AdcClockDiv::div1:
            _div = RCC_CFGR2_ADCPRE34_DIV1;
            break;
          case AdcClockDiv::div2:
            _div = RCC_CFGR2_ADCPRE34_DIV2;
            break;
          case AdcClockDiv::div4:
            _div = RCC_CFGR2_ADCPRE34_DIV4;
            break;
          case AdcClockDiv::div6:
            _div = RCC_CFGR2_ADCPRE34_DIV6;
            break;
          case AdcClockDiv::div8:
            _div = RCC_CFGR2_ADCPRE34_DIV8;
            break;
          case AdcClockDiv::div10:
            _div = RCC_CFGR2_ADCPRE34_DIV10;
            break;
          case AdcClockDiv::div12:
            _div = RCC_CFGR2_ADCPRE34_DIV12;
            break;
          case AdcClockDiv::div16:
            _div = RCC_CFGR2_ADCPRE34_DIV16;
            break;
          case AdcClockDiv::div32:
            _div = RCC_CFGR2_ADCPRE34_DIV32;
            break;
          case AdcClockDiv::div64:
            _div = RCC_CFGR2_ADCPRE34_DIV64;
            break;
          case AdcClockDiv::div128:
            _div = RCC_CFGR2_ADCPRE34_DIV128;
            break;
          case AdcClockDiv::div256:
            _div = RCC_CFGR2_ADCPRE34_DIV256;
            break;
          default:
            break;
        }
        MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE34, _div);
      }
    }
    //MODIFY_REG(RCC->CFGR2, (ADCxSource >> 16U), (ADCxSource & 0x0000FFFFU));
#else
    if (src == AdcClockSrc::ahb) {
      MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12, 0);
    } else {
      MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12, static_cast<uint32_t>(div));
    }
#endif /* RCC_CFGR2_ADCPRE34 */
  }
#endif /* RCC_CFGR_ADCPRE */
#endif /* STM32F3 */

#if defined(RCC_CR_PLLON)
  static IGB_FAST_INLINE void enablePLL() {
    SET_BIT(RCC->CR, RCC_CR_PLLON);
  }

  static IGB_FAST_INLINE void disablePLL() {
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
  }

  static IGB_FAST_INLINE bool isReadyPLL() {
    return (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == (RCC_CR_PLLRDY));
  }
#endif

#if defined(STM32F0) || defined(STM32F3)
#if defined(RCC_PLLSRC_PREDIV1_SUPPORT)
  static IGB_FAST_INLINE void configPllSystemClockDomain(RccPllClockSrc clock_src, RccPllMul mul, RccPllDiv div) {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL, static_cast<uint32_t>(clock_src) | static_cast<uint32_t>(mul));
    MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, static_cast<uint32_t>(div));
  }
#else
  static IGB_FAST_INLINE void configPllSystemClockDomain(RccPllClockSrc clock_src, RccPllMul mul, RccPllDiv div) {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL, (static_cast<uint32_t>(clock_src) & RCC_CFGR_PLLSRC) | static_cast<uint32_t>(mul));
    if (clock_src == RccPllClockSrc::internal) {
      MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, 0);
    } else {
      MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, (static_cast<uint32_t>(div) & RCC_CFGR2_PREDIV));
    }
  }
#endif // RCC_PLLSRC_PREDIV1_SUPPORT
#endif // STM32_SERIES

  //static IGB_FAST_INLINE void setPllMainSrc(RccPllSrc src) {
  //}
};

}
}

#endif /* IGB_STM32_PERIPH_RCC_H */
