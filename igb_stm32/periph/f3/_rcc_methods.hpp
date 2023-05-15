  static IGB_FAST_INLINE RccPllClockSrc getPllSrc() {
    return static_cast<RccPllClockSrc>(RCC->CFGR & RCC_CFGR_PLLSRC);
  }

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

    IGB_MODIFY_REG(RCC->CFGR3, i2c_src, clock_src);
  }
#endif

#if defined(RCC_CFGR3_USART1SW) && STM32_PERIPH_USART1_EXISTS
  static IGB_FAST_INLINE void setUsartClockSrc(UsartType type, UsartClockSrc src) {
    uint32_t usart_src = 0;
    uint32_t clock_src = 0;
    switch(type) {
      case UsartType::usart1:
        usart_src = RCC_CFGR3_USART1SW;
        switch(src) {
          case UsartClockSrc::pclk:
            clock_src = RCC_CFGR3_USART1SW_PCLK;
            break;
          case UsartClockSrc::system:
            clock_src = RCC_CFGR3_USART1SW_SYSCLK;
            break;
          case UsartClockSrc::lse:
            clock_src = RCC_CFGR3_USART1SW_LSE;
            break;
          case UsartClockSrc::hsi:
            clock_src = RCC_CFGR3_USART1SW_HSI;
            break;
        }
        break;
#if defined(RCC_CFGR3_USART2SW) && STM32_PERIPH_USART2_EXISTS
      case UsartType::usart2:
        usart_src = RCC_CFGR3_USART2SW;
        switch(src) {
          case UsartClockSrc::pclk:
            clock_src = RCC_CFGR3_USART2SW_PCLK;
            break;
          case UsartClockSrc::system:
            clock_src = RCC_CFGR3_USART2SW_SYSCLK;
            break;
          case UsartClockSrc::lse:
            clock_src = RCC_CFGR3_USART2SW_LSE;
            break;
          case UsartClockSrc::hsi:
            clock_src = RCC_CFGR3_USART2SW_HSI;
            break;
        }
        break;
#endif
#if defined(RCC_CFGR3_USART3SW) && STM32_PERIPH_USART3_EXISTS
      case UsartType::usart3:
        usart_src = RCC_CFGR3_USART3SW;
        switch(src) {
          case UsartClockSrc::pclk:
            clock_src = RCC_CFGR3_USART3SW_PCLK;
            break;
          case UsartClockSrc::system:
            clock_src = RCC_CFGR3_USART3SW_SYSCLK;
            break;
          case UsartClockSrc::lse:
            clock_src = RCC_CFGR3_USART3SW_LSE;
            break;
          case UsartClockSrc::hsi:
            clock_src = RCC_CFGR3_USART3SW_HSI;
            break;
        }
        break;
#endif
      default:
        break;
    }
    if (usart_src) {
      IGB_MODIFY_REG(RCC->CFGR3, usart_src, clock_src);
    }
  }
#endif

#if defined(RCC_CFGR_ADCPRE)
  static IGB_FAST_INLINE void setAdcClockSrc(AdcType type, AdcClockDiv div = AdcClockDiv::div2) {
    IGB_MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, static_cast<uint32_t>(div));
  }
#elif defined(RCC_CFGR2_ADC1PRES)
  static IGB_FAST_INLINE void setAdcClockSrc(AdcType type, AdcClockSrc src = AdcClockSrc::ahb, AdcClockDiv div = AdcClockDiv::div1) {
    if (src == AdcClockSrc::ahb) {
      IGB_MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADC1PRES, RCC_CFGR2_ADC1PRES_NO);
    } else if (src == AdcClockSrc::pll) {
      IGB_MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADC1PRES, static_cast<uint32_t>(div));
    }
  }
#elif defined(RCC_CFGR2_ADCPRE12) || defined(RCC_CFGR2_ADCPRE34)
  static IGB_FAST_INLINE void setAdcClockSrc(AdcType type, AdcClockSrc src = AdcClockSrc::ahb, AdcClockDiv div = AdcClockDiv::div1) {
#if defined(RCC_CFGR2_ADCPRE34)
    if (type == AdcType::adc1 || type == AdcType::adc2) {
      if (src == AdcClockSrc::ahb) {
        IGB_MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12, 0);
      } else {
        IGB_MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12, static_cast<uint32_t>(div));
      }
    } else {
      if (src == AdcClockSrc::ahb) {
        IGB_MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE34, 0);
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
        IGB_MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE34, _div);
      }
    }
    //IGB_MODIFY_REG(RCC->CFGR2, (ADCxSource >> 16U), (ADCxSource & 0x0000FFFFU));
#else
    if (src == AdcClockSrc::ahb) {
      IGB_MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12, 0);
    } else {
      IGB_MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12, static_cast<uint32_t>(div));
    }
#endif /* RCC_CFGR2_ADCPRE34 */
  }
#endif /* RCC_CFGR_ADCPRE */

#if defined(RCC_PLLSRC_PREDIV1_SUPPORT)
  static IGB_FAST_INLINE void configPllSystemClockDomain(RccPllClockSrc clock_src, RccPllMul mul, RccPllDiv div) {
    IGB_MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL, static_cast<uint32_t>(clock_src) | static_cast<uint32_t>(mul));
    IGB_MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, static_cast<uint32_t>(div));
  }
#else
  static IGB_FAST_INLINE void configPllSystemClockDomain(RccPllClockSrc clock_src, RccPllMul mul, RccPllDiv div) {
    IGB_MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL, (static_cast<uint32_t>(clock_src) & RCC_CFGR_PLLSRC) | static_cast<uint32_t>(mul));
    if (clock_src == RccPllClockSrc::internal) {
      IGB_MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, 0);
    } else {
      IGB_MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, (static_cast<uint32_t>(div) & RCC_CFGR2_PREDIV));
    }
  }
#endif // RCC_PLLSRC_PREDIV1_SUPPORT
