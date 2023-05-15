  static IGB_FAST_INLINE void setI2cClockSrc(I2cType type, I2cClockSrc src) {
    uint32_t clock_src = 0;
    switch(type) {
      case I2cType::i2c1:
      case I2cType::i2c2:
      case I2cType::i2c3:
        switch(src) {
          case I2cClockSrc::internal:
            clock_src = (0b10 << RCC_D2CCIP2R_I2C123SEL_Pos);
            break;
          case I2cClockSrc::system:
            clock_src = (0b00 << RCC_D2CCIP2R_I2C123SEL_Pos);
            break;
          case I2cClockSrc::pll3:
            clock_src = (0b01 << RCC_D2CCIP2R_I2C123SEL_Pos);
            break;
          case I2cClockSrc::csi:
            clock_src = (0b11 << RCC_D2CCIP2R_I2C123SEL_Pos);
            break;
          default:
            break;
        }
        IGB_MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_I2C123SEL_Msk, clock_src);
        break;
      case I2cType::i2c4:
        switch(src) {
          case I2cClockSrc::internal:
            clock_src = (0b10 << RCC_D3CCIPR_I2C4SEL_Pos);
            break;
          case I2cClockSrc::system:
            clock_src = (0b00 << RCC_D3CCIPR_I2C4SEL_Pos);
            break;
          case I2cClockSrc::pll3:
            clock_src = (0b01 << RCC_D3CCIPR_I2C4SEL_Pos);
            break;
          case I2cClockSrc::csi:
            clock_src = (0b11 << RCC_D3CCIPR_I2C4SEL_Pos);
            break;
          default:
            break;
        }
        IGB_MODIFY_REG(RCC->D3CCIPR, RCC_D3CCIPR_I2C4SEL_Msk, clock_src);
        break;
      default:
        break;
    }
  }

  static IGB_FAST_INLINE void setUsartClockSrc(UsartType type, UsartClockSrc src) {
    uint32_t clock_src = 0;
    switch(type) {
      case UsartType::usart1:
      case UsartType::usart6:
        clock_src = (as<uint32_t>(src) << RCC_D2CCIP2R_USART16SEL_Pos);
        IGB_MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_USART16SEL_Msk, clock_src);
        break;
      default:
        clock_src = (as<uint32_t>(src) << RCC_D2CCIP2R_USART28SEL_Pos);
        IGB_MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_USART28SEL_Msk, clock_src);
        break;
    }
  }

  static IGB_FAST_INLINE void setAdcClockSrc(AdcType type, AdcClockSrc src) {
    uint32_t clock_src = (as<uint32_t>(src) << RCC_D3CCIPR_ADCSEL_Pos);
    IGB_MODIFY_REG(RCC->D3CCIPR, RCC_D3CCIPR_ADCSEL_Msk, clock_src);
  }
