  // --- I2C clock source ---

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

  // --- USART clock source ---

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

  // --- ADC clock source ---

  static IGB_FAST_INLINE void setAdcClockSrc(AdcType type, AdcClockSrc src) {
    uint32_t clock_src = (as<uint32_t>(src) << RCC_D3CCIPR_ADCSEL_Pos);
    IGB_MODIFY_REG(RCC->D3CCIPR, RCC_D3CCIPR_ADCSEL_Msk, clock_src);
  }

  // --- PLL source and M dividers (PLLCKSELR) ---

  // Select common input clock source for PLL1/2/3
  static IGB_FAST_INLINE void setPllSrc(PllSrc src) {
    IGB_MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_PLLSRC_Msk,
                   as<uint32_t>(src) << RCC_PLLCKSELR_PLLSRC_Pos);
  }

  // Set PLL1 input pre-divider M (1-63; written directly to register)
  static IGB_FAST_INLINE void setPll1DivM(uint32_t m) {
    IGB_MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM1_Msk,
                   m << RCC_PLLCKSELR_DIVM1_Pos);
  }

  // Set PLL2 input pre-divider M (1-63)
  static IGB_FAST_INLINE void setPll2DivM(uint32_t m) {
    IGB_MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM2_Msk,
                   m << RCC_PLLCKSELR_DIVM2_Pos);
  }

  // Set PLL3 input pre-divider M (1-63)
  static IGB_FAST_INLINE void setPll3DivM(uint32_t m) {
    IGB_MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM3_Msk,
                   m << RCC_PLLCKSELR_DIVM3_Pos);
  }

  // --- PLL1 config (PLLCFGR + PLL1DIVR + PLL1FRACR) ---

  static IGB_FAST_INLINE void setPll1VciRange(PllVciRange range) {
    IGB_MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL1RGE_Msk,
                   as<uint32_t>(range) << RCC_PLLCFGR_PLL1RGE_Pos);
  }

  static IGB_FAST_INLINE void setPll1VcoSel(PllVcoSel sel) {
    IGB_MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL1VCOSEL_Msk,
                   as<uint32_t>(sel) << RCC_PLLCFGR_PLL1VCOSEL_Pos);
  }

  static IGB_FAST_INLINE void enablePll1FracN()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1FRACEN); }
  static IGB_FAST_INLINE void disablePll1FracN() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1FRACEN); }

  static IGB_FAST_INLINE void enablePll1DivP()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVP1EN); }
  static IGB_FAST_INLINE void enablePll1DivQ()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ1EN); }
  static IGB_FAST_INLINE void enablePll1DivR()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVR1EN); }
  static IGB_FAST_INLINE void disablePll1DivP() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVP1EN); }
  static IGB_FAST_INLINE void disablePll1DivQ() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ1EN); }
  static IGB_FAST_INLINE void disablePll1DivR() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVR1EN); }

  // N/P/Q/R: pass the effective divider value; register stores (value - 1)
  static IGB_FAST_INLINE void setPll1DivN(uint32_t n) {
    IGB_MODIFY_REG(RCC->PLL1DIVR, RCC_PLL1DIVR_N1_Msk, (n - 1U) << RCC_PLL1DIVR_N1_Pos);
  }
  static IGB_FAST_INLINE void setPll1DivP(uint32_t p) {
    IGB_MODIFY_REG(RCC->PLL1DIVR, RCC_PLL1DIVR_P1_Msk, (p - 1U) << RCC_PLL1DIVR_P1_Pos);
  }
  static IGB_FAST_INLINE void setPll1DivQ(uint32_t q) {
    IGB_MODIFY_REG(RCC->PLL1DIVR, RCC_PLL1DIVR_Q1_Msk, (q - 1U) << RCC_PLL1DIVR_Q1_Pos);
  }
  static IGB_FAST_INLINE void setPll1DivR(uint32_t r) {
    IGB_MODIFY_REG(RCC->PLL1DIVR, RCC_PLL1DIVR_R1_Msk, (r - 1U) << RCC_PLL1DIVR_R1_Pos);
  }

  // FRACN: written directly (no offset)
  static IGB_FAST_INLINE void setPll1FracN(uint32_t fracn) {
    IGB_MODIFY_REG(RCC->PLL1FRACR, RCC_PLL1FRACR_FRACN1_Msk,
                   fracn << RCC_PLL1FRACR_FRACN1_Pos);
  }

  // --- PLL2 config (PLLCFGR + PLL2DIVR + PLL2FRACR) ---

  static IGB_FAST_INLINE void setPll2VciRange(PllVciRange range) {
    IGB_MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL2RGE_Msk,
                   as<uint32_t>(range) << RCC_PLLCFGR_PLL2RGE_Pos);
  }

  static IGB_FAST_INLINE void setPll2VcoSel(PllVcoSel sel) {
    IGB_MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL2VCOSEL_Msk,
                   as<uint32_t>(sel) << RCC_PLLCFGR_PLL2VCOSEL_Pos);
  }

  static IGB_FAST_INLINE void enablePll2FracN()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2FRACEN); }
  static IGB_FAST_INLINE void disablePll2FracN() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2FRACEN); }

  static IGB_FAST_INLINE void enablePll2DivP()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVP2EN); }
  static IGB_FAST_INLINE void enablePll2DivQ()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ2EN); }
  static IGB_FAST_INLINE void enablePll2DivR()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVR2EN); }
  static IGB_FAST_INLINE void disablePll2DivP() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVP2EN); }
  static IGB_FAST_INLINE void disablePll2DivQ() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ2EN); }
  static IGB_FAST_INLINE void disablePll2DivR() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVR2EN); }

  static IGB_FAST_INLINE void setPll2DivN(uint32_t n) {
    IGB_MODIFY_REG(RCC->PLL2DIVR, RCC_PLL2DIVR_N2_Msk, (n - 1U) << RCC_PLL2DIVR_N2_Pos);
  }
  static IGB_FAST_INLINE void setPll2DivP(uint32_t p) {
    IGB_MODIFY_REG(RCC->PLL2DIVR, RCC_PLL2DIVR_P2_Msk, (p - 1U) << RCC_PLL2DIVR_P2_Pos);
  }
  static IGB_FAST_INLINE void setPll2DivQ(uint32_t q) {
    IGB_MODIFY_REG(RCC->PLL2DIVR, RCC_PLL2DIVR_Q2_Msk, (q - 1U) << RCC_PLL2DIVR_Q2_Pos);
  }
  static IGB_FAST_INLINE void setPll2DivR(uint32_t r) {
    IGB_MODIFY_REG(RCC->PLL2DIVR, RCC_PLL2DIVR_R2_Msk, (r - 1U) << RCC_PLL2DIVR_R2_Pos);
  }

  static IGB_FAST_INLINE void setPll2FracN(uint32_t fracn) {
    IGB_MODIFY_REG(RCC->PLL2FRACR, RCC_PLL2FRACR_FRACN2_Msk,
                   fracn << RCC_PLL2FRACR_FRACN2_Pos);
  }

  // --- PLL3 config (PLLCFGR + PLL3DIVR + PLL3FRACR) ---

  static IGB_FAST_INLINE void setPll3VciRange(PllVciRange range) {
    IGB_MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL3RGE_Msk,
                   as<uint32_t>(range) << RCC_PLLCFGR_PLL3RGE_Pos);
  }

  static IGB_FAST_INLINE void setPll3VcoSel(PllVcoSel sel) {
    IGB_MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL3VCOSEL_Msk,
                   as<uint32_t>(sel) << RCC_PLLCFGR_PLL3VCOSEL_Pos);
  }

  static IGB_FAST_INLINE void enablePll3FracN()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3FRACEN); }
  static IGB_FAST_INLINE void disablePll3FracN() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3FRACEN); }

  static IGB_FAST_INLINE void enablePll3DivP()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVP3EN); }
  static IGB_FAST_INLINE void enablePll3DivQ()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ3EN); }
  static IGB_FAST_INLINE void enablePll3DivR()  { IGB_SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVR3EN); }
  static IGB_FAST_INLINE void disablePll3DivP() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVP3EN); }
  static IGB_FAST_INLINE void disablePll3DivQ() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ3EN); }
  static IGB_FAST_INLINE void disablePll3DivR() { IGB_CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVR3EN); }

  static IGB_FAST_INLINE void setPll3DivN(uint32_t n) {
    IGB_MODIFY_REG(RCC->PLL3DIVR, RCC_PLL3DIVR_N3_Msk, (n - 1U) << RCC_PLL3DIVR_N3_Pos);
  }
  static IGB_FAST_INLINE void setPll3DivP(uint32_t p) {
    IGB_MODIFY_REG(RCC->PLL3DIVR, RCC_PLL3DIVR_P3_Msk, (p - 1U) << RCC_PLL3DIVR_P3_Pos);
  }
  static IGB_FAST_INLINE void setPll3DivQ(uint32_t q) {
    IGB_MODIFY_REG(RCC->PLL3DIVR, RCC_PLL3DIVR_Q3_Msk, (q - 1U) << RCC_PLL3DIVR_Q3_Pos);
  }
  static IGB_FAST_INLINE void setPll3DivR(uint32_t r) {
    IGB_MODIFY_REG(RCC->PLL3DIVR, RCC_PLL3DIVR_R3_Msk, (r - 1U) << RCC_PLL3DIVR_R3_Pos);
  }

  static IGB_FAST_INLINE void setPll3FracN(uint32_t fracn) {
    IGB_MODIFY_REG(RCC->PLL3FRACR, RCC_PLL3FRACR_FRACN3_Msk,
                   fracn << RCC_PLL3FRACR_FRACN3_Pos);
  }

  // --- Bus clock prescalers ---

  // SYSCLK (D1 domain CPU) prescaler — D1CFGR.D1CPRE
  // H7AhbPrescaler values are HPRE field values (pos 0); shift to D1CPRE position (pos 8)
  static IGB_FAST_INLINE void setPrescalerSYSCLK(H7AhbPrescaler prescaler) {
    IGB_MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1CPRE_Msk,
                   as<uint32_t>(prescaler) << RCC_D1CFGR_D1CPRE_Pos);
  }

  // AHB3 (HCLK) prescaler — D1CFGR.HPRE (pos 0, no shift needed)
  static IGB_FAST_INLINE void setPrescalerHCLK(H7AhbPrescaler prescaler) {
    IGB_MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_HPRE_Msk, as<uint32_t>(prescaler));
  }

  // APB3 prescaler — D1CFGR.D1PPRE
  static IGB_FAST_INLINE void setPrescalerAPB3(H7ApbPrescaler prescaler) {
    IGB_MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1PPRE_Msk,
                   as<uint32_t>(prescaler) << RCC_D1CFGR_D1PPRE_Pos);
  }

  // APB1 prescaler — D2CFGR.D2PPRE1
  static IGB_FAST_INLINE void setPrescalerAPB1(H7ApbPrescaler prescaler) {
    IGB_MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE1_Msk,
                   as<uint32_t>(prescaler) << RCC_D2CFGR_D2PPRE1_Pos);
  }

  // APB2 prescaler — D2CFGR.D2PPRE2
  static IGB_FAST_INLINE void setPrescalerAPB2(H7ApbPrescaler prescaler) {
    IGB_MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE2_Msk,
                   as<uint32_t>(prescaler) << RCC_D2CFGR_D2PPRE2_Pos);
  }

  // APB4 prescaler — D3CFGR.D3PPRE
  static IGB_FAST_INLINE void setPrescalerAPB4(H7ApbPrescaler prescaler) {
    IGB_MODIFY_REG(RCC->D3CFGR, RCC_D3CFGR_D3PPRE_Msk,
                   as<uint32_t>(prescaler) << RCC_D3CFGR_D3PPRE_Pos);
  }

  // --- Peripheral clock source selections ---

  // SAI1 clock source — D2CCIP1R.SAI1SEL
  static IGB_FAST_INLINE void setSai1ClockSrc(Sai1ClockSrc src) {
    IGB_MODIFY_REG(RCC->D2CCIP1R, RCC_D2CCIP1R_SAI1SEL_Msk,
                   as<uint32_t>(src) << RCC_D2CCIP1R_SAI1SEL_Pos);
  }

  // SAI2/3 clock source — D2CCIP1R.SAI23SEL
  static IGB_FAST_INLINE void setSai23ClockSrc(Sai23ClockSrc src) {
    IGB_MODIFY_REG(RCC->D2CCIP1R, RCC_D2CCIP1R_SAI23SEL_Msk,
                   as<uint32_t>(src) << RCC_D2CCIP1R_SAI23SEL_Pos);
  }

  // SPI1/2/3 clock source — D2CCIP1R.SPI123SEL
  static IGB_FAST_INLINE void setSpi123ClockSrc(Spi123ClockSrc src) {
    IGB_MODIFY_REG(RCC->D2CCIP1R, RCC_D2CCIP1R_SPI123SEL_Msk,
                   as<uint32_t>(src) << RCC_D2CCIP1R_SPI123SEL_Pos);
  }

  // FMC clock source — D1CCIPR.FMCSEL
  static IGB_FAST_INLINE void setFmcClockSrc(FmcClockSrc src) {
    IGB_MODIFY_REG(RCC->D1CCIPR, RCC_D1CCIPR_FMCSEL_Msk,
                   as<uint32_t>(src) << RCC_D1CCIPR_FMCSEL_Pos);
  }

  // QSPI clock source — D1CCIPR.QSPISEL
  static IGB_FAST_INLINE void setQspiClockSrc(QspiClockSrc src) {
    IGB_MODIFY_REG(RCC->D1CCIPR, RCC_D1CCIPR_QSPISEL_Msk,
                   as<uint32_t>(src) << RCC_D1CCIPR_QSPISEL_Pos);
  }

  // SDMMC clock source — D1CCIPR.SDMMCSEL
  static IGB_FAST_INLINE void setSdmmcClockSrc(SdmmcClockSrc src) {
    IGB_MODIFY_REG(RCC->D1CCIPR, RCC_D1CCIPR_SDMMCSEL_Msk,
                   as<uint32_t>(src) << RCC_D1CCIPR_SDMMCSEL_Pos);
  }

  // USB clock source — D2CCIP2R.USBSEL
  static IGB_FAST_INLINE void setUsbClockSrc(UsbClockSrc src) {
    IGB_MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_USBSEL_Msk,
                   as<uint32_t>(src) << RCC_D2CCIP2R_USBSEL_Pos);
  }
