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
#elif defined(STM32H7)
enum class I2cClockSrc : uint32_t {
  internal = 0,
  system,
  pll3,
  csi
};
#endif

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

enum class UsartClockSrc : uint8_t {
  pclk = 0,
  system = 1,
  lse = 2,
  hsi = 3
};
