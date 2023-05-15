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
