enum class RccClockSrc {
  internal = RCC_CFGR_SW_HSI,
  external = RCC_CFGR_SW_HSE,
  pll = RCC_CFGR_SW_PLL,
};

enum class RccClockSrcStatus {
  internal = RCC_CFGR_SWS_HSI,
  external = RCC_CFGR_SWS_HSE,
  pll = RCC_CFGR_SWS_PLL,
};

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

enum class RccClockPrescalerAPB1 {
  div1 = RCC_CFGR_PPRE1_DIV1,
  div2 = RCC_CFGR_PPRE1_DIV2,
  div4 = RCC_CFGR_PPRE1_DIV4,
  div8 = RCC_CFGR_PPRE1_DIV8,
  div16 = RCC_CFGR_PPRE1_DIV16
};

enum class RccClockPrescalerAPB2 {
  div1 = RCC_CFGR_PPRE2_DIV1,
  div2 = RCC_CFGR_PPRE2_DIV2,
  div4 = RCC_CFGR_PPRE2_DIV4,
  div8 = RCC_CFGR_PPRE2_DIV8,
  div16 = RCC_CFGR_PPRE2_DIV16
};

enum class RccPllClockSrc : uint32_t {
  internal = RCC_PLLCFGR_PLLSRC_HSI,
  external = RCC_PLLCFGR_PLLSRC_HSE,
};

