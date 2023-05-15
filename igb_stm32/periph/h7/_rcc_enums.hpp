enum class RccClockSrc {
  internal = RCC_CFGR_SW_HSI,
  external = RCC_CFGR_SW_HSE,
  csi = RCC_CFGR_SW_CSI,
  pll1 = RCC_CFGR_SW_PLL1
};

enum class RccClockSrcStatus {
  internal = RCC_CFGR_SWS_HSI,
  external = RCC_CFGR_SWS_HSE,
  csi = RCC_CFGR_SWS_CSI,
  pll1 = RCC_CFGR_SWS_PLL1
};

enum class I2cClockSrc {
  system = 0,
  pll3 = 1,
  internal = 2,
  csi = 3
};

enum class UsartClockSrc {
  system = 0,
  pll2 = 1,
  pll3 = 2,
  internal = 3,
  csi = 4,
  lse = 5
};

enum class AdcClockSrc : uint8_t {
  pll2 = 0,
  pll3 = 1,
  per_ck = 2
};
