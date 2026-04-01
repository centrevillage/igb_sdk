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

// PLL input clock source (PLLCKSELR.PLLSRC)
enum class RccPllClockSrc : uint32_t {
  hsi  = 0,
  csi  = 1,
  hse  = 2,
  none = 3
};

// PLL VCI (input) range (PLL1/2/3RGE in PLLCFGR)
// Select based on VCO input frequency = HSE / DIVM
enum class PllVciRange : uint32_t {
  range0 = 0,  // 1 to 2 MHz
  range1 = 1,  // 2 to 4 MHz
  range2 = 2,  // 4 to 8 MHz
  range3 = 3   // 8 to 16 MHz
};

// PLL VCO output range (PLL1/2/3VCOSEL in PLLCFGR)
enum class PllVcoSel : uint32_t {
  wide   = 0,  // 192 to 836 MHz
  medium = 1   // 150 to 420 MHz
};

// AHB/CPU prescaler — 4-bit field encoding (HPRE and D1CPRE).
// Enum values are the HPRE field values (at pos 0); for D1CPRE (pos 8)
// the set method shifts them appropriately.
enum class RccClockPrescalerAHB : uint32_t {
  div1   = RCC_D1CFGR_HPRE_DIV1,
  div2   = RCC_D1CFGR_HPRE_DIV2,
  div4   = RCC_D1CFGR_HPRE_DIV4,
  div8   = RCC_D1CFGR_HPRE_DIV8,
  div16  = RCC_D1CFGR_HPRE_DIV16,
  div64  = RCC_D1CFGR_HPRE_DIV64,
  div128 = RCC_D1CFGR_HPRE_DIV128,
  div256 = RCC_D1CFGR_HPRE_DIV256,
  div512 = RCC_D1CFGR_HPRE_DIV512,
};

// APB prescaler — 3-bit field value (same encoding for APB1/2/3/4).
// Enum values are the raw 3-bit field; set methods shift to the correct bit position.
enum class RccClockPrescalerAPB : uint32_t {
  div1  = 0,
  div2  = 4,  // 0b100
  div4  = 5,  // 0b101
  div8  = 6,  // 0b110
  div16 = 7,  // 0b111
};

// SAI1 clock source (D2CCIP1R.SAI1SEL)
enum class Sai1ClockSrc : uint32_t {
  pll1_q   = 0,
  pll2_p   = 1,
  pll3_p   = 2,
  i2s_ckin = 3,
  per_ck   = 4
};

// SAI2/3 clock source (D2CCIP1R.SAI23SEL)
enum class Sai23ClockSrc : uint32_t {
  pll1_q   = 0,
  pll2_p   = 1,
  pll3_p   = 2,
  i2s_ckin = 3,
  per_ck   = 4
};

// SPI1/2/3 clock source (D2CCIP1R.SPI123SEL)
enum class Spi123ClockSrc : uint32_t {
  pll1_q   = 0,
  pll2_p   = 1,
  pll3_p   = 2,
  i2s_ckin = 3,
  per_ck   = 4
};

// FMC clock source (D1CCIPR.FMCSEL)
enum class FmcClockSrc : uint32_t {
  hclk3  = 0,
  pll1_q = 1,
  pll2_r = 2,
  per_ck = 3
};

// QSPI clock source (D1CCIPR.QSPISEL)
enum class QspiClockSrc : uint32_t {
  hclk3  = 0,
  pll1_q = 1,
  pll2_r = 2,
  per_ck = 3
};

// SDMMC clock source (D1CCIPR.SDMMCSEL)
enum class SdmmcClockSrc : uint32_t {
  pll1_q = 0,
  pll2_r = 1
};

// USB clock source (D2CCIP2R.USBSEL)
enum class UsbClockSrc : uint32_t {
  disabled = 0,
  pll1_q   = 1,
  pll3_q   = 2,
  hsi48    = 3
};
