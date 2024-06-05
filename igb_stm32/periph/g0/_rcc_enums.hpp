enum class RccClockSrc {
  internal = 0UL,
  external = 1UL << RCC_CFGR_SW_Pos,
  pll = 2UL << RCC_CFGR_SW_Pos
};

enum class RccClockSrcStatus {
  internal = 0,
  external = 1UL << RCC_CFGR_SWS_Pos,
  pll = 2UL << RCC_CFGR_SWS_Pos
};

enum class RccClockPrescalerAHB {
  div1 = 0 << RCC_CFGR_HPRE_Pos,
  div2 = 0b1000 << RCC_CFGR_HPRE_Pos,
  div4 = 0b1001 << RCC_CFGR_HPRE_Pos,
  div8 = 0b1010 << RCC_CFGR_HPRE_Pos,
  div16 = 0b1011 << RCC_CFGR_HPRE_Pos,
  div64 = 0b1100 << RCC_CFGR_HPRE_Pos,
  div128 = 0b1101 << RCC_CFGR_HPRE_Pos,
  div256 = 0b1110 << RCC_CFGR_HPRE_Pos,
  div512 = 0b1111 << RCC_CFGR_HPRE_Pos
};

enum class RccClockPrescalerAPB1 {
  div1 = 0 << RCC_CFGR_PPRE_Pos,
  div2 = 0b100 << RCC_CFGR_PPRE_Pos,
  div4 = 0b101 << RCC_CFGR_PPRE_Pos,
  div8 = 0b110 << RCC_CFGR_PPRE_Pos,
  div16 = 0b111 << RCC_CFGR_PPRE_Pos
};

enum class RccPllClockSrc : uint32_t {
  internal = RCC_PLLCFGR_PLLSRC_HSI,
  external = RCC_PLLCFGR_PLLSRC_HSE
};

