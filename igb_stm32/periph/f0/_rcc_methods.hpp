  static IGB_FAST_INLINE RccPllClockSrc getPllSrc() {
    return static_cast<RccPllClockSrc>(RCC->CFGR & RCC_CFGR_PLLSRC);
  }

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
