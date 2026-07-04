enum class AdcStatus : uint32_t {
  ready                  = ADC_ISR_ADRDY,
  endOfSampling          = ADC_ISR_EOSMP,
  endOfConversion        = ADC_ISR_EOC,
  endOfSeqConversions    = ADC_ISR_EOS,
  overrun                = ADC_ISR_OVR,
  injectedEndOfConversion = ADC_ISR_JEOC,
  analogWatchdog1        = ADC_ISR_AWD1,
  analogWatchdog2        = ADC_ISR_AWD2,
  analogWatchdog3        = ADC_ISR_AWD3,
  injectedQueueOverflow  = ADC_ISR_JQOVF,
  ldoReady               = ADC_ISR_LDORDY,
};

enum class AdcCommonStatus : uint32_t {
  readyOnMaster                      = ADC_CSR_ADRDY_MST,
  endOfSamplingOnMaster              = ADC_CSR_EOSMP_MST,
  endOfConversionOnMaster            = ADC_CSR_EOC_MST,
  endOfSeqConversionsOnMaster        = ADC_CSR_EOS_MST,
  overrunOnMaster                    = ADC_CSR_OVR_MST,
  injectedEndOfConversionOnMaster    = ADC_CSR_JEOC_MST,
  injectedEndOfSeqConversionOnMaster = ADC_CSR_JEOS_MST,
  analogWatchdog1OnMaster            = ADC_CSR_AWD1_MST,
  analogWatchdog2OnMaster            = ADC_CSR_AWD2_MST,
  analogWatchdog3OnMaster            = ADC_CSR_AWD3_MST,
  injectedQueueOverflowOnMaster      = ADC_CSR_JQOVF_MST,
  readyOnSlave                       = ADC_CSR_ADRDY_SLV,
  endOfSamplingOnSlave               = ADC_CSR_EOSMP_SLV,
  endOfConversionOnSlave             = ADC_CSR_EOC_SLV,
  endOfSeqConversionsOnSlave         = ADC_CSR_EOS_SLV,
  overrunOnSlave                     = ADC_CSR_OVR_SLV,
  injectedEndOfConversionOnSlave     = ADC_CSR_JEOC_SLV,
  injectedEndOfSeqConversionOnSlave  = ADC_CSR_JEOS_SLV,
  analogWatchdog1OnSlave             = ADC_CSR_AWD1_SLV,
  analogWatchdog2OnSlave             = ADC_CSR_AWD2_SLV,
  analogWatchdog3OnSlave             = ADC_CSR_AWD3_SLV,
  injectedQueueOverflowOnSlave       = ADC_CSR_JQOVF_SLV,
};

enum class AdcInterruptType : uint32_t {
  ready                       = ADC_IER_ADRDYIE,
  endOfSampling               = ADC_IER_EOSMPIE,
  endOfConversion             = ADC_IER_EOCIE,
  endOfSeqConversions         = ADC_IER_EOSIE,
  overrun                     = ADC_IER_OVRIE,
  injectedEndOfConversion     = ADC_IER_JEOCIE,
  injectedEndOfSeqConversions = ADC_IER_JEOSIE,
  analogWatchdog1             = ADC_IER_AWD1IE,
  analogWatchdog2             = ADC_IER_AWD2IE,
  analogWatchdog3             = ADC_IER_AWD3IE,
  injectedQueueOverflow       = ADC_IER_JQOVFIE,
};

// H7: ADCALLIN enables linearity calibration (bit 16 in CR)
enum class AdcCalibrationType : uint32_t {
  singleEnded               = 0,
  differentialEnded         = ADC_CR_ADCALDIF,
  singleEndedWithLinearity  = ADC_CR_ADCALLIN,
  differentialWithLinearity = ADC_CR_ADCALDIF | ADC_CR_ADCALLIN,
};

enum class AdcRegulatorState : uint32_t {
  clear   = 0,
  enable  = 1,
  disable = 2,
};

// H7: DMNGT[1:0] in CFGR replaces DMAEN + DMACFG
enum class AdcDmngtConfig : uint32_t {
  noTransfer  = 0,
  dmaOneShot  = ADC_CFGR_DMNGT_0,
  dfsdm       = ADC_CFGR_DMNGT_1,
  dmaCircular = ADC_CFGR_DMNGT_0 | ADC_CFGR_DMNGT_1,
};

// H7: RES[2:0] is 3-bit (bits 4:2 of CFGR); 16-bit is default (000)
enum class AdcResolution : uint32_t {
  _16bit = 0,
  _14bit = ADC_CFGR_RES_0,
  _12bit = ADC_CFGR_RES_1,
  _10bit = ADC_CFGR_RES_0 | ADC_CFGR_RES_1,
  _8bit  = ADC_CFGR_RES_2,
};

enum class AdcExternalTriggerPolarity : uint32_t {
  disable     = 0,
  risingEdge  = ADC_CFGR_EXTEN_0,
  fallingEdge = ADC_CFGR_EXTEN_1,
  bothEdges   = ADC_CFGR_EXTEN_0 | ADC_CFGR_EXTEN_1,
};

enum class AdcOverrunMode : uint32_t {
  keepOldData        = 0,
  overwriteByNewData = ADC_CFGR_OVRMOD,
};

enum class AdcJsqrMode : uint32_t {
  keepOld    = 0,
  discardOld = ADC_CFGR_JQM,
};

// H7: BOOST[1:0] in CR selects operating clock range
enum class AdcBoostMode : uint32_t {
  clock6mhz  = 0,
  clock12mhz = ADC_CR_BOOST_0,
  clock25mhz = ADC_CR_BOOST_1,
  clock50mhz = ADC_CR_BOOST_0 | ADC_CR_BOOST_1,
};

// H7: OVSR[9:0] in CFGR2 — ratio = OVSR+1 (10-bit raw field value)
enum class AdcOverSamplingRate : uint32_t {
  x1    = 0,
  x2    = 1,
  x4    = 3,
  x8    = 7,
  x16   = 15,
  x32   = 31,
  x64   = 63,
  x128  = 127,
  x256  = 255,
  x512  = 511,
  x1024 = 1023,
};

// H7: OVSS[3:0] in CFGR2 — 11 shifts (0=none ... 10=10-bit)
enum class AdcOverSamplingShift : uint32_t {
  none  = 0,
  _1bit,
  _2bit,
  _3bit,
  _4bit,
  _5bit,
  _6bit,
  _7bit,
  _8bit,
  _9bit,
  _10bit,
};

enum class AdcRegularOverSamplingMode : uint32_t {
  continuous    = 0,
  discontinuous = 1,
};

enum class AdcCommonDualMode : uint32_t {
  independent                = 0,
  syncRegularAndInject       = 1,
  syncRegularAndAltenateTrig = 2,
  syncInterleaveAndInject    = 3,
  reserved                   = 4,
  syncInject                 = 5,
  syncRegular                = 6,
  syncInterleave             = 7,
  alternateTrig              = 9,
};

enum class AdcInjectGrpExtTrigMode : uint32_t {
  noTrig      = 0,
  risingEdge  = ADC_JSQR_JEXTEN_0,
  fallingEdge = ADC_JSQR_JEXTEN_1,
  bothEdge    = ADC_JSQR_JEXTEN_0 | ADC_JSQR_JEXTEN_1,
};

// H7: DAMDF[1:0] in CCR replaces MDMA (dual-mode data format)
enum class AdcCommonDamdfMode : uint32_t {
  disable      = 0,
  reserved     = ADC_CCR_DAMDF_0,
  dfsdmFormat  = ADC_CCR_DAMDF_1,
  packing32bit = ADC_CCR_DAMDF_0 | ADC_CCR_DAMDF_1,
};

enum class AdcCommonClockMode : uint32_t {
  async        = 0,
  syncHclkDiv1 = ADC_CCR_CKMODE_0,
  syncHclkDiv2 = ADC_CCR_CKMODE_1,
  syncHclkDiv4 = ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1,
};

enum class AdcCommonPrescaler : uint32_t {
  div1 = 0,
  div2,
  div4,
  div6,
  div8,
  div10,
  div12,
  div16,
  div32,
  div64,
  div128,
  div256,
};

// H7: channels 0–19 (ch0 added vs F3/G4 which starts at ch1)
enum class AdcChannel : uint32_t {
  ch0 = 0,
  ch1,
  ch2,
  ch3,
  ch4,
  ch5,
  ch6,
  ch7,
  ch8,
  ch9,
  ch10,
  ch11,
  ch12,
  ch13,
  ch14,
  ch15,
  ch16,
  ch17,
  ch18,
  ch19,
};

// H7 sampling times (fractional ADC clock cycles)
enum class AdcSamplingTime : uint8_t {
  clock1_5   = 0,
  clock2_5,
  clock8_5,
  clock16_5,
  clock32_5,
  clock64_5,
  clock387_5,
  clock810_5,
};

struct AdcConf {
  AdcResolution resolution = AdcResolution::_16bit;
  bool auto_delay_mode = false;
  uint8_t external_trigger_select = 0;
  AdcExternalTriggerPolarity external_trigger_polarity = AdcExternalTriggerPolarity::disable;
  bool discontinuous_conv_mode = false;
  uint8_t discontinuous_conv_channel_count = 0;
  bool continuous_conv_mode = false;
  AdcDmngtConfig dmngt_config = AdcDmngtConfig::noTransfer;
  AdcOverrunMode overrun_mode = AdcOverrunMode::overwriteByNewData;
  AdcCommonClockMode clock_mode = AdcCommonClockMode::async;
  AdcCommonPrescaler prescaler = AdcCommonPrescaler::div1;
  AdcBoostMode boost_mode = AdcBoostMode::clock6mhz;
  bool vrefint = false;
  bool temperature_sensor = false;
  bool vbat = false;
  AdcCalibrationType calibration_type = AdcCalibrationType::singleEnded;
  bool enable_interrupt = false;
  uint32_t interrupt_bits = static_cast<uint32_t>(AdcInterruptType::endOfConversion);
  uint8_t interrupt_priority = 1;
  uint8_t lshift = 0; // CFGR2.LSHIFT[3:0]: left-shift result 0–15
  // oversampling (CFGR2.ROVSE / OVSR / OVSS)
  bool oversampling_enable = false;
  AdcOverSamplingRate oversampling_rate = AdcOverSamplingRate::x1;
  AdcOverSamplingShift oversampling_shift = AdcOverSamplingShift::none;
};

struct AdcCommon {
  constexpr static auto addr = ADC12_COMMON_BASE;

  RegEnum<IGB_ADC_COMMON_REG_ADDR(CCR), ADC_CCR_DUAL_Msk, AdcCommonDualMode> dualMode;
  RegValue<IGB_ADC_COMMON_REG_ADDR(CCR), ADC_CCR_DELAY_Msk, ADC_CCR_DELAY_Pos> samplingPhaseDelay;
  RegEnum<IGB_ADC_COMMON_REG_ADDR(CCR), ADC_CCR_DAMDF_Msk, AdcCommonDamdfMode> damdfMode;
  RegEnum<IGB_ADC_COMMON_REG_ADDR(CCR), ADC_CCR_CKMODE_Msk, AdcCommonClockMode> clockMode;
  RegEnum<IGB_ADC_COMMON_REG_ADDR(CCR), ADC_CCR_PRESC_Msk, AdcCommonPrescaler, ADC_CCR_PRESC_Pos> prescaler;
  RegFlag<IGB_ADC_COMMON_REG_ADDR(CCR), ADC_CCR_VREFEN> vrefint;
  RegFlag<IGB_ADC_COMMON_REG_ADDR(CCR), ADC_CCR_TSEN> temperatureSensor;
  RegFlag<IGB_ADC_COMMON_REG_ADDR(CCR), ADC_CCR_VBATEN> vbat;

  RegValue<IGB_ADC_COMMON_REG_ADDR(CDR), ADC_CDR_RDATA_MST_Msk, ADC_CDR_RDATA_MST_Pos> masterData;
  RegValue<IGB_ADC_COMMON_REG_ADDR(CDR), ADC_CDR_RDATA_SLV_Msk, ADC_CDR_RDATA_SLV_Pos> slaveData;

  IGB_FAST_INLINE bool is(AdcCommonStatus status) {
    volatile bool result = IGB_ADC_COMMON->CSR & static_cast<uint32_t>(status);
    return result;
  }
};

struct AdcPinConf {
  AdcChannel ch;
  GpioPinType pin_type;
  AdcSamplingTime sampling_time = AdcSamplingTime::clock32_5;
};

template<AdcType ADC_TYPE>
struct Adc {
  constexpr static auto type = ADC_TYPE;
  constexpr static auto info = STM32_PERIPH_INFO.adc[to_idx(type)];
  constexpr static auto addr = STM32_PERIPH_INFO.adc[to_idx(type)].addr;
  constexpr static auto addr_CR      = IGB_ADC_REG_ADDR(CR);
  constexpr static auto addr_DR      = IGB_ADC_REG_ADDR(DR);
  constexpr static auto addr_ISR     = IGB_ADC_REG_ADDR(ISR);
  constexpr static auto addr_IER     = IGB_ADC_REG_ADDR(IER);
  constexpr static auto addr_CFGR    = IGB_ADC_REG_ADDR(CFGR);
  constexpr static auto addr_CFGR2   = IGB_ADC_REG_ADDR(CFGR2);
  constexpr static auto addr_SMPR1   = IGB_ADC_REG_ADDR(SMPR1);
  constexpr static auto addr_SMPR2   = IGB_ADC_REG_ADDR(SMPR2);
  constexpr static auto addr_PCSEL   = IGB_ADC_REG_ADDR(PCSEL);
  constexpr static auto addr_LTR1    = IGB_ADC_REG_ADDR(LTR1);
  constexpr static auto addr_HTR1    = IGB_ADC_REG_ADDR(HTR1);
  constexpr static auto addr_SQR1    = IGB_ADC_REG_ADDR(SQR1);
  constexpr static auto addr_SQR2    = IGB_ADC_REG_ADDR(SQR2);
  constexpr static auto addr_SQR3    = IGB_ADC_REG_ADDR(SQR3);
  constexpr static auto addr_SQR4    = IGB_ADC_REG_ADDR(SQR4);
  constexpr static auto addr_JSQR    = IGB_ADC_REG_ADDR(JSQR);
  constexpr static auto addr_OFR1    = IGB_ADC_REG_ADDR(OFR1);
  constexpr static auto addr_OFR2    = IGB_ADC_REG_ADDR(OFR2);
  constexpr static auto addr_OFR3    = IGB_ADC_REG_ADDR(OFR3);
  constexpr static auto addr_OFR4    = IGB_ADC_REG_ADDR(OFR4);
  constexpr static auto addr_JDR1    = IGB_ADC_REG_ADDR(JDR1);
  constexpr static auto addr_JDR2    = IGB_ADC_REG_ADDR(JDR2);
  constexpr static auto addr_JDR3    = IGB_ADC_REG_ADDR(JDR3);
  constexpr static auto addr_JDR4    = IGB_ADC_REG_ADDR(JDR4);
  constexpr static auto addr_AWD2CR  = IGB_ADC_REG_ADDR(AWD2CR);
  constexpr static auto addr_AWD3CR  = IGB_ADC_REG_ADDR(AWD3CR);
  constexpr static auto addr_LTR2    = IGB_ADC_REG_ADDR(LTR2);
  constexpr static auto addr_HTR2    = IGB_ADC_REG_ADDR(HTR2);
  constexpr static auto addr_LTR3    = IGB_ADC_REG_ADDR(LTR3);
  constexpr static auto addr_HTR3    = IGB_ADC_REG_ADDR(HTR3);
  constexpr static auto addr_DIFSEL  = IGB_ADC_REG_ADDR(DIFSEL);
  constexpr static auto addr_CALFACT = IGB_ADC_REG_ADDR(CALFACT);
  constexpr static auto addr_CALFACT2 = IGB_ADC_REG_ADDR(CALFACT2);

  // CR
  RegValue<addr_CR, ADC_CR_BITS_PROPERTY_RS, 0> propertyRsClearBits;
  RegFlag<addr_CR, ADC_CR_ADSTART>   convStart;
  RegFlag<addr_CR, ADC_CR_JADSTART>  injectedConvStart;
  RegFlag<addr_CR, ADC_CR_ADSTP>     convStop;
  RegFlag<addr_CR, ADC_CR_JADSTP>    injectedConvStop;
  RegFlag<addr_CR, ADC_CR_DEEPPWD>   deepPowerDown;
  RegFlag<addr_CR, ADC_CR_ADVREGEN>  regulator; // 1-bit in H7
  RegEnum<addr_CR, ADC_CR_BOOST_Msk, AdcBoostMode, ADC_CR_BOOST_Pos> boostMode;
  RegFlag<addr_CR, ADC_CR_ADCALLIN>  linearityCalibration;
  RegFlag<addr_CR, ADC_CR_ADCALDIF>  differentialCalibration;
  RegFlag<addr_CR, ADC_CR_ADCAL>     calibration;

  // CFGR
  RegEnum<addr_CFGR, ADC_CFGR_DMNGT_Msk, AdcDmngtConfig>                        dmngtConfig;
  RegEnum<addr_CFGR, ADC_CFGR_RES_Msk,   AdcResolution>                          resolution;
  RegValue<addr_CFGR, ADC_CFGR_EXTSEL_Msk, ADC_CFGR_EXTSEL_Pos>                  externalTrigSelect;
  RegEnum<addr_CFGR, ADC_CFGR_EXTEN_Msk,  AdcExternalTriggerPolarity>             externalTrigPolarity;
  RegEnum<addr_CFGR, ADC_CFGR_OVRMOD_Msk, AdcOverrunMode>                         overrunMode;
  RegFlag<addr_CFGR, ADC_CFGR_CONT>                                                continuousConvMode;
  RegFlag<addr_CFGR, ADC_CFGR_DISCEN>                                              discontinuousConvMode;
  RegFlag<addr_CFGR, ADC_CFGR_AUTDLY>                                              autoDelayMode;
  RegValue<addr_CFGR, ADC_CFGR_DISCNUM_Msk, ADC_CFGR_DISCNUM_Pos>                discontinuousConvChannelCount;
  RegFlag<addr_CFGR, ADC_CFGR_JDISCEN>                                             injectedDiscontinuousMode;
  RegEnum<addr_CFGR, ADC_CFGR_JQM_Msk,    AdcJsqrMode>                            jsqrMode;
  RegFlag<addr_CFGR, ADC_CFGR_AWD1SGL, false>                                     watchdog1ForAllCh;
  RegFlag<addr_CFGR, ADC_CFGR_AWD1EN>                                              regularWatchdog1;
  RegFlag<addr_CFGR, ADC_CFGR_JAWD1EN>                                             injectWatchdog1;
  RegFlag<addr_CFGR, ADC_CFGR_JAUTO>                                               autoInjectGroupConv;
  RegValue<addr_CFGR, ADC_CFGR_AWD1CH_Msk, ADC_CFGR_AWD1CH_Pos>                  watchdog1Channel;
  RegFlag<addr_CFGR, ADC_CFGR_JQDIS>                                               disableInjectQue;

  // CFGR2
  RegFlag<addr_CFGR2, ADC_CFGR2_ROVSE>                                             enableRegularOverSampling;
  RegFlag<addr_CFGR2, ADC_CFGR2_JOVSE>                                             enableInjectOverSampling;
  RegEnum<addr_CFGR2, ADC_CFGR2_OVSS_Msk, AdcOverSamplingShift, ADC_CFGR2_OVSS_Pos> overSamplingShift;
  RegFlag<addr_CFGR2, ADC_CFGR2_TROVS>                                             triggerRegularOverSampling;
  RegEnum<addr_CFGR2, ADC_CFGR2_ROVSM_Msk, AdcRegularOverSamplingMode, ADC_CFGR2_ROVSM_Pos> regularOverSamplingMode;
  RegEnum<addr_CFGR2, ADC_CFGR2_OVSR_Msk, AdcOverSamplingRate, ADC_CFGR2_OVSR_Pos> overSamplingRate;
  RegValue<addr_CFGR2, ADC_CFGR2_LSHIFT_Msk, ADC_CFGR2_LSHIFT_Pos>               lshift;

  // DR (read-only, full 32-bit to support 16-bit result)
  RegValueRO<addr_DR, ADC_DR_RDATA_Msk, ADC_DR_RDATA_Pos> data;

  // SMPR1: ch0–ch9
  RegValue<addr_SMPR1, ADC_SMPR1_SMP0_Msk, ADC_SMPR1_SMP0_Pos> ch0SamplingTime;
  RegValue<addr_SMPR1, ADC_SMPR1_SMP1_Msk, ADC_SMPR1_SMP1_Pos> ch1SamplingTime;
  RegValue<addr_SMPR1, ADC_SMPR1_SMP2_Msk, ADC_SMPR1_SMP2_Pos> ch2SamplingTime;
  RegValue<addr_SMPR1, ADC_SMPR1_SMP3_Msk, ADC_SMPR1_SMP3_Pos> ch3SamplingTime;
  RegValue<addr_SMPR1, ADC_SMPR1_SMP4_Msk, ADC_SMPR1_SMP4_Pos> ch4SamplingTime;
  RegValue<addr_SMPR1, ADC_SMPR1_SMP5_Msk, ADC_SMPR1_SMP5_Pos> ch5SamplingTime;
  RegValue<addr_SMPR1, ADC_SMPR1_SMP6_Msk, ADC_SMPR1_SMP6_Pos> ch6SamplingTime;
  RegValue<addr_SMPR1, ADC_SMPR1_SMP7_Msk, ADC_SMPR1_SMP7_Pos> ch7SamplingTime;
  RegValue<addr_SMPR1, ADC_SMPR1_SMP8_Msk, ADC_SMPR1_SMP8_Pos> ch8SamplingTime;
  RegValue<addr_SMPR1, ADC_SMPR1_SMP9_Msk, ADC_SMPR1_SMP9_Pos> ch9SamplingTime;

  // SMPR2: ch10–ch19
  RegValue<addr_SMPR2, ADC_SMPR2_SMP10_Msk, ADC_SMPR2_SMP10_Pos> ch10SamplingTime;
  RegValue<addr_SMPR2, ADC_SMPR2_SMP11_Msk, ADC_SMPR2_SMP11_Pos> ch11SamplingTime;
  RegValue<addr_SMPR2, ADC_SMPR2_SMP12_Msk, ADC_SMPR2_SMP12_Pos> ch12SamplingTime;
  RegValue<addr_SMPR2, ADC_SMPR2_SMP13_Msk, ADC_SMPR2_SMP13_Pos> ch13SamplingTime;
  RegValue<addr_SMPR2, ADC_SMPR2_SMP14_Msk, ADC_SMPR2_SMP14_Pos> ch14SamplingTime;
  RegValue<addr_SMPR2, ADC_SMPR2_SMP15_Msk, ADC_SMPR2_SMP15_Pos> ch15SamplingTime;
  RegValue<addr_SMPR2, ADC_SMPR2_SMP16_Msk, ADC_SMPR2_SMP16_Pos> ch16SamplingTime;
  RegValue<addr_SMPR2, ADC_SMPR2_SMP17_Msk, ADC_SMPR2_SMP17_Pos> ch17SamplingTime;
  RegValue<addr_SMPR2, ADC_SMPR2_SMP18_Msk, ADC_SMPR2_SMP18_Pos> ch18SamplingTime;
  RegValue<addr_SMPR2, ADC_SMPR2_SMP19_Msk, ADC_SMPR2_SMP19_Pos> ch19SamplingTime;

  // PCSEL: pre-channel selection (ch0–ch19)
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_0>  ch0PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_1>  ch1PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_2>  ch2PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_3>  ch3PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_4>  ch4PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_5>  ch5PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_6>  ch6PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_7>  ch7PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_8>  ch8PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_9>  ch9PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_10> ch10PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_11> ch11PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_12> ch12PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_13> ch13PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_14> ch14PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_15> ch15PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_16> ch16PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_17> ch17PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_18> ch18PcselEnable;
  RegFlag<addr_PCSEL, ADC_PCSEL_PCSEL_19> ch19PcselEnable;

  // Watchdog thresholds (H7: separate LTR/HTR registers, 26-bit fields)
  RegValue<addr_LTR1, ADC_LTR_LT_Msk, ADC_LTR_LT_Pos> watchdog1LowThreshold;
  RegValue<addr_HTR1, ADC_HTR_HT_Msk, ADC_HTR_HT_Pos> watchdog1HighThreshold;
  RegValue<addr_LTR2, ADC_LTR_LT_Msk, ADC_LTR_LT_Pos> watchdog2LowThreshold;
  RegValue<addr_HTR2, ADC_HTR_HT_Msk, ADC_HTR_HT_Pos> watchdog2HighThreshold;
  RegValue<addr_LTR3, ADC_LTR_LT_Msk, ADC_LTR_LT_Pos> watchdog3LowThreshold;
  RegValue<addr_HTR3, ADC_HTR_HT_Msk, ADC_HTR_HT_Pos> watchdog3HighThreshold;

  // SQR (same as F3/G4)
  RegValue<addr_SQR1, ADC_SQR1_L_Msk,   ADC_SQR1_L_Pos>   seqChLength;
  RegValue<addr_SQR1, ADC_SQR1_SQ1_Msk, ADC_SQR1_SQ1_Pos> seqOrder1Ch;
  RegValue<addr_SQR1, ADC_SQR1_SQ2_Msk, ADC_SQR1_SQ2_Pos> seqOrder2Ch;
  RegValue<addr_SQR1, ADC_SQR1_SQ3_Msk, ADC_SQR1_SQ3_Pos> seqOrder3Ch;
  RegValue<addr_SQR1, ADC_SQR1_SQ4_Msk, ADC_SQR1_SQ4_Pos> seqOrder4Ch;
  RegValue<addr_SQR2, ADC_SQR2_SQ5_Msk, ADC_SQR2_SQ5_Pos> seqOrder5Ch;
  RegValue<addr_SQR2, ADC_SQR2_SQ6_Msk, ADC_SQR2_SQ6_Pos> seqOrder6Ch;
  RegValue<addr_SQR2, ADC_SQR2_SQ7_Msk, ADC_SQR2_SQ7_Pos> seqOrder7Ch;
  RegValue<addr_SQR2, ADC_SQR2_SQ8_Msk, ADC_SQR2_SQ8_Pos> seqOrder8Ch;
  RegValue<addr_SQR2, ADC_SQR2_SQ9_Msk, ADC_SQR2_SQ9_Pos> seqOrder9Ch;
  RegValue<addr_SQR3, ADC_SQR3_SQ10_Msk, ADC_SQR3_SQ10_Pos> seqOrder10Ch;
  RegValue<addr_SQR3, ADC_SQR3_SQ11_Msk, ADC_SQR3_SQ11_Pos> seqOrder11Ch;
  RegValue<addr_SQR3, ADC_SQR3_SQ12_Msk, ADC_SQR3_SQ12_Pos> seqOrder12Ch;
  RegValue<addr_SQR3, ADC_SQR3_SQ13_Msk, ADC_SQR3_SQ13_Pos> seqOrder13Ch;
  RegValue<addr_SQR3, ADC_SQR3_SQ14_Msk, ADC_SQR3_SQ14_Pos> seqOrder14Ch;
  RegValue<addr_SQR4, ADC_SQR4_SQ15_Msk, ADC_SQR4_SQ15_Pos> seqOrder15Ch;
  RegValue<addr_SQR4, ADC_SQR4_SQ16_Msk, ADC_SQR4_SQ16_Pos> seqOrder16Ch;

  // JSQR (same as F3/G4)
  RegValue<addr_JSQR, ADC_JSQR_JL_Msk,      ADC_JSQR_JL_Pos>      injectSeqChLength;
  RegValue<addr_JSQR, ADC_JSQR_JEXTSEL_Msk,  ADC_JSQR_JEXTSEL_Pos> injectGrpExtTrig;
  RegEnum<addr_JSQR,  ADC_JSQR_JEXTEN_Msk,   AdcInjectGrpExtTrigMode> injectGrpExtTrigMode;
  RegValue<addr_JSQR, ADC_JSQR_JSQ1_Msk,     ADC_JSQR_JSQ1_Pos>    injectSeqOrder1Ch;
  RegValue<addr_JSQR, ADC_JSQR_JSQ2_Msk,     ADC_JSQR_JSQ2_Pos>    injectSeqOrder2Ch;
  RegValue<addr_JSQR, ADC_JSQR_JSQ3_Msk,     ADC_JSQR_JSQ3_Pos>    injectSeqOrder3Ch;
  RegValue<addr_JSQR, ADC_JSQR_JSQ4_Msk,     ADC_JSQR_JSQ4_Pos>    injectSeqOrder4Ch;

  // OFR1–4 (H7: 26-bit offset value, SSATE replaces OFFSET_EN)
  RegValue<addr_OFR1, ADC_OFR1_OFFSET1_Msk,    ADC_OFR1_OFFSET1_Pos>    dataOffset1Value;
  RegValue<addr_OFR1, ADC_OFR1_OFFSET1_CH_Msk,  ADC_OFR1_OFFSET1_CH_Pos> dataOffset1Ch;
  RegFlag<addr_OFR1,  ADC_OFR1_SSATE>                                      signedSaturation1;

  RegValue<addr_OFR2, ADC_OFR2_OFFSET2_Msk,    ADC_OFR2_OFFSET2_Pos>    dataOffset2Value;
  RegValue<addr_OFR2, ADC_OFR2_OFFSET2_CH_Msk,  ADC_OFR2_OFFSET2_CH_Pos> dataOffset2Ch;
  RegFlag<addr_OFR2,  ADC_OFR2_SSATE>                                      signedSaturation2;

  RegValue<addr_OFR3, ADC_OFR3_OFFSET3_Msk,    ADC_OFR3_OFFSET3_Pos>    dataOffset3Value;
  RegValue<addr_OFR3, ADC_OFR3_OFFSET3_CH_Msk,  ADC_OFR3_OFFSET3_CH_Pos> dataOffset3Ch;
  RegFlag<addr_OFR3,  ADC_OFR3_SSATE>                                      signedSaturation3;

  RegValue<addr_OFR4, ADC_OFR4_OFFSET4_Msk,    ADC_OFR4_OFFSET4_Pos>    dataOffset4Value;
  RegValue<addr_OFR4, ADC_OFR4_OFFSET4_CH_Msk,  ADC_OFR4_OFFSET4_CH_Pos> dataOffset4Ch;
  RegFlag<addr_OFR4,  ADC_OFR4_SSATE>                                      signedSaturation4;

  // JDR (same as F3/G4)
  RegValue<addr_JDR1, ADC_JDR1_JDATA_Msk, ADC_JDR1_JDATA_Pos> injectData1;
  RegValue<addr_JDR2, ADC_JDR2_JDATA_Msk, ADC_JDR2_JDATA_Pos> injectData2;
  RegValue<addr_JDR3, ADC_JDR3_JDATA_Msk, ADC_JDR3_JDATA_Pos> injectData3;
  RegValue<addr_JDR4, ADC_JDR4_JDATA_Msk, ADC_JDR4_JDATA_Pos> injectData4;

  // AWD2CR: ch0–ch19
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_0>  watchdog2Ch0Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_1>  watchdog2Ch1Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_2>  watchdog2Ch2Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_3>  watchdog2Ch3Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_4>  watchdog2Ch4Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_5>  watchdog2Ch5Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_6>  watchdog2Ch6Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_7>  watchdog2Ch7Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_8>  watchdog2Ch8Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_9>  watchdog2Ch9Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_10> watchdog2Ch10Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_11> watchdog2Ch11Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_12> watchdog2Ch12Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_13> watchdog2Ch13Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_14> watchdog2Ch14Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_15> watchdog2Ch15Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_16> watchdog2Ch16Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_17> watchdog2Ch17Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_18> watchdog2Ch18Enable;
  RegFlag<addr_AWD2CR, ADC_AWD2CR_AWD2CH_19> watchdog2Ch19Enable;

  // AWD3CR: ch0–ch19
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_0>  watchdog3Ch0Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_1>  watchdog3Ch1Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_2>  watchdog3Ch2Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_3>  watchdog3Ch3Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_4>  watchdog3Ch4Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_5>  watchdog3Ch5Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_6>  watchdog3Ch6Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_7>  watchdog3Ch7Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_8>  watchdog3Ch8Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_9>  watchdog3Ch9Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_10> watchdog3Ch10Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_11> watchdog3Ch11Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_12> watchdog3Ch12Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_13> watchdog3Ch13Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_14> watchdog3Ch14Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_15> watchdog3Ch15Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_16> watchdog3Ch16Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_17> watchdog3Ch17Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_18> watchdog3Ch18Enable;
  RegFlag<addr_AWD3CR, ADC_AWD3CR_AWD3CH_19> watchdog3Ch19Enable;

  // DIFSEL: ch0–ch19
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_0>  diffModeCh0Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_1>  diffModeCh1Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_2>  diffModeCh2Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_3>  diffModeCh3Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_4>  diffModeCh4Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_5>  diffModeCh5Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_6>  diffModeCh6Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_7>  diffModeCh7Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_8>  diffModeCh8Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_9>  diffModeCh9Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_10> diffModeCh10Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_11> diffModeCh11Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_12> diffModeCh12Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_13> diffModeCh13Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_14> diffModeCh14Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_15> diffModeCh15Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_16> diffModeCh16Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_17> diffModeCh17Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_18> diffModeCh18Enable;
  RegFlag<addr_DIFSEL, ADC_DIFSEL_DIFSEL_19> diffModeCh19Enable;

  // CALFACT: 11-bit single-ended and differential factors
  RegValue<addr_CALFACT, ADC_CALFACT_CALFACT_S_Msk, ADC_CALFACT_CALFACT_S_Pos> singleendCalibrationFactor;
  RegValue<addr_CALFACT, ADC_CALFACT_CALFACT_D_Msk, ADC_CALFACT_CALFACT_D_Pos> differentialCalibrationFactor;

  // CALFACT2: linearity calibration (H7 only)
  RegValue<addr_CALFACT2, ADC_CALFACT2_LINCALFACT_Msk, ADC_CALFACT2_LINCALFACT_Pos> linearityCalibrationFactor;

  IGB_FAST_INLINE void enable() {
    // RM0433 §25.4.9: clear ADRDY (write-1-to-clear) before setting ADEN,
    // to avoid false-ready detection on warm re-initialization.
    IGB_SET_BIT(IGB_ADC->ISR, ADC_ISR_ADRDY);
    IGB_MODIFY_REG(IGB_ADC->CR, ADC_CR_BITS_PROPERTY_RS, ADC_CR_ADEN);
  }

  IGB_FAST_INLINE void disable() {
    IGB_MODIFY_REG(IGB_ADC->CR, ADC_CR_BITS_PROPERTY_RS, ADC_CR_ADDIS);
  }

  IGB_FAST_INLINE bool is(AdcStatus status) {
    volatile bool result = IGB_ADC->ISR & static_cast<uint32_t>(status);
    return result;
  }

  IGB_FAST_INLINE void clear(AdcStatus status) {
    IGB_ADC->ISR = static_cast<uint32_t>(status);
  }

  IGB_FAST_INLINE void enableIt(AdcInterruptType interrupt) {
    IGB_ADC->IER = IGB_ADC->IER | static_cast<uint32_t>(interrupt);
  }

  IGB_FAST_INLINE void disableIt(AdcInterruptType interrupt) {
    IGB_ADC->IER = IGB_ADC->IER & ~(static_cast<uint32_t>(interrupt));
  }

  // H7: ADVREGEN is 1-bit; must clear DEEPPWD first (reset state: DEEPPWD=1)
  IGB_FAST_INLINE void enableRegulator() {
    regulator(false);
    IGB_CLEAR_BIT(IGB_ADC->CR, ADC_CR_DEEPPWD | ADC_CR_BITS_PROPERTY_RS);
    regulator(true);
  }

  IGB_FAST_INLINE void disableRegulator() {
    IGB_CLEAR_BIT(IGB_ADC->CR, ADC_CR_ADVREGEN | ADC_CR_BITS_PROPERTY_RS);
  }

  // H7: supports ADCALDIF and ADCALLIN for linearity calibration
  IGB_FAST_INLINE void startCalibration(AdcCalibrationType type) {
    IGB_MODIFY_REG(IGB_ADC->CR,
      (ADC_CR_ADCALDIF | ADC_CR_ADCALLIN) | ADC_CR_BITS_PROPERTY_RS,
      ADC_CR_ADCAL | (static_cast<uint32_t>(type) & (ADC_CR_ADCALDIF | ADC_CR_ADCALLIN)));
  }

  IGB_FAST_INLINE void startConversion() {
    (propertyRsClearBits.val(0) | convStart.val(true)).update();
  }

  IGB_FAST_INLINE void startInjectedConversion() {
    (propertyRsClearBits.val(0) | injectedConvStart.val(true)).update();
  }

  IGB_FAST_INLINE void stopConversion() {
    (propertyRsClearBits.val(0) | convStop.val(true)).update();
  }

  IGB_FAST_INLINE void stopInjectedConversion() {
    (propertyRsClearBits.val(0) | injectedConvStop.val(true)).update();
  }

  IGB_FAST_INLINE uint16_t readData() { return data(); }

  static IGB_FAST_INLINE void enableBusClock() {
    STM32_PERIPH_INFO.adc[to_idx(type)].bus.enableBusClock();
  }

  static IGB_FAST_INLINE void prepareGpio(GpioPinType pin_type) {
    GpioPin pin = GpioPin::newPin(pin_type);
    pin.enable();
    pin.setMode(GpioMode::analog);
    pin.setPullMode(GpioPullMode::no);
  }

  static IGB_FAST_INLINE void prepareGpios(auto&& first, auto&&... rest) {
    prepareGpio(first.pin_type);
    prepareGpios(rest...);
  }
  static IGB_FAST_INLINE void prepareGpios() { }

  IGB_FAST_INLINE void _setPcsel(AdcChannel ch) {
    switch (ch) {
      case AdcChannel::ch0:  ch0PcselEnable(true);  break;
      case AdcChannel::ch1:  ch1PcselEnable(true);  break;
      case AdcChannel::ch2:  ch2PcselEnable(true);  break;
      case AdcChannel::ch3:  ch3PcselEnable(true);  break;
      case AdcChannel::ch4:  ch4PcselEnable(true);  break;
      case AdcChannel::ch5:  ch5PcselEnable(true);  break;
      case AdcChannel::ch6:  ch6PcselEnable(true);  break;
      case AdcChannel::ch7:  ch7PcselEnable(true);  break;
      case AdcChannel::ch8:  ch8PcselEnable(true);  break;
      case AdcChannel::ch9:  ch9PcselEnable(true);  break;
      case AdcChannel::ch10: ch10PcselEnable(true); break;
      case AdcChannel::ch11: ch11PcselEnable(true); break;
      case AdcChannel::ch12: ch12PcselEnable(true); break;
      case AdcChannel::ch13: ch13PcselEnable(true); break;
      case AdcChannel::ch14: ch14PcselEnable(true); break;
      case AdcChannel::ch15: ch15PcselEnable(true); break;
      case AdcChannel::ch16: ch16PcselEnable(true); break;
      case AdcChannel::ch17: ch17PcselEnable(true); break;
      case AdcChannel::ch18: ch18PcselEnable(true); break;
      case AdcChannel::ch19: ch19PcselEnable(true); break;
      default: break;
    }
  }

  IGB_FAST_INLINE void _initCh(AdcPinConf pin_conf) {
    uint32_t smp = static_cast<uint32_t>(pin_conf.sampling_time);
    switch (pin_conf.ch) {
      case AdcChannel::ch0:  ch0SamplingTime(smp);  break;
      case AdcChannel::ch1:  ch1SamplingTime(smp);  break;
      case AdcChannel::ch2:  ch2SamplingTime(smp);  break;
      case AdcChannel::ch3:  ch3SamplingTime(smp);  break;
      case AdcChannel::ch4:  ch4SamplingTime(smp);  break;
      case AdcChannel::ch5:  ch5SamplingTime(smp);  break;
      case AdcChannel::ch6:  ch6SamplingTime(smp);  break;
      case AdcChannel::ch7:  ch7SamplingTime(smp);  break;
      case AdcChannel::ch8:  ch8SamplingTime(smp);  break;
      case AdcChannel::ch9:  ch9SamplingTime(smp);  break;
      case AdcChannel::ch10: ch10SamplingTime(smp); break;
      case AdcChannel::ch11: ch11SamplingTime(smp); break;
      case AdcChannel::ch12: ch12SamplingTime(smp); break;
      case AdcChannel::ch13: ch13SamplingTime(smp); break;
      case AdcChannel::ch14: ch14SamplingTime(smp); break;
      case AdcChannel::ch15: ch15SamplingTime(smp); break;
      case AdcChannel::ch16: ch16SamplingTime(smp); break;
      case AdcChannel::ch17: ch17SamplingTime(smp); break;
      case AdcChannel::ch18: ch18SamplingTime(smp); break;
      case AdcChannel::ch19: ch19SamplingTime(smp); break;
      default: break;
    }

    _setPcsel(pin_conf.ch);

    // set single-ended mode (ch19 has no differential mode bit)
    switch (pin_conf.ch) {
      case AdcChannel::ch0:  diffModeCh0Enable(false);  break;
      case AdcChannel::ch1:  diffModeCh1Enable(false);  break;
      case AdcChannel::ch2:  diffModeCh2Enable(false);  break;
      case AdcChannel::ch3:  diffModeCh3Enable(false);  break;
      case AdcChannel::ch4:  diffModeCh4Enable(false);  break;
      case AdcChannel::ch5:  diffModeCh5Enable(false);  break;
      case AdcChannel::ch6:  diffModeCh6Enable(false);  break;
      case AdcChannel::ch7:  diffModeCh7Enable(false);  break;
      case AdcChannel::ch8:  diffModeCh8Enable(false);  break;
      case AdcChannel::ch9:  diffModeCh9Enable(false);  break;
      case AdcChannel::ch10: diffModeCh10Enable(false); break;
      case AdcChannel::ch11: diffModeCh11Enable(false); break;
      case AdcChannel::ch12: diffModeCh12Enable(false); break;
      case AdcChannel::ch13: diffModeCh13Enable(false); break;
      case AdcChannel::ch14: diffModeCh14Enable(false); break;
      case AdcChannel::ch15: diffModeCh15Enable(false); break;
      case AdcChannel::ch16: diffModeCh16Enable(false); break;
      case AdcChannel::ch17: diffModeCh17Enable(false); break;
      case AdcChannel::ch18: diffModeCh18Enable(false); break;
      default: break;
    }
  }

  IGB_FAST_INLINE void _initChannels(auto&& first, auto&&... rest) {
    _initCh(first);
    _initChannels(rest...);
  }
  IGB_FAST_INLINE void _initChannels() { }

  IGB_FAST_INLINE void _setSeqOrder(uint8_t idx, auto&& pin_conf) {
    uint32_t ch = static_cast<uint32_t>(pin_conf.ch);
    switch(idx) {
      case 0:  seqOrder1Ch(ch);  break;
      case 1:  seqOrder2Ch(ch);  break;
      case 2:  seqOrder3Ch(ch);  break;
      case 3:  seqOrder4Ch(ch);  break;
      case 4:  seqOrder5Ch(ch);  break;
      case 5:  seqOrder6Ch(ch);  break;
      case 6:  seqOrder7Ch(ch);  break;
      case 7:  seqOrder8Ch(ch);  break;
      case 8:  seqOrder9Ch(ch);  break;
      case 9:  seqOrder10Ch(ch); break;
      case 10: seqOrder11Ch(ch); break;
      case 11: seqOrder12Ch(ch); break;
      case 12: seqOrder13Ch(ch); break;
      case 13: seqOrder14Ch(ch); break;
      case 14: seqOrder15Ch(ch); break;
      case 15: seqOrder16Ch(ch); break;
      default: break;
    }
  }

  IGB_FAST_INLINE void _setSeqOrders(uint8_t idx, auto&& first, auto&&... rest) {
    _setSeqOrder(idx, first);
    _setSeqOrders(idx+1, rest...);
  }
  IGB_FAST_INLINE void _setSeqOrders(uint8_t idx) { }

  IGB_FAST_INLINE void init(auto&& conf, auto&&... pin_confs) {
    enableBusClock();
    prepareGpios(pin_confs...);

    if (conf.enable_interrupt) {
      NvicCtrl::setPriority(info.irqn, conf.interrupt_priority);
      NvicCtrl::enable(info.irqn);
      IGB_ADC->IER = IGB_ADC->IER | conf.interrupt_bits;
    }

    (
      resolution.val(conf.resolution) |
      autoDelayMode.val(conf.auto_delay_mode)
    ).update();

    (
      externalTrigSelect.val(conf.external_trigger_select) |
      externalTrigPolarity.val(conf.external_trigger_polarity) |
      discontinuousConvMode.val(conf.discontinuous_conv_mode) |
      discontinuousConvChannelCount.val(conf.discontinuous_conv_channel_count) |
      continuousConvMode.val(conf.continuous_conv_mode) |
      dmngtConfig.val(conf.dmngt_config) |
      overrunMode.val(conf.overrun_mode)
    ).update();

    lshift(conf.lshift);

    if (conf.oversampling_enable) {
      enableRegularOverSampling(true);
      overSamplingRate(conf.oversampling_rate);
      overSamplingShift(conf.oversampling_shift);
    }

    AdcCommon adc_common;
    adc_common.clockMode(conf.clock_mode);
    adc_common.prescaler(conf.prescaler);
    adc_common.vrefint(conf.vrefint);
    adc_common.temperatureSensor(conf.temperature_sensor);
    adc_common.vbat(conf.vbat);

    // H7 startup: clear DEEPPWD, enable ADVREGEN, wait for LDO ready
    enableRegulator();
    delay_msec(2);

    boostMode(conf.boost_mode);

    seqChLength((sizeof...(pin_confs)) - 1);
    _setSeqOrders(0, pin_confs...);
    _initChannels(pin_confs...);

    startCalibration(conf.calibration_type);
    while (IGB_ADC->CR & ADC_CR_ADCAL) {}

    enable();
    while (!is(AdcStatus::ready)) {}
  }

  IGB_FAST_INLINE bool checkReady() {
    return is(AdcStatus::ready);
  }

  IGB_FAST_INLINE bool checkEndOfConversion() {
    if (is(AdcStatus::endOfConversion)) {
      clear(AdcStatus::endOfConversion);
      if (is(AdcStatus::endOfSeqConversions)) {
        clear(AdcStatus::endOfSeqConversions);
      }
      return true;
    }
    return false;
  }
};

