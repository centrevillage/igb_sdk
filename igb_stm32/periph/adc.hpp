#ifndef IGB_STM32_PERIPH_ADC_H
#define IGB_STM32_PERIPH_ADC_H

#include <stddef.h>

#include <igb_stm32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/periph/rcc.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>
#include <igb_stm32/periph/systick.hpp>

#define ADC_CR_BITS_PROPERTY_RS (ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART | ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN) /* ADC register CR bits with HW property "rs": Software can read as well as set this bit. Writing '0' has no effect on the bit value. */

namespace igb {
namespace stm32 {

#define IGB_ADC ((ADC_TypeDef*)addr)
#define IGB_ADC_REG_ADDR(member) (addr + offsetof(ADC_TypeDef, member))
#define IGB_ADC_REG(member) ((ADC_TypeDef*)IGB_I2C_REG_ADDR(member))

#define IGB_ADC_COMMON ((ADC_Common_TypeDef*)addr)
#define IGB_ADC_COMMON_REG_ADDR(member) (addr + offsetof(ADC_Common_TypeDef, member))
#define IGB_ADC_COMMON_REG(member) ((ADC_Common_TypeDef*)IGB_I2C_REG_ADDR(member))

// TODO: other series
#if defined(STM32F3)

enum class AdcStatus : uint32_t {
  ready = ADC_ISR_ADRDY,
  endOfSampling = ADC_ISR_EOSMP,
  endOfConversion = ADC_ISR_EOC,
  endOfSeqConversions = ADC_ISR_EOS,
  overrun = ADC_ISR_OVR,
  injectedEndOfConversion = ADC_ISR_JEOC,
  analogWatchdog1 = ADC_ISR_AWD1,
  analogWatchdog2 = ADC_ISR_AWD2,
  analogWatchdog3 = ADC_ISR_AWD3,
  injectedQueueOverflow = ADC_ISR_JQOVF,
};

enum class AdcCommonStatus : uint32_t {
  readyOnMaster = ADC12_CSR_ADRDY_MST,
  endOfSamplingOnMaster = ADC12_CSR_ADRDY_EOSMP_MST,
  endOfConversionOnMaster = ADC12_CSR_ADRDY_EOC_MST,
  endOfSeqConversionsOnMaster = ADC12_CSR_ADRDY_EOS_MST,
  overrunOnMaster = ADC12_CSR_ADRDY_OVR_MST,
  injectedEndOfConversionOnMaster = ADC12_CSR_ADRDY_JEOC_MST,
  injectedEndOfSeqConversionOnMaster = ADC12_CSR_ADRDY_JEOS_MST,
  analogWatchdog1OnMaster = ADC12_CSR_AWD1_MST,
  analogWatchdog2OnMaster = ADC12_CSR_AWD2_MST,
  analogWatchdog3OnMaster = ADC12_CSR_AWD3_MST,
  injectedQueueOverflowOnMaster = ADC12_CSR_JQOVF_MST,

  readyOnSlave = ADC12_CSR_ADRDY_SLV,
  endOfSamplingOnSlave = ADC12_CSR_ADRDY_EOSMP_SLV,
  endOfConversionOnSlave = ADC12_CSR_ADRDY_EOC_SLV,
  endOfSeqConversionsOnSlave = ADC12_CSR_ADRDY_EOS_SLV,
  overrunOnSlave = ADC12_CSR_ADRDY_OVR_SLV,
  injectedEndOfConversionOnSlave = ADC12_CSR_ADRDY_JEOC_SLV,
  injectedEndOfSeqConversionOnSlave = ADC12_CSR_ADRDY_JEOS_SLV,
  analogWatchdog1OnSlave = ADC12_CSR_AWD1_SLV,
  analogWatchdog2OnSlave = ADC12_CSR_AWD2_SLV,
  analogWatchdog3OnSlave = ADC12_CSR_AWD3_SLV,
  injectedQueueOverflowOnSlave = ADC12_CSR_JQOVF_SLV,
};

enum class AdcInterruptType : uint32_t {
  ready = ADC_IER_ADRDYIE,
  endOfSampling = ADC_IER_EOSMPIE,
  endOfConversion = ADC_IER_EOCIE,
  endOfSeqConversions = ADC_IER_EOSIE,
  overrun = ADC_IER_OVRIE,
  injectedEndOfConversion = ADC_IER_JEOCIE,
  injectedEndOfSeqConversions = ADC_IER_JEOSIE,
  analogWatchdog1 = ADC_IER_AWD1IE,
  analogWatchdog2 = ADC_IER_AWD2IE,
  analogWatchdog3 = ADC_IER_AWD3IE,
  injectedQueueOverflow = ADC_IER_JQOVFIE,
};

enum class AdcCalibrationType : uint32_t {
  singleEnded = ADC_CALFACT_CALFACT_S,
  differentialEnded = ADC_CR_ADCALDIF | ADC_CALFACT_CALFACT_D,
  single_and_differential = ADC_CALFACT_CALFACT_S | ADC_CR_ADCALDIF | ADC_CALFACT_CALFACT_D
};

enum class AdcRegulatorState : uint32_t {
  clear = 0,
  enable = ADC_CR_ADVREGEN_0,
  disable = ADC_CR_ADVREGEN_1,
};

enum class AdcDmaConfig : uint32_t {
  oneshot = 0,
  circular = ADC_CFGR_DMACFG
};

enum class AdcResolution : uint32_t {
  _12bit = 0,
  _10bit = ADC_CFGR_RES_0,
  _8bit  = ADC_CFGR_RES_1,
  _6bit  = ADC_CFGR_RES_0 | ADC_CFGR_RES_1,
};

enum class AdcDataAlign : uint32_t {
  right = 0,
  left = ADC_CFGR_ALIGN
};

enum class AdcExternalTriggerEvent : uint32_t {
  none = 0,
  event1 = (1UL << ADC_CFGR_EXTSEL_Pos),
  event2 = (2UL << ADC_CFGR_EXTSEL_Pos),
  event3 = (3UL << ADC_CFGR_EXTSEL_Pos),
  event4 = (4UL << ADC_CFGR_EXTSEL_Pos),
  event5 = (5UL << ADC_CFGR_EXTSEL_Pos),
  event6 = (6UL << ADC_CFGR_EXTSEL_Pos),
  event7 = (7UL << ADC_CFGR_EXTSEL_Pos),
  event8 = (8UL << ADC_CFGR_EXTSEL_Pos),
  event9 = (9UL << ADC_CFGR_EXTSEL_Pos),
  event10 = (10UL << ADC_CFGR_EXTSEL_Pos),
  event11 = (11UL << ADC_CFGR_EXTSEL_Pos),
  event12 = (12UL << ADC_CFGR_EXTSEL_Pos),
  event13 = (13UL << ADC_CFGR_EXTSEL_Pos),
  event14 = (14UL << ADC_CFGR_EXTSEL_Pos),
  event15 = (15UL << ADC_CFGR_EXTSEL_Pos),
};

enum class AdcExternalTriggerPolarity : uint32_t {
  disable = 0,
  risingEdge = ADC_CFGR_EXTEN_0,
  fallingEdge = ADC_CFGR_EXTEN_1,
  bothEdges = ADC_CFGR_EXTEN_0 | ADC_CFGR_EXTEN_1,
};

enum class AdcOverrunMode : uint32_t {
  keepOldData = 0,
  overwriteByNewData = ADC_CFGR_OVRMOD
};

enum class AdcJsqrMode : uint32_t {
  keepOld = 0,
  discardOld = ADC_CFGR_JQM
};

enum class AdcCommonDualMode : uint32_t {
  independent = 0,
  syncRegularAndInject = 1,
  syncRegularAndAltenateTrig = 2,
  syncInterleaveAndInject = 3,
  reserved = 4,
  syncInject = 5,
  syncRegular = 6,
  syncInterleave = 7,
  alternateTrig = 9,
};

enum class AdcInjectGrpExtTrigMode : uint32_t {
  noTrig = 0,
  risingEdge = ADC_JSQR_JEXTEN_0,
  fallingEdge = ADC_JSQR_JEXTEN_1,
  bothEdge = ADC_JSQR_JEXTEN_0 | ADC_JSQR_JEXTEN_1
};

enum class AdcCommonDmaMode : uint32_t {
  oneshot = 0,
  circular = ADC12_CCR_DMACFG
};

enum class AdcCommonMdmaMode : uint32_t {
  disable = 0,
  reserved = 1,
  enable12or10bit = 2,
  enable8or6bit = 3,
};


enum class AdcCommonClockMode : uint32_t {
  async = 0,
  syncHclkDiv1 = ADC12_CCR_CKMODE_0,
  syncHclkDiv2 = ADC12_CCR_CKMODE_1,
  syncHclkDiv4 = ADC12_CCR_CKMODE_0 | ADC12_CCR_CKMODE_1,
};

struct AdcCommon {
  constexpr static auto addr = ADC1_2_COMMON_BASE;

  RegEnum<IGB_ADC_COMMON_REG_ADDR(CCR), ADC12_CCR_MULTI_Msk, AdcCommonDualMode> dualMode;
  RegValue<IGB_ADC_COMMON_REG_ADDR(CCR), ADC12_CCR_DELAY_Msk, ADC12_CCR_DELAY_Pos> samplingPhaseDelay;
  RegEnum<IGB_ADC_COMMON_REG_ADDR(CCR), ADC12_CCR_DMACFG_Msk, AdcCommonDmaMode> dmaMode;
  RegEnum<IGB_ADC_COMMON_REG_ADDR(CCR), ADC12_CCR_MDMA_Msk, AdcCommonMdmaMode> mdmaMode;
  RegEnum<IGB_ADC_COMMON_REG_ADDR(CCR), ADC12_CCR_CKMODE_Msk, AdcCommonClockMode> clockMode;
  RegFlag<IGB_ADC_COMMON_REG_ADDR(CCR), ADC12_CCR_VREFEN> vrefint;
  RegFlag<IGB_ADC_COMMON_REG_ADDR(CCR), ADC12_CCR_TSEN> temperatureSensor;
  RegFlag<IGB_ADC_COMMON_REG_ADDR(CCR), ADC12_CCR_VBATEN> vbat;

  RegValue<IGB_ADC_COMMON_REG_ADDR(CDR), ADC12_CDR_RDATA_MST_Msk, ADC12_CDR_RDATA_MST_Pos> masterData;
  RegValue<IGB_ADC_COMMON_REG_ADDR(CDR), ADC12_CDR_RDATA_SLV_Msk, ADC12_CDR_RDATA_SLV_Pos> slaveData;

  IGB_FAST_INLINE bool is(AdcCommonStatus status) {
    volatile bool result = IGB_ADC_COMMON->CSR & static_cast<uint32_t>(status);
    return result;
  }
};

enum class AdcChannel : uint32_t {
  ch1 = 1,
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
};

struct AdcPinConf {
  AdcChannel ch;
  GpioPinType pin_type;
};


template<AdcType ADC_TYPE>
struct Adc {
  constexpr static auto type = ADC_TYPE;
  constexpr static auto addr = STM32_PERIPH_INFO.adc[static_cast<size_t>(type)].addr;

  RegValue<IGB_ADC_REG_ADDR(CR), ADC_CR_BITS_PROPERTY_RS, 0> propertyRsClearBits;

  RegFlag<IGB_ADC_REG_ADDR(CR), ADC_CR_ADSTART> convStart;
  RegFlag<IGB_ADC_REG_ADDR(CR), ADC_CR_JADSTART> injectedConvStart;
  RegFlag<IGB_ADC_REG_ADDR(CR), ADC_CR_ADSTP> convStop;
  RegFlag<IGB_ADC_REG_ADDR(CR), ADC_CR_JADSTP> injectedConvStop;
  RegEnum<IGB_ADC_REG_ADDR(CR), ADC_CR_ADVREGEN, AdcRegulatorState> regulator;
  RegFlag<IGB_ADC_REG_ADDR(CR), ADC_CR_ADCALDIF> differentialCalibration;
  RegFlag<IGB_ADC_REG_ADDR(CR), ADC_CR_ADCAL> calibration;

  RegFlag<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_DMAEN> dma;
  RegEnum<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_DMACFG_Msk, AdcDmaConfig> dmaConfig;
  RegEnum<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_RES_Msk, AdcResolution> resolution;
  RegEnum<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_ALIGN_Msk, AdcDataAlign> dataAlign;
  RegEnum<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_EXTSEL_Msk, AdcExternalTriggerEvent> externalTrigSelect;
  RegEnum<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_EXTEN_Msk, AdcExternalTriggerPolarity> externalTrigPolarity;
  RegEnum<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_OVRMOD_Msk, AdcOverrunMode> overrunMode;
  // ADC_CFGR_CONT and ADC_CFGR_DISCEN are must not on at same time!
  RegFlag<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_CONT> continuousConvMode;
  RegFlag<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_DISCEN> discontinuousConvMode;
  RegFlag<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_AUTDLY> autoDelayMode;
  RegValue<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_DISCNUM_Msk, ADC_CFGR_DISCNUM_Pos> discontinuousConvChannelCount; // 0 == 1 channel, 7 = 8 channnel
  RegFlag<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_JDISCEN> injectedDiscontinuousMode;
  RegEnum<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_JQM_Msk, AdcJsqrMode> jsqrMode;
  RegFlag<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_AWD1SGL, false> watchdog1ForAllCh;
  RegFlag<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_AWD1EN> regularWatchdog1;
  RegFlag<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_JAWD1EN> injectWatchdog1;
  RegFlag<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_JAUTO> autoInjectGroupConv;
  RegValue<IGB_ADC_REG_ADDR(CFGR), ADC_CFGR_AWD1CH_Msk, ADC_CFGR_AWD1CH_Pos> watchdog1Channel; // 0 == reserved, 1 = channel1, 18 = channel18
  
  RegValueRO<IGB_ADC_REG_ADDR(DR), ADC_DR_RDATA_Msk, ADC_DR_RDATA_Pos> data; // read-only

  // sampling time value:
  // 0: 1.5 ADC Clock
  // 1: 2.5 ADC Clock
  // 2: 4.5 ADC Clock
  // 3: 7.5 ADC Clock
  // 4: 19.5 ADC Clock
  // 5: 61.5 ADC Clock
  // 6: 181.5 ADC Clock
  // 7: 601.5 ADC Clock
  RegValue<IGB_ADC_REG_ADDR(SMPR1), ADC_SMPR1_SMP1_Msk, ADC_SMPR1_SMP1_Pos> ch1SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR1), ADC_SMPR1_SMP2_Msk, ADC_SMPR1_SMP2_Pos> ch2SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR1), ADC_SMPR1_SMP3_Msk, ADC_SMPR1_SMP3_Pos> ch3SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR1), ADC_SMPR1_SMP4_Msk, ADC_SMPR1_SMP4_Pos> ch4SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR1), ADC_SMPR1_SMP5_Msk, ADC_SMPR1_SMP5_Pos> ch5SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR1), ADC_SMPR1_SMP6_Msk, ADC_SMPR1_SMP6_Pos> ch6SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR1), ADC_SMPR1_SMP7_Msk, ADC_SMPR1_SMP7_Pos> ch7SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR1), ADC_SMPR1_SMP8_Msk, ADC_SMPR1_SMP8_Pos> ch8SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR1), ADC_SMPR1_SMP9_Msk, ADC_SMPR1_SMP9_Pos> ch9SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR2), ADC_SMPR2_SMP10_Msk, ADC_SMPR2_SMP10_Pos> ch10SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR2), ADC_SMPR2_SMP11_Msk, ADC_SMPR2_SMP11_Pos> ch11SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR2), ADC_SMPR2_SMP12_Msk, ADC_SMPR2_SMP12_Pos> ch12SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR2), ADC_SMPR2_SMP13_Msk, ADC_SMPR2_SMP13_Pos> ch13SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR2), ADC_SMPR2_SMP14_Msk, ADC_SMPR2_SMP14_Pos> ch14SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR2), ADC_SMPR2_SMP15_Msk, ADC_SMPR2_SMP15_Pos> ch15SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR2), ADC_SMPR2_SMP16_Msk, ADC_SMPR2_SMP16_Pos> ch16SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR2), ADC_SMPR2_SMP17_Msk, ADC_SMPR2_SMP17_Pos> ch17SamplingTime; // 0 ~ 7
  RegValue<IGB_ADC_REG_ADDR(SMPR2), ADC_SMPR2_SMP18_Msk, ADC_SMPR2_SMP18_Pos> ch18SamplingTime; // 0 ~ 7
  
  RegValue<IGB_ADC_REG_ADDR(TR1), ADC_TR1_LT1_Msk, ADC_TR1_LT1_Pos> watchdog1LowThreshold;
  RegValue<IGB_ADC_REG_ADDR(TR1), ADC_TR1_HT1_Msk, ADC_TR1_HT1_Pos> watchdog1HighThreshold;
  RegValue<IGB_ADC_REG_ADDR(TR2), ADC_TR2_LT2_Msk, ADC_TR2_LT2_Pos> watchdog2LowThreshold;
  RegValue<IGB_ADC_REG_ADDR(TR2), ADC_TR2_HT2_Msk, ADC_TR2_HT2_Pos> watchdog2HighThreshold;
  RegValue<IGB_ADC_REG_ADDR(TR3), ADC_TR3_LT3_Msk, ADC_TR3_LT3_Pos> watchdog3LowThreshold;
  RegValue<IGB_ADC_REG_ADDR(TR3), ADC_TR3_HT3_Msk, ADC_TR3_HT3_Pos> watchdog3HighThreshold;

  RegValue<IGB_ADC_REG_ADDR(SQR1), ADC_SQR1_L_Msk, ADC_SQR1_L_Pos> seqChLength;
  RegValue<IGB_ADC_REG_ADDR(SQR1), ADC_SQR1_SQ1_Msk, ADC_SQR1_SQ1_Pos> seqOrder1Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR1), ADC_SQR1_SQ2_Msk, ADC_SQR1_SQ2_Pos> seqOrder2Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR1), ADC_SQR1_SQ3_Msk, ADC_SQR1_SQ3_Pos> seqOrder3Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR1), ADC_SQR1_SQ4_Msk, ADC_SQR1_SQ4_Pos> seqOrder4Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR2), ADC_SQR2_SQ5_Msk, ADC_SQR2_SQ5_Pos> seqOrder5Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR2), ADC_SQR2_SQ6_Msk, ADC_SQR2_SQ6_Pos> seqOrder6Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR2), ADC_SQR2_SQ7_Msk, ADC_SQR2_SQ7_Pos> seqOrder7Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR2), ADC_SQR2_SQ8_Msk, ADC_SQR2_SQ8_Pos> seqOrder8Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR2), ADC_SQR2_SQ9_Msk, ADC_SQR2_SQ9_Pos> seqOrder9Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR3), ADC_SQR3_SQ10_Msk, ADC_SQR3_SQ10_Pos> seqOrder10Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR3), ADC_SQR3_SQ11_Msk, ADC_SQR3_SQ11_Pos> seqOrder11Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR3), ADC_SQR3_SQ12_Msk, ADC_SQR3_SQ12_Pos> seqOrder12Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR3), ADC_SQR3_SQ13_Msk, ADC_SQR3_SQ13_Pos> seqOrder13Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR3), ADC_SQR3_SQ14_Msk, ADC_SQR3_SQ14_Pos> seqOrder14Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR4), ADC_SQR4_SQ15_Msk, ADC_SQR4_SQ15_Pos> seqOrder15Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(SQR4), ADC_SQR4_SQ16_Msk, ADC_SQR4_SQ16_Pos> seqOrder16Ch; // 0 = reserved, 1 = channel1 ..., 18 = channel18

  RegValue<IGB_ADC_REG_ADDR(JSQR), ADC_JSQR_JL_Msk, ADC_JSQR_JL_Pos> injectSeqChLength; // 0 =  1 conversion, 3 = 4 conversion
  RegValue<IGB_ADC_REG_ADDR(JSQR), ADC_JSQR_JEXTSEL_Msk, ADC_JSQR_JEXTSEL_Pos> injectGrpExtTrig; // 0 = event0, 15 = event 15
  RegEnum<IGB_ADC_REG_ADDR(JSQR), ADC_JSQR_JEXTEN_Msk, AdcInjectGrpExtTrigMode> injectGrpExtTrigMode;
  RegValue<IGB_ADC_REG_ADDR(JSQR), ADC_JSQR_JSQ1_Msk, ADC_JSQR_JSQ1_Pos> injectSeqOrder1Ch; // 0 = reserved, 1 = channel1, 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(JSQR), ADC_JSQR_JSQ2_Msk, ADC_JSQR_JSQ2_Pos> injectSeqOrder2Ch; // 0 = reserved, 1 = channel1, 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(JSQR), ADC_JSQR_JSQ3_Msk, ADC_JSQR_JSQ3_Pos> injectSeqOrder3Ch; // 0 = reserved, 1 = channel1, 18 = channel18
  RegValue<IGB_ADC_REG_ADDR(JSQR), ADC_JSQR_JSQ4_Msk, ADC_JSQR_JSQ4_Pos> injectSeqOrder4Ch; // 0 = reserved, 1 = channel1, 18 = channel18

  RegValue<IGB_ADC_REG_ADDR(OFR1), ADC_OFR1_OFFSET1_Msk, ADC_OFR1_OFFSET1_Pos> dataOffset1Value;
  RegValue<IGB_ADC_REG_ADDR(OFR1), ADC_OFR1_OFFSET1_CH_Msk, ADC_OFR1_OFFSET1_CH_Pos> dataOffset1Ch;
  RegFlag<IGB_ADC_REG_ADDR(OFR1), ADC_OFR1_OFFSET1_EN> dataOffset1;

  RegValue<IGB_ADC_REG_ADDR(OFR2), ADC_OFR2_OFFSET2_Msk, ADC_OFR2_OFFSET2_Pos> dataOffset2Value;
  RegValue<IGB_ADC_REG_ADDR(OFR2), ADC_OFR2_OFFSET2_CH_Msk, ADC_OFR2_OFFSET2_CH_Pos> dataOffset2Ch;
  RegFlag<IGB_ADC_REG_ADDR(OFR2), ADC_OFR2_OFFSET2_EN> dataOffset2;

  RegValue<IGB_ADC_REG_ADDR(OFR3), ADC_OFR3_OFFSET3_Msk, ADC_OFR3_OFFSET3_Pos> dataOffset3Value;
  RegValue<IGB_ADC_REG_ADDR(OFR3), ADC_OFR3_OFFSET3_CH_Msk, ADC_OFR3_OFFSET3_CH_Pos> dataOffset3Ch;
  RegFlag<IGB_ADC_REG_ADDR(OFR3), ADC_OFR3_OFFSET3_EN> dataOffset3;

  RegValue<IGB_ADC_REG_ADDR(OFR4), ADC_OFR4_OFFSET4_Msk, ADC_OFR4_OFFSET4_Pos> dataOffset4Value;
  RegValue<IGB_ADC_REG_ADDR(OFR4), ADC_OFR4_OFFSET4_CH_Msk, ADC_OFR4_OFFSET4_CH_Pos> dataOffset4Ch;
  RegFlag<IGB_ADC_REG_ADDR(OFR4), ADC_OFR4_OFFSET4_EN> dataOffset4;

  RegValue<IGB_ADC_REG_ADDR(JDR1), ADC_JDR1_JDATA_Msk, ADC_JDR1_JDATA_Pos> injectData1;
  RegValue<IGB_ADC_REG_ADDR(JDR2), ADC_JDR2_JDATA_Msk, ADC_JDR2_JDATA_Pos> injectData2;
  RegValue<IGB_ADC_REG_ADDR(JDR3), ADC_JDR3_JDATA_Msk, ADC_JDR3_JDATA_Pos> injectData3;
  RegValue<IGB_ADC_REG_ADDR(JDR4), ADC_JDR4_JDATA_Msk, ADC_JDR4_JDATA_Pos> injectData4;

  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_0> watchdog2Ch1Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_1> watchdog2Ch2Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_2> watchdog2Ch3Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_3> watchdog2Ch4Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_4> watchdog2Ch5Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_5> watchdog2Ch6Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_6> watchdog2Ch7Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_7> watchdog2Ch8Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_8> watchdog2Ch9Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_9> watchdog2Ch10Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_10> watchdog2Ch11Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_11> watchdog2Ch12Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_12> watchdog2Ch13Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_13> watchdog2Ch14Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_14> watchdog2Ch15Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_15> watchdog2Ch16Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_16> watchdog2Ch17Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD2CR), ADC_AWD2CR_AWD2CH_17> watchdog2Ch18Enable;

  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_0> watchdog3Ch1Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_1> watchdog3Ch2Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_2> watchdog3Ch3Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_3> watchdog3Ch4Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_4> watchdog3Ch5Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_5> watchdog3Ch6Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_6> watchdog3Ch7Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_7> watchdog3Ch8Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_8> watchdog3Ch9Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_9> watchdog3Ch10Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_10> watchdog3Ch11Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_11> watchdog3Ch12Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_12> watchdog3Ch13Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_13> watchdog3Ch14Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_14> watchdog3Ch15Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_15> watchdog3Ch16Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_16> watchdog3Ch17Enable;
  RegFlag<IGB_ADC_REG_ADDR(AWD3CR), ADC_AWD3CR_AWD3CH_17> watchdog3Ch18Enable;

  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_0> diffModeCh1Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_1> diffModeCh2Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_2> diffModeCh3Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_3> diffModeCh4Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_4> diffModeCh5Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_5> diffModeCh6Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_6> diffModeCh7Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_7> diffModeCh8Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_8> diffModeCh9Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_9> diffModeCh10Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_10> diffModeCh11Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_11> diffModeCh12Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_12> diffModeCh13Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_13> diffModeCh14Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_14> diffModeCh15Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_15> diffModeCh16Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_16> diffModeCh17Enable;
  RegFlag<IGB_ADC_REG_ADDR(DIFSEL), ADC_DIFSEL_DIFSEL_17> diffModeCh18Enable;

  RegValue<IGB_ADC_REG_ADDR(CALFACT), ADC_CALFACT_CALFACT_S_Msk, ADC_CALFACT_CALFACT_S_Pos> singleendCalibrationFactor;
  RegValue<IGB_ADC_REG_ADDR(CALFACT), ADC_CALFACT_CALFACT_D_Msk, ADC_CALFACT_CALFACT_D_Pos> differentialCalibrationFactor;


  IGB_FAST_INLINE void enable() {
    MODIFY_REG(IGB_ADC->CR, ADC_CR_BITS_PROPERTY_RS, ADC_CR_ADEN);
  }

  IGB_FAST_INLINE void disable() {
    MODIFY_REG(IGB_ADC->CR, ADC_CR_BITS_PROPERTY_RS, ADC_CR_ADDIS);
  }

  IGB_FAST_INLINE bool is(AdcStatus status) {
    volatile bool result = IGB_ADC->ISR & static_cast<uint32_t>(status);
    return result;
  }

  IGB_FAST_INLINE void clear(AdcStatus status) {
    IGB_ADC->ISR = static_cast<uint32_t>(status);
  }

  IGB_FAST_INLINE void enable(AdcInterruptType interrupt) {
    IGB_ADC->IER |= static_cast<uint32_t>(interrupt);
  }

  IGB_FAST_INLINE void disable(AdcInterruptType interrupt) {
    IGB_ADC->IER &= ~(static_cast<uint32_t>(interrupt));
  }

  IGB_FAST_INLINE void enableRegulator() {
    regulator(AdcRegulatorState::clear);
    CLEAR_BIT(IGB_ADC->CR, ADC_CR_BITS_PROPERTY_RS);
    regulator(AdcRegulatorState::enable);
  }

  IGB_FAST_INLINE void disableRegulator() {
    CLEAR_BIT(IGB_ADC->CR, (ADC_CR_ADVREGEN | ADC_CR_BITS_PROPERTY_RS));
  }

  IGB_FAST_INLINE void startCalibration(AdcCalibrationType type) {
    MODIFY_REG(IGB_ADC->CR,
               ADC_CR_ADCALDIF | ADC_CR_BITS_PROPERTY_RS,
               ADC_CR_ADCAL | (static_cast<uint32_t>(type) & ADC_CR_ADCALDIF));
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
    STM32_PERIPH_INFO.adc[static_cast<size_t>(type)].bus.enableBusClock();
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

  IGB_FAST_INLINE void initRegularContinuous(auto&&... confs) {
    enableBusClock();
    prepareGpios(confs...);
    (
     resolution.val(AdcResolution::_12bit) |
     dataAlign.val(AdcDataAlign::right) | 
     autoDelayMode.val(false)
    ).update();
    // TODO:
  }

  IGB_FAST_INLINE void initRegularSingle(AdcPinConf conf) {
    enableBusClock();
    prepareGpio(conf.pin_type);
    
    (
     resolution.val(AdcResolution::_12bit) |
     dataAlign.val(AdcDataAlign::right) | 
     autoDelayMode.val(false)
    ).update();

    (
     externalTrigSelect.val(AdcExternalTriggerEvent::none) |
     externalTrigPolarity.val(AdcExternalTriggerPolarity::disable) |
     discontinuousConvMode.val(false) |
     discontinuousConvChannelCount.val(0) |
     continuousConvMode.val(false) |
     dma.val(false) |
     dmaConfig.val(AdcDmaConfig::oneshot) |
     overrunMode.val(AdcOverrunMode::overwriteByNewData)
    ).update();

    AdcCommon adc_common;
    adc_common.clockMode(AdcCommonClockMode::async);
    enableRegulator();
    delay_msec(2);

    (seqChLength.val(0) | seqOrder1Ch.val(static_cast<uint32_t>(conf.ch))).update();
    const uint32_t adc_clock = 4; // 19.5 ADC Clock
    switch (conf.ch) {
      case AdcChannel::ch1:
        ch1SamplingTime(adc_clock);
        break;
      case AdcChannel::ch2:
        ch2SamplingTime(adc_clock);
        break;
      case AdcChannel::ch3:
        ch3SamplingTime(adc_clock);
        break;
      case AdcChannel::ch4:
        ch4SamplingTime(adc_clock);
        break;
      case AdcChannel::ch5:
        ch5SamplingTime(adc_clock);
        break;
      case AdcChannel::ch6:
        ch6SamplingTime(adc_clock);
        break;
      case AdcChannel::ch7:
        ch7SamplingTime(adc_clock);
        break;
      case AdcChannel::ch8:
        ch8SamplingTime(adc_clock);
        break;
      case AdcChannel::ch9:
        ch9SamplingTime(adc_clock);
        break;
      case AdcChannel::ch10:
        ch10SamplingTime(adc_clock);
        break;
      case AdcChannel::ch11:
        ch11SamplingTime(adc_clock);
        break;
      case AdcChannel::ch12:
        ch12SamplingTime(adc_clock);
        break;
      case AdcChannel::ch13:
        ch13SamplingTime(adc_clock);
        break;
      case AdcChannel::ch14:
        ch14SamplingTime(adc_clock);
        break;
      case AdcChannel::ch15:
        ch15SamplingTime(adc_clock);
        break;
      case AdcChannel::ch16:
        ch16SamplingTime(adc_clock);
        break;
      case AdcChannel::ch17:
        ch17SamplingTime(adc_clock);
        break;
      case AdcChannel::ch18:
        ch18SamplingTime(adc_clock);
        break;
      default:
        break;
    }

    switch (conf.ch) {
      case AdcChannel::ch1:
        diffModeCh1Enable(false);
        break;
      case AdcChannel::ch2:
        diffModeCh2Enable(false);
        break;
      case AdcChannel::ch3:
        diffModeCh3Enable(false);
        break;
      case AdcChannel::ch4:
        diffModeCh4Enable(false);
        break;
      case AdcChannel::ch5:
        diffModeCh5Enable(false);
        break;
      case AdcChannel::ch6:
        diffModeCh6Enable(false);
        break;
      case AdcChannel::ch7:
        diffModeCh7Enable(false);
        break;
      case AdcChannel::ch8:
        diffModeCh8Enable(false);
        break;
      case AdcChannel::ch9:
        diffModeCh9Enable(false);
        break;
      case AdcChannel::ch10:
        diffModeCh10Enable(false);
        break;
      case AdcChannel::ch11:
        diffModeCh11Enable(false);
        break;
      case AdcChannel::ch12:
        diffModeCh12Enable(false);
        break;
      case AdcChannel::ch13:
        diffModeCh13Enable(false);
        break;
      case AdcChannel::ch14:
        diffModeCh14Enable(false);
        break;
      case AdcChannel::ch15:
        diffModeCh15Enable(false);
        break;
      case AdcChannel::ch16:
        diffModeCh16Enable(false);
        break;
      case AdcChannel::ch17:
        diffModeCh17Enable(false);
        break;
      case AdcChannel::ch18:
        diffModeCh18Enable(false);
        break;
      default:
        break;
    }

    startCalibration(AdcCalibrationType::singleEnded);

    delay_msec(2);

    enable();

    while(!is(AdcStatus::ready)) {}
  }
};

#endif

#undef IGB_ADC_REG
#undef IGB_ADC_REG_ADDR
#undef IGB_ADC

#undef IGB_ADC_COMMON_REG
#undef IGB_ADC_COMMON_REG_ADDR
#undef IGB_ADC_COMMON

}
}

#endif /* IGB_STM32_PERIPH_ADC_H */
