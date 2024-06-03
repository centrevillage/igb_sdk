#pragma once

#include <tuple>

#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/rcc.hpp>
#include <igb_stm32/periph/nvic.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/reg.hpp>
#include <igb_util/macro.hpp>

namespace igb {
namespace stm32 {

#define IGB_TIM ((TIM_TypeDef*)addr)
#define IGB_TIM_REG_ADDR(member) (addr + offsetof(TIM_TypeDef, member))
#define IGB_TIM_REG(member) ((TIM_TypeDef*)IGB_TIM_REG_ADDR(member))

enum class TimState : uint32_t {
  update  = TIM_SR_UIF,
  cc1     = TIM_SR_CC1IF,
  cc2     = TIM_SR_CC2IF,
  cc3     = TIM_SR_CC3IF,
  cc4     = TIM_SR_CC4IF,
  com     = TIM_SR_COMIF,
  trigger = TIM_SR_TIF,
  _break  = TIM_SR_BIF,
  cc1over = TIM_SR_CC1OF,
  cc2over = TIM_SR_CC2OF,
  cc3over = TIM_SR_CC3OF,
  cc4over = TIM_SR_CC4OF,
};

enum class TimCcCh : uint32_t {
  cc1 = 0,
  cc2,
  cc3,
  cc4,
};

enum class TimChMode : uint32_t {
  output = 0,
  inputTi1,
  inputTi2,
  inputTrc
};

enum class TimOutputCompareMode : uint32_t {
  timing = 0,
  setChannelOutput = 1,
  clearChannelOutput = 2,
  toggleOnMatch = 3,
  forceLow = 4,
  forceHigh = 5,
  pwmMode1 = 6,
  pwmMOde2 = 7
};

enum class TimChInputPrescaler : uint32_t {
  disable = 0,
  two = 1,
  four = 2,
  eight = 3,
};

enum class TimChPolarity : uint32_t {
  high = 0,
  low = 1
};

enum class TimChOutputIdleState : uint32_t {
  low = 0,
  high = 1
};

enum class TimCounterMode : uint32_t {
  up           = 0,
  down         = TIM_CR1_DIR,
  centerUp     = TIM_CR1_CMS_0,
  centerDown   = TIM_CR1_CMS_1,
  centerUpDown = TIM_CR1_CMS,
};

// 列挙値をbitでなく、indexで持った方が共通化しやすいか？
enum class TimInterruptType : uint32_t {
  update  = TIM_DIER_UIE,
  cc1     = TIM_DIER_CC1IE,
  cc2     = TIM_DIER_CC2IE,
  cc3     = TIM_DIER_CC3IE,
  cc4     = TIM_DIER_CC4IE,
  com     = TIM_DIER_COMIE,
  trigger = TIM_DIER_TIE,
  _break  = TIM_DIER_BIE,
};

enum class TimEventGen : uint32_t {
  update  = TIM_EGR_UG,
  cc1     = TIM_EGR_CC1G,
  cc2     = TIM_EGR_CC2G,
  cc3     = TIM_EGR_CC3G,
  cc4     = TIM_EGR_CC4G,
  com     = TIM_EGR_COMG,
  trigger = TIM_EGR_TG,
  _break  = TIM_EGR_BG,
};

enum class TimClockDiv : uint32_t {
  div1 = 0,
  div2 = TIM_CR1_CKD_0,
  div4 = TIM_CR1_CKD_1,
};

enum class TimCounterDir : uint32_t {
  up   = 0,
  down = TIM_CR1_DIR,
};

enum class TimClockSrc : uint32_t {
  internal  = 0,
  extMode1 = (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0),
  extMode2 = TIM_SMCR_ECE,
};

enum class TimTriggerOut : uint32_t {
  reset  = 0,
  enable = TIM_CR2_MMS_0,
  update = TIM_CR2_MMS_1,
  cc1if  = (TIM_CR2_MMS_1 | TIM_CR2_MMS_0),
  oc1ref = TIM_CR2_MMS_2,
  oc2ref = (TIM_CR2_MMS_2 | TIM_CR2_MMS_0),
  oc3ref = (TIM_CR2_MMS_2 | TIM_CR2_MMS_1),
  oc4ref = (TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0),
};

enum class TimEtrConfPolarity : uint32_t {
  noninverted = 0,
  inverted = TIM_SMCR_ETP,
};

enum class TimEtrConfPrescaler : uint32_t {
  div1 = 0,
  div2 = TIM_SMCR_ETPS_0,
  div4 = TIM_SMCR_ETPS_1,
  div8 = TIM_SMCR_ETPS,
};

enum class TimEtrConfFilter : uint32_t {
  fdiv1     = 0,
  fdiv1N2  = TIM_SMCR_ETF_0,
  fdiv1N4  = TIM_SMCR_ETF_1,
  fdiv1N8  = (TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0),
  fdiv2N6  = TIM_SMCR_ETF_2,
  fdiv2N8  = (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_0),
  fdiv4N6  = (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1),
  fdiv4N8  = (TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0),
  fdiv8N6  = TIM_SMCR_ETF_3,
  fdiv8N8  = (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_0),
  fdiv16N5 = (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_1),
  fdiv16N6 = (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0),
  fdiv16N8 = (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2),
  fdiv32N5 = (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2 | TIM_SMCR_ETF_0),
  fdiv32N6 = (TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1),
  fdiv32N8 = TIM_SMCR_ETF,
};

enum class TimBreakPolarity : uint32_t {
  low = 0,
  high = TIM_BDTR_BKP,
};

enum class TimDmaBurstBaseAddr : uint32_t {
  cr1   = 0,
  cr2   = TIM_DCR_DBA_0,
  smcr  = TIM_DCR_DBA_1,
  dier  = (TIM_DCR_DBA_1 |  TIM_DCR_DBA_0),
  sr    = TIM_DCR_DBA_2,
  egr   = (TIM_DCR_DBA_2 | TIM_DCR_DBA_0),
  ccmr1 = (TIM_DCR_DBA_2 | TIM_DCR_DBA_1),
  ccmr2 = (TIM_DCR_DBA_2 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0),
  ccer  = TIM_DCR_DBA_3,
  cnt   = (TIM_DCR_DBA_3 | TIM_DCR_DBA_0),
  psc   = (TIM_DCR_DBA_3 | TIM_DCR_DBA_1),
  arr   = (TIM_DCR_DBA_3 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0),
  rcr   = (TIM_DCR_DBA_3 | TIM_DCR_DBA_2),
  ccr1  = (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 | TIM_DCR_DBA_0),
  ccr2  = (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 | TIM_DCR_DBA_1),
  ccr3  = (TIM_DCR_DBA_3 | TIM_DCR_DBA_2 | TIM_DCR_DBA_1 | TIM_DCR_DBA_0),
  ccr4  = TIM_DCR_DBA_4,
  bdtr  = (TIM_DCR_DBA_4 | TIM_DCR_DBA_0),
};

enum class TimDmaBurstLen : uint32_t {
  l1 = 0,
  l2  = TIM_DCR_DBL_0,
  l3  = TIM_DCR_DBL_1,
  l4  = (TIM_DCR_DBL_1 |  TIM_DCR_DBL_0),
  l5  = TIM_DCR_DBL_2,
  l6  = (TIM_DCR_DBL_2 | TIM_DCR_DBL_0),
  l7  = (TIM_DCR_DBL_2 | TIM_DCR_DBL_1),
  l8  = (TIM_DCR_DBL_2 | TIM_DCR_DBL_1 | TIM_DCR_DBL_0),
  l9  = TIM_DCR_DBL_3,
  l10 = (TIM_DCR_DBL_3 | TIM_DCR_DBL_0),
  l11 = (TIM_DCR_DBL_3 | TIM_DCR_DBL_1),
  l12 = (TIM_DCR_DBL_3 | TIM_DCR_DBL_1 | TIM_DCR_DBL_0),
  l13 = (TIM_DCR_DBL_3 | TIM_DCR_DBL_2),
  l14 = (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 | TIM_DCR_DBL_0),
  l15 = (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 | TIM_DCR_DBL_1),
  l16 = (TIM_DCR_DBL_3 | TIM_DCR_DBL_2 | TIM_DCR_DBL_1 | TIM_DCR_DBL_0),
  l17 = TIM_DCR_DBL_4,
  l18 = (TIM_DCR_DBL_4 |  TIM_DCR_DBL_0),
};

struct TimConf {
  uint32_t prescale = 0;
  uint32_t period = 0;
  TimCounterMode counter_mode = TimCounterMode::up;
  TimClockDiv clock_div = TimClockDiv::div1;
  uint8_t repetition_counter = 0;
  bool arr_preload = false;
  TimClockSrc clock_src = TimClockSrc::internal;
  TimTriggerOut trigger_out = TimTriggerOut::reset;
  bool enable_master_slave = false;

  bool enable_it_update= false;
  bool enable_it_cc1 = false;
  bool enable_it_cc2 = false;
  bool enable_it_cc3 = false;
  bool enable_it_cc4 = false;
  bool enable_it_com = false;
  bool enable_it_trigger = false;
  bool enable_it_break = false;
  uint16_t interrupt_priority = 1;
  bool enable_it = false;
};

template<TimType tim_type>
struct Tim {
  constexpr static auto type = tim_type;
  constexpr static auto info = STM32_PERIPH_INFO.tim[to_idx(type)];
  constexpr static auto addr = STM32_PERIPH_INFO.tim[to_idx(type)].addr;
  constexpr static auto addr_CR1 = IGB_TIM_REG_ADDR(CR1);
  constexpr static auto addr_CR2 = IGB_TIM_REG_ADDR(CR2);
  constexpr static auto addr_SMCR = IGB_TIM_REG_ADDR(SMCR);
  constexpr static auto addr_DIER = IGB_TIM_REG_ADDR(DIER);
  constexpr static auto addr_SR = IGB_TIM_REG_ADDR(SR);
  constexpr static auto addr_EGR = IGB_TIM_REG_ADDR(EGR);
  constexpr static auto addr_CCMR1 = IGB_TIM_REG_ADDR(CCMR1);
  constexpr static auto addr_CCMR2 = IGB_TIM_REG_ADDR(CCMR2);
  constexpr static auto addr_CCER = IGB_TIM_REG_ADDR(CCER);
  constexpr static auto addr_CNT = IGB_TIM_REG_ADDR(CNT);
  constexpr static auto addr_PSC = IGB_TIM_REG_ADDR(PSC);
  constexpr static auto addr_ARR = IGB_TIM_REG_ADDR(ARR);
  constexpr static auto addr_RCR = IGB_TIM_REG_ADDR(RCR);
  constexpr static auto addr_CCR1 = IGB_TIM_REG_ADDR(CCR1);
  constexpr static auto addr_CCR2 = IGB_TIM_REG_ADDR(CCR2);
  constexpr static auto addr_CCR3 = IGB_TIM_REG_ADDR(CCR3);
  constexpr static auto addr_CCR4 = IGB_TIM_REG_ADDR(CCR4);
  constexpr static auto addr_BDTR = IGB_TIM_REG_ADDR(BDTR);
  constexpr static auto addr_DCR  = IGB_TIM_REG_ADDR(DCR);
  constexpr static auto addr_DMAR = IGB_TIM_REG_ADDR(DMAR);
  //constexpr static auto addr_OR   = IGB_TIM_REG_ADDR(OR);

  Reg<addr_PSC> prescaler; 
  RegEnum<addr_CR1, (TIM_CR1_DIR | TIM_CR1_CMS), TimCounterMode> counterMode;
  RegEnum<addr_CR1, TIM_CR1_CKD, TimClockDiv> clockDiv;
  RegFlag<addr_CR1, TIM_CR1_ARPE> enableArrPreload;

  Reg<addr_CCR1> cc1Value;
  Reg<addr_CCR2> cc2Value;
  Reg<addr_CCR3> cc3Value;
  Reg<addr_CCR4> cc4Value;

  // optional tuple interface
  std::tuple<
    Reg<addr_CCR1>,
    Reg<addr_CCR2>,
    Reg<addr_CCR3>,
    Reg<addr_CCR4>
  > ccValues;
  // for dynamic access
  constexpr uint32_t getCcValue(TimCcCh ch) {
    switch (ch) {
      case TimCcCh::cc1:
        return cc1Value();
        break;
      case TimCcCh::cc2:
        return cc2Value();
        break;
      case TimCcCh::cc3:
        return cc3Value();
        break;
      case TimCcCh::cc4:
        return cc4Value();
        break;
      defalut:
        break;
    }
    return 0; // never reach
  };
  constexpr void setCcValue(TimCcCh ch, uint32_t v) { // in mostly cases, v is not constant  so this method isn't evaluated on compile-time
    switch (ch) {
      case TimCcCh::cc1:
        cc1Value(v);
        break;
      case TimCcCh::cc2:
        cc2Value(v);
        break;
      case TimCcCh::cc3:
        cc3Value(v);
        break;
      case TimCcCh::cc4:
        cc4Value(v);
        break;
      defalut:
        break;
    }
  };

  Reg<addr_ARR> autoReload;
  Reg<addr_RCR> repetitionCounter;
  RegEnum<addr_SMCR, TIM_SMCR_SMS | TIM_SMCR_ECE, TimClockSrc> clockSrc;
  RegEnum<addr_CR2, TIM_CR2_MMS, TimTriggerOut> triggerOutput;
  RegFlag<addr_CR2, IGB_BIT(7)> enableTi1Xor;
  RegEnum<addr_CR2, IGB_BIT(8), TimChOutputIdleState, 8> ch1OutputIdleState;
  RegEnum<addr_CR2, IGB_BIT(9), TimChOutputIdleState, 9> ch1ComplementaryOutputIdleState;
  RegEnum<addr_CR2, IGB_BIT(10), TimChOutputIdleState, 10> ch2OutputIdleState;
  RegEnum<addr_CR2, IGB_BIT(11), TimChOutputIdleState, 11> ch2ComplementaryOutputIdleState;
  RegEnum<addr_CR2, IGB_BIT(12), TimChOutputIdleState, 12> ch3OutputIdleState;
  RegEnum<addr_CR2, IGB_BIT(13), TimChOutputIdleState, 13> ch3ComplementaryOutputIdleState;
  RegEnum<addr_CR2, IGB_BIT(14), TimChOutputIdleState, 14> ch4OutputIdleState;
  RegFlag<addr_SMCR, TIM_SMCR_MSM> enableMasterSlave;

  RegEnum<addr_SMCR, TIM_SMCR_ETP, TimEtrConfPolarity> etrConfPolarity;
  RegEnum<addr_SMCR, TIM_SMCR_ETPS, TimEtrConfPrescaler> etrConfPrescaler;
  RegEnum<addr_SMCR, TIM_SMCR_ETF, TimEtrConfFilter> etrConfFilter;

  Reg<addr_SR> reg_SR;

  IGB_FAST_INLINE bool is(TimState state) {
    return !!(reg_SR() & as<uint32_t>(state));
  }

  IGB_FAST_INLINE void clear(TimState state) {
    reg_SR(reg_SR() & ~(as<uint32_t>(state)));
  }

  Reg<addr_CNT> count;

  Reg<addr_EGR> reg_EGR;

  IGB_FAST_INLINE void generateEvent(TimEventGen event) {
    reg_EGR(reg_EGR() | as<uint32_t>(event));
  }

  // TODO
  Reg<addr_CCMR1> reg_CCMR1; 

  RegEnum<addr_CCMR1, IGB_BIT_MASK(2, 0), TimChMode, 0> ch1Mode; 
  RegFlag<addr_CCMR1, IGB_BIT(2)> enableCh1FastOutput; 
  RegFlag<addr_CCMR1, IGB_BIT(3)> enableCh1Preload; 
  RegEnum<addr_CCMR1, IGB_BIT_MASK(3, 4), TimOutputCompareMode, 4> ch1OutputCompareMode; 
  RegFlag<addr_CCMR1, IGB_BIT(7)> enableCh1OutputClear; 
  // input mode
  RegEnum<addr_CCMR1, IGB_BIT_MASK(2, 2), TimChInputPrescaler, 2> ch1InputPrescaler; 
  RegValue<addr_CCMR1, IGB_BIT_MASK(4, 4), 4> ch1InputFilter; 

  RegEnum<addr_CCMR1, IGB_BIT_MASK(2, 8), TimChMode, 8> ch2Mode; 
  // output compare mode
  RegFlag<addr_CCMR1, IGB_BIT(10)> enableCh2FastOutput; 
  RegFlag<addr_CCMR1, IGB_BIT(11)> enableCh2Preload; 
  RegEnum<addr_CCMR1, IGB_BIT_MASK(3, 12), TimOutputCompareMode, 12> ch2OutputCompareMode; 
  RegFlag<addr_CCMR1, IGB_BIT(15)> enableCh2OutputClear; 
  // input mode
  RegEnum<addr_CCMR1, IGB_BIT_MASK(2, 10), TimChInputPrescaler, 10> ch2InputPrescaler; 
  RegValue<addr_CCMR1, IGB_BIT_MASK(4, 12), 12> ch2InputFilter; 

  Reg<addr_CCMR2> reg_CCMR2; 

  RegEnum<addr_CCMR2, IGB_BIT_MASK(2, 0), TimChMode, 0> ch3Mode; 
  // output compare mode
  RegFlag<addr_CCMR2, IGB_BIT(2)> enableCh3FastOutput; 
  RegFlag<addr_CCMR2, IGB_BIT(3)> enableCh3Preload; 
  RegEnum<addr_CCMR2, IGB_BIT_MASK(3, 4), TimOutputCompareMode, 4> ch3OutputCompareMode; 
  RegFlag<addr_CCMR2, IGB_BIT(7)> enableCh3OutputClear; 
  // input mode
  RegEnum<addr_CCMR2, IGB_BIT_MASK(2, 2), TimChInputPrescaler, 2> ch3InputPrescaler; 
  RegValue<addr_CCMR2, IGB_BIT_MASK(4, 4), 4> ch3InputFilter; 

  RegEnum<addr_CCMR2, IGB_BIT_MASK(2, 8), TimChMode, 8> ch4Mode; 
  // output compare mode
  RegFlag<addr_CCMR2, IGB_BIT(10)> enableCh4FastOutput; 
  RegFlag<addr_CCMR2, IGB_BIT(11)> enableCh4Preload; 
  RegEnum<addr_CCMR2, IGB_BIT_MASK(3, 12), TimOutputCompareMode, 12> ch4OutputCompareMode; 
  RegFlag<addr_CCMR2, IGB_BIT(15)> enableCh4OutputClear; 
  // input mode
  RegEnum<addr_CCMR2, IGB_BIT_MASK(2, 10), TimChInputPrescaler, 10> ch4InputPrescaler; 
  RegValue<addr_CCMR2, IGB_BIT_MASK(4, 12), 12> ch4InputFilter; 

  Reg<addr_CCER> reg_CCER; 
  RegFlag<addr_CCER, IGB_BIT(0)> enableCh1;
  RegEnum<addr_CCER, IGB_BIT_MASK(1, 1), TimChPolarity, 1> ch1Polarity;
  RegFlag<addr_CCER, IGB_BIT(2)> enableCh1ComplementaryOutput;
  RegEnum<addr_CCER, IGB_BIT_MASK(1, 3), TimChPolarity, 1> ch1ComplementaryOutputPolarity;

  RegFlag<addr_CCER, IGB_BIT(4)> enableCh2;
  RegEnum<addr_CCER, IGB_BIT_MASK(1, 5), TimChPolarity, 5> ch2Polarity;
  RegFlag<addr_CCER, IGB_BIT(6)> enableCh2ComplementaryOutput;
  RegEnum<addr_CCER, IGB_BIT_MASK(1, 7), TimChPolarity, 7> ch2ComplementaryOutputPolarity;

  RegFlag<addr_CCER, IGB_BIT(8)> enableCh3;
  RegEnum<addr_CCER, IGB_BIT_MASK(1, 9), TimChPolarity, 9> ch3Polarity;
  RegFlag<addr_CCER, IGB_BIT(10)> enableCh3ComplementaryOutput;
  RegEnum<addr_CCER, IGB_BIT_MASK(1, 11), TimChPolarity, 11> ch3ComplementaryOutputPolarity;

  RegFlag<addr_CCER, IGB_BIT(12)> enableCh4;
  RegEnum<addr_CCER, IGB_BIT_MASK(1, 13), TimChPolarity, 13> ch4Polarity;

  Reg<addr_DIER> reg_DIER;

  RegFlag<addr_DIER, TIM_DIER_UIE>   enableItUpdate;
  RegFlag<addr_DIER, TIM_DIER_CC1IE> enableItCc1;
  RegFlag<addr_DIER, TIM_DIER_CC2IE> enableItCc2;
  RegFlag<addr_DIER, TIM_DIER_CC3IE> enableItCc3;
  RegFlag<addr_DIER, TIM_DIER_CC4IE> enableItCc4;
  RegFlag<addr_DIER, TIM_DIER_COMIE> enableItCom;
  RegFlag<addr_DIER, TIM_DIER_TIE>   enableItTrigger;
  RegFlag<addr_DIER, TIM_DIER_BIE>   enableItBreak;;

  void enableIt(TimInterruptType interrupt) {
    reg_DIER(reg_DIER() | as<uint32_t>(interrupt));
  }

  void disableIt(TimInterruptType interrupt) {
    reg_DIER(reg_DIER() & ~(as<uint32_t>(interrupt)));
  }

  RegValue<addr_BDTR, TIM_BDTR_DTG_Msk, TIM_BDTR_DTG_Pos> deadTime;

  RegFlag<addr_BDTR, TIM_BDTR_BKE> enableBreak;
  RegEnum<addr_BDTR, TIM_BDTR_BKP_Msk, TimBreakPolarity> breakPolarity;
  RegFlag<addr_BDTR, TIM_BDTR_OSSI> enableOffStateIdle;
  RegFlag<addr_BDTR, TIM_BDTR_OSSR> enableOffStateRun;
  RegFlag<addr_BDTR, TIM_BDTR_AOE> enableAutoOut;
  RegFlag<addr_BDTR, TIM_BDTR_MOE> enableAllOut;

  RegEnum<addr_DCR, TIM_DCR_DBL_Msk, TimDmaBurstLen> dmaBurstLen;
  RegEnum<addr_DCR, TIM_DCR_DBA_Msk, TimDmaBurstBaseAddr> dmaBurstBaseAddr;

  Reg<addr_DMAR> reg_DMAR;

  RegFlag<addr_CR1, TIM_CR1_CEN> enable;

  void start() {
    count(0);
    enable(true);
  }
  void stop() {
    enable(false);
  }

  void init(auto&& conf) {
    const auto& info = STM32_PERIPH_INFO.tim[to_idx(type)];
    info.bus.enableBusClock();

    auto it_bits = (
      enableItUpdate.val(conf.enable_it_update) |
      enableItCc1.val(conf.enable_it_cc1) |
      enableItCc2.val(conf.enable_it_cc2) |
      enableItCc3.val(conf.enable_it_cc3) |
      enableItCc4.val(conf.enable_it_cc4) |
      enableItCom.val(conf.enable_it_com) |
      enableItTrigger.val(conf.enable_it_trigger) |
      enableItBreak.val(conf.enable_it_break)
    );

    if (it_bits.value() || conf.enable_it) {
      NvicCtrl::setPriority(info.irqn, conf.interrupt_priority);
      NvicCtrl::enable(info.irqn);
      it_bits.update();
    }
    count(0);
    prescaler(conf.prescale);
    autoReload(conf.period);
    counterMode(conf.counter_mode);
    clockDiv(conf.clock_div);
    repetitionCounter(conf.repetition_counter);
    if (conf.arr_preload) {
      enableArrPreload(true);
    } else {
      enableArrPreload(false);
    }
    clockSrc(conf.clock_src);
    triggerOutput(conf.trigger_out);
    enableMasterSlave(conf.enable_master_slave);
  }
};

#undef IGB_TIM_REG
#undef IGB_TIM_REG_ADDR
#undef IGB_TIM

} // namespace stm32
} // namespace igb

