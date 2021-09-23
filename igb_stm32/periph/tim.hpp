#ifndef IGB_STM32_PERIPH_TIM_H
#define IGB_STM32_PERIPH_TIM_H

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

  bool enable_update_interrupt = false;
  uint16_t interrupt_priority = 1;
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
  constexpr static auto addr_OR   = IGB_TIM_REG_ADDR(OR);

  Reg<addr_PSC> prescaler; 
  RegEnum<addr_CR1, (TIM_CR1_DIR | TIM_CR1_CMS), TimCounterMode> counterMode;
  RegEnum<addr_CR1, TIM_CR1_CKD, TimClockDiv> clockDiv;
  RegFlag<addr_CR1, TIM_CR1_ARPE> enableArrPreload;

  std::tuple<
    Reg<addr_CCR1>,
    Reg<addr_CCR2>,
    Reg<addr_CCR3>,
    Reg<addr_CCR4>
  > ccValues;

  Reg<addr_ARR> autoReload;
  Reg<addr_RCR> repetitionCounter;
  RegEnum<addr_SMCR, TIM_SMCR_SMS | TIM_SMCR_ECE, TimClockSrc> clockSrc;
  RegEnum<addr_CR2, TIM_CR2_MMS, TimTriggerOut> triggerOutput;
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

  Reg<addr_DIER> reg_DIER;

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

    if (conf.enable_update_interrupt) {
      NvicCtrl::setPriority(info.irqn, conf.interrupt_priority);
      NvicCtrl::enable(info.irqn);
      enableIt(TimInterruptType::update);
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

#endif /* IGB_STM32_PERIPH_TIM_H */
