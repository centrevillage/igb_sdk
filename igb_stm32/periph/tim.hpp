#ifndef IGB_STM32_PERIPH_TIM_H
#define IGB_STM32_PERIPH_TIM_H

#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/rcc.hpp>
#include <igb_stm32/periph/nvic.hpp>
#include <igb_util/cast.hpp>

namespace igb {
namespace stm32 {

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

// TODO: デバイス依存の記述になっているので一旦機能削除
//enum class TimRemapInput : uint32_t { 
//  TO_GPIO = TIM14_OR_RMP_MASK,
//  TO_RTC_CLK = (TIM14_OR_TI1_RMP_0  | TIM14_OR_RMP_MASK),
//  TO_HSE = (TIM14_OR_TI1_RMP_1  | TIM14_OR_RMP_MASK),
//  TO_MCO = (TIM14_OR_TI1_RMP_0  | TIM14_OR_TI1_RMP_1  | TIM14_OR_RMP_MASK),
//};

struct TimConf {
  uint32_t prescale = 0;
  uint32_t period = 0;
  TimCounterMode counter_mode = TimCounterMode::up;
  TimClockDiv clock_div = TimClockDiv::div1;
  uint8_t repetition_counter = 0;
  bool arr_preload = false;
  TimClockSrc clock_src = TimClockSrc::internal;
  TimTriggerOut trigger_out = TimTriggerOut::reset;
  bool master_slave_mode = false;

  bool enable_update_interrupt = false;
  uint16_t interrupt_priority = 1;
};

struct Tim {
  TIM_TypeDef* p_tim;

  void setPrescaler(uint32_t prescale) {
    p_tim->PSC = prescale;
  }

  void setCounterMode(TimCounterMode mode) {
    MODIFY_REG(p_tim->CR1, (TIM_CR1_DIR | TIM_CR1_CMS), as<uint32_t>(mode));
  }

  void setClockDiv(TimClockDiv div) {
    MODIFY_REG(p_tim->CR1, TIM_CR1_CKD, as<uint32_t>(div));
  }

  void enableArrPreload() {
    SET_BIT(p_tim->CR1, TIM_CR1_ARPE);
  }

  void disableArrPreload() {
    CLEAR_BIT(p_tim->CR1, TIM_CR1_ARPE);
  }

  void setCcValue(TimCcCh ch, uint32_t value) {
    __IO uint32_t* ptr = &(p_tim->CCR1) + as<uint32_t>(ch);
    (*ptr) = value;
  }

  uint32_t getCcValue(TimCcCh ch) {
    __IO uint32_t* ptr = &(p_tim->CCR1) + as<uint32_t>(ch);
    return *ptr;
  }

  void setAutoreload(uint32_t value) {
    p_tim->ARR = value;
  }

  void setRepetitionCounter(uint8_t value) {
    p_tim->RCR = value;
  }

  void setClockSrc(TimClockSrc src) {
    MODIFY_REG(p_tim->SMCR, TIM_SMCR_SMS | TIM_SMCR_ECE, as<uint32_t>(src));
  }

  void setTriggerOutput(TimTriggerOut out) {
    MODIFY_REG(p_tim->CR2, TIM_CR2_MMS, as<uint32_t>(out));
  }

  void setMasterSlaveMode(bool is_master_slave) {
    if (is_master_slave) {
      p_tim->SMCR |= TIM_SMCR_MSM;
    } else {
      p_tim->SMCR &= ~TIM_SMCR_MSM;
    }
  }

  void setEtrConf(TimEtrConfPolarity polarity, TimEtrConfPrescaler prescaler, TimEtrConfFilter filter) {
    MODIFY_REG(p_tim->SMCR,
        TIM_SMCR_ETP | TIM_SMCR_ETPS | TIM_SMCR_ETF,
        as<uint32_t>(polarity) | as<uint32_t>(prescaler) | as<uint32_t>(filter));
  }

  bool is(TimState state) {
    return !!(p_tim->SR & as<uint32_t>(state));
  }

  void clear(TimState state) {
    p_tim->SR &= ~(as<uint32_t>(state));
  }

  void setCount(uint32_t count) {
    p_tim->CNT = count;
  }

  uint32_t getCount() {
    return p_tim->CNT;
  }

  void generateEvent(TimEventGen event) {
    p_tim->EGR |= as<uint32_t>(event);
  }

  void enableIt(TimInterruptType interrupt) {
    p_tim->DIER |= as<uint32_t>(interrupt);
  }

  void disableIt(TimInterruptType interrupt) {
    p_tim->DIER &= ~(as<uint32_t>(interrupt));
  }

  void setDeadtime(uint8_t dead_time) {
    MODIFY_REG(p_tim->BDTR, TIM_BDTR_DTG, as<uint32_t>(dead_time));
  }

  void enableBreak() {
    p_tim->BDTR |= TIM_BDTR_BKE;
    __IO uint32_t tmp = p_tim->BDTR;
  }

  void disableBreak() {
    p_tim->BDTR &= ~TIM_BDTR_BKE;
    __IO uint32_t tmp = p_tim->BDTR;
  }

  void configBreak(TimBreakPolarity polarity) {
    MODIFY_REG(p_tim->BDTR, TIM_BDTR_BKP, as<uint32_t>(polarity));
    __IO uint32_t tmp = p_tim->BDTR;
  }

  void setOffStates(bool off_state_idle, bool off_state_run) {
    MODIFY_REG(p_tim->BDTR, TIM_BDTR_OSSI | TIM_BDTR_OSSR, (off_state_idle ? TIM_BDTR_OSSI : 0) | (off_state_run ? TIM_BDTR_OSSR : 0));
  }

  void enableAutoOut() {
    p_tim->BDTR |= TIM_BDTR_AOE;
  }
  void disableAutoOut() {
    p_tim->BDTR &= ~TIM_BDTR_AOE;
  }

  void enableAllOut() {
    p_tim->BDTR |= TIM_BDTR_MOE;
  }
  void disableAllOut() {
    p_tim->BDTR &= ~TIM_BDTR_MOE;
  }

  void configDmaBurst(TimDmaBurstBaseAddr addr, TimDmaBurstLen len) {
    MODIFY_REG(p_tim->DCR, (TIM_DCR_DBL | TIM_DCR_DBA), (as<uint32_t>(addr) | as<uint32_t>(len)));
  }

//  void setRemap(TimRemapInput remap) {
//    MODIFY_REG(p_tim->OR, (as<uint32_t>(remap) >> 16U), (as<uint32_t>(remap) & 0x0000FFFFU));
//  }

  void enable(bool flag) {
    if (flag) {
      enable();
    } else {
      disable();
    }
  }

  void enable() {
    p_tim->CR1 |= TIM_CR1_CEN;
  }
  void disable() {
    p_tim->CR1 &= ~TIM_CR1_CEN;
  }
  void start() {
    setCount(0);
    enable();
  }
  void stop() {
    disable();
  }

  void init(TimType type, auto&& conf) {
    const auto& info = STM32_PERIPH_INFO.tim[as<uint32_t>(type)];
    p_tim = info.p_tim;
    info.bus.enableBusClock();

    if (conf.enable_update_interrupt) {
      NvicCtrl::setPriority(info.irqn, conf.interrupt_priority);
      NvicCtrl::enable(info.irqn);
      enableIt(TimInterruptType::update);
    }
    setCount(0);
    setPrescaler(conf.prescale);
    setAutoreload(conf.period);
    setCounterMode(conf.counter_mode);
    setClockDiv(conf.clock_div);
    setRepetitionCounter(conf.repetition_counter);
    if (conf.arr_preload) {
      enableArrPreload();
    } else {
      disableArrPreload();
    }
    setClockSrc(conf.clock_src);
    setTriggerOutput(conf.trigger_out);
    setMasterSlaveMode(conf.master_slave_mode);
  }

  // TODO: コンパイル時の依存を減らすため、外部関数化＆外部ファイル化すべき？
  static Tim newIntervalTimer(TimType type, uint16_t prescale, uint32_t period, uint16_t priority) {
    const auto& info = STM32_PERIPH_INFO.tim[as<uint32_t>(type)];
    info.bus.enableBusClock();

    NvicCtrl::setPriority(info.irqn, priority);
    NvicCtrl::enable(info.irqn);

    auto timer = Tim { info.p_tim };
    timer.enableIt(TimInterruptType::update);
    timer.setPrescaler(prescale);
    timer.setCounterMode(TimCounterMode::up);
    timer.setAutoreload(period);
    timer.setClockDiv(TimClockDiv::div1);
    timer.setRepetitionCounter(0);
    timer.disableArrPreload();
    timer.setClockSrc(TimClockSrc::internal);
    timer.setTriggerOutput(TimTriggerOut::reset);
    timer.setMasterSlaveMode(false);

    return timer;
  }
};

} // namespace stm32
} // namespace igb

#endif /* IGB_STM32_PERIPH_TIM_H */
