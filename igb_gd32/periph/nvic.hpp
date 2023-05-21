#ifndef IGB_GD32_PERIPH_NVIC_H
#define IGB_GD32_PERIPH_NVIC_H

#include <igb_gd32/base.hpp>

namespace igb {
namespace gd32 {

struct NvicCtrl {
  static inline void setPriorityGrouping(uint32_t group) {
    NVIC_SetPriorityGrouping(group);
  }
  static inline uint32_t getPriorityGrouping() {
    return NVIC_GetPriorityGrouping();
  }
  static inline void enable(IRQn_Type irqn) {
    NVIC_EnableIRQ(irqn);
  }
  //static inline bool isEnabled(IRQn_Type irqn) {
  //  return NVIC_GetEnableIRQ(irqn);
  //}
  static inline void disable(IRQn_Type irqn) {
    NVIC_DisableIRQ(irqn);
  }
  static inline bool isPending(IRQn_Type irqn) {
    return NVIC_GetPendingIRQ(irqn);
  }
  static inline void pend(IRQn_Type irqn) {
    NVIC_SetPendingIRQ(irqn);
  }
  static inline void clearPending(IRQn_Type irqn) {
    NVIC_ClearPendingIRQ(irqn);
  }
  static inline bool isActive(IRQn_Type irqn) {
    return NVIC_GetActive(irqn);
  }
  static inline void setPriority(IRQn_Type irqn, uint32_t priority) {
    NVIC_SetPriority(irqn, priority);
  }
  static inline uint32_t getPriority(IRQn_Type irqn) {
    return NVIC_GetPriority(irqn);
  }
  static inline void systemReset() {
    NVIC_SystemReset();
  }
};

template <typename CtrlType = NvicCtrl>
struct NvicIrqn {
  IRQn_Type irqn;

  inline void enable() {
    CtrlType::enable(irqn);
  }
  inline bool isEnabled() {
    return CtrlType::isEnabled(irqn);
  }
  inline void disable() {
    CtrlType::disable(irqn);
  }
  inline bool isPending() {
    return CtrlType::isPending(irqn);
  }
  inline void pend() {
    CtrlType::pend(irqn);
  }
  inline void clearPending() {
    CtrlType::clearPending(irqn);
  }
  inline bool isActive() {
    return CtrlType::isActive(irqn);
  }
  inline void setPriority(uint32_t priority) {
    CtrlType::setPriority(irqn, priority);
  }
  inline uint32_t getPriority() {
    return CtrlType::getPriority(irqn);
  }
};

}
}

#endif /* IGB_GD32_PERIPH_NVIC_H */
