#ifndef IGB_UTIL_REG_H
#define IGB_UTIL_REG_H

#include <igb_util/macro.hpp>

namespace igb {

#define IGB_ACC_REG_PTR ((volatile uint32_t *)(reg_addr))

template<uint32_t reg_addr>
struct Reg {
  // getter
  IGB_FAST_INLINE uint32_t operator()() {
    return (*IGB_ACC_REG_PTR);
  }

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_REG_PTR) = v;
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask>
struct RegBit {
  // getter
  IGB_FAST_INLINE uint32_t operator()() {
    return (*IGB_ACC_REG_PTR) & (bit_mask);
  }

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_REG_PTR) = (*IGB_ACC_REG_PTR) & (~bit_mask) | (v & bit_mask);
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask, bool invert_logic = false>
struct RegFlag {
  // getter
  IGB_FAST_INLINE bool operator()() {
    if (invert_logic) {
      return !((*IGB_ACC_REG_PTR) & (bit_mask));
    }
    return (*IGB_ACC_REG_PTR) & (bit_mask);
  }

  IGB_FAST_INLINE void enable() {
    if (invert_logic) {
      (*IGB_ACC_REG_PTR) &= ~bit_mask;
    } else {
      (*IGB_ACC_REG_PTR) |= bit_mask;
    }
  }

  IGB_FAST_INLINE void disable() {
    if (invert_logic) {
      (*IGB_ACC_REG_PTR) |= bit_mask;
    } else {
      (*IGB_ACC_REG_PTR) &= ~bit_mask;
    }
  }

  // setter
  IGB_FAST_INLINE void operator()(bool flag) {
    if (flag) {
      enable();
    } else {
      disable();
    }
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask, const uint32_t bit_pos>
struct RegValue {
  IGB_FAST_INLINE uint32_t operator()() {
    return ((*IGB_ACC_REG_PTR) & (bit_mask)) >> bit_pos;
  }

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_REG_PTR) = (*IGB_ACC_REG_PTR) & (~bit_mask) | ((v << bit_pos) & bit_mask);
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask, typename ENUM_TYPE>
struct RegEnum {
  // getter
  IGB_FAST_INLINE ENUM_TYPE operator()() {
    return static_cast<ENUM_TYPE>((*IGB_ACC_REG_PTR) & (bit_mask));
  }

  // setter
  IGB_FAST_INLINE void operator()(ENUM_TYPE v) {
    (*IGB_ACC_REG_PTR) = (*IGB_ACC_REG_PTR) & (~bit_mask) | (static_cast<uint32_t>(v) & bit_mask);
  }
};

#undef IGB_ACC_REG_PTR

}

#endif /* IGB_UTIL_REG_H */
