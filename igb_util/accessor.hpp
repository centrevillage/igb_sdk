#ifndef IGB_UTIL_ACCESSOR_ACCESSOR_H
#define IGB_UTIL_ACCESSOR_ACCESSOR_H

#include <igb_util/macro.hpp>

namespace igb {

template<volatile uint32_t* const p_reg_value>
struct RegAccessor {
  // getter
  IGB_FAST_INLINE uint32_t operator()() {
    return (*p_reg_value);
  }

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*p_reg_value) = v;
  }
};

template<volatile uint32_t* const p_reg_value, const uint32_t bit_mask>
struct RegBitAccessor {
  // getter
  IGB_FAST_INLINE uint32_t operator()() {
    return (*p_reg_value) & (bit_mask);
  }

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*p_reg_value) = (*p_reg_value) & (~bit_mask) | (v & bit_mask);
  }
};

template<volatile uint32_t* const p_reg_value, const uint32_t bit_mask, bool invert_logic = false>
struct RegFlagAccessor {
  // getter
  IGB_FAST_INLINE bool operator()() {
    if (invert_logic) {
      return !((*p_reg_value) & (bit_mask));
    }
    return (*p_reg_value) & (bit_mask);
  }

  IGB_FAST_INLINE void enable() {
    if (invert_logic) {
      (*p_reg_value) &= ~bit_mask;
    } else {
      (*p_reg_value) |= bit_mask;
    }
  }

  IGB_FAST_INLINE void disable() {
    if (invert_logic) {
      (*p_reg_value) |= bit_mask;
    } else {
      (*p_reg_value) &= ~bit_mask;
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

template<volatile uint32_t* const p_reg_value, const uint32_t bit_mask, const uint32_t bit_pos>
struct RegValueAccessor {
  IGB_FAST_INLINE uint32_t operator()() {
    return ((*p_reg_value) & (bit_mask)) >> bit_pos;
  }

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*p_reg_value) = (*p_reg_value) & (~bit_mask) | ((v << bit_pos) & bit_mask);
  }
};

template<volatile uint32_t* const p_reg_value, const uint32_t bit_mask, typename ENUM_TYPE>
struct RegEnumAccessor {
  // getter
  IGB_FAST_INLINE ENUM_TYPE operator()() {
    return static_cast<ENUM_TYPE>((*p_reg_value) & (bit_mask));
  }

  // setter
  IGB_FAST_INLINE void operator()(ENUM_TYPE v) {
    (*p_reg_value) = (*p_reg_value) & (~bit_mask) | (static_cast<uint32_t>(v) & bit_mask);
  }
};

}

#endif /* IGB_UTIL_ACCESSOR_ACCESSOR_H */
