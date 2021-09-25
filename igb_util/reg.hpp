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

template<uint32_t reg_addr>
struct RegRO {
  IGB_FAST_INLINE uint32_t operator()() {
    return (*IGB_ACC_REG_PTR);
  }
};

template<uint32_t reg_addr>
struct RegWO {
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_REG_PTR) = v;
  }
};

template<uint32_t reg_addr>
struct RegFragment {
  const uint32_t bit_mask;
  const uint32_t v;

  IGB_FAST_INLINE RegFragment<reg_addr> operator | (const RegFragment<reg_addr> fragment) {
    return RegFragment<reg_addr> { bit_mask | fragment.bit_mask, v | fragment.v };
  }

  IGB_FAST_INLINE void update() {
    (*IGB_ACC_REG_PTR) = ((*IGB_ACC_REG_PTR) & (~bit_mask)) | (v & bit_mask);
  }

  uint32_t mask() const noexcept {
    return bit_mask;
  }
  uint32_t value() const noexcept {
    return v;
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
    (*IGB_ACC_REG_PTR) = ((*IGB_ACC_REG_PTR) & (~bit_mask)) | (v & bit_mask);
  }

  IGB_FAST_INLINE RegFragment<reg_addr> val(uint32_t v) {
    return RegFragment<reg_addr> { bit_mask, v & bit_mask } ;
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask>
struct RegBitRO {
  IGB_FAST_INLINE uint32_t operator()() {
    return (*IGB_ACC_REG_PTR) & (bit_mask);
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask>
struct RegBitWO {
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_REG_PTR) = ((*IGB_ACC_REG_PTR) & (~bit_mask)) | (v & bit_mask);
  }

  IGB_FAST_INLINE RegFragment<reg_addr> val(uint32_t v) {
    return RegFragment<reg_addr> { bit_mask, v & bit_mask } ;
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
      (*IGB_ACC_REG_PTR) = (*IGB_ACC_REG_PTR) & ~bit_mask;
    } else {
      (*IGB_ACC_REG_PTR) = (*IGB_ACC_REG_PTR) | bit_mask;
    }
  }

  IGB_FAST_INLINE void disable() {
    if (invert_logic) {
      (*IGB_ACC_REG_PTR) = (*IGB_ACC_REG_PTR) | bit_mask;
    } else {
      (*IGB_ACC_REG_PTR) = (*IGB_ACC_REG_PTR) & ~bit_mask;
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

  IGB_FAST_INLINE RegFragment<reg_addr> val(bool flag) {
    return RegFragment<reg_addr> { bit_mask, (flag == invert_logic) ? (0) : (bit_mask) } ;
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask, bool invert_logic = false>
struct RegFlagRO {
  IGB_FAST_INLINE bool operator()() {
    if (invert_logic) {
      return !((*IGB_ACC_REG_PTR) & (bit_mask));
    }
    return (*IGB_ACC_REG_PTR) & (bit_mask);
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask, bool invert_logic = false>
struct RegFlagWO {
  IGB_FAST_INLINE void enable() {
    if (invert_logic) {
      (*IGB_ACC_REG_PTR) = (*IGB_ACC_REG_PTR) & ~bit_mask;
    } else {
      (*IGB_ACC_REG_PTR) = (*IGB_ACC_REG_PTR) | bit_mask;
    }
  }

  IGB_FAST_INLINE void disable() {
    if (invert_logic) {
      (*IGB_ACC_REG_PTR) = (*IGB_ACC_REG_PTR) | bit_mask;
    } else {
      (*IGB_ACC_REG_PTR) = (*IGB_ACC_REG_PTR) & ~bit_mask;
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

  IGB_FAST_INLINE RegFragment<reg_addr> val(bool flag) {
    return RegFragment<reg_addr> { bit_mask, (flag == invert_logic) ? (0) : (bit_mask) } ;
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask, const uint32_t bit_pos>
struct RegValue {
  IGB_FAST_INLINE uint32_t operator()() {
    return ((*IGB_ACC_REG_PTR) & (bit_mask)) >> bit_pos;
  }

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_REG_PTR) = ((*IGB_ACC_REG_PTR) & (~bit_mask)) | ((v << bit_pos) & bit_mask);
  }

  IGB_FAST_INLINE RegFragment<reg_addr> val(uint32_t v) {
    return RegFragment<reg_addr> { bit_mask, (v << bit_pos) & bit_mask } ;
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask, const uint32_t bit_pos>
struct RegValueRO {
  IGB_FAST_INLINE uint32_t operator()() {
    return ((*IGB_ACC_REG_PTR) & (bit_mask)) >> bit_pos;
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask, const uint32_t bit_pos>
struct RegValueWO {
  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_REG_PTR) = ((*IGB_ACC_REG_PTR) & (~bit_mask)) | ((v << bit_pos) & bit_mask);
  }

  IGB_FAST_INLINE RegFragment<reg_addr> val(uint32_t v) {
    return RegFragment<reg_addr> { bit_mask, (v << bit_pos) & bit_mask } ;
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask, typename ENUM_TYPE, const uint32_t bit_pos = 0>
struct RegEnum {
  // getter
  IGB_FAST_INLINE ENUM_TYPE operator()() {
    return static_cast<ENUM_TYPE>(((*IGB_ACC_REG_PTR) & (bit_mask)) >> bit_pos);
  }

  // setter
  IGB_FAST_INLINE void operator()(ENUM_TYPE v) {
    (*IGB_ACC_REG_PTR) = ((*IGB_ACC_REG_PTR) & (~bit_mask)) | ((static_cast<uint32_t>(v) << bit_pos) & bit_mask);
  }

  IGB_FAST_INLINE RegFragment<reg_addr> val(ENUM_TYPE v) {
    return RegFragment<reg_addr> { bit_mask, (static_cast<uint32_t>(v) << bit_pos) & bit_mask } ;
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask, typename ENUM_TYPE, const uint32_t bit_pos = 0>
struct RegEnumRO {
  IGB_FAST_INLINE ENUM_TYPE operator()() {
    return static_cast<ENUM_TYPE>(((*IGB_ACC_REG_PTR) & (bit_mask)) >> bit_pos);
  }
};

template<uint32_t reg_addr, const uint32_t bit_mask, typename ENUM_TYPE, const uint32_t bit_pos = 0>
struct RegEnumWO {
  // setter
  IGB_FAST_INLINE void operator()(ENUM_TYPE v) {
    (*IGB_ACC_REG_PTR) = ((*IGB_ACC_REG_PTR) & (~bit_mask)) | ((static_cast<uint32_t>(v) << bit_pos) & bit_mask);
  }

  IGB_FAST_INLINE RegFragment<reg_addr> val(ENUM_TYPE v) {
    return RegFragment<reg_addr> { bit_mask, (static_cast<uint32_t>(v) << bit_pos) & bit_mask } ;
  }
};

#undef IGB_ACC_REG_PTR

}

#endif /* IGB_UTIL_REG_H */
