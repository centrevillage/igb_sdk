#pragma once

#include <cstdint>
#include <igb_util/macro.hpp>

namespace igb {

#define IGB_ACC_DYNAMIC_REG_PTR ((volatile uint32_t *)(reg_addr))

struct DReg {
  const uint32_t reg_addr;

  // getter
  IGB_FAST_INLINE uint32_t operator()() {
    return (*IGB_ACC_DYNAMIC_REG_PTR);
  }

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_DYNAMIC_REG_PTR) = v;
  }
};

struct DRegRO {
  const uint32_t reg_addr;

  IGB_FAST_INLINE uint32_t operator()() {
    return (*IGB_ACC_DYNAMIC_REG_PTR);
  }
};

struct DRegWO {
  const uint32_t reg_addr;

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_DYNAMIC_REG_PTR) = v;
  }
};

template<const uint32_t bit_mask>
struct DRegBit {
  const uint32_t reg_addr;

  // getter
  IGB_FAST_INLINE uint32_t operator()() {
    return (*IGB_ACC_DYNAMIC_REG_PTR) & (bit_mask);
  }

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_DYNAMIC_REG_PTR) = ((*IGB_ACC_DYNAMIC_REG_PTR) & (~bit_mask)) | (v & bit_mask);
  }
};

template<const uint32_t bit_mask>
struct DRegBitRO {
  const uint32_t reg_addr;

  IGB_FAST_INLINE uint32_t operator()() {
    return (*IGB_ACC_DYNAMIC_REG_PTR) & (bit_mask);
  }
};

template<const uint32_t bit_mask>
struct DRegBitWO {
  const uint32_t reg_addr;

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_DYNAMIC_REG_PTR) = ((*IGB_ACC_DYNAMIC_REG_PTR) & (~bit_mask)) | (v & bit_mask);
  }
};

template<const uint32_t bit_mask, bool invert_logic = false>
struct DRegFlag {
  const uint32_t reg_addr;

  // getter
  IGB_FAST_INLINE bool operator()() {
    if (invert_logic) {
      return !((*IGB_ACC_DYNAMIC_REG_PTR) & (bit_mask));
    }
    return (*IGB_ACC_DYNAMIC_REG_PTR) & (bit_mask);
  }

  IGB_FAST_INLINE void enable() {
    if (invert_logic) {
      (*IGB_ACC_DYNAMIC_REG_PTR) = (*IGB_ACC_DYNAMIC_REG_PTR) & ~bit_mask;
    } else {
      (*IGB_ACC_DYNAMIC_REG_PTR) = (*IGB_ACC_DYNAMIC_REG_PTR) | bit_mask;
    }
  }

  IGB_FAST_INLINE void disable() {
    if (invert_logic) {
      (*IGB_ACC_DYNAMIC_REG_PTR) = (*IGB_ACC_DYNAMIC_REG_PTR) | bit_mask;
    } else {
      (*IGB_ACC_DYNAMIC_REG_PTR) = (*IGB_ACC_DYNAMIC_REG_PTR) & ~bit_mask;
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

template<const uint32_t bit_mask, bool invert_logic = false>
struct DRegFlagRO {
  const uint32_t reg_addr;

  // getter
  IGB_FAST_INLINE bool operator()() {
    if (invert_logic) {
      return !((*IGB_ACC_DYNAMIC_REG_PTR) & (bit_mask));
    }
    return (*IGB_ACC_DYNAMIC_REG_PTR) & (bit_mask);
  }
};

template<const uint32_t bit_mask, bool invert_logic = false>
struct DRegFlagWO {
  const uint32_t reg_addr;

  IGB_FAST_INLINE void enable() {
    if (invert_logic) {
      (*IGB_ACC_DYNAMIC_REG_PTR) = (*IGB_ACC_DYNAMIC_REG_PTR) & ~bit_mask;
    } else {
      (*IGB_ACC_DYNAMIC_REG_PTR) = (*IGB_ACC_DYNAMIC_REG_PTR) | bit_mask;
    }
  }

  IGB_FAST_INLINE void disable() {
    if (invert_logic) {
      (*IGB_ACC_DYNAMIC_REG_PTR) = (*IGB_ACC_DYNAMIC_REG_PTR) | bit_mask;
    } else {
      (*IGB_ACC_DYNAMIC_REG_PTR) = (*IGB_ACC_DYNAMIC_REG_PTR) & ~bit_mask;
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

template<const uint32_t bit_mask, const uint32_t bit_pos>
struct DRegValue {
  const uint32_t reg_addr;

  IGB_FAST_INLINE uint32_t operator()() {
    return ((*IGB_ACC_DYNAMIC_REG_PTR) & (bit_mask)) >> bit_pos;
  }

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_DYNAMIC_REG_PTR) = ((*IGB_ACC_DYNAMIC_REG_PTR) & (~bit_mask)) | ((v << bit_pos) & bit_mask);
  }
};

template<const uint32_t bit_mask, const uint32_t bit_pos>
struct DRegValueRO {
  const uint32_t reg_addr;

  IGB_FAST_INLINE uint32_t operator()() {
    return ((*IGB_ACC_DYNAMIC_REG_PTR) & (bit_mask)) >> bit_pos;
  }
};

template<const uint32_t bit_mask, const uint32_t bit_pos>
struct DRegValueWO {
  const uint32_t reg_addr;

  // setter
  IGB_FAST_INLINE void operator()(uint32_t v) {
    (*IGB_ACC_DYNAMIC_REG_PTR) = ((*IGB_ACC_DYNAMIC_REG_PTR) & (~bit_mask)) | ((v << bit_pos) & bit_mask);
  }
};

template<const uint32_t bit_mask, typename ENUM_TYPE, const uint32_t bit_pos = 0>
struct DRegEnum {
  const uint32_t reg_addr;

  // getter
  IGB_FAST_INLINE ENUM_TYPE operator()() {
    return static_cast<ENUM_TYPE>(((*IGB_ACC_DYNAMIC_REG_PTR) & (bit_mask)) >> bit_pos);
  }

  // setter
  IGB_FAST_INLINE void operator()(ENUM_TYPE v) {
    (*IGB_ACC_DYNAMIC_REG_PTR) = ((*IGB_ACC_DYNAMIC_REG_PTR) & (~bit_mask)) | ((static_cast<uint32_t>(v) << bit_pos) & bit_mask);
  }
};

template<const uint32_t bit_mask, typename ENUM_TYPE, const uint32_t bit_pos = 0>
struct DRegEnumRO {
  const uint32_t reg_addr;

  // getter
  IGB_FAST_INLINE ENUM_TYPE operator()() {
    return static_cast<ENUM_TYPE>(((*IGB_ACC_DYNAMIC_REG_PTR) & (bit_mask)) >> bit_pos);
  }
};

template<const uint32_t bit_mask, typename ENUM_TYPE, const uint32_t bit_pos = 0>
struct DRegEnumWO {
  const uint32_t reg_addr;

  // setter
  IGB_FAST_INLINE void operator()(ENUM_TYPE v) {
    (*IGB_ACC_DYNAMIC_REG_PTR) = ((*IGB_ACC_DYNAMIC_REG_PTR) & (~bit_mask)) | ((static_cast<uint32_t>(v) << bit_pos) & bit_mask);
  }
};

#undef IGB_ACC_DYNAMIC_REG_PTR

}

