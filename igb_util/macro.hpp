#ifndef IGB_STM32_UTIL_MACRO_H
#define IGB_STM32_UTIL_MACRO_H

#define UNWRAP(id) id

#define IGB_FAST_INLINE inline __attribute__((always_inline, optimize("Ofast"))) 

#define IGB_UNUSED __attribute__((unused))

#define MODIFY_REG_SIMPLE(reg, param_name, value) \
  MODIFY_REG(reg, param_name ## _Msk, static_cast<uint32_t>(value) << param_name ## _Pos);

// c++20 compatible macros (REG must be constant variable, not expression!)
#define IGB_READ_REG(REG)         ((REG))
#define IGB_WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define IGB_CLEAR_REG(REG, VAL)   ((REG) = (0x0))
#define IGB_READ_BIT(REG, BIT)    ((REG) & (BIT))
#define IGB_CLEAR_BIT(REG, BIT)   ((REG) = (REG) & ~(BIT))
#define IGB_SET_BIT(REG, BIT)     ((REG) = (REG) | (BIT))
#define IGB_MODIFY_REG(REG, CLEARMASK, SETMASK) (REG = ((IGB_READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK))

#endif /* IGB_STM32_UTIL_MACRO_H */
