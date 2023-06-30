#ifndef IGB_STM32_UTIL_MACRO_H
#define IGB_STM32_UTIL_MACRO_H

#define UNWRAP(id) id

#define IGB_FAST_INLINE inline __attribute__((always_inline, optimize("Ofast"))) 
#define IGB_SIZE_OPTIMIZE __attribute__((optimize("Os"))) 

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

// David Mazières FOR_EACH implementation
// from: https://www.scs.stanford.edu/~dm/blog/va-opt.html
#define IGB_PARENS ()
// Rescan macro tokens 256 times
#define IGB_EXPAND(arg) IGB_EXPAND1(IGB_EXPAND1(IGB_EXPAND1(IGB_EXPAND1(arg))))
#define IGB_EXPAND1(arg) IGB_EXPAND2(IGB_EXPAND2(IGB_EXPAND2(IGB_EXPAND2(arg))))
#define IGB_EXPAND2(arg) IGB_EXPAND3(IGB_EXPAND3(IGB_EXPAND3(IGB_EXPAND3(arg))))
#define IGB_EXPAND3(arg) IGB_EXPAND4(IGB_EXPAND4(IGB_EXPAND4(IGB_EXPAND4(arg))))
#define IGB_EXPAND4(arg) arg
#define IGB_FOR_EACH(macro, ...) \
  __VA_OPT__(IGB_EXPAND(IGB_FOR_EACH_HELPER(macro, __VA_ARGS__)))
#define IGB_FOR_EACH_HELPER(macro, a1, ...) \
  macro(a1) \
  __VA_OPT__(IGB_FOR_EACH_AGAIN IGB_PARENS (macro, __VA_ARGS__))
#define IGB_FOR_EACH_AGAIN() IGB_FOR_EACH_HELPER

#define IGB_FOR_EACH_WITH_IDX(macro, ...) \
  __VA_OPT__(IGB_EXPAND(IGB_FOR_EACH_WITH_IDX_HELPER(macro, 0, __VA_ARGS__)))
#define IGB_FOR_EACH_WITH_IDX_HELPER(macro, idx, a1, ...) \
  macro(a1, idx) \
  __VA_OPT__(IGB_FOR_EACH_WITH_IDX_AGAIN IGB_PARENS (macro, (idx+1), __VA_ARGS__))
#define IGB_FOR_EACH_WITH_IDX_AGAIN() IGB_FOR_EACH_WITH_IDX_HELPER

#define IGB_PLUS_ONE(arg) +1
#define IGB_ARG_COUNT(...) \
  (IGB_FOR_EACH(IGB_PLUS_ONE, __VA_ARGS__))

#endif /* IGB_STM32_UTIL_MACRO_H */
