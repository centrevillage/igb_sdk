#ifndef IGB_GD32_PERIPH_RCC_H
#define IGB_GD32_PERIPH_RCC_H

#include <igb_gd32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>

namespace igb {
namespace gd32 {

struct RccCtrl {
  static IGB_FAST_INLINE void enableBusClock(const auto& periph_bus_info) {
    periph_bus_info.enableBusClock();
  }

  static IGB_FAST_INLINE void disableBusClock(const auto& periph_bus_info) {
    periph_bus_info.disableBusClock();
  }
};

}
}

#endif /* IGB_GD32_PERIPH_RCC_H */
