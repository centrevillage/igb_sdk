#ifndef IGB_GD32_PERIPH_DMA_H
#define IGB_GD32_PERIPH_DMA_H

#include <igb_gd32/base.hpp>
#include <igb_util/cast.hpp>
#include <igb_util/macro.hpp>
#include <igb_util/reg.hpp>
#include <igb_util/dynamic_reg.hpp>
#include <igb_gd32/periph/nvic.hpp>

#define IGB_DMA ((DMA_TypeDef*)addr)
#define IGB_DMA_REG_ADDR(member) (addr + offsetof(DMA_TypeDef, member))
#define IGB_DMA_REG(member) ((DMA_TypeDef*)IGB_DMA_REG_ADDR(member))

#define IGB_DMA_CH_REG_ADDR(member) ((uint32_t)(p_dma_channel) + offsetof(DMA_Channel_TypeDef, member))

namespace igb {
namespace gd32 {

enum class DmaStatus : uint32_t {
  globalInterrupt = 0b0001,
  complete = 0b0010,
  completeHalf = 0b0100,
  error = 0b1000
};

enum class DmaInterrupt : uint32_t {
  complete = 0b0010,
  completeHalf = 0b0100,
  error = 0b1000
};

enum class DmaTransDir : uint32_t {
  fromPeripheral = 0,
  fromMemory = 1,
};

enum class DmaChannelType : uint32_t {
  ch0 = 0,
  ch1,
  ch2,
  ch3,
  ch4,
  ch5,
  ch6,
};

enum class DmaPeriphBitWidth: uint32_t {
  _8bit = 0,
  _16bit = 1,
  _32bit = 2,
};

enum class DmaMemoryBitWidth : uint32_t {
  _8bit = 0,
  _16bit = 1,
  _32bit = 2,
};

enum class DmaChannelPriority : uint32_t {
  low = 0,
  mid = 1,
  high = 2,
  veryHigh = 3 
};

struct DmaChannelConf {
  DmaChannelType channel;
  DmaTransDir direction = DmaTransDir::fromPeripheral;
  bool circular = false;
  bool periph_increment = false;
  bool memory_increment = false;
  DmaPeriphBitWidth periph_bit_width = DmaPeriphBitWidth::_32bit;
  DmaMemoryBitWidth memory_bit_width = DmaMemoryBitWidth::_32bit;
  DmaChannelPriority priority = DmaChannelPriority::high;
  bool memory_to_memory = false;
  uint32_t data_count = 1;
  uint32_t periph_address;
  uint32_t memory_address;
  bool interrupt_complete = true;
  bool interrupt_complete_half = true;
  bool interrupt_error = true;
  uint32_t nvic_priority = 1;
};

struct DmaChannel {
  DMA_Channel_TypeDef* const p_dma_channel;

  DRegFlag<IGB_BIT(0)> enable {IGB_DMA_CH_REG_ADDR(CCR)};

  IGB_FAST_INLINE void enableIt(DmaInterrupt interrupt) {
    p_dma_channel->CCR = p_dma_channel->CCR | static_cast<uint32_t>(interrupt);
  }

  IGB_FAST_INLINE void disableIt(DmaInterrupt interrupt) {
    p_dma_channel->CCR = p_dma_channel->CCR & ~(static_cast<uint32_t>(interrupt));
  }

  DRegEnum<IGB_BIT_MASK(1, 4), DmaTransDir, 4> direction {IGB_DMA_CH_REG_ADDR(CCR)};
  DRegFlag<IGB_BIT(5)> circular {IGB_DMA_CH_REG_ADDR(CCR)};
  DRegFlag<IGB_BIT(6)> periphIncrement {IGB_DMA_CH_REG_ADDR(CCR)};
  DRegFlag<IGB_BIT(7)> memoryIncrement {IGB_DMA_CH_REG_ADDR(CCR)};
  DRegEnum<IGB_BIT_MASK(2, 8), DmaPeriphBitWidth, 8> periphBitWidth {IGB_DMA_CH_REG_ADDR(CCR)};
  DRegEnum<IGB_BIT_MASK(2, 10), DmaMemoryBitWidth, 10> memoryBitWidth {IGB_DMA_CH_REG_ADDR(CCR)};
  DRegEnum<IGB_BIT_MASK(2, 12), DmaChannelPriority, 12> priority {IGB_DMA_CH_REG_ADDR(CCR)};
  DRegFlag<IGB_BIT(14)> memoryToMemory {IGB_DMA_CH_REG_ADDR(CCR)};

  DRegValue<IGB_BIT_MASK(16, 0), 0> dataCount {IGB_DMA_CH_REG_ADDR(CNDTR)};
  DReg periphAddress {IGB_DMA_CH_REG_ADDR(CPAR)};
  DReg memoryAddress {IGB_DMA_CH_REG_ADDR(CMAR)};

  IGB_FAST_INLINE void configure(auto&& conf) {
    direction(conf.direction);
    circular(conf.circular);
    periphIncrement(conf.periph_increment);
    memoryIncrement(conf.memory_increment);
    periphBitWidth(conf.periph_bit_width);
    memoryBitWidth(conf.memory_bit_width);
    priority(conf.priority);
    memoryToMemory(conf.memory_to_memory);
    dataCount(conf.data_count);
    periphAddress(conf.periph_address);
    memoryAddress(conf.memory_address);

    if (conf.interrupt_complete) {
      enableIt(DmaInterrupt::complete);
    }
    if (conf.interrupt_complete_half) {
      enableIt(DmaInterrupt::completeHalf);
    }
    if (conf.interrupt_error) {
      enableIt(DmaInterrupt::error);
    }
    enable(true);
  }
};

template<DmaType DMA_TYPE>
struct Dma {
  constexpr static auto type = DMA_TYPE;
  constexpr static auto addr = GD32_PERIPH_INFO.dma[to_idx(type)].addr;
  constexpr static auto _channels = GD32_PERIPH_INFO.dma[to_idx(type)].channels;

  IGB_FAST_INLINE bool is(DmaChannelType ch_type, DmaStatus status) {
    const auto ch_idx = static_cast<uint32_t>(ch_type);
    const auto status_idx = static_cast<uint32_t>(status);
    return IGB_DMA->ISR & (status_idx << (ch_idx * 4));
  }

  IGB_FAST_INLINE void clear(DmaChannelType ch_type, DmaStatus status) {
    const auto ch_idx = static_cast<uint32_t>(ch_type);
    const auto status_idx = static_cast<uint32_t>(status);
    IGB_DMA->IFCR = IGB_DMA->IFCR | (status_idx << (ch_idx * 4));
  }

  IGB_FAST_INLINE void enableNvic(DmaChannelType ch_type, uint32_t priority = 1) {
    const auto& channel = _channels[static_cast<uint32_t>(ch_type)];
    NvicCtrl::setPriority(channel.irqn, priority);
    NvicCtrl::enable(channel.irqn);
  }

  IGB_FAST_INLINE void enableBusClock() {
    const auto& dma_info = GD32_PERIPH_INFO.dma[to_idx(type)];
    dma_info.bus.enableBusClock();
  }

  IGB_FAST_INLINE DmaChannel ch(DmaChannelType ch_type) {
    return DmaChannel {_channels[static_cast<uint32_t>(ch_type)].p_dma_channel};
  }

  IGB_FAST_INLINE void init(auto&&... confs) {
    enableBusClock();
    configure(confs...);
  }

  IGB_FAST_INLINE void configure(auto&& first, auto&&... rest) {
    auto channel = DmaChannel {_channels[static_cast<uint32_t>(first.channel)].p_dma_channel};
    enableNvic(first.channel, first.nvic_priority);
    channel.configure(first);
    configure(rest...);
  }
  IGB_FAST_INLINE void configure() { }

};

} /* gd32 */
} /* igb */

#undef IGB_DMA_CH_REG_ADDR

#undef IGB_DMA_REG
#undef IGB_DMA_REG_ADDR
#undef IGB_DMA

#endif /* IGB_GD32_PERIPH_DMA_H */
