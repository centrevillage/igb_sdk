#pragma once

#if defined(STM32H7)

#include <igb_stm32/base.hpp>
#include <igb_util/macro.hpp>
#include <igb_stm32/periph/nvic.hpp>
#include <functional>

namespace igb {
namespace stm32 {

// DMAMUX1 request IDs (STM32H7xx RM Table 121)
enum class DmaMux1ReqId : uint32_t {
  adc1    = 9,
  adc2    = 10,
  spi1_rx = 37,
  spi1_tx = 38,
  spi2_rx = 39,
  spi2_tx = 40,
  spi3_rx = 61,
  spi3_tx = 62,
  spi4_rx = 83,
  spi4_tx = 84,
  spi5_rx = 85,
  spi5_tx = 86,
};

// STM32H7 DMA Stream wrapper (DMA1/DMA2, stream 0-7)
// Handles DMAMUX1 routing, NVIC setup, and TC interrupt.
//
// Usage:
//   DmaStream<DmaType::dma1, 0> dma;
//   dma.on_complete = []() { /* called from DMA TC ISR */ };
//   dma.init(DmaMux1ReqId::spi1_tx);
//   // In ISR: extern "C" void DMA1_Stream0_IRQHandler() { dma.handleIrq(); }
template<DmaType DMA_TYPE, uint8_t STREAM_IDX>
struct DmaStream {
  static_assert(STREAM_IDX < 8, "Stream index must be 0-7");

  constexpr static auto type = DMA_TYPE;
  constexpr static auto addr = STM32_PERIPH_INFO.dma[to_idx(type)].addr;

  // DMAMUX1 channel: DMA1 streams → ch0-7, DMA2 streams → ch8-15
  constexpr static uint8_t dmamux_ch_idx =
    (DMA_TYPE == DmaType::dma1 ? 0 : 8) + STREAM_IDX;

  // Interrupt flag bit shift within LISR/HISR
  // Streams 0,4→shift 0; 1,5→6; 2,6→16; 3,7→22
  constexpr static uint32_t flag_shift =
    (STREAM_IDX % 4 == 0) ? 0u :
    (STREAM_IDX % 4 == 1) ? 6u :
    (STREAM_IDX % 4 == 2) ? 16u : 22u;

  constexpr static uint32_t tc_flag   = DMA_LISR_TCIF0 << flag_shift;
  constexpr static uint32_t all_flags = 0x3FUL         << flag_shift;

  constexpr static IRQn_Type get_irqn() {
    if constexpr (DMA_TYPE == DmaType::dma1) {
      if constexpr (STREAM_IDX == 0) return DMA1_Stream0_IRQn;
      else if constexpr (STREAM_IDX == 1) return DMA1_Stream1_IRQn;
      else if constexpr (STREAM_IDX == 2) return DMA1_Stream2_IRQn;
      else if constexpr (STREAM_IDX == 3) return DMA1_Stream3_IRQn;
      else if constexpr (STREAM_IDX == 4) return DMA1_Stream4_IRQn;
      else if constexpr (STREAM_IDX == 5) return DMA1_Stream5_IRQn;
      else if constexpr (STREAM_IDX == 6) return DMA1_Stream6_IRQn;
      else                                 return DMA1_Stream7_IRQn;
    } else {
      if constexpr (STREAM_IDX == 0) return DMA2_Stream0_IRQn;
      else if constexpr (STREAM_IDX == 1) return DMA2_Stream1_IRQn;
      else if constexpr (STREAM_IDX == 2) return DMA2_Stream2_IRQn;
      else if constexpr (STREAM_IDX == 3) return DMA2_Stream3_IRQn;
      else if constexpr (STREAM_IDX == 4) return DMA2_Stream4_IRQn;
      else if constexpr (STREAM_IDX == 5) return DMA2_Stream5_IRQn;
      else if constexpr (STREAM_IDX == 6) return DMA2_Stream6_IRQn;
      else                                 return DMA2_Stream7_IRQn;
    }
  }
  constexpr static IRQn_Type irqn = get_irqn();

  // Called from DMA TC ISR — set before calling init()
  std::function<void()> on_complete;

  DMA_Stream_TypeDef* p_stream() const {
    constexpr uint32_t base = (DMA_TYPE == DmaType::dma1) ? DMA1_BASE : DMA2_BASE;
    return reinterpret_cast<DMA_Stream_TypeDef*>(base + 0x10UL + STREAM_IDX * 0x18UL);
  }

  DMAMUX_Channel_TypeDef* p_dmamux_ch() const {
    //return reinterpret_cast<DMAMUX_Channel_TypeDef*>(
    //  DMAMUX1_BASE + dmamux_ch_idx * sizeof(DMAMUX_Channel_TypeDef));
    return reinterpret_cast<DMAMUX_Channel_TypeDef*>(
      DMAMUX1_BASE + dmamux_ch_idx * 4);
  }

  volatile uint32_t& isr_reg() const {
    DMA_TypeDef* dma = (DMA_TYPE == DmaType::dma1) ? DMA1 : DMA2;
    return (STREAM_IDX < 4) ? dma->LISR : dma->HISR;
  }

  volatile uint32_t& ifcr_reg() const {
    DMA_TypeDef* dma = (DMA_TYPE == DmaType::dma1) ? DMA1 : DMA2;
    return (STREAM_IDX < 4) ? dma->LIFCR : dma->HIFCR;
  }

  IGB_FAST_INLINE void enableBusClock() {
    const auto& dma_info = STM32_PERIPH_INFO.dma[to_idx(type)];
    dma_info.bus.enableBusClock();
  }

  void init(DmaMux1ReqId req_id, uint32_t nvic_priority = 1) {
    // Enable DMA clock (DMAMUX1 is clocked via the same AHB1 bus)
    enableBusClock();

    // Configure DMAMUX1 channel with request ID
    p_dmamux_ch()->CCR = static_cast<uint32_t>(req_id) & DMAMUX_CxCR_DMAREQ_ID_Msk;

    // Enable NVIC
    NvicCtrl::setPriority(irqn, nvic_priority);
    NvicCtrl::enable(irqn);
  }

  // TODO: refactoring: pass config struct value same as the dma.hpp
  // Start mem→periph 8-bit DMA transfer (TX only)
  void startTx(uint32_t mem_addr, uint32_t periph_addr, uint16_t count) {
    auto* s = p_stream();
    // Disable and wait until stream is off
    IGB_CLEAR_BIT(s->CR, DMA_SxCR_EN);
    while (s->CR & DMA_SxCR_EN) {}
    // Clear all interrupt flags
    ifcr_reg() = all_flags;
    // Configure addresses and count
    s->PAR  = periph_addr;
    s->M0AR = mem_addr;
    s->NDTR = count;
    // mem→periph, 8-bit×8-bit, mem increment, TC interrupt, enable
    s->CR = DMA_SxCR_DIR_0  // memory to peripheral
          | DMA_SxCR_MINC   // memory address increment
          | DMA_SxCR_TCIE   // transfer complete interrupt
          | DMA_SxCR_EN;    // enable stream
  }

  // Start periph→mem 16-bit circular DMA (RX, e.g. ADC)
  void startRxCircular(uint32_t periph_addr, uint32_t mem_addr, uint16_t count) {
    auto* s = p_stream();
    IGB_CLEAR_BIT(s->CR, DMA_SxCR_EN);
    while (s->CR & DMA_SxCR_EN) {}
    ifcr_reg() = all_flags;
    s->PAR  = periph_addr;
    s->M0AR = mem_addr;
    s->NDTR = count;
    // periph→mem (no DIR), 16-bit×16-bit, mem increment, circular, enable
    s->CR = DMA_SxCR_CIRC    // circular mode
          | DMA_SxCR_MINC    // memory address increment
          | DMA_SxCR_PSIZE_0 // periph 16-bit
          | DMA_SxCR_MSIZE_0 // memory 16-bit
          | DMA_SxCR_EN;
  }

  bool isBusy() const {
    return (p_stream()->CR & DMA_SxCR_EN) != 0;
  }

  // Call from ISR: extern "C" void DMAx_StreamN_IRQHandler() { dma.handleIrq(); }
  void handleIrq() {
    ifcr_reg() = all_flags;
    if (on_complete) on_complete();
  }
};

} // namespace stm32
} // namespace igb

#endif // STM32H7
