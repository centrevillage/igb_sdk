#pragma once

#if defined(STM32H7)

#include <igb_stm32/base.hpp>
#include <igb_util/macro.hpp>
#include <igb_stm32/periph/nvic.hpp>
#include <functional>

namespace igb {
namespace stm32 {

// DMAMUX1 request IDs (STM32H7xx RM0433 Table 122)
enum class DmaMux1ReqId : uint32_t {
  none             = 0,

  // DMAMUX1 request generators (1-8)
  dmamux1ReqGen0   = 1,
  dmamux1ReqGen1   = 2,
  dmamux1ReqGen2   = 3,
  dmamux1ReqGen3   = 4,
  dmamux1ReqGen4   = 5,
  dmamux1ReqGen5   = 6,
  dmamux1ReqGen6   = 7,
  dmamux1ReqGen7   = 8,

  // ADC (9-10)
  adc1             = 9,
  adc2             = 10,

  // TIM1 (11-17)
  tim1Ch1          = 11,
  tim1Ch2          = 12,
  tim1Ch3          = 13,
  tim1Ch4          = 14,
  tim1Up           = 15,
  tim1Trig         = 16,
  tim1Com          = 17,

  // TIM2 (18-22)
  tim2Ch1          = 18,
  tim2Ch2          = 19,
  tim2Ch3          = 20,
  tim2Ch4          = 21,
  tim2Up           = 22,

  // TIM3 (23-28)
  tim3Ch1          = 23,
  tim3Ch2          = 24,
  tim3Ch3          = 25,
  tim3Ch4          = 26,
  tim3Up           = 27,
  tim3Trig         = 28,

  // TIM4 (29-32)
  tim4Ch1          = 29,
  tim4Ch2          = 30,
  tim4Ch3          = 31,
  tim4Up           = 32,

  // I2C1-2 (33-36)
  i2c1Rx           = 33,
  i2c1Tx           = 34,
  i2c2Rx           = 35,
  i2c2Tx           = 36,

  // SPI1-2 (37-40)
  spi1Rx           = 37,
  spi1Tx           = 38,
  spi2Rx           = 39,
  spi2Tx           = 40,

  // USART1-2 (41-44)
  usart1Rx         = 41,
  usart1Tx         = 42,
  usart2Rx         = 43,
  usart2Tx         = 44,

  // USART3 (45-46)
  usart3Rx         = 45,
  usart3Tx         = 46,

  // TIM8 (47-53)
  tim8Ch1          = 47,
  tim8Ch2          = 48,
  tim8Ch3          = 49,
  tim8Ch4          = 50,
  tim8Up           = 51,
  tim8Trig         = 52,
  tim8Com          = 53,

  // 54: Reserved

  // TIM5 (55-60)
  tim5Ch1          = 55,
  tim5Ch2          = 56,
  tim5Ch3          = 57,
  tim5Ch4          = 58,
  tim5Up           = 59,
  tim5Trig         = 60,

  // SPI3 (61-62)
  spi3Rx           = 61,
  spi3Tx           = 62,

  // UART4-5 (63-66)
  uart4Rx          = 63,
  uart4Tx          = 64,
  uart5Rx          = 65,
  uart5Tx          = 66,

  // DAC (67-68)
  dacCh1           = 67,
  dacCh2           = 68,

  // TIM6-7 (69-70)
  tim6Up           = 69,
  tim7Up           = 70,

  // USART6 (71-72)
  usart6Rx         = 71,
  usart6Tx         = 72,

  // I2C3 (73-74)
  i2c3Rx           = 73,
  i2c3Tx           = 74,

  // DCMI / CRYP / HASH (75-78)
  dcmi             = 75,
  crypIn           = 76,
  crypOut          = 77,
  hashIn           = 78,

  // UART7-8 (79-82)
  uart7Rx          = 79,
  uart7Tx          = 80,
  uart8Rx          = 81,
  uart8Tx          = 82,

  // SPI4-5 (83-86)
  spi4Rx           = 83,
  spi4Tx           = 84,
  spi5Rx           = 85,
  spi5Tx           = 86,

  // SAI1-2 (87-90)
  sai1A            = 87,
  sai1B            = 88,
  sai2A            = 89,
  sai2B            = 90,

  // SWPMI (91-92)
  swpmiRx          = 91,
  swpmiTx          = 92,

  // SPDIF (93-94)
  spdifrxDat       = 93,
  spdifrxCtrl      = 94,

  // HR_REQ (95-100)
  hrReq1           = 95,
  hrReq2           = 96,
  hrReq3           = 97,
  hrReq4           = 98,
  hrReq5           = 99,
  hrReq6           = 100,

  // DFSDM1 (101-104)
  dfsdm1Dma0       = 101,
  dfsdm1Dma1       = 102,
  dfsdm1Dma2       = 103,
  dfsdm1Dma3       = 104,

  // TIM15 (105-108)
  tim15Ch1         = 105,
  tim15Up          = 106,
  tim15Trig        = 107,
  tim15Com         = 108,

  // TIM16-17 (109-112)
  tim16Ch1         = 109,
  tim16Up          = 110,
  tim17Ch1         = 111,
  tim17Up          = 112,

  // SAI3 (113-114)
  sai3A            = 113,
  sai3B            = 114,

  // ADC3 (115)
  adc3             = 115,
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
