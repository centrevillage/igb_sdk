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

// ============================================================
// DMA Stream configuration enums (RM0433 §15.5.5 DMA_SxCR)
// ============================================================

// DMA_SxCR DIR[1:0] — data transfer direction
enum class DmaStreamDir : uint32_t {
  periphToMem = 0,
  memToPeriph = 1,
  memToMem    = 2,
};

// DMA_SxCR PSIZE[1:0] / MSIZE[1:0] — data size
enum class DmaStreamDataSize : uint32_t {
  _8bit  = 0,
  _16bit = 1,
  _32bit = 2,
};

// DMA_SxCR MBURST[1:0] / PBURST[1:0] — burst transfer
enum class DmaStreamBurst : uint32_t {
  single = 0,
  incr4  = 1,
  incr8  = 2,
  incr16 = 3,
};

// DMA_SxCR PL[1:0] — priority level
enum class DmaStreamPriority : uint32_t {
  low      = 0,
  medium   = 1,
  high     = 2,
  veryHigh = 3,
};

// DMA_SxFCR FTH[1:0] — FIFO threshold selection
enum class DmaStreamFifoThreshold : uint32_t {
  quarter      = 0,
  half         = 1,
  threeQuarter = 2,
  full         = 3,
};

// ============================================================
// DMA Stream configuration struct
// ============================================================

struct DmaStreamConf {
  // DMA_SxCR fields
  DmaStreamDir       direction        = DmaStreamDir::periphToMem;
  DmaStreamDataSize  periphSize       = DmaStreamDataSize::_8bit;
  DmaStreamDataSize  memSize          = DmaStreamDataSize::_8bit;
  bool               periphIncrement  = false;
  bool               memIncrement     = true;
  bool               circular         = false;
  DmaStreamPriority  priority         = DmaStreamPriority::low;
  DmaStreamBurst     memBurst         = DmaStreamBurst::single;
  DmaStreamBurst     periphBurst      = DmaStreamBurst::single;
  bool               doubleBuffer     = false;
  bool               periphFlowCtrl   = false;
  bool               bufferable       = false;   // TRBUFF: set for UART/USART/LPUART

  // DMA_SxFCR fields
  bool                    enableFifo      = false;   // DMDIS=1 → FIFO enabled
  DmaStreamFifoThreshold  fifoThreshold   = DmaStreamFifoThreshold::half;

  // Interrupt enables
  bool  interruptComplete      = false;  // TCIE
  bool  interruptHalfTransfer  = false;  // HTIE
  bool  interruptError         = false;  // TEIE
  bool  interruptDirectMode    = false;  // DMEIE
  bool  interruptFifo          = false;  // FEIE
};

// ============================================================
// DMA Stream wrapper (DMA1/DMA2, stream 0-7)
//
// Usage:
//   DmaStream<DmaType::dma1, 0> dma;
//   dma.on_complete = []() { /* TC ISR */ };
//   dma.on_half_transfer = []() { /* HT ISR */ };
//   dma.init(DmaMux1ReqId::sai1A);
//   dma.start(periphAddr, mem0Addr, count, conf);
//
//   // In ISR:
//   extern "C" void DMA1_Stream0_IRQHandler() { dma.handleIrq(); }
// ============================================================

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

  constexpr static uint32_t tc_flag    = DMA_LISR_TCIF0  << flag_shift;
  constexpr static uint32_t ht_flag    = DMA_LISR_HTIF0  << flag_shift;
  constexpr static uint32_t te_flag    = DMA_LISR_TEIF0  << flag_shift;
  constexpr static uint32_t dme_flag   = DMA_LISR_DMEIF0 << flag_shift;
  constexpr static uint32_t fe_flag    = DMA_LISR_FEIF0  << flag_shift;
  constexpr static uint32_t all_flags  = 0x3DUL          << flag_shift; // bits 0,2,3,4,5

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

  // TODO: fix these namings by snake case
  // Callbacks — set before calling init() / start()
  std::function<void()> on_complete;
  std::function<void()> on_half_transfer;

  // ----- Hardware access helpers -----

  DMA_Stream_TypeDef* p_stream() const {
    constexpr uint32_t base = (DMA_TYPE == DmaType::dma1) ? DMA1_BASE : DMA2_BASE;
    return reinterpret_cast<DMA_Stream_TypeDef*>(base + 0x10UL + STREAM_IDX * 0x18UL);
  }

  DMAMUX_Channel_TypeDef* p_dmamux_ch() const {
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

  // ----- Init / control -----

  IGB_FAST_INLINE void enableBusClock() {
    const auto& dma_info = STM32_PERIPH_INFO.dma[to_idx(type)];
    dma_info.bus.enableBusClock();
  }

  // Initialize DMAMUX1 routing and NVIC.  Call once before start().
  void init(DmaMux1ReqId reqId, uint32_t nvicPriority = 1) {
    enableBusClock();
    p_dmamux_ch()->CCR = static_cast<uint32_t>(reqId) & DMAMUX_CxCR_DMAREQ_ID_Msk;
    NvicCtrl::setPriority(irqn, nvicPriority);
    NvicCtrl::enable(irqn);
  }

  // Disable stream and wait until hardware confirms it is off.
  IGB_FAST_INLINE void stop() {
    auto* s = p_stream();
    IGB_CLEAR_BIT(s->CR, DMA_SxCR_EN);
    while (s->CR & DMA_SxCR_EN) {}
  }

  // Start a single-buffer DMA transfer.
  void start(uint32_t periphAddr, uint32_t memAddr, uint16_t count,
             const DmaStreamConf& conf)
  {
    auto* s = p_stream();

    // 1. Disable stream and wait (RM0433 §15.3.19 step 1)
    stop();

    // 2. Clear all interrupt flags
    ifcr_reg() = all_flags;

    // 3. Addresses and data count (steps 2-4)
    s->PAR  = periphAddr;
    s->M0AR = memAddr;
    s->NDTR = count;

    // 4. FIFO control register (step 8)
    uint32_t fcr = 0;
    if (conf.enableFifo) {
      fcr |= DMA_SxFCR_DMDIS;
      fcr |= (static_cast<uint32_t>(conf.fifoThreshold) << DMA_SxFCR_FTH_Pos);
    }
    if (conf.interruptFifo) {
      fcr |= DMA_SxFCR_FEIE;
    }
    s->FCR = fcr;

    // 5. Build CR (step 9)
    uint32_t cr = buildCr(conf);

    // 6. Enable (step 10)
    s->CR = cr | DMA_SxCR_EN;
  }

  // Start a double-buffer DMA transfer.
  // Circular mode is automatically forced by hardware when DBM=1.
  void start(uint32_t periphAddr, uint32_t mem0Addr, uint32_t mem1Addr,
             uint16_t count, const DmaStreamConf& conf)
  {
    auto* s = p_stream();

    stop();
    ifcr_reg() = all_flags;

    s->PAR  = periphAddr;
    s->M0AR = mem0Addr;
    s->M1AR = mem1Addr;
    s->NDTR = count;

    uint32_t fcr = 0;
    if (conf.enableFifo) {
      fcr |= DMA_SxFCR_DMDIS;
      fcr |= (static_cast<uint32_t>(conf.fifoThreshold) << DMA_SxFCR_FTH_Pos);
    }
    if (conf.interruptFifo) {
      fcr |= DMA_SxFCR_FEIE;
    }
    s->FCR = fcr;

    uint32_t cr = buildCr(conf);
    cr |= DMA_SxCR_DBM;

    s->CR = cr | DMA_SxCR_EN;
  }

  // ----- Status -----

  bool isBusy() const {
    return (p_stream()->CR & DMA_SxCR_EN) != 0;
  }

  // Current target in double-buffer mode: 0 = M0AR, 1 = M1AR
  uint8_t currentTarget() const {
    return (p_stream()->CR & DMA_SxCR_CT) ? 1 : 0;
  }

  // Remaining data items to transfer
  uint16_t remaining() const {
    return static_cast<uint16_t>(p_stream()->NDTR);
  }

  // ----- ISR handler -----

  // Call from the DMAx_StreamN_IRQHandler.
  // Dispatches to on_half_transfer / on_complete based on which flags fired.
  void handleIrq() {
    uint32_t isr = isr_reg();
    ifcr_reg() = all_flags;

    if ((isr & ht_flag) && on_half_transfer) {
      on_half_transfer();
    }
    if ((isr & tc_flag) && on_complete) {
      on_complete();
    }
  }

private:
  // Build DMA_SxCR value from config (without EN bit).
  IGB_FAST_INLINE uint32_t buildCr(const DmaStreamConf& conf) const {
    uint32_t cr = 0;

    cr |= (static_cast<uint32_t>(conf.direction)   << DMA_SxCR_DIR_Pos);
    cr |= (static_cast<uint32_t>(conf.periphSize)   << DMA_SxCR_PSIZE_Pos);
    cr |= (static_cast<uint32_t>(conf.memSize)       << DMA_SxCR_MSIZE_Pos);
    cr |= (static_cast<uint32_t>(conf.priority)      << DMA_SxCR_PL_Pos);
    cr |= (static_cast<uint32_t>(conf.memBurst)      << DMA_SxCR_MBURST_Pos);
    cr |= (static_cast<uint32_t>(conf.periphBurst)   << DMA_SxCR_PBURST_Pos);

    if (conf.periphIncrement)  cr |= DMA_SxCR_PINC;
    if (conf.memIncrement)     cr |= DMA_SxCR_MINC;
    if (conf.circular)         cr |= DMA_SxCR_CIRC;
    if (conf.doubleBuffer)     cr |= DMA_SxCR_DBM;
    if (conf.periphFlowCtrl)   cr |= DMA_SxCR_PFCTRL;
    if (conf.bufferable)       cr |= DMA_SxCR_TRBUFF;

    if (conf.interruptComplete)      cr |= DMA_SxCR_TCIE;
    if (conf.interruptHalfTransfer)  cr |= DMA_SxCR_HTIE;
    if (conf.interruptError)         cr |= DMA_SxCR_TEIE;
    if (conf.interruptDirectMode)    cr |= DMA_SxCR_DMEIE;

    return cr;
  }
};

} // namespace stm32
} // namespace igb

#endif // STM32H7
