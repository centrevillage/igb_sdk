#pragma once

// SAI1 + DMA audio pipeline for Daisy Seed2 DFM (PCM3060 codec).
//
// Hardware:
//   SAI1 Block A: Master TX (PE6 SD_A)
//   SAI1 Block B: Slave  RX (PE3 SD_B)
//   Shared:  PE2 MCLK, PE4 FS, PE5 SCK  (all AF6)
//   PB11: PCM3060 de-emphasis disable (GPIO output low)
//
// Format: I2S Left-Justified (MSB Justified), 24-bit audio in 32-bit slot, stereo.
//
// Clocking assumes the SAI kernel clock is PLL3P ≈ 49.152 MHz (the standard
// audio PLL value, configured by sys_init at the application level). MCKDIV
// is computed from the requested sample_rate so that fs = PLL3P / (MCKDIV*2*256)
// hits the target. Verified for 48 kHz (MCKDIV=2) and 96 kHz (MCKDIV=1).
//
// DMA: circular mode with HT+TC interrupts (single buffer split in half).
// The DMA stream types are template parameters so the application owns
// stream allocation. The class itself does not register IRQ handlers —
// register e.g. `extern "C" void DMA1_Stream0_IRQHandler()` at the
// application level and dispatch to `audio.dma_tx.handleIrq()`.

#include <cstdint>
#include <cstddef>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/sai.hpp>
#include <igb_stm32/periph/gpio.hpp>

namespace igb::daisy {

// PLL3P assumed for SAI kernel clock (audio-friendly 49.152 MHz).
inline constexpr uint32_t pcm3060_sai_kernel_hz = 49'152'000U;

// MCK = PLL3P / (mckDiv * 2), fs = MCK / 256
//   → mckDiv = PLL3P / (sample_rate * 512)
constexpr uint8_t pcm3060_calc_mck_div(uint32_t sample_rate) {
  return (uint8_t)(pcm3060_sai_kernel_hz / (sample_rate * 512U));
}

template <
  typename DmaTxStreamT,
  typename DmaRxStreamT,
  uint32_t sample_rate = 48000,
  size_t block_size = 48
>
struct AudioPcm3060 {
  static constexpr size_t channels = 2;                          // PCM3060 is stereo
  static constexpr size_t dma_size = block_size * channels * 2;  // *2 for two halves
  static constexpr uint8_t mck_div = pcm3060_calc_mck_div(sample_rate);

  static_assert(mck_div >= 1, "sample_rate too high for PLL3P=49.152MHz audio clock");

  // Stereo-interleaved int32 (24-bit data left-justified in 32-bit slots).
  // size = block_size * channels (one half-buffer worth of samples).
  using Callback = void (*)(const int32_t* in, int32_t* out, size_t size);

  igb::stm32::Sai<igb::stm32::SaiType::sai1> sai;
  DmaTxStreamT dma_tx;  // SAI1_A TX (mem → periph)
  DmaRxStreamT dma_rx;  // SAI1_B RX (periph → mem)

  int32_t* const rx_buf;  // length: dma_size, must be in DMA-capable memory
  int32_t* const tx_buf;  // length: dma_size, must be in DMA-capable memory

  Callback callback = nullptr;

  AudioPcm3060(int32_t* rx_buf_, int32_t* tx_buf_)
    : rx_buf(rx_buf_), tx_buf(tx_buf_) {}

  void initGpio() {
    // SAI1 GPIO: AF6
    sai.prepareGpio(igb::stm32::GpioPinType::pe2);  // MCLK
    sai.prepareGpio(igb::stm32::GpioPinType::pe3);  // SD_B (RX)
    sai.prepareGpio(igb::stm32::GpioPinType::pe4);  // FS
    sai.prepareGpio(igb::stm32::GpioPinType::pe5);  // SCK
    sai.prepareGpio(igb::stm32::GpioPinType::pe6);  // SD_A (TX)

    // PB11: PCM3060 de-emphasis disable
    auto deemp = igb::stm32::GpioPin::newPin(igb::stm32::GpioPinType::pb11);
    deemp.enable();
    deemp.setMode(igb::stm32::GpioMode::output);
    deemp.setOutputMode(igb::stm32::GpioOutputMode::pushpull);
    deemp.low();
  }

  void initSai() {
    sai.enableBusClock();

    // --- Block A: Master TX ---
    sai.blockA.initBlock({
      .mode       = igb::stm32::SaiBlockMode::masterTransmit,
      .protocol   = igb::stm32::SaiProtocol::free,
      .dataSize   = igb::stm32::SaiDataSize::_32bit,
      .clockStrobing = true, // CKSTR=1: HAL inverts for TX, so direct CKSTR=1 needed
      .syncMode   = igb::stm32::SaiSyncMode::async,
      .outputDrive = false,
      .mckDiv     = mck_div,
      .mckEnable  = true,
      .osr        = true,  // 512×fs oversampling: FFS = PLL3P/(MCKDIV×512) = 48k.
                           // Without it OSR=0 gives 256×fs → FFS doubles to 96k
                           // (mck_div is computed for the 512×fs relation). #110
    });

    // I2S Left-Justified (MSB Justified) frame:
    //   64 SCK cycles per frame (32 per channel)
    //   FS active high for left channel
    //   FS on first bit of slot 0 (SAI_FS_FIRSTBIT = FSOFF=0)
    sai.blockA.initFrame({
      .frameLength  = 63,   // FRL = 64 - 1
      .activeLength = 31,   // FSALL = 32 - 1
      .fsDefinition = true, // channel identification
      .fsPol        = true, // active high = left
      .fsOffset     = false, // FSOFF=0: FS on first bit (MSB Justified)
    });

    sai.blockA.initSlot({
      .firstBitOffset = 0,
      .slotSize       = igb::stm32::SaiSlotSize::_32bit,
      .numSlots       = 1,      // NBSLOT = 2 - 1
      .slotEnable     = 0x0003, // enable slot 0 and 1
    });

    sai.blockA.fifoThreshold(igb::stm32::SaiFifoThreshold::quarter);
    sai.blockA.dmaEnable(true);
    sai.blockA.flushFifo();

    // --- Block B: Slave RX (synchronized to Block A) ---
    sai.blockB.initBlock({
      .mode       = igb::stm32::SaiBlockMode::slaveReceive,
      .protocol   = igb::stm32::SaiProtocol::free,
      .dataSize   = igb::stm32::SaiDataSize::_32bit,
      .clockStrobing = true, // sample on rising edge (PCM3060 changes data on falling)
      .syncMode   = igb::stm32::SaiSyncMode::internal,  // sync with Block A
    });

    sai.blockB.initFrame({
      .frameLength  = 63,
      .activeLength = 31,
      .fsDefinition = true,
      .fsPol        = true,
      .fsOffset     = false,
    });

    sai.blockB.initSlot({
      .firstBitOffset = 0,
      .slotSize       = igb::stm32::SaiSlotSize::_32bit,
      .numSlots       = 1,
      .slotEnable     = 0x0003,
    });

    sai.blockB.fifoThreshold(igb::stm32::SaiFifoThreshold::quarter);
    sai.blockB.dmaEnable(true);
    sai.blockB.flushFifo();
  }

  void initDma() {
    // TX: SAI1_A (mem → periph)
    dma_tx.init(igb::stm32::DmaMux1ReqId::sai1A, 1);

    // RX: SAI1_B (periph → mem). RX drives the callback.
    dma_rx.on_half_transfer = [this]() {
      if (callback) {
        callback(rx_buf, tx_buf, block_size * channels);
      }
    };
    dma_rx.on_complete = [this]() {
      if (callback) {
        constexpr size_t half = block_size * channels;
        callback(rx_buf + half, tx_buf + half, half);
      }
    };
    dma_rx.init(igb::stm32::DmaMux1ReqId::sai1B, 1);
  }

  // TX FIFO preload word count (LilaCRepeater issue #250).
  //
  // The callback is driven by the RX DMA half/complete events, but TX is an
  // independent circular DMA. Without a preload, dma_tx.start() below finds an
  // empty TX FIFO with the request already asserted and instantly prefetches
  // several words BEFORE the SAI is even enabled; combined with the RX FIFO
  // threshold lag this leaves the TX read pointer ~6 words (31 us at 192 kHz
  // word rate) AHEAD of the RX half boundary at every callback entry. The
  // effective deadline for writing the tx half-buffer was therefore 18 words
  // (93.7 us), not 24 (125 us) — callbacks longer than that had their leading
  // samples consumed before they were written (device-measured: 64.9% of
  // blocks raced in the worst configuration; audible as a fizzing noise).
  //
  // Preloading k zero words keeps the FIFO level above the threshold at DMA
  // start, so the initial request burst never happens and the TX pointer runs
  // k words later permanently (FIFO level = preload + fetched - consumed, and
  // the DMA regulates level to the threshold — so fetched = consumed +
  // threshold - preload). k = 6 cancels the measured lead, restoring the full
  // 125 us deadline. Side effect: +6 words (~31 us) of output latency and 6
  // silent words at boot — both negligible.
  //
  // Verified on device via the parent firmware's T1+T3 overlay: entry_min
  // (words until TX enters the half being written, at callback entry) must
  // read ~0x17-0x18 (23-24) after this change, and the race count must stay 0.
  // Do NOT preload more than the measured lead: overshooting makes TX LAG the
  // boundary, and a callback that finishes faster than the lag (5.2 us/word)
  // would overwrite tail words the previous window still needs.
  //
  // MUST BE EVEN: the buffer is interleaved L,R and the preload shifts which
  // word lands on which slot. An even count shifts by whole frames (channel
  // mapping preserved); an odd count would SWAP left and right permanently.
  static constexpr uint32_t tx_fifo_preload_words = 6;
  static_assert(tx_fifo_preload_words % 2 == 0,
                "odd preload would swap L/R channel mapping");
  static_assert(tx_fifo_preload_words <= 8, "SAI FIFO is 8 words deep");

  void start(Callback cb) {
    callback = cb;

    // Clear buffers
    for (size_t i = 0; i < dma_size; ++i) {
      tx_buf[i] = 0;
      rx_buf[i] = 0;
    }

    // Issue #250: preload the TX FIFO with silence BEFORE the TX DMA starts
    // (see tx_fifo_preload_words above). SAI_xDR writes load the FIFO
    // regardless of SAIEN; the FIFO was flushed in initSai().
    for (uint32_t i = 0; i < tx_fifo_preload_words; ++i) {
      *reinterpret_cast<volatile uint32_t*>(sai.blockA.addr_DR) = 0u;
    }

    // DMA configuration: 32-bit, circular, HT+TC interrupts
    igb::stm32::DmaStreamConf tx_conf {
      .direction       = igb::stm32::DmaStreamDir::memToPeriph,
      .periphSize      = igb::stm32::DmaStreamDataSize::_32bit,
      .memSize         = igb::stm32::DmaStreamDataSize::_32bit,
      .memIncrement    = true,
      .circular        = true,
      .priority        = igb::stm32::DmaStreamPriority::high,
      .interruptComplete     = true,
      .interruptHalfTransfer = true,
    };

    igb::stm32::DmaStreamConf rx_conf {
      .direction       = igb::stm32::DmaStreamDir::periphToMem,
      .periphSize      = igb::stm32::DmaStreamDataSize::_32bit,
      .memSize         = igb::stm32::DmaStreamDataSize::_32bit,
      .memIncrement    = true,
      .circular        = true,
      .priority        = igb::stm32::DmaStreamPriority::high,
      .interruptComplete     = true,
      .interruptHalfTransfer = true,
    };

    uint32_t sai_a_dr = sai.blockA.addr_DR;
    uint32_t sai_b_dr = sai.blockB.addr_DR;

    dma_tx.start(sai_a_dr, reinterpret_cast<uint32_t>(tx_buf), dma_size, tx_conf);
    dma_rx.start(sai_b_dr, reinterpret_cast<uint32_t>(rx_buf), dma_size, rx_conf);

    // Enable SAI: slave (Block B) first, then master (Block A)
    sai.blockB.enable();
    sai.blockA.enable();
  }

  void init(Callback cb) {
    initGpio();
    initSai();
    initDma();
    start(cb);
  }
};

}  // namespace igb::daisy
