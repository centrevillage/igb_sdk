#pragma once

// SAI1 + DMA audio pipeline for the Daisy Seed family (codec-agnostic part).
//
// Hardware (all AF6):
//   PE2 MCLK, PE4 FS, PE5 SCK  -- driven by Block A (master); Block B is the
//                                 slave, synchronised internally to A
//   PE6 SD_A, PE3 SD_B
// Which block transmits is a board property (AudioSai1Direction):
//   blockA_tx : A = master TX (PE6), B = slave RX (PE3)
//               Daisy Seed rev4 (AK4556), Seed2 DFM (PCM3060), Seed3 (TAC5242)
//   blockA_rx : A = master RX (PE6), B = slave TX (PE3)
//               Daisy Seed 1.1 (WM8731)
// Codec-specific preparation (PB11 reset pulse or de-emphasis pin, I2C
// register setup) is the caller's business; see AudioPcm3060 for the DFM.
//
// Format: I2S Left-Justified (MSB justified), 32-bit slots, stereo. The
// codecs' 24-bit words sit in the upper bits of each int32. These are the
// register values libDaisy's HAL produced for SAI_I2S_MSBJUSTIFIED / 24 bit:
// FS active high on the left channel, FS on the first bit, 64 SCK per frame,
// CKSTR = 1 on both blocks (data changes on the falling SCK edge, is sampled
// on the rising edge) whichever direction they run in.
//
// Clocking assumes the SAI kernel clock is PLL3P ~ 49.152 MHz (configured by
// the application's sys_init). On the H7 SAI (RM0433 51.4.7, NODIV = 0):
//   MCLK = SAI_CK / MCKDIV,   FS = MCLK / (256 * (OSR + 1))
// so the MCLK pin carries 256 x fs (OSR = 0) or 512 x fs (OSR = 1); FS and
// SCK are the same either way. The ratio is the mclk_fs_ratio template
// parameter. 512 is the value LilaCRepeater ships with (PCM3060 at 48 kHz,
// MCLK 24.576 MHz; the codec auto-detects the ratio). 256 is what libDaisy's
// HAL produced (MckOverSampling disabled, MCKDIV = SAI_CK / (fs * 256)) and
// what a codec with a fixed ratio needs: the WM8731 (Daisy Seed 1.1) only
// knows 256 / 384 x fs, and at 512 x fs its converters run at twice the
// LRCLK rate, which shows up as a raised noise floor.
//
// DMA: one stream per SAI block (dma_a <-> SAI1_A, dma_b <-> SAI1_B), both
// circular with HT+TC interrupts on a single buffer split in half; the RX
// stream drives the callback. The stream types are template parameters so the
// application owns stream allocation. The class does not register IRQ
// handlers -- define e.g. `extern "C" void DMA1_Stream0_IRQHandler()` in the
// application and dispatch to `audio.dma_a.handleIrq()`.

#include <cstdint>
#include <cstddef>
#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/sai.hpp>
#include <igb_stm32/periph/gpio.hpp>

namespace igb::daisy {

// PLL3P assumed for the SAI kernel clock (audio-friendly 49.152 MHz).
inline constexpr uint32_t audio_sai1_kernel_hz = 49'152'000U;

// MCLK = PLL3P / mckDiv = mclk_fs_ratio * fs
//   -> mckDiv = PLL3P / (sample_rate * mclk_fs_ratio)
constexpr uint8_t audio_sai1_calc_mck_div(uint32_t sample_rate, uint16_t mclk_fs_ratio = 512) {
  return (uint8_t)(audio_sai1_kernel_hz / (sample_rate * (uint32_t)mclk_fs_ratio));
}

enum class AudioSai1Direction : uint8_t {
  blockA_tx = 0,  // A = master TX, B = slave RX
  blockA_rx = 1,  // A = master RX, B = slave TX
};

template <
  typename DmaAStreamT,   // stream wired to SAI1_A (DmaMux1ReqId::sai1A)
  typename DmaBStreamT,   // stream wired to SAI1_B (DmaMux1ReqId::sai1B)
  uint32_t sample_rate = 48000,
  size_t block_size = 48,
  // TX FIFO preload word count (0 = stock behaviour, no preload).
  //
  // The audio callback is driven by the RX DMA half/complete events, but TX
  // is an independent circular DMA. At the TX stream start the empty TX FIFO
  // has its DMA request already asserted, so the stream prefetches words into
  // the FIFO before the SAI is even enabled; the RX FIFO threshold adds lag on
  // the other side. The TX read pointer therefore runs several words AHEAD of
  // the RX event phase, which silently shortens the effective deadline for
  // writing the tx half-buffer below the nominal half period. Preloading k
  // zero words into the TX FIFO before starting the DMA suppresses the
  // initial prefetch burst and shifts the TX pointer k words later,
  // permanently (fetched = consumed + threshold - preload).
  //
  // The correct k is a phase COMPENSATION and must be measured on the target
  // system: it depends on both FIFO thresholds, the DMA/SAI enable order, and
  // the silicon (RM0433 51.4.9 pins the threshold fill levels, but the
  // shift-register accounting is +-1-2 words in practice). Constraints:
  //  - must be EVEN, or the interleaved L/R channel mapping swaps permanently
  //  - at most 8 (FIFO depth)
  //  - do not exceed the measured lead: overshooting makes TX LAG the
  //    boundary, and a callback finishing faster than the lag (one word
  //    period per word) overwrites tail words the previous window still needs
  // Writing SAI_xDR while the SAI is disabled is ST-sanctioned (the official
  // HAL's SAI_FillFifo() does the same; the FIFO is flushed in initSai()).
  uint32_t tx_fifo_preload_words = 0,
  // NVIC priority of both DMA stream interrupts.
  uint8_t dma_irq_priority = 1,
  // MCLK pin frequency as a multiple of fs: 512 (OSR = 1) or 256 (OSR = 0).
  // See the clocking note at the top of the file.
  uint16_t mclk_fs_ratio = 512
>
struct AudioSai1 {
  static constexpr size_t channels = 2;
  static constexpr size_t dma_size = block_size * channels * 2;  // *2 for two halves
  static constexpr uint8_t mck_div = audio_sai1_calc_mck_div(sample_rate, mclk_fs_ratio);

  static_assert(mclk_fs_ratio == 256 || mclk_fs_ratio == 512,
                "mclk_fs_ratio must be 256 or 512 (SAI OSR bit)");
  static_assert(mck_div >= 1, "sample_rate too high for PLL3P=49.152MHz audio clock");
  static_assert(tx_fifo_preload_words % 2 == 0,
                "odd preload would swap L/R channel mapping");
  static_assert(tx_fifo_preload_words <= 8, "SAI FIFO is 8 words deep");

  // Stereo-interleaved int32 (24-bit data left-justified in 32-bit slots).
  // size = block_size * channels (one half-buffer worth of samples).
  using Callback = void (*)(const int32_t* in, int32_t* out, size_t size);

  igb::stm32::Sai<igb::stm32::SaiType::sai1> sai;
  DmaAStreamT dma_a;  // SAI1_A: TX (mem -> periph) for blockA_tx, RX for blockA_rx
  DmaBStreamT dma_b;  // SAI1_B: the other way round

  int32_t* const rx_buf;  // length: dma_size, must be in DMA-capable memory
  int32_t* const tx_buf;  // length: dma_size, must be in DMA-capable memory

  Callback callback = nullptr;
  AudioSai1Direction direction = AudioSai1Direction::blockA_tx;

  AudioSai1(int32_t* rx_buf_, int32_t* tx_buf_)
    : rx_buf(rx_buf_), tx_buf(tx_buf_) {}

  bool isBlockATx() const { return direction == AudioSai1Direction::blockA_tx; }

  void initGpio() {
    // SAI1 GPIO: AF6
    sai.prepareGpio(igb::stm32::GpioPinType::pe2);  // MCLK
    sai.prepareGpio(igb::stm32::GpioPinType::pe3);  // SD_B
    sai.prepareGpio(igb::stm32::GpioPinType::pe4);  // FS
    sai.prepareGpio(igb::stm32::GpioPinType::pe5);  // SCK
    sai.prepareGpio(igb::stm32::GpioPinType::pe6);  // SD_A
  }

  void initSai(AudioSai1Direction dir) {
    direction = dir;
    const bool a_tx = isBlockATx();

    sai.enableBusClock();

    // --- Block A: master (drives MCLK / FS / SCK), TX or RX ---
    sai.blockA.initBlock({
      .mode       = a_tx ? igb::stm32::SaiBlockMode::masterTransmit
                         : igb::stm32::SaiBlockMode::masterReceive,
      .protocol   = igb::stm32::SaiProtocol::free,
      .dataSize   = igb::stm32::SaiDataSize::_32bit,
      .clockStrobing = true, // CKSTR=1 for both directions (libDaisy HAL value)
      .syncMode   = igb::stm32::SaiSyncMode::async,
      .outputDrive = false,
      .mckDiv     = mck_div,
      .mckEnable  = true,
      .osr        = (mclk_fs_ratio == 512),  // FS = MCLK / (256 * (OSR + 1));
                                             // mck_div is computed for the
                                             // same ratio, so FS stays put.
    });

    // I2S Left-Justified (MSB Justified) frame:
    //   64 SCK cycles per frame (32 per channel)
    //   FS active high for left channel
    //   FS on first bit of slot 0 (FSOFF=0)
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

    // --- Block B: slave synchronised to Block A, the opposite direction ---
    sai.blockB.initBlock({
      .mode       = a_tx ? igb::stm32::SaiBlockMode::slaveReceive
                         : igb::stm32::SaiBlockMode::slaveTransmit,
      .protocol   = igb::stm32::SaiProtocol::free,
      .dataSize   = igb::stm32::SaiDataSize::_32bit,
      .clockStrobing = true,
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
    dma_a.init(igb::stm32::DmaMux1ReqId::sai1A, dma_irq_priority);
    dma_b.init(igb::stm32::DmaMux1ReqId::sai1B, dma_irq_priority);

    // The RX stream drives the callback.
    auto on_half = [this]() {
      if (callback) {
        callback(rx_buf, tx_buf, block_size * channels);
      }
    };
    auto on_complete = [this]() {
      if (callback) {
        constexpr size_t half = block_size * channels;
        callback(rx_buf + half, tx_buf + half, half);
      }
    };
    if (isBlockATx()) {
      dma_b.on_half_transfer = on_half;
      dma_b.on_complete = on_complete;
      dma_a.on_half_transfer = nullptr;
      dma_a.on_complete = nullptr;
    } else {
      dma_a.on_half_transfer = on_half;
      dma_a.on_complete = on_complete;
      dma_b.on_half_transfer = nullptr;
      dma_b.on_complete = nullptr;
    }
  }

  void start(Callback cb) {
    callback = cb;

    // Clear buffers
    for (size_t i = 0; i < dma_size; ++i) {
      tx_buf[i] = 0;
      rx_buf[i] = 0;
    }

    const uint32_t sai_a_dr = sai.blockA.addr_DR;
    const uint32_t sai_b_dr = sai.blockB.addr_DR;
    const uint32_t tx_dr = isBlockATx() ? sai_a_dr : sai_b_dr;

    // Preload the TX FIFO with silence BEFORE the TX DMA starts (see the
    // tx_fifo_preload_words template parameter). No-op when 0.
    for (uint32_t i = 0; i < tx_fifo_preload_words; ++i) {
      *reinterpret_cast<volatile uint32_t*>(tx_dr) = 0u;
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

    if (isBlockATx()) {
      dma_a.start(sai_a_dr, reinterpret_cast<uint32_t>(tx_buf), dma_size, tx_conf);
      dma_b.start(sai_b_dr, reinterpret_cast<uint32_t>(rx_buf), dma_size, rx_conf);
    } else {
      dma_a.start(sai_a_dr, reinterpret_cast<uint32_t>(rx_buf), dma_size, rx_conf);
      dma_b.start(sai_b_dr, reinterpret_cast<uint32_t>(tx_buf), dma_size, tx_conf);
    }

    // Enable SAI: slave (Block B) first, then master (Block A)
    sai.blockB.enable();
    sai.blockA.enable();
  }

  void init(Callback cb, AudioSai1Direction dir = AudioSai1Direction::blockA_tx) {
    initGpio();
    initSai(dir);
    initDma();
    start(cb);
  }
};

}  // namespace igb::daisy
