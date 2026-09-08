#pragma once

// SAI1 + DMA audio pipeline for Daisy Seed2 DFM (PCM3060 codec).
//
// Thin wrapper over AudioSai1 (audio_sai1.hpp), which holds the codec-agnostic
// SAI / DMA pipeline. The DFM specifics are:
//   - Block A = master TX (PE6 SD_A), Block B = slave RX (PE3 SD_B)
//   - PB11: PCM3060 de-emphasis disable (GPIO output low)
// Everything else (format, clocking, DMA, TX FIFO preload) is documented in
// audio_sai1.hpp. The members dma_tx / dma_rx keep the historical names:
// dma_tx is the SAI1_A stream, dma_rx the SAI1_B stream.

#include <cstdint>
#include <cstddef>
#include <igb_daisy/periph/audio_sai1.hpp>

namespace igb::daisy {

// Kept for callers of the old names.
inline constexpr uint32_t pcm3060_sai_kernel_hz = audio_sai1_kernel_hz;

constexpr uint8_t pcm3060_calc_mck_div(uint32_t sample_rate) {
  return audio_sai1_calc_mck_div(sample_rate);
}

template <
  typename DmaTxStreamT,
  typename DmaRxStreamT,
  uint32_t sample_rate = 48000,
  size_t block_size = 48,
  // TX FIFO preload word count, see AudioSai1.
  uint32_t tx_fifo_preload_words = 0
>
struct AudioPcm3060
  : AudioSai1<DmaTxStreamT, DmaRxStreamT, sample_rate, block_size, tx_fifo_preload_words> {
  using Base = AudioSai1<DmaTxStreamT, DmaRxStreamT, sample_rate, block_size, tx_fifo_preload_words>;
  using Callback = typename Base::Callback;

  DmaTxStreamT& dma_tx = this->dma_a;  // SAI1_A TX (mem -> periph)
  DmaRxStreamT& dma_rx = this->dma_b;  // SAI1_B RX (periph -> mem)

  AudioPcm3060(int32_t* rx_buf_, int32_t* tx_buf_)
    : Base(rx_buf_, tx_buf_) {}

  void initGpio() {
    Base::initGpio();

    // PB11: PCM3060 de-emphasis disable
    auto deemp = igb::stm32::GpioPin::newPin(igb::stm32::GpioPinType::pb11);
    deemp.enable();
    deemp.setMode(igb::stm32::GpioMode::output);
    deemp.setOutputMode(igb::stm32::GpioOutputMode::pushpull);
    deemp.low();
  }

  void initSai() {
    Base::initSai(AudioSai1Direction::blockA_tx);
  }

  void init(Callback cb) {
    initGpio();
    initSai();
    Base::initDma();
    Base::start(cb);
  }
};

}  // namespace igb::daisy
