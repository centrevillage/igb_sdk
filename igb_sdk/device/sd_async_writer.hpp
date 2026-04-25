#pragma once

// Async block writer. Wraps SdCard::beginWriteBlocks so callers can submit
// writes by (start_sector + file_offset_bytes) without touching block
// addresses directly. Templated on the SdCard implementation so test mocks
// can substitute.

#include <cstdint>
#include <cstddef>

namespace igb::sdk {

template <typename SdCardT>
struct SdAsyncWriter {
  SdCardT& sd_card;

  explicit SdAsyncWriter(SdCardT& c) : sd_card(c) {}

  // Submit an async write of `bytes` bytes (multiple of 512) from `buf` to
  // the SD starting at `start_sector + file_offset_bytes / 512`. Returns
  // false if submission to the DMA engine fails.
  bool beginWrite(uint32_t start_sector, size_t file_offset_bytes,
                  const void* buf, size_t bytes) {
    uint32_t target_sector = start_sector + (uint32_t)(file_offset_bytes / 512);
    uint32_t block_count   = (uint32_t)(bytes / 512);
    return sd_card.beginWriteBlocks(
        reinterpret_cast<const uint32_t*>(buf),
        target_sector,
        block_count);
  }

  bool isComplete() { return sd_card.isTransferComplete(); }

  bool endWrite() { return sd_card.endTransfer(); }

  // Block until any in-flight transfer completes, then finalize. Intended
  // for cancellation/synchronous drain points. Uses __WFI() to sleep-wait
  // on the SDMMC IRQ.
  bool drain() {
    while (!sd_card.isTransferComplete()) { __WFI(); }
    return sd_card.endTransfer();
  }
};

}  // namespace igb::sdk
