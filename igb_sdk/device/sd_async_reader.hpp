#pragma once

// Async block reader. Wraps SdCard::beginReadBlocks so callers can request
// reads by (start_sector + file_offset_bytes) rather than block addresses.
// Templated on the SdCard implementation so test mocks can substitute.

#include <cstdint>
#include <cstddef>

namespace igb::sdk {

template <typename SdCardT>
struct SdAsyncReader {
  SdCardT& sd_card;

  explicit SdAsyncReader(SdCardT& c) : sd_card(c) {}

  // Submit an async read of `bytes` bytes (multiple of 512) into `buf`
  // starting at `start_sector + file_offset_bytes / 512`. Returns false
  // if submission fails.
  bool beginRead(uint32_t start_sector, size_t file_offset_bytes,
                 void* buf, size_t bytes) {
    uint32_t target_sector = start_sector + (uint32_t)(file_offset_bytes / 512);
    uint32_t block_count   = (uint32_t)(bytes / 512);
    return sd_card.beginReadBlocks(
        reinterpret_cast<uint32_t*>(buf), target_sector, block_count);
  }

  bool isComplete() { return sd_card.isTransferComplete(); }
  bool endRead()    { return sd_card.endTransfer(); }

  // Block until any in-flight transfer completes, then finalize. For abort
  // paths only; never in the steady-state load loop.
  bool drain() {
    while (!sd_card.isTransferComplete()) { __WFI(); }
    return sd_card.endTransfer();
  }
};

}  // namespace igb::sdk
