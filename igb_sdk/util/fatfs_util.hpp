#pragma once

// FatFs internal-structure helpers for SD direct-sector writes.
//
// Bypasses FatFs for bulk WAV data body writes: callers use f_expand to
// allocate a contiguous cluster range, read the start cluster from the FIL,
// convert it to an absolute sector via FATFS database/csize, then write via
// the SD driver's async API. These accessors encapsulate the FatFs internal
// fields so an upgrade audit is localized here.
//
// Verified against FatFs R0.12c.

#include "ff.h"
#include <cstdint>

namespace igb::sdk::fatfs_util {

// Absolute SD sector number of `cluster` in fs. Cluster numbers 0/1 are reserved.
inline uint32_t cluster_to_sector(FATFS* fs, uint32_t cluster) {
  return fs->database + (cluster - 2) * fs->csize;
}

// Start sector of the (already-opened, expanded) file `fp`. Assumes contiguous
// allocation via f_expand(fp, _, 1); for fragmented files this returns only the
// first cluster's sector.
inline uint32_t file_start_sector(FIL* fp) {
  return cluster_to_sector(fp->obj.fs, fp->obj.sclust);
}

}  // namespace igb::sdk::fatfs_util
