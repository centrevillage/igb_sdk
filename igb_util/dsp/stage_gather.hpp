#pragma once

#include <cstdint>
#include <igb_util/dsp/q32_pos.hpp>

namespace igb::dsp {

// LilaC issue #225: seam between a DSP stage that wants a strided run of
// samples and a platform-specific engine that can fetch it in the
// background (on LilaC: MDMA, SDRAM -> DTCM).
//
// The contract is deliberately data-first: the provider PUBLISHES a window
// of already-staged items into a plain struct, and the consumer reads that
// struct as ordinary memory. There is no per-item and no per-frame indirect
// call — the consumer only calls poll() when its index leaves the published
// window (LilaC #201/#202: an indirect call into flash on the audio IRQ
// costs a veneer and an I-cache miss, so the hot path must not have one).
//
// Nothing here knows what the samples mean; the whole file is transport.

// One linear run: `count` items at src[0], src[stride], src[2*stride], ...
// The CONSUMER resolves its own addressing (wrapping, windowing, whatever
// it means by a position) and hands over plain pointers, so a provider needs
// to know nothing about the source container — which keeps the provider on
// the hardware side of the layering, where it belongs.
template<typename T>
struct StageRun {
  const T* src    = nullptr;
  uint32_t stride = 1;
  uint32_t count  = 0;
};

// The provider's published view: items [first, first + count) of the
// CURRENT request are readable at data[0 .. count). A provider that stages
// the whole request in one buffer leaves `first` at 0 and grows `count`; a
// provider that recycles a small ring slides both.
template<typename T>
struct StageWindow {
  const T* data  = nullptr;
  uint32_t first = 0;
  uint32_t count = 0;
};

// Installed by the OWNER (wiring, not state — a reset must not clear it).
// A null `begin` means "no provider": consumers fall back to reading the
// source directly, which is always the reference behaviour.
//
// Call sites and expected frequency (LilaC #225):
//   begin()  once per staged request (a few times per search)
//   poll()   only when the consumer's index leaves the window
//
// There is deliberately NO cancel(): abandoning a request is a plain
// consumer-side state drop, and the provider cleans up lazily inside the
// next begin(). That keeps every hook call — and therefore every hardware
// access — in the consumer's own context (LilaC: the audio IRQ), even
// though a reset can come from the main loop. An abandoned transfer simply
// finishes into a buffer nobody reads.
template<typename T>
struct StageHooks {
  // Take over the runs (concatenated, in order, `total` items overall) and
  // start fetching. false = cannot serve this request (disabled, too many
  // runs, hardware not idle yet) — the consumer then uses its direct path
  // for the WHOLE request. Must also make the provider forget any previous
  // request, without blocking. The runs array is the caller's and is only
  // valid for the duration of the call.
  bool (*begin)(void* ctx, const StageRun<T>* runs, uint32_t n_runs,
                uint32_t total) = nullptr;
  // Advance the published window if more data has landed. Must be cheap and
  // must never block.
  void (*poll)(void* ctx) = nullptr;

  void*           ctx = nullptr;
  StageWindow<T>* win = nullptr;
};

} // namespace igb::dsp
