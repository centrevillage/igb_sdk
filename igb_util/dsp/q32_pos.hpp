#pragma once

#include <cstdint>
#include <igb_util/macro.hpp>

namespace igb::dsp {

// Q32.32 signed fixed-point playback-position math (LilaCRepeater issue #198,
// docs/198 §9 Layer 1). Value = q / 2^32: upper 32 bits = integer sample
// index, lower 32 bits = fractional position. Signed so reverse rates and
// pre-wrap negative positions work.
//
// LIBCALL RULE (docs/198 §4-7): a plain `(int64_t)double` / `(double)int64_t`
// / `(float)int64_t` cast emits an __aeabi_* libcall on ARM32 — newlib code
// resident in QSPI flash, which must never be reachable from the audio hot
// path (the #170 lesson). Every conversion here is therefore built ONLY from
// u32-domain vcvt steps plus integer/FP arithmetic. Keep it that way when
// editing; the build is checked with `grep __aeabi main.lst` (docs/198 §6-c).

using q32_t = int64_t;

constexpr q32_t q32_one = q32_t(1) << 32;

// Integer sample index (upper word). Precondition: q >= 0 — call only on
// wrapped positions (a negative q would arithmetic-shift to a huge index).
IGB_FAST_INLINE uint32_t q32_idx(q32_t q) {
  return (uint32_t)(q >> 32);
}

// Interpolation coefficient from the fractional word. Bit-identical to
// rounding the exact fraction lo/2^32 to float (scaling by a power of two
// preserves the u32->f32 rounding), which is what the double path's
// `(float)(read_pos - pos_i)` produced for 2^-32-grid positions.
IGB_FAST_INLINE float q32_frac_to_float(q32_t q) {
  return (float)(uint32_t)q * (1.0f / 4294967296.0f);
}

// Full value -> float (pos_snapshot). Sign-correct so a (degenerate-state)
// negative position snapshots as the old `(float)pos_double` did. Two u32
// vcvt + one fma + sign select; may differ from the old cast by 1 ULP from
// double rounding (harmless: PLL margin and UI pixel scale are orders above).
IGB_FAST_INLINE float q32_to_float(q32_t q) {
  bool neg = q < 0;
  uint64_t a = neg ? (uint64_t)(-q) : (uint64_t)q;
  float f = (float)(uint32_t)(a >> 32)
          + (float)(uint32_t)a * (1.0f / 4294967296.0f);
  return neg ? -f : f;
}

// double -> Q32.32, round-to-nearest half-away (llround-equivalent) without
// the __aeabi_d2lz libcall. Precondition: |v| < 2^31 (positions/rates are
// bounded by max_samples_per_track ≈ 2^22, so this is never tight).
IGB_FAST_INLINE q32_t q32_from_double(double v) {
  bool neg = v < 0.0;
  double a = neg ? -v : v;
  uint32_t hi = (uint32_t)a;                       // vcvt, trunc toward zero
  double frac = a - (double)hi;                    // exact, in [0, 1)
  double scaled = frac * 4294967296.0 + 0.5;       // exact (< 2^53)
  uint32_t lo;
  if (scaled >= 4294967296.0) {                    // fraction rounded up to 1.0
    ++hi;
    lo = 0u;
  } else {
    lo = (uint32_t)scaled;                         // floor(frac*2^32 + 0.5)
  }
  q32_t q = (q32_t)(((uint64_t)hi << 32) | lo);
  return neg ? -q : q;
}

// Q32.32 -> double (main-loop / test use; NOT for the audio hot path — the
// hot path never needs the double form back). Exact whenever the value has
// <= 53 significant bits — always true for values produced by
// q32_from_double from a double source.
IGB_FAST_INLINE double q32_to_double(q32_t q) {
  bool neg = q < 0;
  uint64_t a = neg ? (uint64_t)(-q) : (uint64_t)q;
  double d = (double)(uint32_t)(a >> 32)
           + (double)(uint32_t)a * (1.0 / 4294967296.0);
  return neg ? -d : d;
}

// Advance + single conditional wrap; returns whether a wrap occurred.
// EXACTLY movePos()'s semantics (loop_buffer.hpp): one subtract/add only, so
// a transiently out-of-ring pos (window shrink, #166) re-rings one len per
// call — do NOT "fix" this into a while loop. len == 0 degenerates to
// "subtract 0, report wrapped" exactly like the double code (callers guard
// the empty state).
IGB_FAST_INLINE bool q32_advance_wrap(q32_t& pos, q32_t rate, q32_t len) {
  pos += rate;
  if (pos >= len) {
    pos -= len;
    return true;
  } else if (pos < 0) {
    pos += len;
    return true;
  }
  return false;
}

// Single-wrap for derived read positions (readLoopAhead semantics).
// Precondition: pos within ±1 len of the ring (|ahead + offset| < len for
// in-ring bases; out-of-ring transients stay out-of-ring, as in the double
// path — downstream _winIdx handles them).
IGB_FAST_INLINE q32_t q32_wrap_once(q32_t pos, q32_t len) {
  if (pos >= len) return pos - len;
  if (pos < 0)    return pos + len;
  return pos;
}

// Linear-interp tap pair for a window-relative read position (docs/198 §9.2;
// pure so the Phase 0 harness can verify the index math against the frozen
// double reference before LoopBufferStereo is converted — the Phase 1
// _interpTapsQ method is a thin wrapper passing _wl_q).
struct InterpTapsQ {
  uint32_t i0;   // window-relative index of the first tap
  uint32_t i1;   // partner tap; wraps to 0 at the (possibly FRACTIONAL) window end
  float t;       // interpolation coefficient [0, 1)
};

// Mirrors readLoopAhead's index math (loop_buffer.hpp): the partner wraps
// where the fractional window ends — i1 == 0 once (i0 + 1) would reach or
// exceed win_len — so the seam interpolates window-end -> window-start.
// Precondition: read_pos_q >= 0 (post q32_wrap_once). A transiently
// past-the-window position (#166) yields i0 past the window with i1 == 0,
// identical to the double path; _winIdx tolerates both.
IGB_FAST_INLINE InterpTapsQ q32_interp_taps(q32_t read_pos_q, q32_t win_len_q) {
  InterpTapsQ taps;
  taps.i0 = q32_idx(read_pos_q);
  taps.t  = q32_frac_to_float(read_pos_q);
  uint32_t next = taps.i0 + 1u;
  taps.i1 = ((q32_t)((uint64_t)next << 32) >= win_len_q) ? 0u : next;
  return taps;
}

// ---- cross-context 64-bit access (docs/198 §3.2) ----
// The ONLY approved PRIMASK use in the codebase: a save/restore bracket
// around a single 64-bit DTCM/SRAM access, so the main loop can never
// observe or publish a torn q32 value across main-loop <-> audio-IRQ.
// Rules (docs/198 §3.2 — enforced by keeping the bracket INSIDE these two
// helpers only):
//   - nothing else goes inside the bracket (no SDRAM access, no libcalls);
//   - call sites are exactly the §3.2 audit table; IRQ-context code uses
//     plain accesses (the audio IRQ is never preempted);
//   - raw __disable_irq() / cpsid at call sites is forbidden.
// Host builds fall back to plain access (concurrency is not host-testable;
// covered by on-device checks, docs/198 §6-e).
#if defined(__arm__)
IGB_FAST_INLINE q32_t q32_atomic_load(const q32_t& src) {
  uint32_t pm;
  asm volatile("mrs %0, PRIMASK\n\tcpsid i" : "=r"(pm) : : "memory");
  q32_t v = src;
  asm volatile("msr PRIMASK, %0" : : "r"(pm) : "memory");
  return v;
}
IGB_FAST_INLINE void q32_atomic_store(q32_t& dst, q32_t v) {
  uint32_t pm;
  asm volatile("mrs %0, PRIMASK\n\tcpsid i" : "=r"(pm) : : "memory");
  dst = v;
  asm volatile("msr PRIMASK, %0" : : "r"(pm) : "memory");
}
#else
IGB_FAST_INLINE q32_t q32_atomic_load(const q32_t& src) { return src; }
IGB_FAST_INLINE void q32_atomic_store(q32_t& dst, q32_t v) { dst = v; }
#endif

}
