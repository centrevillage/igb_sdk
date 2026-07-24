#pragma once

#include <cstdint>
#include <utility>
#include <igb_util/macro.hpp>
#include <igb_util/dsp/q32_pos.hpp>

namespace igb::dsp {

// LilaC issue #200 (Phase B): granular time-stretch renderer.
//
// Synthesis-hop granular with a constant TWO overlapping grains at 50%
// overlap under a Hann window — which is mathematically an equal-sum
// crossfade between the outgoing and incoming grain, so it is implemented as
// exactly that: one envelope fetch + two interp reads + a lerp per sample
// (docs/200_granular_engine_design.md §1).
//
// Division of labor (docs/198 §9.3):
//   - transport: the OWNING code advances buf.pos_q via movePos() at the
//     transport rate (tempo ratio + PLL mod, reverse-signed). This engine
//     never advances the transport; call renderIo() BEFORE movePos(), the
//     same convention as readLoop()/readLoopAhead().
//   - grains: window-relative read heads owned here, reading through the
//     state-free interp core buf._readInterpQ() after one _syncWin() per
//     frame. Grain rate = the user pitch (±2^(st/12)), captured at spawn so
//     a pitch edit lands click-free on the next grain (LilaC Q9).
//
// Phase alignment (design §3): a grain spawned at output time t0 has its
// Hann centroid at t0 + L/2 reading anchor + p·L/2, while the transport sits
// at pos(t0) + r·L/2. anchor = pos + (r − p)·L/2 makes the perceived source
// position track pos exactly — at r == p the engine degenerates to a sample-
// exact passthrough of the direct read (the host-test invariant).
template <typename LoopBuf>
struct GranularStretch {
  // Tunable (design §8): grain length L in 48k frames; hop = L/2. Even, and
  // small against the minimum play window (1 step) so the spawn lead stays
  // a fraction of the ring.
  constexpr static uint32_t default_grain_len = 1024;   // ~21.3 ms @48k

  struct Grain {
    q32_t src = 0;      // window-relative read pos (Q32.32)
    q32_t rate = 0;     // per 48k frame, signed (reverse reads backwards)
    bool active = false;
  };

  Grain _out_g;          // fading out (weight 1−e)
  Grain _in_g;           // fading in  (weight e)
  uint32_t _k = 0;       // 0..hop−1: crossfade phase
  uint32_t _hop = default_grain_len / 2;
  float _env_step = 256.0f / (float)(default_grain_len / 2);
  // Next spawn's rate. Written by the main loop (setPitch), read by the audio
  // context (_spawn) — a 64-bit cross-context pair, so the writer goes
  // through q32_atomic_store (#198 §3.2 PRIMASK discipline; plain read on
  // the IRQ side).
  alignas(8) q32_t _pitch_q = q32_one;

  // Edit-time (main loop): grains in flight keep their captured rate — a
  // pitch change lands click-free on the next spawn (LilaC Q9).
  void setPitch(q32_t p) { q32_atomic_store(_pitch_q, p); }

  // --- WSOLA alignment search (design §9) ------------------------------------
  // Plain OLA splices grains at arbitrary phase: the two live grains read
  // positions offset by a constant (r−p)·hop, which on real hardware (PLL-
  // wobbled r) is a few samples even at pitch ±0 — a hop-rate-modulated
  // 2-tap comb ("gritty" broadband noise, worst near the original pitch) —
  // and periodic material splices out of phase every hop (hop-rate sidebands
  // on every harmonic). The fix is the "S" of WSOLA: pick the spawn anchor
  // within ±wsola_half_range that best continues what the outgoing grain is
  // about to play.
  //
  // Budget discipline (v2, after the first device spike): SDRAM reads are
  // the scarce resource — one 125 µs audio IRQ carries SIX loop frames × four
  // tracks, so even "a few reads per frame" multiplies by 24. The candidate
  // region is therefore copied ONCE into the SRAM _scratch (a handful of
  // SDRAM reads per frame), and the AMDF scan runs entirely on the scratch
  // (cheap core-local float ops, also micro-budgeted). Two-stage scan
  // (coarse stride-2, then a ±2 stride-1 refine) keeps the op count small at
  // 1-sample final resolution. Stretch tracks ban OD, so the buffer content
  // is immutable during playback and the look-ahead copy is race-free (a
  // reseed cancels the search).
  //
  // Acceptance rule: the aligned anchor is used only when its AMDF beats the
  // nominal anchor's by wsola_accept_num — periodic/near-passthrough content
  // aligns (the artifact cases), while aperiodic material (where the edge of
  // the window would win a meaningless linear race, e.g. ramps) falls back
  // to the phase-exact nominal anchor. A mild multiplicative center bias
  // breaks periodic ties toward the nominal.
  bool wsola_enabled = true;                          // device A/B switch
  constexpr static uint32_t wsola_half_range = 96;    // ±samples (~2 ms)
  constexpr static uint32_t wsola_coarse_step = 2;    // coarse lag stride
  constexpr static uint32_t wsola_fine_reach = 2;     // refine best ±2 @ stride 1
  constexpr static uint32_t wsola_taps = 16;          // template length
  constexpr static float wsola_accept_num = 0.5f;     // best < 0.5 × nominal
  constexpr static float wsola_center_bias = 0.001f;  // per-sample tie penalty
  // Per-frame micro-budgets, per track (scaled up for tiny hops).
  // LilaC issue #208 (perf audit C1): 4/8 → 3/6 — the SAME total search work
  // spread over more frames (results identical, so quality-invariant; only
  // the per-IRQ worst-case slice shrinks). At the standard hop 512 the plan
  // still fits at scale 1: frames_at_1x 300 → 398, search_lead 308 → 406
  // ≤ hop. Tiny hops compress via _recalcSearchPlan exactly as before.
  constexpr static uint32_t wsola_fill_per_frame = 3;   // SDRAM reads
  constexpr static uint32_t wsola_taps_per_frame = 6;   // scratch AMDF taps
  // Scratch spans the candidate walk in BOTH directions (reverse grains read
  // backwards): taps·|p|max(4) margin on each side of the 2W lag range.
  constexpr static uint32_t wsola_scratch_len =
      2 * wsola_half_range + 2 * wsola_taps * 4 + 16;   // = 336

  // --- Wide pre-pass (LilaC issue #219: low-frequency splice rumble) --------
  // The grain gap (r−p)·hop folded mod the material period T lands outside
  // ±wsola_half_range whenever T is long (f0 under ~240 Hz) and the pitch
  // offset is big — every hop then splices out of phase (hop-rate "rumble").
  // Widening the fine search 4× would blow the lead ≤ hop plan, so a COARSE
  // pre-pass scans ±wsola_pre_half_range at wsola_pre_stride on a DECIMATED
  // scratch first, then the normal coarse/fine machinery refines
  // ±wsola_refine_reach around the pre winner at full resolution (reusing
  // _scratch). Long-period alignment tolerates the ±stride/2 pre quantization
  // (T=800: ±4 samples ≈ π/100 of phase — inaudible).
  //
  // Engagement: only when the gap exceeds wsola_half_range — below that the
  // needed correction ALWAYS folds inside the fine reach (T > 2·range → the
  // fold is the gap itself; T ≤ 2·range → fold ≤ T/2 ≤ range), so the classic
  // single-stage pipeline runs untouched (the #200 near-unity grit cases stay
  // bit-exact, pinned by test). The decimated metric sees aliasing above
  // fs/(2·stride); a misled pre winner fails the full-res acceptance and
  // falls back to the nominal anchor — never worse than the pre-#219 output.
  constexpr static uint32_t wsola_pre_stride = 8;        // scratch decimation
  constexpr static uint32_t wsola_pre_half_range = 400;  // ±samples (covers T≤800 folds)
  constexpr static uint32_t wsola_refine_reach = 8;      // full-res ± around pre winner
  // Pre-metric geometry (retuned after the first device listen — the noisy-
  // material probe showed broadband content dragging the AMDF off the
  // fundamental's alignment lattice): the template observes LONG (taps ×
  // tap_stride·stride·p = 320·p source samples ≈ T/5 at T=800, p=0.5) and
  // averages MORE taps, paid for by a coarser lag walk (lag_stride·stride =
  // 16 samples; winner accuracy ±8 = exactly wsola_refine_reach, so the
  // full-res refine still lands sample-exact).
  constexpr static uint32_t wsola_pre_taps = 20;         // pre template length
  constexpr static uint32_t wsola_pre_tap_stride = 2;    // decimated units per tap
  constexpr static uint32_t wsola_pre_lag_stride = 2;    // decimated units per lag
  static_assert(wsola_pre_lag_stride * wsola_pre_stride / 2 <= wsola_refine_reach,
                "refine reach must cover the pre winner quantization");
  // Search eligibility: every scratch margin assumes |p| ≤ 4, but bend+macro
  // stacking can push the effective pitch to ±48 st = ratio 16 (#216/#220)
  // where clipped taps read 0.0f and bias the metrics asymmetrically — the
  // search abstains entirely (nominal anchors) beyond ratio 4.
  constexpr static q32_t wsola_max_abs_pitch = q32_t(4) << 32;
  // Sweep tolerance (#219 v3): a pitch edit between search freeze and spawn
  // used to discard the result outright — a CONTINUOUS bend/macro sweep
  // (#216/#220 performance gestures) therefore ran with no alignment at
  // all. A small pitch delta only shifts the predicted anchor by ≈ Δp·hop
  // samples, so results within a bounded anchor error are kept instead:
  // ±2 samples on the classic path (its targets are few-sample PLL-grit
  // corrections), ±8 on the wide path (long-period phase tolerates it).
  // A slow bend (~4 st/s) moves ≈1.3 samples/hop — inside both bounds.
  constexpr static uint32_t wsola_sweep_tol_classic = 2;   // samples
  constexpr static uint32_t wsola_sweep_tol_wide = 8;      // samples
  constexpr static uint32_t wsola_pre_scratch_len =
      2 * wsola_pre_half_range / wsola_pre_stride
      + 2 * wsola_pre_taps * wsola_pre_tap_stride * 4 + 16;   // = 436
  constexpr static uint32_t wsola_refine_scratch_len =
      2 * (wsola_refine_reach + wsola_fine_reach) + 2 * wsola_taps * 4 + 16;  // = 164
  static_assert(wsola_refine_scratch_len <= wsola_scratch_len,
                "wide-path refine reuses _scratch");
  // The wide pre-pass predicts further ahead (±(pre range + template span));
  // require a window comfortably beyond that span or stay on the classic
  // path (short windows keep today's behavior).
  constexpr static uint32_t wsola_wide_min_win = 4096;

  enum class SearchPhase : uint8_t {
    idle, fill, ref_capture, coarse, fine, ready,
    pre_fill, pre_ref, pre_coarse, center_ref,   // wide-path extras (#219)
  };
  SearchPhase _sphase = SearchPhase::idle;
  float _scratch[wsola_scratch_len];   // mono (L+R) candidate region, SRAM
  float _ref[wsola_taps];              // outgoing grain's predicted continuation
  float _pre_scratch[wsola_pre_scratch_len];   // decimated wide region (#219)
  float _pre_ref[wsola_pre_taps];      // continuation, tap_stride·stride·p apart
  q32_t _scratch_base = 0;   // window-relative pos of _scratch[0] (integer q32)
  q32_t _cand0_off = 0;      // lag-0 candidate offset inside the scratch (q32)
  q32_t _pre_base = 0;       // window-relative pos of _pre_scratch[0] (integer)
  q32_t _anchor_pred = 0;    // frozen nominal-anchor prediction (#219)
  q32_t _ref_start = 0;      // frozen template start (outgoing grain @ hop)
  q32_t _fill_cursor = 0;
  q32_t _ref_cursor = 0;
  q32_t _search_rate = 0;    // p frozen at search start
  // Active-scan view consumed by _lagStep — the classic path sets it once in
  // idle; the wide path repoints it per stage (pre scan → full-res refine).
  const float* _scan_buf = nullptr;
  const float* _scan_ref = nullptr;
  q32_t _scan_cand0 = 0;     // lag-domain-center candidate offset (scan units)
  q32_t _tap_step = 0;       // cursor step per tap (scan units; pre: ×tap_stride)
  uint32_t _scan_taps = wsola_taps;   // template length of the active scan
  uint32_t _scan_len = 0;    // valid floats in _scan_buf (idx guard)
  uint32_t _scan_half = 0;   // lag value of the scan center
  uint32_t _scan_end = 0;    // last coarse lag
  uint32_t _scan_step = 0;   // coarse lag stride
  uint32_t _scan_unit = 1;   // source samples per lag unit (pre: stride)
  uint32_t _fine_max = 0;    // fine-lag upper clamp
  int32_t _bias_base = 0;    // scan-center distance from nominal (samples)
  uint32_t _fill_target = 0; // floats to fill into _scratch this stage
  bool _wide = false;        // this search runs the #219 wide pre-pass
  uint32_t _fill_idx = 0;
  uint32_t _ref_idx = 0;
  uint32_t _lag = 0;         // current lag (samples, 0..2W)
  uint32_t _fine_end = 0;
  uint32_t _tap_idx = 0;     // partial-lag resume point
  float _amdf_acc = 0.0f;
  // Schedule state — the defaults MUST equal what _recalcSearchPlan derives
  // for the default hop, so they come from the same _plan* maths below
  // (LilaC issue #208 C1: a stale hand-set lead default silently starved the
  // search when the per-frame budgets shrank — the engine runs on these
  // defaults in production, setGrainLen is a tuning-only entry).
  uint32_t _fill_pf = wsola_fill_per_frame * _planScale(default_grain_len / 2);
  uint32_t _taps_pf = wsola_taps_per_frame * _planScale(default_grain_len / 2);
  uint32_t _search_lead = _planLead(default_grain_len / 2);
  uint32_t _search_lead_wide = _planLeadWide(default_grain_len / 2);
  q32_t _sweep_tol_classic_q = _planSweepTol(wsola_sweep_tol_classic,
                                             default_grain_len / 2);
  q32_t _sweep_tol_wide_q = _planSweepTol(wsola_sweep_tol_wide,
                                          default_grain_len / 2);
  float _best_metric = 0.0f;
  float _center_metric = 0.0f;
  // Wide path (#219): the pre scan's own best/center pair. Its template is
  // longer and denser than the full-res one, so on noisy material it is the
  // BETTER-conditioned acceptance evidence — _spawn accepts when either
  // metric pair clears wsola_accept_num (a rejected wide hop falls back to
  // the nominal anchor, which in the wide regime is a guaranteed full-depth
  // splice error, so false rejection costs more than false acceptance).
  float _pre_best_metric = 0.0f;
  float _pre_center_metric = 0.0f;
  uint32_t _best_lag = 0;
  // Sub-sample refinement (#219 v3): raw (unbiased) AMDF of each fine-scan
  // lag, kept so _spawn can interpolate a fractional lag around the accepted
  // winner. Integer lags leave up to ±0.5 sample of splice error whose phase
  // impact scales with harmonic number — the "sizzle" on pitch offsets with
  // fractional grain gaps. Grain anchors are Q32.32, so applying the
  // fraction is free.
  float _fine_metrics[8] = {};
  uint32_t _fine_start = 0;
  float _last_amdf = 0.0f;

  // Tuning entry (spike/listening): resets the render state.
  void setGrainLen(uint32_t len) {
    if (len < 128) len = 128;
    len &= ~1u;                        // even → exact 50% hop
    _hop = len / 2;
    _env_step = 256.0f / (float)_hop;
    _recalcSearchPlan();
    reset();
  }
  uint32_t grainLen() const { return _hop * 2; }

  // Fit the amortized schedule into the frames before each hop; tiny hops
  // compress it by raising the per-frame budgets proportionally. constexpr
  // so the member defaults above are derived from the SAME maths (they are
  // the production values — setGrainLen/_recalcSearchPlan is tuning-only).
  constexpr static uint32_t _planFrames1x() {
    const uint32_t coarse_lags = wsola_half_range / wsola_coarse_step * 2 + 1;
    const uint32_t fine_lags = 2 * wsola_fine_reach + 1;
    return (wsola_scratch_len + wsola_fill_per_frame - 1) / wsola_fill_per_frame
         + (wsola_taps + wsola_fill_per_frame - 1) / wsola_fill_per_frame
         + ((coarse_lags + fine_lags) * wsola_taps + wsola_taps_per_frame - 1)
               / wsola_taps_per_frame
         + 8;
  }
  // Wide-path (#219) frame count: pre fill + pre template + decimated scan,
  // then refine fill + template + full-res center + the ±refine_reach scan.
  // At the default hop this is ~440 ≤ 512 with the SAME per-frame budgets —
  // reach ×4 paid in busy frames, not in per-IRQ cost.
  constexpr static uint32_t _planFrames1xWide() {
    const uint32_t pre_lags =
        2 * (wsola_pre_half_range / wsola_pre_stride) / wsola_pre_lag_stride + 1;
    const uint32_t refine_lags = wsola_refine_reach / wsola_coarse_step * 2 + 1;
    const uint32_t fine_lags = 2 * wsola_fine_reach + 1;
    return (wsola_pre_scratch_len + wsola_fill_per_frame - 1) / wsola_fill_per_frame
         + (wsola_pre_taps + wsola_fill_per_frame - 1) / wsola_fill_per_frame
         + (pre_lags * wsola_pre_taps + wsola_taps_per_frame - 1)
               / wsola_taps_per_frame
         + (wsola_refine_scratch_len + wsola_fill_per_frame - 1) / wsola_fill_per_frame
         + 2 * ((wsola_taps + wsola_fill_per_frame - 1) / wsola_fill_per_frame)
         + ((refine_lags + fine_lags) * wsola_taps + wsola_taps_per_frame - 1)
               / wsola_taps_per_frame
         + 8;
  }
  // Scale from the WIDE plan (the larger of the two) so tiny hops compress
  // both paths; at the default hop both scales are 1 (production unchanged).
  constexpr static uint32_t _planScale(uint32_t hop) {
    const uint32_t budget = (hop > 24) ? (hop - 16) : 8;
    return (_planFrames1xWide() + budget - 1) / budget;
  }
  constexpr static uint32_t _planLead(uint32_t hop) {
    const uint32_t lead = _planFrames1x() / _planScale(hop) + 8;
    return (lead > hop) ? hop : lead;
  }
  constexpr static uint32_t _planLeadWide(uint32_t hop) {
    const uint32_t lead = _planFrames1xWide() / _planScale(hop) + 8;
    return (lead > hop) ? hop : lead;
  }
  // Per-hop pitch-delta bound equivalent to `tol` samples of anchor error.
  constexpr static q32_t _planSweepTol(uint32_t tol_samples, uint32_t hop) {
    return ((q32_t)tol_samples << 32) / (q32_t)hop;
  }

  void _recalcSearchPlan() {
    // The wide plan must fit one default hop WITHOUT a budget raise — if a
    // tuning change breaks this, the lead cap would silently starve the
    // search (#208 C1 class). Checked here because in-class static_assert
    // cannot call member constexpr functions (incomplete class).
    static_assert(_planFrames1xWide() + 8 <= default_grain_len / 2,
                  "wide WSOLA plan must fit one hop at per-frame budget 1x");
    const uint32_t scale = _planScale(_hop);
    _fill_pf = wsola_fill_per_frame * scale;
    _taps_pf = wsola_taps_per_frame * scale;
    _search_lead = _planLead(_hop);
    _search_lead_wide = _planLeadWide(_hop);
    _sweep_tol_classic_q = _planSweepTol(wsola_sweep_tol_classic, _hop);
    _sweep_tol_wide_q = _planSweepTol(wsola_sweep_tol_wide, _hop);
  }

  // Reseed hook (docs/v2_timestretch §5): pos discontinuities (undo swap,
  // step-jump, stutter repin, window commit, loopset swap, …) drop the
  // in-flight grains; the next render spawns fresh and Hann-ramps in
  // (~hop/48k s) — click-free by construction. Cancels any in-flight
  // alignment search (its predictions are stale).
  void reset() {
    _out_g.active = false;
    _in_g.active = false;
    _k = 0;
    _sphase = SearchPhase::idle;
  }

  // One loop-frame render: emits the io-rate pair (canonical + half-step
  // sub-frame, issue #111 structure — state advances ONCE per call). Reads
  // buf.pos_q / buf.tape_speed_q directly (audio context, plain access).
  IGB_FAST_INLINE void renderIo(LoopBuf& buf, std::pair<float, float>* out2) {
    buf._syncWin();
    const q32_t wl = buf.winLenQ();
    if (wl <= 0) {
      out2[0] = {0.0f, 0.0f};
      out2[1] = {0.0f, 0.0f};
      return;
    }
    // Spawn/rotate at the FRAME HEAD so a fresh grain's first read happens
    // at the same pos its anchor was derived from — spawning at the frame
    // tail would lag the incoming grain by one transport step (a small but
    // measurable phase error the passthrough test catches).
    if (!_in_g.active) {
      _spawn(buf, wl);                    // first grain after reset()
    } else if (_k >= _hop) {              // hop: incoming becomes outgoing
      _out_g = _in_g;
      _spawn(buf, wl);
      _k = 0;
    }

    const float e0 = _env((float)_k * _env_step);
    const float e1 = _env(((float)_k + 0.5f) * _env_step);
    const auto o0 = _readG(buf, wl, _out_g, 0);
    const auto o1 = _readG(buf, wl, _out_g, 1);
    const auto i0 = _readG(buf, wl, _in_g, 0);
    const auto i1 = _readG(buf, wl, _in_g, 1);
    out2[0] = { (1.0f - e0) * o0.first  + e0 * i0.first,
                (1.0f - e0) * o0.second + e0 * i0.second };
    out2[1] = { (1.0f - e1) * o1.first  + e1 * i1.first,
                (1.0f - e1) * o1.second + e1 * i1.second };

    if (_out_g.active) _out_g.src = q32_wrap_once(_out_g.src + _out_g.rate, wl);
    _in_g.src = q32_wrap_once(_in_g.src + _in_g.rate, wl);
    ++_k;                                 // hop handled at the next frame head

    // Amortized WSOLA scan for the NEXT spawn (a few lags per frame; see the
    // member-block comment). Runs after the render so its SDRAM traffic sits
    // in the same budget slot every frame.
    if (wsola_enabled && _in_g.active && _k < _hop) _searchAdvance(buf, wl);
  }

  // --- internals -----------------------------------------------------------

  IGB_FAST_INLINE void _spawn(LoopBuf& buf, q32_t wl) {
    // anchor = pos + (r − p)·L/2 (design §3). |r−p|·L/2 can exceed one
    // window on extreme rate deltas × short windows, so wrap is a bounded
    // loop here (q32_wrap_once is a ±1-window helper).
    const q32_t nominal =
        buf.pos_q + (buf.tape_speed_q - _pitch_q) * (q32_t)_hop;
    q32_t anchor = nominal;
    q32_t dp = _pitch_q - _search_rate;
    if (dp < 0) dp = -dp;
    if (_sphase == SearchPhase::ready
        && dp <= (_wide ? _sweep_tol_wide_q : _sweep_tol_classic_q)) {
      // Acceptance rule (member-block comment): only a decisively better
      // splice replaces the phase-exact nominal anchor. _cand0_off points at
      // the scan center (classic: nominal; wide: the pre winner) and the
      // final scan's lag domain is centered at _scan_half on both paths.
      // Wide dual acceptance: the pre metric pair also qualifies (see the
      // _pre_best_metric member comment — a wide rejection is a guaranteed
      // full-depth splice error, so it needs only ONE convincing witness).
      bool ok = _best_metric < wsola_accept_num * _center_metric;
      if (!ok && _wide)
        ok = _pre_best_metric < wsola_accept_num * _pre_center_metric;
      if (ok) {
        // Sub-sample interp (#219 v3) from the fine scan's raw AMDF
        // neighbors. An L1 metric has a V-shaped minimum, so the exact
        // sub-lag is the piecewise-linear intersection, NOT the parabola
        // fit (which underestimates V minima). Only when both neighbors
        // were scanned; skipped at window edges and on degenerate shapes.
        // |p| ≥ 1 ONLY: below unity the template taps stride < 1 sample and
        // floor-read DUPLICATE source samples — the AMDF bottom flattens,
        // carries no sub-sample information, and the interp fits noise
        // (probe-measured: −7 st got 2× WORSE with it, +5 st 5× better).
        q32_t frac_q = 0;
        if ((_search_rate >= q32_one || _search_rate <= -q32_one)
            && _best_lag > _fine_start && _best_lag < _fine_end) {
          const uint32_t bi = _best_lag - _fine_start;
          if (bi + 1 < 8) {
            const float m0 = _fine_metrics[bi - 1];
            const float m1 = _fine_metrics[bi];
            const float m2 = _fine_metrics[bi + 1];
            // Symmetric V intersection + a depth gate. The AMDF minimum of
            // real (multi-harmonic) material is smooth, where the symmetric
            // form is the better estimator (probe: +5 st 0.021 vs 0.044 for
            // the two-slope variant). Its failure mode — an exactly-integer
            // minimum with asymmetric slopes manufactures a spurious
            // fraction (+12 st: 0.000 → 0.089) — is gated out by depth:
            // when m1 is already far below both neighbors the minimum IS
            // the integer lag, so the fraction is skipped.
            float fr = 0.0f;
            const float lo_n = (m0 < m2) ? m0 : m2;
            if (m1 > 0.05f * lo_n) {
              const float hi = (m0 > m2) ? m0 : m2;
              const float den = 2.0f * (hi - m1);
              if (den > 1e-20f) fr = (m0 - m2) / den;
            }
            fr = (fr > 0.5f) ? 0.5f : ((fr < -0.5f) ? -0.5f : fr);
            // Two-step float→q32: a single f64 cast would be an int64
            // libcall on the device (#198 discipline).
            frac_q = (q32_t)(int32_t)(fr * 65536.0f) * (q32_t)65536;
          }
        }
        anchor = _scratch_base + _cand0_off
               + ((q32_t)(_best_lag) << 32)
               - ((q32_t)_scan_half << 32)
               + frac_q;
      }
    }
    _sphase = SearchPhase::idle;   // consume; the next cycle re-arms
    _in_g.src = _wrapBounded(anchor, wl);
    _in_g.rate = _pitch_q;
    _in_g.active = true;
  }

  // Alignment-tap read from the loop buffer: nearest-sample mono sum (L+R).
  // Interp is unnecessary for AMDF alignment; the ±0.5-sample quantization
  // is far below the comb offsets being corrected.
  IGB_FAST_INLINE float _tapAt(const LoopBuf& buf, q32_t p, q32_t wl) const {
    q32_t w = _wrapBounded(p, wl);
    if (w < 0) w = 0;
    // Member reads through the pointer, NOT `auto v = *ptr`: the pair copy
    // constructs, and inside the out-of-line _searchAdvance the optimize
    // attribute boundary made GCC emit it as a CALL into flash (+veneer,
    // the #202 class — caught by the nm audit).
    const auto* v = buf.buf + buf._winIdx(q32_idx(w));
    return v->first + v->second;
  }

  // One AMDF lag evaluated against the active scan buffer, resumable mid-lag
  // via _tap_idx (the per-frame budget can be smaller than one lag).
  // Candidate tap positions are scan-relative: _scan_cand0 + (lag − center)
  // + k·p — in the decimated pre scan the SAME arithmetic applies because
  // one decimated unit per lag/tap equals stride source samples.
  IGB_FAST_INLINE bool _lagStep(uint32_t budget_taps) {
    const q32_t base = _scan_cand0
        + (((q32_t)_lag - (q32_t)_scan_half) << 32);
    q32_t cp = base + (q32_t)_tap_idx * _tap_step;
    const float* sbuf = _scan_buf;
    const float* ref = _scan_ref;
    const uint32_t slen = _scan_len;
    const uint32_t taps = _scan_taps;
    uint32_t done = 0;
    while (_tap_idx < taps && done < budget_taps) {
      const uint32_t idx = (uint32_t)(cp >> 32);
      const float d = ((idx < slen) ? sbuf[idx] : 0.0f) - ref[_tap_idx];
      _amdf_acc += (d < 0.0f) ? -d : d;
      cp += _tap_step;
      ++_tap_idx;
      ++done;
    }
    if (_tap_idx < taps) return false;   // resume next frame
    // Lag complete: score with the scale-free center bias. The distance is
    // measured from the NOMINAL anchor in source samples on every path
    // (_bias_base carries the pre winner's offset in the wide refine).
    int32_t off_i = _bias_base
        + ((int32_t)_lag - (int32_t)_scan_half) * (int32_t)_scan_unit;
    if (off_i < 0) off_i = -off_i;
    const float metric = _amdf_acc * (1.0f + wsola_center_bias * (float)off_i);
    // The classic path measures the nominal's own AMDF here; the wide path
    // gets its full-res center from the dedicated center_ref stage, and the
    // pre scan records its own (decimated) nominal metric for the dual
    // acceptance in _spawn. The refine scan records neither (its center lag
    // is the pre winner, not the nominal).
    if (_lag == _scan_half) {
      if (!_wide) _center_metric = _amdf_acc;
      else if (_sphase == SearchPhase::pre_coarse) _pre_center_metric = _amdf_acc;
    }
    if (metric < _best_metric) {
      _best_metric = metric;
      _best_lag = _lag;
    }
    _last_amdf = _amdf_acc;   // raw value for the fine-scan sub-sample interp
    _amdf_acc = 0.0f;
    _tap_idx = 0;
    return true;
  }

  // OUT-OF-LINE on purpose (#219): renderIo is force-inlined at two call
  // sites per track, and the wide-path state machine is big enough that
  // duplicating it there costs ~5KB of ITCM. One ITCM-resident copy plus a
  // BL is a few cycles against a 3-fill/6-tap frame budget — noise. IGB_ITCM
  // keeps it out of flash (no per-frame veneer, the #201 class); the inner
  // helpers (_tapAt/_lagStep) stay always_inline INSIDE this body.
  IGB_ITCM __attribute__((noinline, optimize("Ofast")))
  void _searchAdvance(LoopBuf& buf, q32_t wl) {
    switch (_sphase) {
      case SearchPhase::idle: {
        // #219 eligibility: every scratch margin assumes |p| ≤ 4 — beyond
        // that (bend+macro stacking, up to ratio 16) clipped taps would bias
        // the metrics, so the search abstains entirely (nominal anchors).
        const q32_t p = _pitch_q;
        if (p > wsola_max_abs_pitch || p < -wsola_max_abs_pitch) return;
        // #219 wide-path decision on the constant grain gap (r−p)·hop:
        // within ±wsola_half_range the fold always lands inside the fine
        // reach and the classic pipeline runs unchanged. Dual lead — the
        // longer wide lead applies only when the pre-pass engages, so
        // classic searches freeze on the same frame as before.
        const q32_t r = buf.tape_speed_q;
        q32_t gap = (r - p) * (q32_t)_hop;
        if (gap < 0) gap = -gap;
        // >= not >: a FRACTIONAL gap in [range, range+1) floors to `range`
        // yet its correction can exceed the classic reach — the probe caught
        // +3 st (gap 96.87) fully unaligned on the classic path.
        const bool wide = ((uint32_t)(gap >> 32) >= wsola_half_range)
            && ((uint32_t)(wl >> 32) >= wsola_wide_min_win);
        if (_hop - _k > (wide ? _search_lead_wide : _search_lead)) return;
        // Degenerate windows (shorter than the candidate span) skip the
        // search — real play windows are ≥ one step (thousands of samples).
        if ((uint32_t)(wl >> 32) < 2048) return;
        // Freeze the plan: predict where the outgoing grain (= the current
        // incoming one) and the nominal anchor will be at the hop. Content
        // is immutable during stretch playback (OD is gated off), so the
        // look-ahead reads stay valid; r may drift a hair over ≤10 ms, which
        // only biases the window the accepted lag was searched in.
        const q32_t remain = (q32_t)(_hop - _k);
        _search_rate = p;
        // Timing: _in_g.src is post-advance for THIS frame while pos_q is
        // pre-movePos, and the spawn-frame renderIo runs before that frame's
        // movePos — so the grain advances `remain` more times but pos
        // advances `remain + 1` times before the spawn reads them.
        _ref_start = _in_g.src + remain * _search_rate;
        _ref_cursor = _ref_start;
        _anchor_pred =
            buf.pos_q + (remain + 1) * r + (r - _search_rate) * (q32_t)_hop;
        _wide = wide;
        _fill_idx = 0;
        _ref_idx = 0;
        _lag = 0;
        _tap_idx = 0;
        _amdf_acc = 0.0f;
        _best_metric = 3.4e38f;
        _center_metric = 0.0f;
        if (wide) {
          // Decimated pre-scan setup: _pre_scratch covers anchor_pred ±
          // (pre range + template span). One decimated unit is
          // wsola_pre_stride source samples; the division is exact in q32
          // (anchor − base > 0), so candidate flooring stays grid-aligned.
          const q32_t plo = _anchor_pred
              - ((q32_t)(wsola_pre_half_range
                         + wsola_pre_taps * wsola_pre_tap_stride * 4
                               * wsola_pre_stride) << 32);
          _pre_base = (q32_t)((uint64_t)plo & ~0xFFFFFFFFull);
          _scan_cand0 =
              (q32_t)((uint64_t)(_anchor_pred - _pre_base) / wsola_pre_stride);
          _scan_buf = _pre_scratch;
          _scan_ref = _pre_ref;
          _scan_taps = wsola_pre_taps;
          _tap_step = _search_rate * (q32_t)wsola_pre_tap_stride;
          _scan_len = wsola_pre_scratch_len;
          _scan_half = wsola_pre_half_range / wsola_pre_stride;
          _scan_end = 2 * (wsola_pre_half_range / wsola_pre_stride);
          _scan_step = wsola_pre_lag_stride;
          _scan_unit = wsola_pre_stride;
          _bias_base = 0;
          _best_lag = _scan_half;
          _fill_cursor = _pre_base;
          _sphase = SearchPhase::pre_fill;
          return;
        }
        // Classic single-stage setup (#200) — data flow unchanged.
        // Scratch covers [anchor_pred − W − taps·4, … + W + taps·4]: the
        // integer-floored base keeps candidate flooring identical to the
        // direct-read path (passthrough AMDF must stay exactly 0).
        const q32_t lo = _anchor_pred
            - ((q32_t)(wsola_half_range + wsola_taps * 4) << 32);
        _scratch_base = (q32_t)((uint64_t)lo & ~0xFFFFFFFFull);
        _cand0_off = _anchor_pred - _scratch_base;   // ≥ 0 by construction
        _scan_cand0 = _cand0_off;
        _scan_buf = _scratch;
        _scan_ref = _ref;
        _scan_taps = wsola_taps;
        _tap_step = _search_rate;
        _scan_len = wsola_scratch_len;
        _scan_half = wsola_half_range;
        _scan_end = 2 * wsola_half_range;
        _scan_step = wsola_coarse_step;
        _scan_unit = 1;
        _fine_max = 2 * wsola_half_range;
        _bias_base = 0;
        _best_lag = wsola_half_range;
        _fill_cursor = _scratch_base;
        _fill_target = wsola_scratch_len;
        _sphase = SearchPhase::fill;
        return;
      }
      case SearchPhase::pre_fill: {
        for (uint32_t n = 0;
             n < _fill_pf && _fill_idx < wsola_pre_scratch_len; ++n) {
          _pre_scratch[_fill_idx++] = _tapAt(buf, _fill_cursor, wl);
          _fill_cursor += (q32_t)wsola_pre_stride << 32;
        }
        if (_fill_idx >= wsola_pre_scratch_len) _sphase = SearchPhase::pre_ref;
        return;
      }
      case SearchPhase::pre_ref: {
        // Template = the outgoing grain's continuation sampled every
        // stride·tap_stride output frames (source step = tap_stride decimated
        // units per tap, matching the pre scan's tap walk).
        for (uint32_t n = 0; n < _fill_pf && _ref_idx < wsola_pre_taps; ++n) {
          _pre_ref[_ref_idx++] = _tapAt(buf, _ref_cursor, wl);
          _ref_cursor +=
              _search_rate * (q32_t)(wsola_pre_stride * wsola_pre_tap_stride);
        }
        if (_ref_idx >= wsola_pre_taps) {
          _ref_idx = 0;                       // reused by ref_capture below
          _sphase = SearchPhase::pre_coarse;
        }
        return;
      }
      case SearchPhase::pre_coarse: {
        uint32_t budget = _taps_pf;
        while (budget > 0) {
          const uint32_t before = _tap_idx;
          if (!_lagStep(budget)) return;       // budget exhausted mid-lag
          budget -= (_scan_taps - before);
          if (_lag + _scan_step > _scan_end) {
            // Pre scan done → full-res refine around the winner, reusing
            // _scratch (wsola_refine_scratch_len ≤ wsola_scratch_len; the
            // _scan_len guard keeps stale floats beyond it unreadable).
            _pre_best_metric = _best_metric;   // dual-acceptance evidence
            _bias_base = ((int32_t)_best_lag - (int32_t)_scan_half)
                       * (int32_t)wsola_pre_stride;
            // (multiply, not <<: _bias_base is signed and a negative left
            // shift is UB before C++20)
            const q32_t center = _anchor_pred + (q32_t)_bias_base * q32_one;
            const q32_t lo = center
                - ((q32_t)(wsola_refine_reach + wsola_fine_reach
                           + wsola_taps * 4) << 32);
            _scratch_base = (q32_t)((uint64_t)lo & ~0xFFFFFFFFull);
            _cand0_off = center - _scratch_base;
            _scan_cand0 = _cand0_off;
            _scan_buf = _scratch;
            _scan_ref = _ref;
            _scan_taps = wsola_taps;
            _tap_step = _search_rate;
            _scan_len = wsola_refine_scratch_len;
            _scan_half = wsola_half_range;     // refine lag-domain center
            _scan_end = wsola_half_range + wsola_refine_reach;
            _scan_step = wsola_coarse_step;
            _scan_unit = 1;
            _fine_max =
                wsola_half_range + wsola_refine_reach + wsola_fine_reach;
            _best_metric = 3.4e38f;
            _best_lag = _scan_half;
            _lag = wsola_half_range - wsola_refine_reach;
            _tap_idx = 0;
            _amdf_acc = 0.0f;
            _fill_cursor = _scratch_base;
            _fill_idx = 0;
            _fill_target = wsola_refine_scratch_len;
            _ref_cursor = _ref_start;
            _sphase = SearchPhase::fill;
            return;
          }
          _lag += _scan_step;
        }
        return;
      }
      case SearchPhase::fill: {
        for (uint32_t n = 0; n < _fill_pf && _fill_idx < _fill_target; ++n) {
          _scratch[_fill_idx++] = _tapAt(buf, _fill_cursor, wl);
          _fill_cursor += q32_one;
        }
        if (_fill_idx >= _fill_target) _sphase = SearchPhase::ref_capture;
        return;
      }
      case SearchPhase::ref_capture: {
        for (uint32_t n = 0; n < _fill_pf && _ref_idx < wsola_taps; ++n) {
          _ref[_ref_idx++] = _tapAt(buf, _ref_cursor, wl);
          _ref_cursor += _search_rate;
        }
        if (_ref_idx >= wsola_taps)
          _sphase = _wide ? SearchPhase::center_ref : SearchPhase::coarse;
        return;
      }
      case SearchPhase::center_ref: {
        // Wide path only: the NOMINAL anchor's full-res AMDF via direct
        // reads (it may lie outside the refine scratch) — keeps the
        // acceptance semantics identical to the classic path (best <
        // accept_num × center, both full-res). Flooring matches a scratch
        // read: _tapAt floors, and scratch bases are integer q32.
        for (uint32_t n = 0; n < _fill_pf && _tap_idx < wsola_taps; ++n) {
          const float d =
              _tapAt(buf, _anchor_pred + (q32_t)_tap_idx * _search_rate, wl)
              - _ref[_tap_idx];
          _amdf_acc += (d < 0.0f) ? -d : d;
          ++_tap_idx;
        }
        if (_tap_idx < wsola_taps) return;
        _center_metric = _amdf_acc;
        _amdf_acc = 0.0f;
        _tap_idx = 0;
        _sphase = SearchPhase::coarse;
        return;
      }
      case SearchPhase::coarse: {
        uint32_t budget = _taps_pf;
        while (budget > 0) {
          const uint32_t before = _tap_idx;
          if (!_lagStep(budget)) return;       // budget exhausted mid-lag
          budget -= (_scan_taps - before);
          if (_lag + _scan_step > _scan_end) {
            // Coarse pass done → refine around the best (stride 1).
            const uint32_t b = _best_lag;
            _lag = (b > wsola_fine_reach) ? (b - wsola_fine_reach) : 0;
            _fine_end = b + wsola_fine_reach;
            if (_fine_end > _fine_max) _fine_end = _fine_max;
            _fine_start = _lag;
            _sphase = SearchPhase::fine;
            return;
          }
          _lag += _scan_step;
        }
        return;
      }
      case SearchPhase::fine: {
        uint32_t budget = _taps_pf;
        while (budget > 0) {
          const uint32_t before = _tap_idx;
          if (!_lagStep(budget)) return;
          budget -= (_scan_taps - before);
          const uint32_t fi = _lag - _fine_start;
          if (fi < 8) _fine_metrics[fi] = _last_amdf;
          if (_lag >= _fine_end) {
            _sphase = SearchPhase::ready;
            return;
          }
          ++_lag;
        }
        return;
      }
      case SearchPhase::ready:
        return;
    }
  }

  IGB_FAST_INLINE std::pair<float, float> _readG(const LoopBuf& buf, q32_t wl,
                                                 const Grain& g, uint32_t sub) const {
    if (!g.active) return {0.0f, 0.0f};
    // Sub-frame 1 reads half a rate step ahead (io oversample, #111) —
    // wrapped per read since the base src is only wrapped once per frame.
    q32_t p = (sub == 0) ? g.src : q32_wrap_once(g.src + (g.rate >> 1), wl);
    if (p < 0) p = 0;   // negative-saturate like readLoopAhead (#198)
    return buf._readInterpQ(p);
  }

  static q32_t _wrapBounded(q32_t v, q32_t len) {
    while (v >= len) v -= len;
    while (v < 0)    v += len;
    return v;
  }

  // LilaC issue #208 (perf audit B4): the envelope LUT pointer. Defaults to
  // the in-class constexpr table (.rodata — host/parity paths unchanged); on
  // the device the owner repoints it at a zero-wait TCM mirror of the same
  // values (the table is read ×2 per rendered frame, and an XIP-flash
  // .rodata read is D-cache-missable on the hot path). Wiring, not state —
  // reset() must not touch it; owners re-wire after reconstruction (the
  // WowFlutterFx tape-ring class of hooks).
  const float* _env_lut = hann_ramp_lut;
  void setEnvLut(const float* lut257) { _env_lut = lut257; }

  // Half-Hann rise 0→1 over index 0..256, linear-interpolated. Literal table
  // (bit-identical host/device, .rodata — no boot-time trig); reads go
  // through _env_lut (same values wherever it points, so still bit-exact).
  IGB_FAST_INLINE float _env(float idx256) const {
    if (idx256 <= 0.0f) return 0.0f;
    if (idx256 >= 256.0f) return 1.0f;
    const uint32_t i = (uint32_t)idx256;
    const float t = idx256 - (float)i;
    const float* lut = _env_lut;
    return lut[i] + t * (lut[i + 1] - lut[i]);
  }

  constexpr static float hann_ramp_lut[257] = {
  0.00000000f, 0.00003765f, 0.00015059f, 0.00033881f, 0.00060227f, 0.00094094f, 0.00135477f, 0.00184369f,
  0.00240764f, 0.00304651f, 0.00376023f, 0.00454868f, 0.00541175f, 0.00634929f, 0.00736118f, 0.00844726f,
  0.00960736f, 0.01084131f, 0.01214893f, 0.01353002f, 0.01498437f, 0.01651176f, 0.01811197f, 0.01978474f,
  0.02152983f, 0.02334698f, 0.02523591f, 0.02719634f, 0.02922797f, 0.03133049f, 0.03350360f, 0.03574696f,
  0.03806023f, 0.04044307f, 0.04289512f, 0.04541601f, 0.04800535f, 0.05066277f, 0.05338785f, 0.05618019f,
  0.05903937f, 0.06196495f, 0.06495650f, 0.06801357f, 0.07113569f, 0.07432240f, 0.07757322f, 0.08088765f,
  0.08426519f, 0.08770535f, 0.09120759f, 0.09477140f, 0.09839623f, 0.10208155f, 0.10582679f, 0.10963139f,
  0.11349477f, 0.11741637f, 0.12139558f, 0.12543180f, 0.12952444f, 0.13367286f, 0.13787646f, 0.14213459f,
  0.14644661f, 0.15081188f, 0.15522973f, 0.15969950f, 0.16422052f, 0.16879211f, 0.17341358f, 0.17808423f,
  0.18280336f, 0.18757026f, 0.19238420f, 0.19724448f, 0.20215035f, 0.20710107f, 0.21209590f, 0.21713409f,
  0.22221488f, 0.22733751f, 0.23250119f, 0.23770516f, 0.24294863f, 0.24823081f, 0.25355090f, 0.25890811f,
  0.26430163f, 0.26973064f, 0.27519434f, 0.28069188f, 0.28622245f, 0.29178522f, 0.29737934f, 0.30300398f,
  0.30865828f, 0.31434140f, 0.32005248f, 0.32579066f, 0.33155507f, 0.33734485f, 0.34315913f, 0.34899703f,
  0.35485766f, 0.36074016f, 0.36664362f, 0.37256717f, 0.37850991f, 0.38447095f, 0.39044938f, 0.39644431f,
  0.40245484f, 0.40848006f, 0.41451906f, 0.42057093f, 0.42663476f, 0.43270965f, 0.43879466f, 0.44488890f,
  0.45099143f, 0.45710134f, 0.46321772f, 0.46933963f, 0.47546616f, 0.48159639f, 0.48772939f, 0.49386423f,
  0.50000000f, 0.50613577f, 0.51227061f, 0.51840361f, 0.52453384f, 0.53066037f, 0.53678228f, 0.54289866f,
  0.54900857f, 0.55511110f, 0.56120534f, 0.56729035f, 0.57336524f, 0.57942907f, 0.58548094f, 0.59151994f,
  0.59754516f, 0.60355569f, 0.60955062f, 0.61552905f, 0.62149009f, 0.62743283f, 0.63335638f, 0.63925984f,
  0.64514234f, 0.65100297f, 0.65684087f, 0.66265515f, 0.66844493f, 0.67420934f, 0.67994752f, 0.68565860f,
  0.69134172f, 0.69699602f, 0.70262066f, 0.70821478f, 0.71377755f, 0.71930812f, 0.72480566f, 0.73026936f,
  0.73569837f, 0.74109189f, 0.74644910f, 0.75176919f, 0.75705137f, 0.76229484f, 0.76749881f, 0.77266249f,
  0.77778512f, 0.78286591f, 0.78790410f, 0.79289893f, 0.79784965f, 0.80275552f, 0.80761580f, 0.81242974f,
  0.81719664f, 0.82191577f, 0.82658642f, 0.83120789f, 0.83577948f, 0.84030050f, 0.84477027f, 0.84918812f,
  0.85355339f, 0.85786541f, 0.86212354f, 0.86632714f, 0.87047556f, 0.87456820f, 0.87860442f, 0.88258363f,
  0.88650523f, 0.89036861f, 0.89417321f, 0.89791845f, 0.90160377f, 0.90522860f, 0.90879241f, 0.91229465f,
  0.91573481f, 0.91911235f, 0.92242678f, 0.92567760f, 0.92886431f, 0.93198643f, 0.93504350f, 0.93803505f,
  0.94096063f, 0.94381981f, 0.94661215f, 0.94933723f, 0.95199465f, 0.95458399f, 0.95710488f, 0.95955693f,
  0.96193977f, 0.96425304f, 0.96649640f, 0.96866951f, 0.97077203f, 0.97280366f, 0.97476409f, 0.97665302f,
  0.97847017f, 0.98021526f, 0.98188803f, 0.98348824f, 0.98501563f, 0.98646998f, 0.98785107f, 0.98915869f,
  0.99039264f, 0.99155274f, 0.99263882f, 0.99365071f, 0.99458825f, 0.99545132f, 0.99623977f, 0.99695349f,
  0.99759236f, 0.99815631f, 0.99864523f, 0.99905906f, 0.99939773f, 0.99966119f, 0.99984941f, 0.99996235f,
  1.00000000f,
  };
};

}
