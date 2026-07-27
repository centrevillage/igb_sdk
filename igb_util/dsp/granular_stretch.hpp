#pragma once

#include <cstdint>
#include <utility>
#include <igb_util/macro.hpp>
#include <igb_util/dsp/q32_pos.hpp>
#include <igb_util/dsp/stage_gather.hpp>

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

  // LilaC #222: one-shot hand-off seed for a direct-read → grain engage
  // (varispeed pitch-mod). Consumed at the next renderIo head, where the
  // LIVE pos/r are captured in the audio context — a main-loop capture
  // would lag by up to one block and open a phase step at the switch.
  // Single-core bool cross-context write: plain store suffices.
  bool _seed_passthrough = false;
  void armPassthroughSeed() { _seed_passthrough = true; }

  // LilaC #222: constant read-anchor offset in source samples (Q32.32).
  // The direct path reads at pos + read_head_offset (LilaC #68, 16 samples
  // — scatter-write separation), while grains anchor on pos itself: an
  // instant direct↔grain switch therefore TIME-JUMPS the waveform by the
  // offset (the audible click the LilaC #222 probe pinned; the sync-flip
  // crossfade used to mask it). The vari-mod owner sets the same offset
  // here so seed/spawn/search all present the direct path's alignment and
  // the hand-off is continuous by construction. True stretch tracks keep 0
  // (the #200/#219 pins stay bit-exact). Written on engage transitions
  // only (main loop, atomic store; per-spawn plain read in the audio
  // context). Wiring-adjacent but resettable state: reset() keeps it — the
  // owner re-arms it on every engage edge.
  alignas(8) q32_t _anchor_off = 0;
  void setAnchorOffset(q32_t off) { q32_atomic_store(_anchor_off, off); }

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
  // LilaC issue #225 (stage 2): when a stage is served from staging its reads
  // are TCM loads, not SDRAM line fills, so the per-frame slice can be wider
  // for free — and the frames that buys are spent thinning the ONE stage that
  // cannot be staged. pre_ref's taps are rate·16 apart, so its span is up to
  // 64x its tap count and gathering it would move more traffic than it saves;
  // it stays on direct reads, at a third of the rate. Neither number changes
  // WHICH samples are read or in what order, so results stay bit-identical —
  // only the frame each read lands on moves.
  constexpr static uint32_t wsola_fill_per_frame_staged = 4;
  constexpr static uint32_t wsola_pre_ref_per_frame_staged = 1;
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

  // --- Search-phase stagger (LilaC issue #223 C2) --------------------------
  // Synchronized stretch tracks share hop and reseed instant, so every
  // instance walks the SAME search stage on the SAME frame — the per-IRQ cost
  // of any stage that still touches SDRAM (after #225 that is `pre_ref`, one
  // read/frame whose taps are 16·|p| samples apart = a fresh cache line at
  // every playback rate) multiplies by the track count on those frames, and
  // the max-hold reading is exactly what catches it.
  //
  // Fix: START each instance's search this many frames EARLIER than the plan
  // requires. Starting early can only ever GIVE the schedule frames (the
  // freeze predicts `remain = hop − k` frames ahead for any remain), so no
  // stage can starve; the search simply finishes and waits in `ready`. The
  // cost is a slightly longer constant-rate assumption between freeze and
  // spawn (48 frames ≈ 1 ms more), which the sweep tolerance already covers.
  //
  // Owner-set WIRING, not state: reset() must not clear it (the setEnvLut /
  // setStageHooks class). Default 0 = the pre-#223 schedule, so host pins
  // that don't set it stay bit-identical.
  uint32_t _search_stagger = 0;
  void setSearchStagger(uint32_t frames) { _search_stagger = frames; }
  // Clamped to the room the hop actually has (lead + 1 ≤ hop): a lead at or
  // past the hop would start the search on the spawn frame itself, which is
  // harmless but buys no separation. Tiny hops (planScale > 1) run leads
  // close to the hop and therefore stagger by less — or not at all.
  IGB_FAST_INLINE uint32_t _staggerFrames(uint32_t lead) const {
    const uint32_t room = (_hop > lead + 1) ? (_hop - lead - 1) : 0;
    return (_search_stagger < room) ? _search_stagger : room;
  }

  // --- Background staging of the fill reads (LilaC issue #225) ------------
  // The two FILL stages walk the loop buffer at a FIXED INTEGER stride from
  // a base frozen at plan time (pre_fill: stride 8 over the decimated wide
  // region; fill: stride 1 over the candidate region), which makes them a
  // pure gather — no speculation, every address known up front. On the
  // device that is handed to a DMA that can write TCM, so the reads leave
  // the audio IRQ's critical path entirely (the SDRAM line fills they cause
  // are the dominant term of the search's cost; see docs/225 §2).
  //
  // The engine stays platform-agnostic: an owner may install a provider,
  // and if none is installed — or it declines a request — the direct read
  // path below runs exactly as before. The staged data is a byte copy of
  // the same pairs in the same order, and the L+R sum is still taken here,
  // so a staged search is BIT-IDENTICAL to a direct one (the host tests pin
  // exactly that).
  //
  // Wiring, not state: reset() must not clear the hooks (the setEnvLut
  // class), only the in-flight request.
  // Wrapping splits a strided walk into at most (span/window + span/content
  // + 1) linear runs; the widest gather here spans 3488 samples against a
  // window of at least 2048, so 4 covers it with room. A walk that needs
  // more is simply not staged.
  constexpr static uint32_t stage_max_runs = 4;
  using StageT = StageHooks<std::pair<float, float>>;
  StageT* _stage = nullptr;
  void setStageHooks(StageT* hooks) { _stage = hooks; }
  // True while the CURRENT fill stage is being served from staging. Decided
  // once per request at plan time and never mid-stage: a half-staged stage
  // would put the SDRAM reads back one frame at a time, which is the cost
  // this whole mechanism exists to remove.
  bool _stage_active = false;
  // Whether THIS search's first fill was staged. The per-frame budgets that
  // depend on staging are chosen from it once, at freeze, so a stage that
  // later falls back cannot end up reading SDRAM at the wider staged rate.
  bool _staged_plan = false;
  // Base of the staged span for the reference stages (integer q32): a tap at
  // window-relative p reads staging[q32_idx(p - _ref_stage_base)].
  q32_t _ref_stage_base = 0;

  // --- Background staging of the GRAIN reads (LilaC issue #227) ------------
  // The render's own reads are the other half of the picture #225 opened. At
  // grain rate 4 (+24 st, the true worst) each read walks 32 B forward, so
  // every read owns a fresh D-cache line of which it uses 8-16 B: ~48 lines
  // per IRQ stream through the 16 KB cache and evict the MAIN LOOP's working
  // set, which is why the main loop measures ~30 µs slower there than its CPU
  // share predicts (#223 Phase 1). Reuse cannot fix that — the stride IS one
  // line per read — so the fix is to move the traffic to a DMA that writes
  // TCM, where the CPU allocates nothing.
  //
  // What makes this plannable: `Grain::rate` is frozen at _spawn and pitch
  // edits only land at the NEXT spawn, so a grain's read positions are the
  // exact arithmetic sequence src + k·rate for a whole block, known one block
  // ahead. Requests are therefore issued at the END of a block for the next
  // one (stageBlock), giving the transfer the IRQ-free remainder of the
  // period to land — one channel per slot, no ping-pong.
  //
  // Slots vs trajectories: at a hop renderIo does `_out_g = _in_g`, so the
  // incoming grain CONTINUES as the outgoing one — one arithmetic sequence
  // spanning both roles. The staging instance therefore follows the
  // trajectory (the _gs_out rotation below), and only the freshly spawned
  // grain — whose anchor the search decides at the spawn instant, after the
  // plan was made — falls back to direct reads for the rest of that block.
  //
  // Safety: a staged read is served ONLY when the request's window identity
  // still matches and the position maps inside the request. Anything else
  // (a window commit, a content swap, a not-yet-landed transfer, a span that
  // crosses the window wrap, a rate too fast for the buffer) falls back to
  // the direct read, so a stale or mislabelled buffer can never return the
  // wrong samples — it can only cost the fill it was meant to save.
  //
  // Span cap: at 6 frames the span is (5·|rate| + 2) samples, so 32 items
  // covers |rate| ≤ 6 — past the ±24 st worst the whole feature targets, and
  // past the point where the fetched span would move more bytes than the
  // demand misses it replaces. Beyond it (bend+macro stacking reaches ±48 st
  // = rate 16) the request is simply not made.
  constexpr static uint32_t grain_stage_max_items = 32;
  // A/B switch (LilaC #227). It gates the WHOLE feature, not just the
  // hardware request: with it off nothing is planned, so the resolved
  // pointers stay null and the render reads exactly as it did before #227.
  // That is what makes a device A/B a SAME-SESSION control — the only way to
  // separate a real cost from the ~6.7 µs of session-to-session drift. Owner
  // wiring, not state: reset() must not touch it.
  bool grain_stage_enabled = true;
  constexpr static uint8_t grain_slot_count = 2;   // outgoing / incoming
  StageT* _grain_stage[grain_slot_count] = { nullptr, nullptr };
  void setGrainStageHooks(uint8_t slot, StageT* hooks) {
    if (slot < grain_slot_count) _grain_stage[slot] = hooks;
  }
  // Rate floor (LilaC #227, device round 1): staging only pays when the reads
  // actually miss. A read takes 2 samples (16 B) and a cache line holds 4, so
  // a grain at rate r opens a new line every 4/r frames — at r = 1 the CPU
  // hits the same line for four frames and there is nothing to save, while
  // the plan still costs. Below this the request is not made at all.
  constexpr static q32_t grain_stage_min_rate = q32_t(2) << 32;
  // Per-INSTANCE request state (rotates with _gs_out, so it always describes
  // the trajectory the instance currently carries).
  q32_t _gs_base[grain_slot_count] = { 0, 0 };        // floored, wrapped
  uint32_t _gs_base_idx[grain_slot_count] = { 0, 0 };
  uint32_t _gs_count[grain_slot_count] = { 0, 0 };    // 0 = nothing staged
  // Window identity the request was planned against (see Safety above).
  size_t _gs_wstart[grain_slot_count] = { 0, 0 };
  q32_t _gs_wl[grain_slot_count] = { 0, 0 };
  // Resolved staging base, or nullptr while the request cannot serve reads
  // (not landed, window moved, nothing planned). Resolved ONCE PER FRAME by
  // _gsResolve — the per-READ path may then cost no more than a null test and
  // a range check (device round 1: doing the validation per read, through a
  // noinline helper, cost ~13 µs/block — MORE than the line fills it saved.
  // The four reads per frame sit inside the hottest loop in the engine; any
  // call there also breaks its register allocation, the #201 spike-2 shape).
  const std::pair<float, float>* _gs_ptr[grain_slot_count] = { nullptr, nullptr };
  uint8_t _gs_out = 0;   // instance serving the OUTGOING grain; in = ^1
  // Diagnostics: staged reads that had a plan but whose data had NOT landed
  // in time, so the direct read served them anyway. This is the "staging ran
  // but did not help" case #225 learned to make visible — it cannot be told
  // apart from "staging never ran" by a timing number alone. Incremented only
  // on that rare branch (never on the fast path), plain counter, not reset by
  // reset() (it is a measurement, not state).
  uint32_t gs_stat_late = 0;

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
  // LilaC #225 stage 2 companions of _fill_pf (see the constants above).
  uint32_t _fill_pf_staged =
      wsola_fill_per_frame_staged * _planScale(default_grain_len / 2);
  uint32_t _pre_ref_pf =
      wsola_pre_ref_per_frame_staged * _planScale(default_grain_len / 2);
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
  // LilaC #225 (stage 2): the wide plan's WORST staged mix — the first fill
  // staged (so pre_ref is thinned to 1/frame) but every later stage declining
  // and falling back to direct reads at the legacy rate. This must still fit
  // the SAME lead the legacy plan sets: the lead is a frozen contract, since
  // it fixes the search freeze point and moving that moves every alignment
  // result. Staging may only make the schedule shorter, never the lead.
  constexpr static uint32_t _planFramesStagedWorstWide() {
    const uint32_t pre_lags =
        2 * (wsola_pre_half_range / wsola_pre_stride) / wsola_pre_lag_stride + 1;
    const uint32_t refine_lags = wsola_refine_reach / wsola_coarse_step * 2 + 1;
    const uint32_t fine_lags = 2 * wsola_fine_reach + 1;
    return (wsola_pre_scratch_len + wsola_fill_per_frame_staged - 1)
               / wsola_fill_per_frame_staged
         + (wsola_pre_taps + wsola_pre_ref_per_frame_staged - 1)
               / wsola_pre_ref_per_frame_staged
         + (pre_lags * wsola_pre_taps + wsola_taps_per_frame - 1)
               / wsola_taps_per_frame
         + (wsola_refine_scratch_len + wsola_fill_per_frame - 1)
               / wsola_fill_per_frame
         + 2 * ((wsola_taps + wsola_fill_per_frame - 1) / wsola_fill_per_frame)
         + ((refine_lags + fine_lags) * wsola_taps + wsola_taps_per_frame - 1)
               / wsola_taps_per_frame
         + 8;
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
    // LilaC #225: staging must never need MORE lead than the legacy plan.
    static_assert(_planFramesStagedWorstWide()
                      <= _planLeadWide(default_grain_len / 2),
                  "staged wide schedule must fit the legacy search lead");
    const uint32_t scale = _planScale(_hop);
    _fill_pf = wsola_fill_per_frame * scale;
    _taps_pf = wsola_taps_per_frame * scale;
    _fill_pf_staged = wsola_fill_per_frame_staged * scale;
    _pre_ref_pf = wsola_pre_ref_per_frame_staged * scale;
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
    // LilaC #225: drop the in-flight staging request. This is a plain bool
    // store on purpose — reset() is reachable from the MAIN LOOP (the
    // reseedStretch hooks), and the provider owns hardware that only the
    // audio context may touch. An abandoned transfer just finishes into a
    // buffer nobody reads; the next begin() cleans up.
    _stage_active = false;
    // LilaC #227: the grains themselves are gone, so every staged trajectory
    // is stale. Same plain-store reasoning as above (main-loop reachable).
    _gsDrop(0);
    _gsDrop(1);
    // LilaC #222: a reseed invalidates an armed hand-off seed too — after a
    // pos jump the passthrough continuation is the WRONG trajectory; the
    // ramp-in above is the click-free entry. (Engage arms AFTER reset().)
    _seed_passthrough = false;
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
    _gsResolve(buf, wl);   // LilaC #227: per-frame, so the reads stay cheap
    // LilaC #222: consume an armed hand-off seed — the outgoing grain
    // becomes a passthrough continuation of the direct read (src = pos,
    // rate = r, full weight at k = 0: _env(0) == 0 so this frame's output
    // is bit-equal to the direct read), and the incoming grain spawns at
    // the frame head below, Hann-morphing r → p over one hop. This replaces
    // the dual-render crossfade for the varispeed engage at zero extra
    // render cost.
    if (_seed_passthrough) {
      _seed_passthrough = false;
      _out_g.src = _wrapBounded(buf.pos_q + _anchor_off, wl);
      _out_g.rate = buf.tape_speed_q;
      _out_g.active = true;
      _in_g.active = false;               // force the fresh spawn below
      _k = 0;
      _sphase = SearchPhase::idle;        // stale predictions (reset() rule)
      _gsDrop(0);                         // #227: both trajectories replaced
      _gsDrop(1);
    }
    // Spawn/rotate at the FRAME HEAD so a fresh grain's first read happens
    // at the same pos its anchor was derived from — spawning at the frame
    // tail would lag the incoming grain by one transport step (a small but
    // measurable phase error the passthrough test catches).
    if (!_in_g.active) {
      _spawn(buf, wl);                    // first grain after reset()
      // LilaC #227: nothing was planned for a grain that did not exist.
      _gsDrop(_gs_out ^ 1u);
    } else if (_k >= _hop) {              // hop: incoming becomes outgoing
      _out_g = _in_g;
      _spawn(buf, wl);
      _k = 0;
      // LilaC #227: staging follows the TRAJECTORY, not the slot — the
      // instance that carried the incoming grain now carries the outgoing
      // one (same src sequence, so its plan stays exactly valid), and the
      // freed instance has nothing for the grain just spawned (its anchor
      // was decided here, after the plan was made).
      _gs_out ^= 1u;
      _gsDrop(_gs_out ^ 1u);
    }

    const float e0 = _env((float)_k * _env_step);
    const float e1 = _env(((float)_k + 0.5f) * _env_step);
    // LilaC #224: at |rate| ≥ 4 (beyond ±24 st) the sub-frame read strides
    // onto its own D-cache line every frame (SDRAM loop buffer) — half of
    // the high-pitch fill traffic (device-measured +13.5 µs at ±48 st) buys
    // a ZOH'd sub-frame on a signal whose resampling imaging already
    // dominates at those ratios. Fills can NOT be hidden instead: the M7
    // has two linefill buffers and the render loop is track-major
    // (back-to-back frames), so demand misses keep the buffers saturated
    // and PLD hints are dropped (the #224 probe measured exactly zero).
    // LilaC #227: two read paths, chosen once per frame. With nothing staged
    // for this frame — every playback rate below the staging floor, i.e. the
    // ordinary case — the else branch is the pre-#227 code EXACTLY, so it
    // costs one predicted branch and nothing else: no extra loads in the
    // read, and its own register allocation (device round 2 measured ~5 µs
    // of allocation pressure when the two paths shared one body).
    // SCALARS, not std::pair values: the pair helpers are not always_inline,
    // and at this inline budget GCC outlines their ctor/assign to flash and
    // veneer-calls them per frame (the #202 class — the nm audit caught
    // exactly that on the first build of this branch). `g4` is never
    // address-taken, so it stays in registers.
    //
    // Device round 3: splitting this into a staged and a direct body (with
    // the staged one out-of-line) cost 4.5 µs of the staging advantage at
    // +24 st and recovered nothing at unity — handing eight floats across a
    // call forces them through memory. One body with the staged branch
    // inlined per read is the cheaper shape.
    const uint8_t inst_out = _gs_out;
    const uint8_t inst_in = (uint8_t)(_gs_out ^ 1u);
    float g4[8];   // o0 l/r, o1 l/r, i0 l/r, i1 l/r
    _readG(buf, wl, _out_g, 0, inst_out, g4[0], g4[1]);
    if (_subZoh(_out_g)) { g4[2] = g4[0]; g4[3] = g4[1]; }
    else _readG(buf, wl, _out_g, 1, inst_out, g4[2], g4[3]);
    _readG(buf, wl, _in_g, 0, inst_in, g4[4], g4[5]);
    if (_subZoh(_in_g)) { g4[6] = g4[4]; g4[7] = g4[5]; }
    else _readG(buf, wl, _in_g, 1, inst_in, g4[6], g4[7]);
    out2[0].first  = (1.0f - e0) * g4[0] + e0 * g4[4];
    out2[0].second = (1.0f - e0) * g4[1] + e0 * g4[5];
    out2[1].first  = (1.0f - e1) * g4[2] + e1 * g4[6];
    out2[1].second = (1.0f - e1) * g4[3] + e1 * g4[7];

    if (_out_g.active) _out_g.src = q32_wrap_once(_out_g.src + _out_g.rate, wl);
    _in_g.src = q32_wrap_once(_in_g.src + _in_g.rate, wl);
    ++_k;                                 // hop handled at the next frame head

    // Amortized WSOLA scan for the NEXT spawn (a few lags per frame; see the
    // member-block comment). Runs after the render so its SDRAM traffic sits
    // in the same budget slot every frame.
    if (wsola_enabled && _in_g.active && _k < _hop) _searchAdvance(buf, wl);
  }

  // --- internals -----------------------------------------------------------

  // noinline/ITCM (LilaC #227): a spawn happens once per HOP — 512 frames —
  // but renderIo is force-inlined at two call sites, so an inlined body puts
  // two copies of the acceptance test and the sub-sample interpolation in the
  // scarcest memory. The same trade #219 made for _searchAdvance.
  IGB_ITCM __attribute__((noinline))
  void _spawn(LoopBuf& buf, q32_t wl) {
    // anchor = pos + (r − p)·L/2 (design §3). |r−p|·L/2 can exceed one
    // window on extreme rate deltas × short windows, so wrap is a bounded
    // loop here (q32_wrap_once is a ±1-window helper).
    // LilaC #222: _anchor_off shifts the whole anchor lattice to the direct
    // path's read position (see the member comment); 0 in true stretch.
    const q32_t nominal =
        buf.pos_q + _anchor_off + (buf.tape_speed_q - _pitch_q) * (q32_t)_hop;
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

  // LilaC #225: ask the provider to stage a fill stage's whole gather. Sets
  // _stage_active for the stage; a false return leaves it clear and the
  // stage reads directly, on the unchanged per-frame budget.
  //
  // The window/content wrapping is resolved HERE (planStrideSegments applies
  // exactly the addressing _tapAt would), so the provider receives plain
  // pointers and never needs to know what a loop buffer is.
  // noinline: keeps the wrap solver from duplicating into the 4 call sites
  // inside ITCM-resident _searchAdvance (~1.6 KB of the scarcest memory).
  // ITCM (LilaC #225): this runs from the audio IRQ every few IRQs during
  // steady-state stretch playback — PERIODIC, not event-driven, so flash
  // placement is the #201 spike class (a cold QSPI I-fetch of its ~13 cache
  // lines costs ~5 us inside the IRQ). A flash-resident version of this very
  // function contaminated BOTH sides of the #225 device A/B and nearly
  // cemented the wrong architectural verdict — "called once every few IRQs"
  // is not a cold path; only event-driven code is.
  IGB_ITCM __attribute__((noinline))
  void _stageBegin(LoopBuf& buf, q32_t base, uint32_t stride,
                                   uint32_t count, q32_t wl) {
    _stage_active = false;
    if (!_stage || !_stage->begin || !_stage->win) return;
    // A background reader bypasses the D-cache, so it must not run while the
    // source may still hold dirty lines (LilaC #225 §6.5 — the owner cleans
    // and clears this from the main loop).
    if (buf.content_dirty) return;
    typename LoopBuf::StrideSegment segs[stage_max_runs];
    const uint32_t n =
        buf.planStrideSegments(base, stride, count, wl, segs, stage_max_runs);
    if (!n) return;
    StageRun<std::pair<float, float>> runs[stage_max_runs];
    for (uint32_t i = 0; i < n; ++i) {
      runs[i] = { buf.buf + segs[i].start_idx, stride, segs[i].count };
    }
    _stage_active = _stage->begin(_stage->ctx, runs, n, count);
  }

  // LilaC #227: plan the NEXT block's grain reads and hand them to the
  // providers. Called at the END of an audio block, after its frames were
  // rendered (see the member-block comment for why that timing lets one
  // channel per slot suffice).
  //
  // Every path out of here leaves the instances it did not stage invalidated,
  // so "no plan" always means "direct reads", never "stale plan".
  // ITCM/noinline on the device for the same reason _stageBegin is: this runs
  // on EVERY audio block (#201/#225 cold-I-fetch class).
  IGB_ITCM __attribute__((noinline))
  void stageBlock(LoopBuf& buf, uint32_t nframes) {
    // A plan that never resolved is the "staged but did not help" case — the
    // block ran on direct reads. Counted once per request, here, rather than
    // per read (the read path must stay branch-and-compare only).
    for (uint8_t i = 0; i < grain_slot_count; ++i) {
      if (_gs_count[i] && !_gs_ptr[i]) ++gs_stat_late;
      _gsDrop(i);
    }
    if (!grain_stage_enabled) return;
    if (!nframes || !_grain_stage[0] || !_grain_stage[1]) return;
    // A background reader bypasses the D-cache, so it must not run while the
    // source may still hold dirty lines (#225 §6.5 — the owner cleans and
    // clears this from the main loop).
    if (buf.content_dirty) return;
    buf._syncWin();
    const q32_t wl = buf.winLenQ();
    if (wl <= 0) return;
    // The hop fires at the head of the frame whose _k reaches _hop, so the
    // outgoing grain lives for `remain` more frames and the incoming one
    // covers the whole block (it continues as the outgoing grain past the
    // hop — the rotation in renderIo hands it the same staging instance).
    const uint32_t remain = (_k < _hop) ? (_hop - _k) : 0u;
    const uint32_t out_frames = (remain < nframes) ? remain : nframes;
    _stageGrain(buf, wl, _gs_out, _out_g, out_frames);
    _stageGrain(buf, wl, (uint8_t)(_gs_out ^ 1u), _in_g, nframes);
  }

  // Drop any staged plan (the owner calls this for a block the track will not
  // render through the engine, so a later block cannot inherit one).
  void dropGrainStage() {
    _gsDrop(0);
    _gsDrop(1);
  }

  // One grain's block span. `frames` may be shorter than the block (the
  // outgoing grain stops at the hop). noinline: two call sites, and the wrap
  // solver it inlines is ~0.5 KB of the scarcest memory (the #219 lesson).
  IGB_ITCM __attribute__((noinline))
  void _stageGrain(LoopBuf& buf, q32_t wl, uint8_t inst, const Grain& g,
                   uint32_t frames) {
    if (!frames || !g.active) return;
    // Rate floor: below it the CPU's own reads hit the cache and the plan
    // would be pure overhead (see grain_stage_min_rate).
    if (g.rate < grain_stage_min_rate && g.rate > -grain_stage_min_rate) return;
    // Extremes of this block's read positions, UNWRAPPED: src + k·rate for
    // k = 0..frames-1, plus the half-step sub-frame read where #224's ZOH
    // gate has not removed it. `rate >> 1` is exactly what _readG uses (an
    // arithmetic shift, so a reverse grain's half step floors the same way).
    const q32_t last = g.src + (q32_t)(frames - 1u) * g.rate
                     + (_subZoh(g) ? (q32_t)0 : (g.rate >> 1));
    const q32_t lo = (last < g.src) ? last : g.src;
    const q32_t hi = (last < g.src) ? g.src : last;
    // A span that crosses the window wrap is NOT staged. The staged index is
    // a plain `i0 - base_idx` subtraction; making it survive the wrap would
    // mean reasoning about a FRACTIONAL window length in index space, and the
    // failure mode of getting that wrong is silently reading the wrong
    // samples. A grain crosses the wrap once per window pass (thousands of
    // blocks), so the direct-read fallback costs nothing measurable.
    if (lo < 0 || hi >= wl) return;
    const q32_t base = (q32_t)((uint64_t)lo & ~0xFFFFFFFFull);
    // +1 to include the integer part of `hi`, +1 for its interpolation
    // partner (the direct read pairs i0 with i0+1 — see q32_interp_taps).
    const uint32_t count = q32_idx(hi - base) + 2u;
    if (count > grain_stage_max_items) return;    // rate too fast to be worth it
    typename LoopBuf::StrideSegment segs[stage_max_runs];
    const uint32_t n =
        buf.planStrideSegments(base, 1, count, wl, segs, stage_max_runs);
    if (!n) return;
    StageRun<std::pair<float, float>> runs[stage_max_runs];
    for (uint32_t i = 0; i < n; ++i) {
      runs[i] = { buf.buf + segs[i].start_idx, 1, segs[i].count };
    }
    StageT* h = _grain_stage[inst];
    if (!h->begin || !h->win) return;
    if (!h->begin(h->ctx, runs, n, count)) return;
    _gs_base[inst] = base;
    _gs_base_idx[inst] = q32_idx(base);
    _gs_wstart[inst] = buf._wstart;
    _gs_wl[inst] = wl;
    _gs_count[inst] = count;
  }

  // Drop one instance's plan (both halves together — a live pointer with a
  // dead count would be a contradiction waiting to be read).
  IGB_FAST_INLINE void _gsDrop(uint8_t inst) {
    _gs_count[inst] = 0;
    _gs_ptr[inst] = nullptr;
  }

  // Once per FRAME: decide, per instance, whether staging can serve this
  // frame's reads, and publish the base pointer the read path uses. Every
  // check that is not per-read lives here — window identity, the provider's
  // window, the poll — so the read itself is a null test plus a range check.
  //
  // Requiring the WHOLE request to have landed (not just the item in hand)
  // is deliberate: it is the natural granularity here (one request per grain
  // per block) and it collapses the per-read test to one comparison.
  IGB_FAST_INLINE void _gsResolve(const LoopBuf& buf, q32_t wl) {
    for (uint8_t i = 0; i < grain_slot_count; ++i) {
      if (_gs_ptr[i] || !_gs_count[i]) continue;   // resolved, or nothing to do
      // A window commit between plan and read remaps every index, so the
      // staged copy describes a different window and must be dropped.
      if (_gs_wl[i] != wl || _gs_wstart[i] != buf._wstart) { _gsDrop(i); continue; }
      StageT* h = _grain_stage[i];
      const StageWindow<std::pair<float, float>>* win = h->win;
      if (win->first != 0 || win->count < _gs_count[i]) {
        if (!h->poll) { _gsDrop(i); continue; }
        h->poll(h->ctx);
        if (win->first != 0 || win->count < _gs_count[i]) continue;  // retry next frame
      }
      _gs_ptr[i] = win->data;
    }
  }

  // LilaC #225 (stage 2): stage the SPAN the reference taps walk. Their step
  // is the pitch itself, so the walk is fractional — not a gather — but it
  // covers at most taps·|p|max + 1 = 61 contiguous samples, which is small
  // enough that fetching the span costs about what fetching the taps would.
  // (pre_ref is the opposite case and stays direct: its step is 16x larger.)
  // The base is floored, exactly like _scratch_base, so a staged tap index is
  // a plain q32_idx difference.
  IGB_FAST_INLINE void _stageRefSpan(LoopBuf& buf, q32_t base, q32_t wl) {
    _stage_active = false;
    if (!_staged_plan) return;   // legacy plan: legacy reads and rates
    const q32_t last = base + (q32_t)(wsola_taps - 1) * _search_rate;
    const q32_t lo = (last < base) ? last : base;
    const q32_t hi = (last < base) ? base : last;
    _ref_stage_base = (q32_t)((uint64_t)lo & ~0xFFFFFFFFull);
    _stageBegin(buf, _ref_stage_base, 1,
                q32_idx(hi - _ref_stage_base) + 1u, wl);
  }

  // One staged reference tap, or nullptr while it has not landed. Reverse
  // grains walk the span backwards, so this must not assume the index only
  // grows — which is why the provider is required to publish a request that
  // fits its chunk in a single window (see stage_gather.hpp).
  IGB_FAST_INLINE const std::pair<float, float>* _stagedTap(q32_t p) {
    // A provider that has already slid its window past the start of the span
    // can never serve a backwards walk again, so give staging up for this
    // stage and let the direct path finish it. Waiting instead would stall
    // until the hop and silently lose the search.
    if (_stage->win->first != 0) {
      _stage_active = false;
      return nullptr;
    }
    uint32_t avail = 0;
    return _stagedRun(q32_idx(p - _ref_stage_base), avail);
  }

  // Consecutive staged items readable from `i`, or nullptr if the provider
  // has not caught up yet (the stage then simply idles this frame — the
  // plan's slack absorbs it). At most ONE poll per call: the window is
  // plain memory, so the common case costs three loads and no call.
  IGB_FAST_INLINE const std::pair<float, float>* _stagedRun(uint32_t i,
                                                            uint32_t& avail) {
    const auto* win = _stage->win;
    if (i < win->first || i - win->first >= win->count) {
      if (!_stage->poll) return nullptr;
      _stage->poll(_stage->ctx);
      if (i < win->first || i - win->first >= win->count) return nullptr;
    }
    const uint32_t off = i - win->first;
    avail = win->count - off;
    return win->data + off;
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
        // LilaC #223 C2: + this instance's stagger (0 by default — see the
        // member comment). The lead is still the CONTRACT for how late a
        // search may start; the stagger only lets it start earlier.
        const uint32_t lead = wide ? _search_lead_wide : _search_lead;
        if (_hop - _k > lead + _staggerFrames(lead)) return;
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
        // LilaC #222: same _anchor_off as _spawn's nominal — the scan stays
        // centered on the lattice the spawn will actually use.
        _anchor_pred = buf.pos_q + _anchor_off
            + (remain + 1) * r + (r - _search_rate) * (q32_t)_hop;
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
          _stageBegin(buf, _pre_base, wsola_pre_stride, wsola_pre_scratch_len, wl);
          _staged_plan = _stage_active;
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
        _stageBegin(buf, _scratch_base, 1, _fill_target, wl);
        _staged_plan = _stage_active;
        _sphase = SearchPhase::fill;
        return;
      }
      case SearchPhase::pre_fill: {
        // LilaC #225: staged reads are the SAME pairs summed in the SAME
        // order as _tapAt — bit-identical, only the memory they come from
        // differs. When staging is active the direct path is not used as a
        // partial fallback (see _stage_active).
        if (_stage_active) {
          uint32_t avail = 0;
          const auto* s = _stagedRun(_fill_idx, avail);
          for (uint32_t n = 0; s && n < _fill_pf_staged && n < avail
                               && _fill_idx < wsola_pre_scratch_len; ++n) {
            _pre_scratch[_fill_idx++] = s->first + s->second;
            ++s;
            _fill_cursor += (q32_t)wsola_pre_stride << 32;
          }
        } else {
          for (uint32_t n = 0;
               n < _fill_pf && _fill_idx < wsola_pre_scratch_len; ++n) {
            _pre_scratch[_fill_idx++] = _tapAt(buf, _fill_cursor, wl);
            _fill_cursor += (q32_t)wsola_pre_stride << 32;
          }
        }
        if (_fill_idx >= wsola_pre_scratch_len) {
          _stage_active = false;
          _sphase = SearchPhase::pre_ref;
        }
        return;
      }
      case SearchPhase::pre_ref: {
        // Template = the outgoing grain's continuation sampled every
        // stride·tap_stride output frames (source step = tap_stride decimated
        // units per tap, matching the pre scan's tap walk).
        const uint32_t budget = _staged_plan ? _pre_ref_pf : _fill_pf;
        for (uint32_t n = 0; n < budget && _ref_idx < wsola_pre_taps; ++n) {
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
            _stageBegin(buf, _scratch_base, 1, _fill_target, wl);
            _ref_cursor = _ref_start;
            _sphase = SearchPhase::fill;
            return;
          }
          _lag += _scan_step;
        }
        return;
      }
      case SearchPhase::fill: {
        if (_stage_active) {
          uint32_t avail = 0;
          const auto* s = _stagedRun(_fill_idx, avail);
          for (uint32_t n = 0; s && n < _fill_pf_staged && n < avail
                               && _fill_idx < _fill_target; ++n) {
            _scratch[_fill_idx++] = s->first + s->second;
            ++s;
            _fill_cursor += q32_one;
          }
        } else {
          for (uint32_t n = 0; n < _fill_pf && _fill_idx < _fill_target; ++n) {
            _scratch[_fill_idx++] = _tapAt(buf, _fill_cursor, wl);
            _fill_cursor += q32_one;
          }
        }
        if (_fill_idx >= _fill_target) {
          _stageRefSpan(buf, _ref_start, wl);
          _sphase = SearchPhase::ref_capture;
        }
        return;
      }
      case SearchPhase::ref_capture: {
        const uint32_t budget = _stage_active ? _fill_pf_staged : _fill_pf;
        for (uint32_t n = 0; n < budget && _ref_idx < wsola_taps; ++n) {
          if (_stage_active) {
            const auto* s = _stagedTap(_ref_cursor);
            if (!s) break;                     // not landed yet: idle a frame
            _ref[_ref_idx++] = s->first + s->second;
          } else {
            _ref[_ref_idx++] = _tapAt(buf, _ref_cursor, wl);
          }
          _ref_cursor += _search_rate;
        }
        if (_ref_idx >= wsola_taps) {
          if (_wide) {
            _stageRefSpan(buf, _anchor_pred, wl);
            _sphase = SearchPhase::center_ref;
          } else {
            _stage_active = false;
            _sphase = SearchPhase::coarse;
          }
        }
        return;
      }
      case SearchPhase::center_ref: {
        // Wide path only: the NOMINAL anchor's full-res AMDF via direct
        // reads (it may lie outside the refine scratch) — keeps the
        // acceptance semantics identical to the classic path (best <
        // accept_num × center, both full-res). Flooring matches a scratch
        // read: _tapAt floors, and scratch bases are integer q32.
        const uint32_t budget = _stage_active ? _fill_pf_staged : _fill_pf;
        for (uint32_t n = 0; n < budget && _tap_idx < wsola_taps; ++n) {
          const q32_t p = _anchor_pred + (q32_t)_tap_idx * _search_rate;
          float v;
          if (_stage_active) {
            const auto* s = _stagedTap(p);
            if (!s) break;                     // not landed yet: idle a frame
            v = s->first + s->second;
          } else {
            v = _tapAt(buf, p, wl);
          }
          const float d = v - _ref[_tap_idx];
          _amdf_acc += (d < 0.0f) ? -d : d;
          ++_tap_idx;
        }
        if (_tap_idx < wsola_taps) return;
        _stage_active = false;
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

  // LilaC #224: sub-frame ZOH gate — see the renderIo read block. ±4.0 in
  // grain-rate units (samples/frame); the vari-mod engage seed carries the
  // TRANSPORT rate (|r| ≤ ~2.2), so hand-off parity is never gated.
  constexpr static q32_t subread_zoh_min_rate = q32_t(4) << 32;
  IGB_FAST_INLINE static bool _subZoh(const Grain& g) {
    return g.rate >= subread_zoh_min_rate || g.rate <= -subread_zoh_min_rate;
  }

  // `inst` = the staging instance carrying this grain's trajectory (LilaC
  // #227). The staged and direct paths differ ONLY in where the two samples
  // come from — same taps, same expression, same order — so a staged read is
  // bit-identical to _readInterpQ by construction, and the extra ITCM per
  // inlined copy is a null test, a subtraction and a compare.
  IGB_FAST_INLINE void _readG(const LoopBuf& buf, q32_t wl, const Grain& g,
                              uint32_t sub, uint8_t inst,
                              float& l, float& r) const {
    if (!g.active) { l = 0.0f; r = 0.0f; return; }
    // Sub-frame 1 reads half a rate step ahead (io oversample, #111) —
    // wrapped per read since the base src is only wrapped once per frame.
    q32_t p = (sub == 0) ? g.src : q32_wrap_once(g.src + (g.rate >> 1), wl);
    if (p < 0) p = 0;   // negative-saturate like readLoopAhead (#198)
    const InterpTapsQ taps = q32_interp_taps(p, wl);
    const std::pair<float, float>* v0;
    const std::pair<float, float>* v1;
    const std::pair<float, float>* st = _gs_ptr[inst];
    uint32_t j = 0;
    // Unsigned subtraction: a position BELOW the staged base wraps to a huge
    // index and fails the same bound, so one compare covers both ends.
    if (st && (j = taps.i0 - _gs_base_idx[inst]) + 1u < _gs_count[inst]) {
      v0 = st + j;
      v1 = st + j + 1u;
    } else {
      v0 = buf.buf + buf._winIdx(taps.i0);
      v1 = buf.buf + buf._winIdx(taps.i1);
    }
    // Member reads through the pointers, NOT pair copies (the #202 class:
    // std::pair's helpers are not always_inline and get outlined once the
    // ITCM caller runs out of inline budget).
    l = (1.0f - taps.t) * v0->first  + taps.t * v1->first;
    r = (1.0f - taps.t) * v0->second + taps.t * v1->second;
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
