#pragma once

#include <utility>
#include <igb_util/dsp/env_follower.hpp>

namespace igb::dsp {

struct LevelAnalyzer {
  EnvFollower ef;
  EnvFollower::Context ef_ctx[2];

  std::pair<float, float> levels = {0.0f, 0.0f};

  // attack_time / release_time are EnvFollower time constants (seconds). Defaults
  // preserve the original 1 ms / 1 ms behavior; callers can pass a slower release
  // for a smoother level-meter fall (see LilaCRepeater issue #157).
  void init(float attack_time = 0.001f, float release_time = 0.001f) {
    ef.init(attack_time, release_time);
  }

  // Explicit tick rate: process() need not run at the dsp Config rate — a
  // display meter fed once per audio BLOCK ticks at rate/block_frames and must
  // map the same seconds onto that cadence (LilaCRepeater issue #223).
  void init(float attack_time, float release_time, uint32_t sampling_rate) {
    ef.init(attack_time, release_time, sampling_rate);
  }

  void process(std::pair<float, float> signals) {
    processLR(signals.first, signals.second);
  }

  // Scalar entry for audio-IRQ callers (LilaCRepeater issue #223): building a
  // std::pair temporary at the call site is what drags the pair helpers — not
  // always_inline — out to flash once the ITCM caller exceeds GCC's inline
  // budget, and the IRQ then veneer-calls them (the #62 / #202 class).
  IGB_FAST_INLINE void processLR(float l, float r) {
    levels.first = ef.process(ef_ctx[0], l);
    levels.second = ef.process(ef_ctx[1], r);
  }

  std::pair<float, float> getLevels() const {
    return levels;
  }
};

}  // namespace igb::dsp
