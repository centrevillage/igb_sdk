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

  void process(std::pair<float, float> signals) {
    levels = {
      ef.process(ef_ctx[0], signals.first),
      ef.process(ef_ctx[1], signals.second)
    };
  }

  std::pair<float, float> getLevels() const {
    return levels;
  }
};

}  // namespace igb::dsp
