#pragma once

#include <utility>
#include <igb_util/dsp/env_follower.hpp>

namespace igb::dsp {

struct LevelAnalyzer {
  EnvFollower ef;
  EnvFollower::Context ef_ctx[2];

  std::pair<float, float> levels = {0.0f, 0.0f};

  void init() {
    ef.init(0.001f, 0.001f);
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
