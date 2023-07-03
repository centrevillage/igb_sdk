#pragma once

#include <cmath>
#include <igb_util/math.hpp>
#include <igb_util/dsp/config.hpp>
#include <igb_util/dsp/math.hpp>

namespace igb::dsp {

struct EnvFollower {
  float attack_coeff = 1.0f;
  float release_coeff = 1.0f;

  struct Context {
    float y1 = 0.0f;
  };

  void init(float attack_time, float release_time) {
    changeAttack(attack_time);
    changeRelease(release_time);
  }

  void changeAttack(float attack_time) {
    attack_coeff = igb::dsp::tau2pole(attack_time);
  }

  void changeRelease(float release_time) {
    release_coeff = igb::dsp::tau2pole(release_time);
  }

  float process(Context& ctx, float x) {
    x = std::abs(x);
    float coeff = (x > ctx.y1) ? attack_coeff : release_coeff;
    ctx.y1 = (1.0f - coeff) * x + coeff * ctx.y1;
    return ctx.y1;
  }
};

}
