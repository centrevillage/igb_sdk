#pragma once

#include <cmath>
#include <igb_util/math.hpp>

template<uint32_t sampling_rate>
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
    attack_coeff = igb::tau2pole(attack_time, sampling_rate);
  }

  void changeRelease(float release_time) {
    release_coeff = igb::tau2pole(release_time, sampling_rate);
  }

  float process(Context& ctx, float x) {
    x = std::abs(x);
    float coeff = (x > ctx.y1) ? attack_coeff : release_coeff;
    ctx.y1 = (1.0f - coeff) * x + coeff * ctx.y1;
    return ctx.y1;
  }
};
