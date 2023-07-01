#pragma once

#include <igb_util/algorithm.hpp>

namespace igb {

struct Smoother {
  float coeff = 0.99f;

  struct Context {
    float y1 = 0.0f;
  };

  void setCoeff(float c) {
    coeff = igb::clamp(c, 0.0f, 1.0f);
  }

  float process(Context& ctx, float x0) {
    float y0 = igb::lerp(x0, ctx.y1, coeff);
    ctx.y1 = y0;
    return y0;
  }
};

}
