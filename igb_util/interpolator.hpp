#pragma once

#include <algorithm>
#include <utility>
#include <cmath>

namespace igb {

template<typename PosType = float>
float intp_linear_f(const float* tbl, size_t tbl_size, PosType v) {
  uint32_t idx = v;
  float rate = v - (PosType)idx;
  float end_v = tbl[(idx + 1) % tbl_size];
  float start_v = tbl[idx];
  return std::lerp(start_v, end_v, rate);
}

template<typename PosType = float>
std::pair<float, float> intp_linear_stereo_f(const float* tbl /* tbl[tbl_size][2] */, size_t tbl_size, PosType v) {
  uint32_t idx = v;
  float rate = v - (PosType)idx;
  uint32_t start_idx = idx * 2;
  uint32_t end_idx  = ((idx + 1) % tbl_size) * 2;
  return {
    std::lerp(tbl[start_idx], tbl[end_idx], rate),
    std::lerp(tbl[start_idx+1], tbl[end_idx+1], rate)
  };
}

}
