#pragma once

#include <algorithm>
#include <utility>
#include <cmath>

namespace igb {

template<typename PosType = float>
float intp_linear_f(const float* tbl, size_t tbl_size, PosType pos) {
  uint32_t idx = pos;
  float rate = pos - (PosType)idx;
  float end_v = tbl[(idx + 1) % tbl_size];
  float start_v = tbl[idx];
  return std::lerp(start_v, end_v, rate);
}

template<typename PosType = float>
std::pair<float, float> intp_linear_stereo_f(const float* tbl /* tbl[tbl_size][2] */, size_t tbl_size, PosType pos) {
  uint32_t idx = pos;
  float rate = pos - (PosType)idx;
  uint32_t start_idx = idx * 2;
  uint32_t end_idx  = ((idx + 1) % tbl_size) * 2;
  return {
    std::lerp(tbl[start_idx], tbl[end_idx], rate),
    std::lerp(tbl[start_idx+1], tbl[end_idx+1], rate)
  };
}

// Hermite Interpolation
// https://github.com/pichenettes/stmlib/blob/master/dsp/dsp.h
//
// Copyright 2012 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
template<typename PosType = float>
float intp_hermite_f(const float* tbl, size_t tbl_size, PosType pos) {
  uint32_t idx = pos;
  float rate = pos - (PosType)idx;
  const float xm1 = tbl[(idx - 1 + tbl_size) % tbl_size];
  const float x0 = tbl[idx + 0];
  const float x1 = tbl[(idx + 1) % tbl_size];
  const float x2 = tbl[(idx + 2) % tbl_size];
  const float c = (x1 - xm1) * 0.5f;
  const float v = x0 - x1;
  const float w = c + v;
  const float a = w + v + (x2 - x0) * 0.5f;
  const float b_neg = w + a;
  const float f = rate;
  return (((a * f) - b_neg) * f + c) * f + x0;
}

template<typename PosType = float>
std::pair<float, float> intp_hermite_stereo_f(const float* tbl /* tbl[tbl_size][2] */, size_t tbl_size, PosType pos) {
  uint32_t idx = pos;
  float rate = pos - (PosType)idx;
  float results[2] = {0.0f, 0.0f};
  for (uint32_t i = 0; i < 2; ++i) {
    const float xm1 = tbl[(((idx - 1 + tbl_size) % tbl_size) * 2) + i];
    const float x0 = tbl[((idx + 0) * 2) + i];
    const float x1 = tbl[(((idx + 1) % tbl_size) * 2) + i];
    const float x2 = tbl[(((idx + 2) % tbl_size) * 2) + i];
    const float c = (x1 - xm1) * 0.5f;
    const float v = x0 - x1;
    const float w = c + v;
    const float a = w + v + (x2 - x0) * 0.5f;
    const float b_neg = w + a;
    const float f = rate;
    results[i] = (((a * f) - b_neg) * f + c) * f + x0;
  }

  return {results[0], results[1]};
}

}
