#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <igb_util/dsp/dsp_tbl_func.hpp>
#include <igb_util/math.hpp>

TEST_CASE("dsp table test") {
  SECTION("sin_wave_bi") {
    double avg_error = 0.0;
    double max_error = 0.0;
    float max_error_pos = 0.0f;
    uint32_t test_size = 10000;

    for (uint32_t i = 0; i < test_size; ++i) {
      float pos = ((float)i / (float)test_size) * 4.0f - 2.0f;
      float diff = std::abs(igb::dsp::sin_wave_bi(pos) - std::sin(pos * 2.0f * igb::numbers::pi));
      if (diff > max_error) {
        max_error = diff;
        max_error_pos = pos;
      }
      avg_error += diff;
    }

    std::cout << "sin_wave_bi[" << igb::dsp::dsp_func_tbl_size << "] test: avg_error = " << (avg_error / (float)test_size) ;
    std::cout << "; max_error = " << max_error << "; max_error_pos = " << max_error_pos << std::endl; 
  }

  SECTION("cos_wave_bi") {
    double avg_error = 0.0;
    double max_error = 0.0;
    float max_error_pos = 0.0f;
    uint32_t test_size = 10000;

    for (uint32_t i = 0; i < test_size; ++i) {
      float pos = ((float)i / (float)test_size) * 4.0f - 2.0f;
      float diff = std::abs(igb::dsp::cos_wave_bi(pos) - std::cos(pos * 2.0f * igb::numbers::pi));
      if (diff > max_error) {
        max_error = diff;
        max_error_pos = pos;
      }
      avg_error += diff;
    }

    std::cout << "cos_wave_bi[" << igb::dsp::dsp_func_tbl_size << "] test: avg_error = " << (avg_error / (float)test_size) ;
    std::cout << "; max_error = " << max_error << "; max_error_pos = " << max_error_pos << std::endl; 
  }
}
