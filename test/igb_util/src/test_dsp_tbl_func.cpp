#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <igb_util/dsp/dsp_tbl_func.hpp>
#include <igb_util/math.hpp>

TEST_CASE("dsp table test") {
  SECTION("dsp_sin") {
    double avg_error = 0.0;
    double max_error = 0.0;
    float max_error_pos = 0.0f;
    uint32_t test_size = 10000;

    for (uint32_t i = 0; i < test_size; ++i) {
      float pos = ((float)i / (float)test_size) * 4.0f - 2.0f;
      float diff = std::abs(igb::dsp_sin(pos) - std::sin(pos * 2.0f * igb::numbers::pi));
      if (diff > max_error) {
        max_error = diff;
        max_error_pos = pos;
      }
      avg_error += diff;
    }

    std::cout << "dsp_sin[" << igb::dsp_func_tbl_size << "] test: avg_error = " << (avg_error / (float)test_size) ;
    std::cout << "; max_error = " << max_error << "; max_error_pos = " << max_error_pos << std::endl; 
  }

  SECTION("dsp_cos") {
    double avg_error = 0.0;
    double max_error = 0.0;
    float max_error_pos = 0.0f;
    uint32_t test_size = 10000;

    for (uint32_t i = 0; i < test_size; ++i) {
      float pos = ((float)i / (float)test_size) * 4.0f - 2.0f;
      float diff = std::abs(igb::dsp_cos(pos) - std::cos(pos * 2.0f * igb::numbers::pi));
      if (diff > max_error) {
        max_error = diff;
        max_error_pos = pos;
      }
      avg_error += diff;
    }

    std::cout << "dsp_cos[" << igb::dsp_func_tbl_size << "] test: avg_error = " << (avg_error / (float)test_size) ;
    std::cout << "; max_error = " << max_error << "; max_error_pos = " << max_error_pos << std::endl; 
  }
}
