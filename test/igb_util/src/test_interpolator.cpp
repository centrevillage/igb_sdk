#include <catch2/catch_test_macros.hpp>

#include <igb_util/interpolator.hpp>

TEST_CASE("intp_linear_f") {
  const size_t tbl_size = 256;
  float tbl[tbl_size];

  for (size_t i = 0; i < tbl_size; ++i) {
    tbl[i] = i;
  }

  float v = igb::intp_linear_f(tbl, tbl_size, 100.8);
  REQUIRE(std::abs(v - 100.8f) < 0.00001f);

  v = igb::intp_linear_f(tbl, tbl_size, 255.5);
  REQUIRE(std::abs(v - 127.5f) < 0.00001f);
}

TEST_CASE("intp_linear_stereo_f") {
  const size_t tbl_size = 256;
  float tbl[tbl_size][2];

  for (size_t i = 0; i < tbl_size; ++i) {
    tbl[i][0] = i;
    tbl[i][1] = (tbl_size - 1) - i;
  }

  auto [l, r] = igb::intp_linear_stereo_f(&(tbl[0][0]), tbl_size, 100.8);
  REQUIRE(std::abs(l - 100.8f) < 0.00001f);
  REQUIRE(std::abs(r - 154.2f) < 0.00001f);
}

TEST_CASE("intp_hermite_f") {
  const size_t tbl_size = 256;
  float tbl[tbl_size];

  for (size_t i = 0; i < tbl_size; ++i) {
    tbl[i] = i;
  }

  float v = igb::intp_hermite_f(tbl, tbl_size, 100.8);
  REQUIRE(std::abs(v - 100.8f) < 0.00001f);

  v = igb::intp_hermite_f(tbl, tbl_size, 255.5);
  REQUIRE(std::abs(v - 127.5f) < 0.00001f);
}

TEST_CASE("intp_hermite_stereo_f") {
  const size_t tbl_size = 256;
  float tbl[tbl_size][2];

  for (size_t i = 0; i < tbl_size; ++i) {
    tbl[i][0] = i;
    tbl[i][1] = (tbl_size - 1) - i;
  }

  auto [l, r] = igb::intp_hermite_stereo_f(&(tbl[0][0]), tbl_size, 100.8);
  REQUIRE(std::abs(l - 100.8f) < 0.00001f);
  REQUIRE(std::abs(r - 154.2f) < 0.00001f);
}
