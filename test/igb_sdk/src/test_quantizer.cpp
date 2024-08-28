#include <catch2/catch_test_macros.hpp>

#include <igb_sdk/util/quantizer.hpp>

#include <iostream>
#include <bitset>
#include <array>

using namespace igb;
using namespace igb::sdk;

TEST_CASE("ScaleQuantizer::updateScale", "[scale_bit]") {
  ScaleQuantizer quantizer;

  SECTION( "scale CDEFA" ) {
    const uint16_t scale_bit = 0b0000001000110101;
    quantizer.updateScale(scale_bit);

    std::bitset<16> bit(scale_bit);
    std::cout << "scale bits = " << bit << std::endl;
    std::cout << "scale_map = ";
    for (const auto& e : quantizer._scale_map) {
      std::cout << (int)e << ",";
    }
    std::cout << std::endl;
    REQUIRE((int)quantizer.quantize(0) == 0);
    REQUIRE((int)quantizer.quantize(1) == 0);
    REQUIRE((int)quantizer.quantize(2) == 2);
    REQUIRE((int)quantizer.quantize(3) == 2);
    REQUIRE((int)quantizer.quantize(4) == 4);
    REQUIRE((int)quantizer.quantize(5) == 5);
    REQUIRE((int)quantizer.quantize(6) == 5);
    REQUIRE((int)quantizer.quantize(7) == 5);
    REQUIRE((int)quantizer.quantize(8) == 9);
    REQUIRE((int)quantizer.quantize(9) == 9);
    REQUIRE((int)quantizer.quantize(10) == 9);
    REQUIRE((int)quantizer.quantize(11) == 12);
  }

  SECTION( "scale only E" ) {
    const uint16_t scale_bit = 0b0000000000000100;
    quantizer.updateScale(scale_bit);

    std::bitset<16> bit(scale_bit);
    std::cout << "scale bits = " << bit << std::endl;
    std::cout << "scale_map = ";
    for (const auto& e : quantizer._scale_map) {
      std::cout << (int)e << ",";
    }
    std::cout << std::endl;
    REQUIRE((int)quantizer.quantize(0) == 2);
    REQUIRE((int)quantizer.quantize(1) == 2);
    REQUIRE((int)quantizer.quantize(2) == 2);
    REQUIRE((int)quantizer.quantize(3) == 2);
    REQUIRE((int)quantizer.quantize(4) == 2);
    REQUIRE((int)quantizer.quantize(5) == 2);
    REQUIRE((int)quantizer.quantize(6) == 2);
    REQUIRE((int)quantizer.quantize(7) == 2);
    REQUIRE((int)quantizer.quantize(8) == 14);
    REQUIRE((int)quantizer.quantize(9) == 14);
    REQUIRE((int)quantizer.quantize(10) == 14);
    REQUIRE((int)quantizer.quantize(11) == 14);
  }

}


TEST_CASE("WeightedScaleQuantizer::updateScale", "[priorities]") {
  constexpr static size_t resolution = 8;
  WeightedScaleQuantizer<resolution> quantizer;

  SECTION("scale C(+)DE(+)F(-)A" ) {
    std::array<uint8_t, 12> priorities = {
      127, // C
      0, // C#
      84, // D 
      0, // D#
      127, // E
      43, // F
      0, // F#
      0, // G
      0, // G#
      84, // A
      0, // A#
      0 // B
    };

    quantizer.updateScale(priorities);
    std::cout << "WeightedScaleQuantizer (C(+)DE(+)F(-)A):" << std::endl;
    std::cout << "scale_map = ";
    for (const auto& e : quantizer._scale_map) {
      std::cout << (int)e << ",";
    }
    std::cout << std::endl;

    REQUIRE((int)quantizer.quantize(0 * resolution) == 0 * resolution); // C
    REQUIRE((int)quantizer.quantize(1 * resolution) == 0 * resolution);
    REQUIRE((int)quantizer.quantize(2 * resolution) == 0 * resolution);
    REQUIRE((int)quantizer.quantize(3 * resolution) == 0 * resolution);
    REQUIRE((int)quantizer.quantize(4 * resolution) == 2 * resolution); // D
    REQUIRE((int)quantizer.quantize(5 * resolution) == 2 * resolution); // D
    REQUIRE((int)quantizer.quantize(6 * resolution) == 4 * resolution); // E
    REQUIRE((int)quantizer.quantize(7 * resolution) == 4 * resolution);
    REQUIRE((int)quantizer.quantize(8 * resolution) == 4 * resolution);
    REQUIRE((int)quantizer.quantize(9 * resolution) == 5 * resolution); // F
    REQUIRE((int)quantizer.quantize(10 * resolution) == 9 * resolution); // A
    REQUIRE((int)quantizer.quantize(11 * resolution) == 9 * resolution);
    REQUIRE((int)quantizer.quantize(12 * resolution - 1) == 9 * resolution);
    REQUIRE((int)quantizer.quantize(12 * resolution) == 12 * resolution); // C(octave up)
  }

  SECTION( "scale only E" ) {
    std::array<uint8_t, 12> priorities = {
      0, // C
      0, // C#
      0, // D 
      0, // D#
      84, // E
      0, // F
      0, // F#
      0, // G
      0, // G#
      0, // A
      0, // A#
      0 // B
    };

    quantizer.updateScale(priorities);
    std::cout << "WeightedScaleQuantizer (E):" << std::endl;
    std::cout << "scale_map = ";
    for (const auto& e : quantizer._scale_map) {
      std::cout << (int)e << ",";
    }
    std::cout << std::endl;

    for (uint16_t i = 0; i < 12; ++i) {
      REQUIRE((int)quantizer.quantize(i * resolution) == 4 * resolution); // E
    }
  }
}
