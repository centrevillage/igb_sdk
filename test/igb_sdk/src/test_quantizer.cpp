#include <catch2/catch_test_macros.hpp>

#include <igb_sdk/util/quantizer.hpp>
#include <igb_util/bitmagic.hpp>

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

  SECTION( "rot test" ) {
    // scale CDEFA

    const uint8_t rot = 3;
    const uint16_t scale_bit = 0b0000001000110101;
    ScaleQuantizer quantizer1;
    ScaleQuantizer quantizer2;

    quantizer1.updateScale(bit_left_rotate_with_length_u16(scale_bit, rot, 12));
    quantizer2.updateScale(scale_bit);

    std::cout << "rot test:";
    std::cout << std::endl;
    std::cout << " - quantizer1: ";
    for (uint8_t i = 0; i < 12; ++i) {
      std::cout << (int)quantizer1.quantize(i) << ",";
    }
    std::cout << std::endl;
    std::cout << " - quantizer2: ";
    for (uint8_t i = 0; i < 12; ++i) {
      std::cout << (int)quantizer2.quantize(i, rot) << ",";
    }
    std::cout << std::endl;

    REQUIRE((int)quantizer1.quantize(0) == (int)quantizer2.quantize(0, rot));
    REQUIRE((int)quantizer1.quantize(1) == (int)quantizer2.quantize(1, rot));
    REQUIRE((int)quantizer1.quantize(2) == (int)quantizer2.quantize(2, rot));
    REQUIRE((int)quantizer1.quantize(3) == (int)quantizer2.quantize(3, rot));
    REQUIRE((int)quantizer1.quantize(4) == (int)quantizer2.quantize(4, rot));
    REQUIRE((int)quantizer1.quantize(5) == (int)quantizer2.quantize(5, rot));
    REQUIRE((int)quantizer1.quantize(6) == (int)quantizer2.quantize(6, rot));
    REQUIRE((int)quantizer1.quantize(7) == (int)quantizer2.quantize(7, rot));
    REQUIRE((int)quantizer1.quantize(8) == (int)quantizer2.quantize(8, rot));
    REQUIRE((int)quantizer1.quantize(9) == (int)quantizer2.quantize(9, rot));
    //REQUIRE((int)quantizer1.quantize(10) == (int)quantizer2.quantize(10, rot));
    REQUIRE((int)quantizer1.quantize(11) == (int)quantizer2.quantize(11, rot));
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
    //std::cout << "scale_map = ";
    //for (const auto& e : quantizer._scale_map) {
    //  std::cout << (int)e << ",";
    //}
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
    //std::cout << "scale_map = ";
    //for (const auto& e : quantizer._scale_map) {
    //  std::cout << (int)e << ",";
    //}
    std::cout << std::endl;

    for (uint16_t i = 0; i < 12; ++i) {
      REQUIRE((int)quantizer.quantize(i * resolution) == 4 * resolution); // E
    }
  }

  SECTION( "rot test" ) {
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
    std::array<uint8_t, 12> priorities_rot = {
      84,
      0,
      0,
      127,
      0,
      84,
      0,
      127,
      43,
      0,
      0,
      0,
    };
    const uint8_t rot = 3;

    WeightedScaleQuantizer<resolution> quantizer1;
    WeightedScaleQuantizer<resolution> quantizer2;

    quantizer1.updateScale(priorities_rot);
    quantizer2.updateScale(priorities);

    //std::cout << "rot test(weighted):";
    //std::cout << std::endl;
    //std::cout << " - quantizer1: ";
    //for (uint16_t i = 24; i < 36 * resolution; ++i) {
    //  std::cout << (int)(quantizer1.quantize(i) / resolution) << ",";
    //}
    //std::cout << std::endl;
    //std::cout << " - quantizer2: ";
    //for (uint16_t i = 24; i < 36 * resolution; ++i) {
    //  std::cout << (int)(quantizer2.quantize(i, rot) / resolution) << ",";
    //}
    //std::cout << std::endl;

    REQUIRE((int)quantizer1.quantize(0) == (int)quantizer2.quantize(0, rot));
    REQUIRE((int)quantizer1.quantize(1) == (int)quantizer2.quantize(1, rot));
    REQUIRE((int)quantizer1.quantize(2) == (int)quantizer2.quantize(2, rot));
    REQUIRE((int)quantizer1.quantize(3) == (int)quantizer2.quantize(3, rot));
    REQUIRE((int)quantizer1.quantize(4) == (int)quantizer2.quantize(4, rot));
    REQUIRE((int)quantizer1.quantize(5) == (int)quantizer2.quantize(5, rot));
    REQUIRE((int)quantizer1.quantize(6) == (int)quantizer2.quantize(6, rot));
    REQUIRE((int)quantizer1.quantize(7) == (int)quantizer2.quantize(7, rot));
    REQUIRE((int)quantizer1.quantize(8) == (int)quantizer2.quantize(8, rot));
    REQUIRE((int)quantizer1.quantize(9) == (int)quantizer2.quantize(9, rot));
    REQUIRE((int)quantizer1.quantize(10) == (int)quantizer2.quantize(10, rot));
    REQUIRE((int)quantizer1.quantize(11) == (int)quantizer2.quantize(11, rot));
  }
}
