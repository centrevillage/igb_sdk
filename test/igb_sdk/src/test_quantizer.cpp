#include <catch2/catch_test_macros.hpp>

#include <igb_sdk/util/quantizer.hpp>

#include <iostream>
#include <bitset>

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
