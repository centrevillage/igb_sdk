#include <catch2/catch_test_macros.hpp>

#include <igb_sdk/ui/button_history.hpp>

#include <iostream>

using namespace igb;
using namespace igb::sdk;

enum class TestButtonId {
  test = 0,
  dummy,
};

typedef ButtonHistory<TestButtonId, 8> TestButtonHistory;
typedef TestButtonHistory::Info TestButtonInfo;

TEST_CASE("ButtonHistory::findMorseCode", "[id, long_press_tick, max_interval_tick]") {
  SECTION( "A" ) {
    TestButtonHistory history;
    history.add(TestButtonInfo {.tick = 0, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 500, .id = TestButtonId::test, .state = 0});
    history.add(TestButtonInfo {.tick = 1000, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 2000, .id = TestButtonId::test, .state = 0});

    REQUIRE(history.findMorseCode(TestButtonId::test, 1000, 5000) == 'A');
  }

  SECTION( "null" ) {
    TestButtonHistory history;
    history.add(TestButtonInfo {.tick = 0, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 500, .id = TestButtonId::test, .state = 0});
    history.add(TestButtonInfo {.tick = 1000, .id = TestButtonId::dummy, .state = 1});
    history.add(TestButtonInfo {.tick = 2000, .id = TestButtonId::test, .state = 0});

    REQUIRE(history.findMorseCode(TestButtonId::test, 1000, 5000) == (char)0);
  }

  SECTION( "C" ) {
    TestButtonHistory history;
    history.add(TestButtonInfo {.tick = 0, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 1000, .id = TestButtonId::test, .state = 0});
    history.add(TestButtonInfo {.tick = 1500, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 2000, .id = TestButtonId::test, .state = 0});
    history.add(TestButtonInfo {.tick = 2500, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 3500, .id = TestButtonId::test, .state = 0});
    history.add(TestButtonInfo {.tick = 4000, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 4500, .id = TestButtonId::test, .state = 0});

    REQUIRE(history.findMorseCode(TestButtonId::test, 1000, 5000) == 'C');
  }

  SECTION( "G" ) {
    TestButtonHistory history;
    history.add(TestButtonInfo {.tick = 0, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 1000, .id = TestButtonId::test, .state = 0});
    history.add(TestButtonInfo {.tick = 1500, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 2500, .id = TestButtonId::test, .state = 0});
    history.add(TestButtonInfo {.tick = 3000, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 3500, .id = TestButtonId::test, .state = 0});

    REQUIRE(history.findMorseCode(TestButtonId::test, 1000, 5000) == 'G');
  }

  SECTION( "O" ) {
    TestButtonHistory history;
    history.add(TestButtonInfo {.tick = 0, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 1000, .id = TestButtonId::test, .state = 0});
    history.add(TestButtonInfo {.tick = 1500, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 2500, .id = TestButtonId::test, .state = 0});
    history.add(TestButtonInfo {.tick = 3000, .id = TestButtonId::test, .state = 1});
    history.add(TestButtonInfo {.tick = 4000, .id = TestButtonId::test, .state = 0});

    REQUIRE(history.findMorseCode(TestButtonId::test, 1000, 5000) == 'O');
  }
}
