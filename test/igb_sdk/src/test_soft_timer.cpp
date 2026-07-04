#include <catch2/catch_test_macros.hpp>

#include <igb_sdk/util/soft_timer.hpp>

using namespace igb;
using namespace igb::sdk;

// SoftTimer::process() の wrap-safe な期日判定に対するテスト。
// 特に「未到来(未来 start)のタイマーが符号なし減算のアンダーフローで即発火する」
// 潜在バグの回帰防止(process() のコールバック内から同一タイマーへ積むケースで顕在化)。

TEST_CASE("SoftTimer::process basic timing", "") {
  SECTION("oneshot fires exactly at start + interval") {
    SoftTimer<2> timer;
    int fired = 0;
    timer.oneshotCallback(100, 0, [&]() { ++fired; });

    timer.process(50);   // 未到来
    REQUIRE(fired == 0);
    timer.process(99);   // 未到来
    REQUIRE(fired == 0);
    timer.process(100);  // 到達
    REQUIRE(fired == 1);
    timer.process(500);  // oneshot は inactive 化済み。再発火しない
    REQUIRE(fired == 1);
  }

  SECTION("interval fires repeatedly") {
    SoftTimer<2> timer;
    int fired = 0;
    timer.intervalCallback(100, 0, [&]() { ++fired; });

    timer.process(100);
    REQUIRE(fired == 1);
    timer.process(150);  // 次周期は 200
    REQUIRE(fired == 1);
    timer.process(200);
    REQUIRE(fired == 2);
  }
}

TEST_CASE("SoftTimer::process does not underflow-fire", "") {
  // 未来 start のタイマーは発火しないこと(バグ本体の最小再現)。
  // 修正前は current_msec_ - _start_msec が符号なしでアンダーフローし即発火していた。
  SECTION("a timer whose start is in the future does not fire early") {
    SoftTimer<1> timer;
    int fired = 0;
    timer.oneshotCallback(50, 200, [&]() { ++fired; });  // fire = 250

    timer.process(100);  // start(200) が未来。修正前はここで誤発火していた
    REQUIRE(fired == 0);
    timer.process(249);
    REQUIRE(fired == 0);
    timer.process(250);
    REQUIRE(fired == 1);
  }

  // process() のコールバック内から同一タイマーへ積んだタイマーが、同じ周回で
  // 即発火しないこと(CLK OUT のパルス潰れの実シナリオ)。
  SECTION("rescheduling inside a process callback does not fire in the same pass") {
    SoftTimer<2> timer;
    long a_at = -1;
    long b_at = -1;
    uint32_t cur = 0;
    constexpr uint32_t gap = 3;  // コールバック内で tick を読み直す際のずれ

    timer.oneshotCallback(100, 0, [&]() {
      a_at = cur;
      // A のコールバック内で、読み直した tick (cur + gap) を起点に B を積む
      timer.oneshotCallback(50, cur + gap, [&]() { b_at = cur; });
    });

    for (uint32_t t = 1; t <= 300; ++t) {
      cur = t;
      timer.process(t);
    }

    REQUIRE(a_at == 100);              // A は 100 で発火
    REQUIRE(b_at == 100 + gap + 50);   // B は自分の期日(153)で発火。同一周回の即発火(=100)ではない
    REQUIRE(b_at != a_at);
  }
}

TEST_CASE("SoftTimer::process is wrap-safe", "") {
  // fire_time (= _start_msec + interval) が uint32 をラップしても正しく判定できること。
  SECTION("fires at the correct time across the uint32 boundary") {
    SoftTimer<1> timer;
    int fired = 0;
    const uint32_t start = 0xFFFFFF00;
    const uint32_t interval = 0x200;  // fire_time = 0x100000100 -> uint32 で 0x100 にラップ
    timer.oneshotCallback(interval, start, [&]() { ++fired; });

    timer.process(0xFFFFFF80);  // elapsed=0x80 < interval
    REQUIRE(fired == 0);
    timer.process(0x000000FF);  // ラップ後だが elapsed=0x1FF < interval
    REQUIRE(fired == 0);
    timer.process(0x00000100);  // elapsed=0x200 == interval -> 発火
    REQUIRE(fired == 1);
  }
}
