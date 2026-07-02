#include <catch2/catch_test_macros.hpp>

#include <igb_sdk/util/midi.hpp>
#include <igb_sdk/util/midi_usart.hpp>

#include <initializer_list>
#include <vector>

using namespace igb::sdk;

// Byte-level tests for Midi::getEvent() (found via LilaC #147): the
// "2 byte message?" switch compared the raw status byte (channel nibble
// included) against IGB_MIDI_PROG_CHG (0xC0), so Program Change / Channel
// Pressure completed only on channel 1 (0xC0 / 0xD0 exactly); on any other
// channel the parser kept waiting for a second data byte and the pending
// event was silently discarded by the next status byte. Completed messages
// also kept their data bytes, so a running-status continuation re-emitted
// stale data instead of starting a fresh message.

namespace {
// Feed bytes into the parser's RX ring and collect every completed event.
// Parser state (a partial message) persists on `midi` across calls, matching
// how MidiUsart::process() drains the ring incrementally.
std::vector<MidiEvent> parse(Midi& midi, std::initializer_list<uint8_t> bytes) {
  std::vector<MidiEvent> out;
  for (uint8_t b : bytes) midi.rx_buffer.add(b);
  while (auto e = midi.getEvent()) out.push_back(e.value());
  return out;
}
}  // namespace

TEST_CASE("MidiParser: Program Change parses on every channel, not just ch1",
          "[midi]") {
  Midi midi;
  // ch1 (0xC0) — the one case that worked pre-fix.
  auto ev = parse(midi, {0xC0, 5});
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].status == 0xC0);
  CHECK(ev[0].data1 == 5);
  CHECK(ev[0].data2 == MidiEvent::noData);
  // ch2 (0xC1) — the regression: PCs on ch != 1 never parsed.
  ev = parse(midi, {0xC1, 7});
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].status == 0xC1);
  CHECK(ev[0].data1 == 7);
  // ch16 (0xCF), max program.
  ev = parse(midi, {0xCF, 127});
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].status == 0xCF);
  CHECK(ev[0].data1 == 127);
}

TEST_CASE("MidiParser: Channel Pressure parses on every channel", "[midi]") {
  Midi midi;
  auto ev = parse(midi, {0xD2, 0x40});  // ch3
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].status == 0xD2);
  CHECK(ev[0].data1 == 0x40);
}

TEST_CASE("MidiParser: consecutive non-ch1 PCs each emit one event", "[midi]") {
  Midi midi;
  auto ev = parse(midi, {0xC1, 1, 0xC1, 2});
  REQUIRE(ev.size() == 2);
  CHECK(ev[0].data1 == 1);
  CHECK(ev[1].data1 == 2);
}

TEST_CASE("MidiParser: non-ch1 PC followed by a 3-byte message emits both",
          "[midi]") {
  Midi midi;
  // Pre-fix the pending PC was silently clobbered by the next status byte.
  auto ev = parse(midi, {0xC2, 9, 0x91, 60, 100});
  REQUIRE(ev.size() == 2);
  CHECK(ev[0].status == 0xC2);
  CHECK(ev[0].data1 == 9);
  CHECK(ev[1].status == 0x91);
  CHECK(ev[1].data1 == 60);
  CHECK(ev[1].data2 == 100);
}

TEST_CASE("MidiParser: running-status PCs parse each program", "[midi]") {
  Midi midi;
  // Same-value double PC with the repeated status byte elided.
  auto ev = parse(midi, {0xC1, 5, 5});
  REQUIRE(ev.size() == 2);
  CHECK(ev[0].status == 0xC1);
  CHECK(ev[0].data1 == 5);
  CHECK(ev[1].status == 0xC1);
  CHECK(ev[1].data1 == 5);
  // Different values: the 2nd event must carry ITS program, not the 1st's.
  ev = parse(midi, {0xC2, 3, 9});
  REQUIRE(ev.size() == 2);
  CHECK(ev[1].status == 0xC2);
  CHECK(ev[1].data1 == 9);
}

TEST_CASE("MidiParser: running-status 3-byte messages parse each data pair",
          "[midi]") {
  Midi midi;
  // NoteOn 60 vel 100, then (running status) NoteOn 62 vel 0.
  auto ev = parse(midi, {0x91, 60, 100, 62, 0});
  REQUIRE(ev.size() == 2);
  CHECK(ev[0].status == 0x91);
  CHECK(ev[0].data1 == 60);
  CHECK(ev[0].data2 == 100);
  CHECK(ev[1].status == 0x91);
  CHECK(ev[1].data1 == 62);
  CHECK(ev[1].data2 == 0);
}

TEST_CASE("MidiParser: 3-byte channel messages still wait for both data bytes",
          "[midi]") {
  Midi midi;
  auto ev = parse(midi, {0xB1, 7});  // CC ch2, data2 pending
  CHECK(ev.empty());
  ev = parse(midi, {100});
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].status == 0xB1);
  CHECK(ev[0].data1 == 7);
  CHECK(ev[0].data2 == 100);
}

TEST_CASE("MidiParser: system common statuses stay unmasked", "[midi]") {
  Midi midi;
  // Song Select (0xF3) is a 2-byte system common message — masking it to 0xF0
  // (SysEx start) would break it, hence the < 0xF0 guard on the mask.
  auto ev = parse(midi, {0xF3, 3});
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].status == 0xF3);
  CHECK(ev[0].data1 == 3);
  // Song Position (0xF2) is 3-byte and must NOT complete after one data byte.
  ev = parse(midi, {0xF2, 1});
  CHECK(ev.empty());
  ev = parse(midi, {2});
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].status == 0xF2);
  CHECK(ev[0].data1 == 1);
  CHECK(ev[0].data2 == 2);
}

TEST_CASE("MidiParser: realtime byte between PC status and data keeps the PC",
          "[midi]") {
  Midi midi;
  // A clock byte may interleave anywhere; between a PC's status and its data
  // byte the parser state survives (data1 is still empty, so the status-byte
  // reset is a no-op) and both events come out.
  auto ev = parse(midi, {0xC1, 0xF8, 5});
  REQUIRE(ev.size() == 2);
  CHECK(ev[0].status == 0xF8);
  CHECK(ev[1].status == 0xC1);
  CHECK(ev[1].data1 == 5);
}

// ---- LilaC #172: realtime interleave must not destroy a partial message ----

TEST_CASE("MidiParser: clock between data bytes keeps the 3-byte message",
          "[midi]") {
  Midi midi;
  // Pre-fix the 0xF8 reset data1, so the NoteOn lost its note number and the
  // following bytes re-paired into phantom events.
  auto ev = parse(midi, {0x91, 60, 0xF8, 100});
  REQUIRE(ev.size() == 2);
  CHECK(ev[0].status == 0xF8);
  CHECK(ev[1].status == 0x91);
  CHECK(ev[1].data1 == 60);
  CHECK(ev[1].data2 == 100);
}

TEST_CASE("MidiParser: running-status note stream survives scattered clocks",
          "[midi]") {
  Midi midi;
  auto ev = parse(midi, {0x91, 60, 0xF8, 100, 62, 0xF8, 0});
  REQUIRE(ev.size() == 4);
  CHECK(ev[0].status == 0xF8);
  CHECK(ev[1].status == 0x91);
  CHECK(ev[1].data1 == 60);
  CHECK(ev[1].data2 == 100);
  CHECK(ev[2].status == 0xF8);
  CHECK(ev[3].status == 0x91);
  CHECK(ev[3].data1 == 62);
  CHECK(ev[3].data2 == 0);
}

TEST_CASE("MidiParser: defined realtime bytes each emit a 1-byte event",
          "[midi]") {
  Midi midi;
  auto ev = parse(midi, {0xF8, 0xFA, 0xFB, 0xFC, 0xFE, 0xFF});
  REQUIRE(ev.size() == 6);
  CHECK(ev[0].status == 0xF8);
  CHECK(ev[1].status == 0xFA);
  CHECK(ev[2].status == 0xFB);
  CHECK(ev[3].status == 0xFC);
  CHECK(ev[4].status == 0xFE);
  CHECK(ev[5].status == 0xFF);
}

TEST_CASE("MidiParser: undefined status bytes are discarded without clobbering",
          "[midi]") {
  Midi midi;
  // 0xF9 / 0xFD (undefined realtime) and 0xF4 / 0xF5 (undefined system
  // common) mid-message: no event, and the pending NoteOn stays intact.
  auto ev = parse(midi, {0x91, 60, 0xF9, 0xFD, 0xF4, 0xF5, 100});
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].status == 0x91);
  CHECK(ev[0].data1 == 60);
  CHECK(ev[0].data2 == 100);
}

TEST_CASE("MidiParser: data bytes before any status byte are dropped",
          "[midi]") {
  Midi midi;
  // Power-on mid-stream / hot-plug: the tail of a partial message must not
  // form a status-0 garbage event (it would be forwarded on the chain wire).
  auto ev = parse(midi, {10, 20, 30});
  CHECK(ev.empty());
  // The parser recovers at the first real status byte.
  ev = parse(midi, {0x90, 60, 100});
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].status == 0x90);
  CHECK(ev[0].data1 == 60);
  CHECK(ev[0].data2 == 100);
}

// ---- LilaC #173: MidiUsart sysex drain robustness ----

namespace {
// Minimal fakes for MidiUsart's template params. Tests never call
// initUsart()/irqHandler(), so only the members process() touches are needed
// (is() gating txHandler, txData() through it).
enum class FakeUsartState { rxNotEmpty, txEmpty };
struct FakeUsartConf {};
struct FakeUsart {
  bool is(FakeUsartState) const { return false; }
  void txData(uint8_t) {}
  uint8_t rxData() { return 0; }
  void enable(bool) {}
};
using TestMidiUsart = MidiUsart<FakeUsart, int, FakeUsartConf, FakeUsartState>;

struct UsartHarness {
  TestMidiUsart mu;
  std::vector<MidiEvent> received;
  std::vector<uint8_t> sysex_body;
  int sysex_starts = 0;
  int sysex_ends = 0;

  UsartHarness() {
    mu.on_receive = [this](const MidiEvent& e) { received.push_back(e); };
    mu.on_sysex_start = [this]() { ++sysex_starts; };
    mu.on_sysex_end = [this]() { ++sysex_ends; };
    mu.on_sysex_receive = [this](uint8_t b) { sysex_body.push_back(b); };
  }

  // Queue bytes as the RX IRQ would, then run enough process() iterations to
  // drain them (the normal path completes at most one event per call).
  void feed(std::initializer_list<uint8_t> bytes) {
    for (uint8_t b : bytes) mu.midi.rx_buffer.add(b);
    for (int i = 0; i < 64; ++i) mu.process();
  }
};
}  // namespace

TEST_CASE("MidiUsart: a message after a sysex in the same batch survives",
          "[midi]") {
  UsartHarness h;
  // Pre-fix the drain loop kept eating past F7, so the NoteOn's status was
  // discarded and its data bytes leaked into on_sysex_receive.
  h.feed({0xF0, 1, 2, 3, 0xF7, 0x90, 60, 100});
  CHECK(h.sysex_starts == 1);
  CHECK(h.sysex_ends == 1);
  CHECK(h.sysex_body == std::vector<uint8_t>{1, 2, 3});
  REQUIRE(h.received.size() == 1);
  CHECK(h.received[0].status == 0x90);
  CHECK(h.received[0].data1 == 60);
  CHECK(h.received[0].data2 == 100);
}

TEST_CASE("MidiUsart: realtime inside a sysex is dispatched, sysex continues",
          "[midi]") {
  UsartHarness h;
  // Pre-fix a clock byte falsely terminated the sysex AND was swallowed.
  h.feed({0xF0, 1, 0xF8, 2, 0xF7});
  CHECK(h.sysex_starts == 1);
  CHECK(h.sysex_ends == 1);
  CHECK(h.sysex_body == std::vector<uint8_t>{1, 2});
  REQUIRE(h.received.size() == 1);
  CHECK(h.received[0].status == 0xF8);
}

TEST_CASE("MidiUsart: a status byte ends an unterminated sysex and starts "
          "the next message", "[midi]") {
  UsartHarness h;
  // Pre-fix the terminating status byte was consumed and dropped, so the
  // NoteOn was lost and its data bytes misparsed.
  h.feed({0xF0, 1, 2, 0x91, 60, 100});
  CHECK(h.sysex_ends == 1);
  CHECK(h.sysex_body == std::vector<uint8_t>{1, 2});
  REQUIRE(h.received.size() == 1);
  CHECK(h.received[0].status == 0x91);
  CHECK(h.received[0].data1 == 60);
  CHECK(h.received[0].data2 == 100);
}

TEST_CASE("MidiUsart: a 1-byte message ending a sysex is itself dispatched",
          "[midi]") {
  UsartHarness h;
  h.feed({0xF0, 1, 0xF6, 0x90, 60, 100});  // Tune Request ends the sysex
  CHECK(h.sysex_ends == 1);
  REQUIRE(h.received.size() == 2);
  CHECK(h.received[0].status == 0xF6);
  CHECK(h.received[1].status == 0x90);
  CHECK(h.received[1].data1 == 60);
  CHECK(h.received[1].data2 == 100);
}

TEST_CASE("MidiUsart: F0 inside a sysex restarts a new sysex", "[midi]") {
  UsartHarness h;
  h.feed({0xF0, 1, 0xF0, 2, 0xF7});
  CHECK(h.sysex_starts == 2);
  CHECK(h.sysex_ends == 2);
  CHECK(h.sysex_body == std::vector<uint8_t>{1, 2});
  CHECK(h.received.empty());
}
