#include <catch2/catch_test_macros.hpp>

#include <igb_sdk/util/midi.hpp>

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
