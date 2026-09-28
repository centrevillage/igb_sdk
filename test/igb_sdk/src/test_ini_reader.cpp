#include <catch2/catch_test_macros.hpp>

#include <igb_sdk/util/ini_reader.hpp>

#include <string>

namespace ini = igb::sdk::ini_reader;

namespace {
std::string str(const char* sec, const char* key) {
  const char* v = nullptr;
  size_t n = 0;
  ini::getStr(sec, key, &v, &n);
  return v ? std::string(v, n) : std::string("<none>");
}
}  // namespace

TEST_CASE("ini_reader strict form (the form LilaCRepeater wrote)", "") {
  const char* buf =
    "[config]\n"
    "loopsync=bar\n"
    "autostop=16\n"
    "[other]\n"
    "autostop=3\n";
  const char* sec = ini::findSection(buf, "config");
  REQUIRE(sec != nullptr);
  REQUIRE(ini::getInt(sec, "autostop", -1) == 16);
  REQUIRE(str(sec, "loopsync") == "bar");
  SECTION("keys of a later section do not leak") {
    REQUIRE(ini::getInt(sec, "missing", -1) == -1);
    REQUIRE(ini::getInt(ini::findSection(buf, "other"), "autostop", -1) == 3);
  }
  SECTION("missing section / key") {
    REQUIRE(ini::findSection(buf, "midi") == nullptr);
    REQUIRE(ini::getInt(nullptr, "autostop", 7) == 7);
    REQUIRE(str(nullptr, "loopsync") == "<none>");
  }
  SECTION("a key is not matched by its prefix or by a longer key") {
    REQUIRE(ini::getInt(sec, "auto", -1) == -1);
    REQUIRE(ini::getInt(sec, "autostops", -1) == -1);
  }
  SECTION("getInt keeps strtol's reading of a non-number") {
    const char* b2 = "[a]\nx=abc\ny=12abc\n";
    const char* s2 = ini::findSection(b2, "a");
    REQUIRE(ini::getInt(s2, "x", -1) == 0);
    REQUIRE(ini::getInt(s2, "y", -1) == 12);
  }
}

TEST_CASE("ini_reader tolerates hand-written files (SproutFX issue #31)", "") {
  SECTION("blanks around = and at the line start, CRLF, trailing blanks") {
    const char* buf =
      "; SproutFX config\r\n"
      "  [midi]  \r\n"
      "\tchannel = 5 \r\n"
      "  velocity\t=\toff\t\r\n";
    const char* sec = ini::findSection(buf, "midi");
    REQUIRE(sec != nullptr);
    REQUIRE(ini::getInt(sec, "channel", -1) == 5);
    REQUIRE(str(sec, "velocity") == "off");
  }
  SECTION("UTF-8 BOM before the first section") {
    const char* buf = "\xEF\xBB\xBF[midi]\nchannel=3\n";
    REQUIRE(ini::getInt(ini::findSection(buf, "midi"), "channel", -1) == 3);
  }
  SECTION("letter case of section names and keys") {
    const char* buf = "[MIDI]\nChannel=9\n";
    REQUIRE(ini::getInt(ini::findSection(buf, "midi"), "channel", -1) == 9);
  }
  SECTION("blanks inside the brackets") {
    const char* buf = "[ midi ]\nchannel=2\n";
    REQUIRE(ini::getInt(ini::findSection(buf, "midi"), "channel", -1) == 2);
  }
  SECTION("an indented section header ends the previous section") {
    const char* buf = "[midi]\nchannel=2\n  [other]\nvelocity=on\n";
    const char* sec = ini::findSection(buf, "midi");
    REQUIRE(str(sec, "velocity") == "<none>");
  }
  SECTION("comments and empty values") {
    const char* buf = "[midi]\n; channel=4\n# channel=6\nvelocity=\n";
    const char* sec = ini::findSection(buf, "midi");
    REQUIRE(ini::getInt(sec, "channel", -1) == -1);
    REQUIRE(str(sec, "velocity") == "");
  }
  SECTION("no newline at the end of the buffer") {
    const char* buf = "[midi]\nchannel = 16";
    REQUIRE(ini::getInt(ini::findSection(buf, "midi"), "channel", -1) == 16);
  }
}

TEST_CASE("ini_reader tryGetInt accepts whole integers only", "") {
  const char* buf =
    "[a]\n"
    "ok = 12 \r\n"
    "neg=-3\n"
    "plus=+4\n"
    "word=abc\n"
    "tail=12abc\n"
    "empty=\n"
    "float=1.5\n";
  const char* sec = ini::findSection(buf, "a");
  long v = 99;
  REQUIRE(ini::tryGetInt(sec, "ok", &v));
  REQUIRE(v == 12);
  REQUIRE(ini::tryGetInt(sec, "neg", &v));
  REQUIRE(v == -3);
  REQUIRE(ini::tryGetInt(sec, "plus", &v));
  REQUIRE(v == 4);
  v = 99;
  REQUIRE_FALSE(ini::tryGetInt(sec, "word", &v));
  REQUIRE_FALSE(ini::tryGetInt(sec, "tail", &v));
  REQUIRE_FALSE(ini::tryGetInt(sec, "empty", &v));
  REQUIRE_FALSE(ini::tryGetInt(sec, "float", &v));
  REQUIRE_FALSE(ini::tryGetInt(sec, "missing", &v));
  REQUIRE_FALSE(ini::tryGetInt(nullptr, "ok", &v));
  REQUIRE(v == 99);
}
