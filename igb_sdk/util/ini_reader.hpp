#pragma once

// Minimal zero-allocation INI reader. Operates on a pre-loaded, NUL-
// terminated char buffer (reader never owns the buffer). Supports:
//   - [section] lookup
//   - key=value (int and string variants)
//   - Stops at the next `[section]` boundary so keys from a later section
//     don't leak into the current one.
//
// Hand-written files are tolerated (SproutFX issue #31): a UTF-8 BOM at the
// start of the buffer, blanks (space / tab) at the start of a line, around
// `=` and inside `[ ... ]`, trailing blanks and CR after a value, and any
// letter case in section names and keys. A file that parsed before reads
// the same (the strict `key=value` form is a special case).

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace igb::sdk::ini_reader {

namespace detail {

inline bool isBlank(char c) { return c == ' ' || c == '\t'; }

inline const char* skipBlank(const char* p, const char* end) {
  while (p < end && isBlank(*p)) ++p;
  return p;
}

inline char lower(char c) {
  return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

// `p` (length >= n, not checked past `end`) equals `name` ignoring case.
inline bool equalsNoCase(const char* p, const char* end, const char* name, size_t n) {
  if ((size_t)(end - p) < n) return false;
  for (size_t i = 0; i < n; ++i) {
    if (lower(p[i]) != lower(name[i])) return false;
  }
  return true;
}

inline const char* lineEnd(const char* p) {
  const char* e = std::strchr(p, '\n');
  return e ? e : p + std::strlen(p);
}

inline const char* nextLine(const char* lend) {
  return (*lend == '\n') ? lend + 1 : lend;
}

inline const char* skipBom(const char* p) {
  if ((uint8_t)p[0] == 0xEF && (uint8_t)p[1] == 0xBB && (uint8_t)p[2] == 0xBF) {
    return p + 3;
  }
  return p;
}

// Finds `key = value` from `section_start` up to the next `[section]` line.
// On a match: *v = the value's first non-blank char, *v_end = past its last
// char with trailing blanks / CR removed.
inline bool findValue(const char* section_start, const char* key,
                      const char** v, const char** v_end) {
  if (!section_start) return false;
  const size_t klen = std::strlen(key);
  const char* p = section_start;
  while (*p) {
    const char* lend = lineEnd(p);
    const char* q = skipBlank(p, lend);
    if (q < lend && *q == '[') break;
    if (equalsNoCase(q, lend, key, klen)) {
      const char* r = skipBlank(q + klen, lend);
      if (r < lend && *r == '=') {
        const char* b = skipBlank(r + 1, lend);
        const char* e = lend;
        while (e > b && (isBlank(e[-1]) || e[-1] == '\r')) --e;
        *v = b;
        *v_end = e;
        return true;
      }
    }
    p = nextLine(lend);
  }
  return false;
}

}  // namespace detail

// Locate the line just after `[section]` within `buf`. Returns nullptr
// if the section is not present.
inline const char* findSection(const char* buf, const char* name) {
  const size_t nlen = std::strlen(name);
  const char* p = detail::skipBom(buf);
  while (*p) {
    const char* lend = detail::lineEnd(p);
    const char* q = detail::skipBlank(p, lend);
    if (q < lend && *q == '[') {
      const char* r = detail::skipBlank(q + 1, lend);
      if (detail::equalsNoCase(r, lend, name, nlen)) {
        const char* s = detail::skipBlank(r + nlen, lend);
        if (s < lend && *s == ']') {
          return detail::nextLine(lend);
        }
      }
    }
    p = detail::nextLine(lend);
  }
  return nullptr;
}

// From `section_start` (inside a section), find `key=`. Returns the
// integer value or `def` if absent. Stops at the next `[section]`. A value
// that is not a number reads as strtol reads it (e.g. 0 for "abc"); use
// tryGetInt() to tell those apart.
inline long getInt(const char* section_start, const char* key, long def) {
  const char* v = nullptr;
  const char* e = nullptr;
  if (!detail::findValue(section_start, key, &v, &e)) return def;
  return std::strtol(v, nullptr, 10);
}

// Like getInt(), but true only when the key is present and its whole value
// is a decimal integer (optional sign; blanks / CR after it are allowed).
// *out is left untouched otherwise.
inline bool tryGetInt(const char* section_start, const char* key, long* out) {
  const char* v = nullptr;
  const char* e = nullptr;
  if (!detail::findValue(section_start, key, &v, &e)) return false;
  if (v == e) return false;
  char* num_end = nullptr;
  const long n = std::strtol(v, &num_end, 10);
  if (num_end == v || num_end != e) return false;
  *out = n;
  return true;
}

// From `section_start`, find `key=value`. On match sets `*out` to a
// pointer inside the buffer and `*out_len` to the trimmed length (leading
// and trailing blanks and CR removed). On miss leaves them {nullptr, 0}.
inline void getStr(const char* section_start, const char* key,
                   const char** out, size_t* out_len) {
  *out = nullptr; *out_len = 0;
  const char* v = nullptr;
  const char* e = nullptr;
  if (!detail::findValue(section_start, key, &v, &e)) return;
  *out = v;
  *out_len = (size_t)(e - v);
}

}  // namespace igb::sdk::ini_reader
