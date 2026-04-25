#pragma once

// Minimal zero-allocation INI reader. Operates on a pre-loaded, NUL-
// terminated char buffer (reader never owns the buffer). Supports:
//   - [section] lookup
//   - key=value (int and string variants)
//   - Stops at the next `[section]` boundary so keys from a later section
//     don't leak into the current one.

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace igb::sdk::ini_reader {

// Locate the line just after `[section]` within `buf`. Returns nullptr
// if the section is not present.
inline const char* findSection(const char* buf, const char* name) {
  size_t nlen = std::strlen(name);
  const char* p = buf;
  while (*p) {
    const char* lend = std::strchr(p, '\n');
    if (!lend) lend = p + std::strlen(p);
    if ((size_t)(lend - p) >= nlen + 2 && p[0] == '['
        && std::memcmp(p + 1, name, nlen) == 0 && p[1 + nlen] == ']') {
      return (*lend == '\n') ? lend + 1 : lend;
    }
    p = (*lend == '\n') ? lend + 1 : lend;
  }
  return nullptr;
}

// From `section_start` (inside a section), find `key=`. Returns the
// integer value or `def` if absent. Stops at the next `[section]`.
inline long getInt(const char* section_start, const char* key, long def) {
  if (!section_start) return def;
  size_t klen = std::strlen(key);
  const char* p = section_start;
  while (p && *p) {
    if (*p == '[') break;
    if ((size_t)std::strlen(p) >= klen + 1
        && std::memcmp(p, key, klen) == 0 && p[klen] == '=') {
      return std::strtol(p + klen + 1, nullptr, 10);
    }
    const char* lend = std::strchr(p, '\n');
    if (!lend) break;
    p = lend + 1;
  }
  return def;
}

// From `section_start`, find `key=value`. On match sets `*out` to a
// pointer inside the buffer and `*out_len` to the trimmed length (CR
// removed). On miss leaves them {nullptr, 0}.
inline void getStr(const char* section_start, const char* key,
                   const char** out, size_t* out_len) {
  *out = nullptr; *out_len = 0;
  if (!section_start) return;
  size_t klen = std::strlen(key);
  const char* p = section_start;
  while (p && *p) {
    if (*p == '[') break;
    if ((size_t)std::strlen(p) >= klen + 1
        && std::memcmp(p, key, klen) == 0 && p[klen] == '=') {
      const char* v = p + klen + 1;
      const char* lend = std::strchr(v, '\n');
      if (!lend) lend = v + std::strlen(v);
      size_t L = (size_t)(lend - v);
      if (L > 0 && v[L - 1] == '\r') --L;
      *out = v; *out_len = L;
      return;
    }
    const char* lend = std::strchr(p, '\n');
    if (!lend) break;
    p = lend + 1;
  }
}

}  // namespace igb::sdk::ini_reader
