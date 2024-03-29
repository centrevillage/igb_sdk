#pragma once

namespace igb {

template<typename T, typename N>
class BitStruct {
private:
  N bit;

  BitStruct(N initialBit) : bit(initialBit) { }
public:

  static N construct() { return 0; }
  static N construct(const T group, const auto... groups) { return (1UL << static_cast<N>(group)) | construct(groups...); }

  BitStruct(const auto... groups) : bit(construct(groups...)) {
  }

  BitStruct operator = (const BitStruct s) noexcept {
    bit = s.bit;
    return *this;
  }

  BitStruct operator | (const BitStruct s) const noexcept {
    return BitStruct { bit | s.bit };
  }

  BitStruct operator & (const BitStruct s) const noexcept {
    return BitStruct { bit & s.bit };
  }

  BitStruct operator ^ (const BitStruct s) const noexcept {
    return BitStruct { bit ^ s.bit };
  }

  BitStruct operator | (const N n) const noexcept {
    return BitStruct { bit | n };
  }

  BitStruct operator & (const N n) const noexcept {
    return BitStruct { bit & n };
  }

  BitStruct operator ^ (const N n) const noexcept {
    return BitStruct { bit ^ n };
  }

  BitStruct operator ~() const noexcept {
    return BitStruct { ~bit };
  }

  explicit operator bool() const noexcept {
    return !!bit;
  }

  bool operator !() const noexcept {
    return !bit;
  }

  N get() const noexcept {
    return bit;
  }

  bool has(T group) const noexcept {
    return bit & (1UL << static_cast<N>(group));
  }

  void add(T group) noexcept {
    bit = bit | (1UL << static_cast<N>(group));
  }

  void clear(T group) noexcept {
    bit = bit & ~(1UL << static_cast<N>(group));
  }
};

}

