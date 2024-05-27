#pragma once

#include <type_traits>

namespace igb {
  template <typename T> constexpr static T as(auto&& value) {
    return static_cast<T>(value);
  }

  // TODO: コンパイルエラーになるので要修正
  template <typename T> constexpr static auto enum_cast(const T& v) {
    return std::underlying_type<T>::type(v);
  }
}

