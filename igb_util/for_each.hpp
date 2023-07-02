#pragma once

#include <tuple>

namespace igb {

template<size_t N = 0, typename T>
void for_each_tuple(const T& tuple_val, auto&& func) {
  if constexpr(N < std::tuple_size<T>::value) {
    const auto& v = std::get<N>(tuple_val);
    func(N, v);
    for_each_tuple<N+1>(tuple_val, func);
  }
}

}

