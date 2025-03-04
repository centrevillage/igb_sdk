#pragma once

namespace igb {

// lambda overload pattern
template <class... Ts> struct overload : Ts... { using Ts::operator()...; };
template <class... Ts> overload(Ts...) -> overload<Ts...>;

} // namespace igb
