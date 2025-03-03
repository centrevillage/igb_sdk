#pragma once

#include <string.h>
#include <cstddef>
#include <cstdint>
#include <array>
#include <type_traits>
#include <igb_util/macro.hpp>

namespace igb {

// TODO: move these concepts to common utility file
template<class T>
struct is_array : std::is_array<T>{};

template<class T, std::size_t N>
struct is_array<std::array<T,N>> : std::true_type {};

template <typename T> struct is_std_array : std::false_type{};
template<class T, std::size_t N>
struct is_std_array<std::array<T,N>> : std::true_type {};

template <class T>
concept SerializedBufSize = requires (T& x) {
  T::serializedBufSize();
};

template <class T>
concept Serializable = requires (T& x) {
  T::serializedBufSize();
  x.serialize((uint8_t*)0, (size_t)0);
};

template <class T>
concept Deserializable = requires (T& x) {
  T::serializedBufSize();
  x.deserialize((uint8_t*)0, (size_t)0);
};

template<typename E>
concept EnumType = std::is_enum_v<E>;

template <class T>
concept ArithmeticType = std::is_arithmetic_v<T>;

template <class T>
concept NativeArrayType = std::is_array_v<T>;

template <class T>
concept StdArrayType = is_std_array<T>::value;

template <class T>
concept ArrayType = is_array<T>::value;

struct Serializer {
  
  template <ArithmeticType T>
  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const T v) {
    memcpy(buf, &v, sizeof(v));
    return sizeof(v);
  }

  template <ArithmeticType T>
  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, T& v) {
    memcpy(&v, buf, sizeof(v));
    return sizeof(v);
  }

  template<ArithmeticType T>
  constexpr static auto size() -> size_t {
    return sizeof(T);
  }

  template<ArithmeticType T>
  constexpr static auto size(const T v) -> size_t {
    return sizeof(v);
  }

  // for enum
  template<EnumType T>
  constexpr static auto size() -> size_t {
    return sizeof(std::underlying_type_t<T>);
  }

  template<EnumType T>
  constexpr static auto size(const T& v) -> size_t {
    return sizeof(std::underlying_type_t<T>);
  }

  template<EnumType T>
  constexpr static size_t serialize(uint8_t* buf, const T& type) {
    return serialize(buf, (std::underlying_type_t<T>)type);
  }

  template<EnumType T>
  constexpr static size_t deserialize(uint8_t* buf, T& type) {
    std::underlying_type_t<T> v;
    auto ret = deserialize(buf, v);
    type = (T)v;
    return ret;
  }

  template<ArrayType T>
  constexpr static size_t size(const T& v) {
    return size(v[0]) * std::size(v);
  }

  template<StdArrayType T>
  constexpr static size_t size() {
    return size<std::tuple_element_t<0, T>>() * std::tuple_size_v<T>;
  }

  template<ArrayType T>
  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const T& values) {
    for (const auto& v : values) {
      buf += serialize(buf, v);
    }
    return size(values);
  }

  // TODO: smart implementation for N >= 2 dim array
  template<typename T, size_t array_size1, size_t array_size2>
  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const std::array<std::array<T, array_size2>, array_size1>& values) {
    for (const auto& v : values) {
      buf += serialize(buf, v);
    }
    return size<T>() * array_size1 * array_size2;
  }
  template<typename T, size_t array_size1, size_t array_size2>
  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const T(&values)[array_size1][array_size2]) {
    for (const auto& v : values) {
      buf += serialize(buf, v);
    }
    return size<T>() * array_size1 * array_size2;
  }

  template<ArrayType T>
  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, T& values) {
    for (auto& v : values) {
      buf += deserialize(buf, v);
    }
    return size(values);
  }

  // TODO: smart implementation for N >= 2 dim array
  template<typename T, size_t array_size1, size_t array_size2>
  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, std::array<std::array<T, array_size2>, array_size1>& values) {
    for (auto& v : values) {
      buf += deserialize(buf, v);
    }
    return size<T>() * array_size1 * array_size2;
  }
  template<typename T, size_t array_size1, size_t array_size2>
  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, T(&values)[array_size1][array_size2]) {
    for (auto& v : values) {
      buf += deserialize(buf, v);
    }
    return size<T>() * array_size1 * array_size2;
  }

  // for serializable object
  template<SerializedBufSize T>
  constexpr static size_t size(const T& type) {
    return T::serializedBufSize();
  }

  template<SerializedBufSize T>
  constexpr static auto size() -> size_t {
    return T::serializedBufSize();
  }

  template<Serializable T>
  constexpr static size_t serialize(uint8_t* buf, const T& type) {
    return type.serialize(buf, size<T>());
  }

  template<Deserializable T>
  constexpr static size_t deserialize(uint8_t* buf, T& type) {
    return type.deserialize(buf, size<T>());
  }
};

}
