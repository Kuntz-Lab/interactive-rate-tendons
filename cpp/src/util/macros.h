#ifndef UTIL_MACROS_H
#define UTIL_MACROS_H

#include <type_traits>

#ifndef UNUSED_VAR
#define UNUSED_VAR(x) (void)x
#endif // UNUSED_VAR

// This macro is to allow easily defining common operators for enum types
// intended to be used as flags with bitwise and and or
#define ENUM_FLAG_OPERATORS(T) \
    inline T operator~ (T a) { \
      return static_cast<T>( ~static_cast<std::underlying_type<T>::type>(a) ); \
    } \
    inline T operator| (T a, T b) { \
      return static_cast<T>( \
          static_cast<std::underlying_type<T>::type>(a) \
          | static_cast<std::underlying_type<T>::type>(b) ); \
    } \
    inline T operator& (T a, T b) { \
      return static_cast<T>( \
          static_cast<std::underlying_type<T>::type>(a) \
          & static_cast<std::underlying_type<T>::type>(b) ); \
    } \
    inline T operator^ (T a, T b) { \
      return static_cast<T>( \
          static_cast<std::underlying_type<T>::type>(a) \
          ^ static_cast<std::underlying_type<T>::type>(b) ); \
    } \
    inline T& operator|= (T& a, T b) { \
      return reinterpret_cast<T&>( \
          reinterpret_cast<std::underlying_type<T>::type&>(a) \
              |= static_cast<std::underlying_type<T>::type>(b) ); \
    } \
    inline T& operator&= (T& a, T b) { \
      return reinterpret_cast<T&>( \
          reinterpret_cast<std::underlying_type<T>::type&>(a) \
              &= static_cast<std::underlying_type<T>::type>(b) ); \
    } \
    inline T& operator^= (T& a, T b) { \
      return reinterpret_cast<T&>( \
          reinterpret_cast<std::underlying_type<T>::type&>(a) \
              ^= static_cast<std::underlying_type<T>::type>(b) ); \
    }

#endif // UTIL_MACROS_H
