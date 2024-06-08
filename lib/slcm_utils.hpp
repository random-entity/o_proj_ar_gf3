#pragma once

#include <cxxabi.h>
#include <time.h>

#include <map>
#include <optional>
#include <string>

namespace som {

/// @brief A collection of utilty functions.
///        All methods are declared as static.
struct Utils {
  /// @brief Get precise current time in seconds.
  static double GetNow() {
    struct timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<double>(ts.tv_sec) +
           static_cast<double>(ts.tv_nsec) * 1e-9;
  }

  /// @brief Safely get value by key from a map.
  /// @return Optional object that might hold the desired value.
  ///         Use boolean operator or .has_value() method
  ///         to check if the get operation succeeded.
  template <typename K, typename V>
  static std::optional<V> SafeAt(const std::map<K, V>& map, const K& key) {
    std::optional<V> maybe_value;
    try {
      maybe_value.emplace(map.at(key));
    } catch (std::out_of_range e) {
      // Key not found, but do nothing.
    }
    return maybe_value;
  }

  /// @brief Get class name of an object pointed to by the given pointer.
  /// @return The name of the class at success, an empty string elsewhere.
  template <class T>
  static std::string GetClassName(T* ptr) {
    int status;
    char* class_name_buffer = static_cast<char*>(malloc(32));
    abi::__cxa_demangle(typeid(*ptr).name(), class_name_buffer, nullptr,
                        &status);
    std::string class_name_string =
        status == 0 ? static_cast<std::string>(class_name_buffer) : "";
    free(class_name_buffer);
    return class_name_string;
  }

  /// @brief Check system endianness.
  static bool IsLittleEndian() {
    static union {
      uint32_t four_byte_int;
      char four_chars[4];
    } random_entity = {0x00cafe01};

    return random_entity.four_chars[0];
  }
};

}  // namespace som
