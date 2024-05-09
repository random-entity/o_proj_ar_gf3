#pragma once

#include <time.h>

#include <map>
#include <optional>

namespace som {
namespace helpers {

static double GetNow() {  // Get precise current time in seconds
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) * 1e-9;
}

template <typename Key, typename Value>
static std::optional<Value> SafeAt(const std::map<Key, Value>& map,
                                   const Key& key) {
  std::optional<Value> result;
  try {
    result = map.at(key);
  } catch (std::out_of_range e) {
    // key not found
  }
  return result;
}

}  // namespace helpers
}  // namespace som
