#pragma once

#include "imports.h"

namespace gf3 ::utils {

double GetTime() {
  struct timespec ts;
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) * 1e-9;
}

template <typename K, typename V>
static std::optional<V> SafeAt(const std::map<K, V>& map, const K& key) {
  auto it = map.find(key);
  if (it == map.end()) {
    return std::nullopt;
  } else {
    return it->second;
  }
}

static bool IsLittleEndian() {
  static union {
    uint64_t eight_byte_int;
    char eight_chars[8];
  } random_entity = {0x00feeddeadbeef01};

  return random_entity.eight_chars[0];
}

template <typename T>
void Merge(std::vector<T>& v1, const std::vector<T>& v2) {
  for (const auto& t : v2) v1.push_back(t);
}

class Beat {
 public:
  Beat(const double& interval)
      : next_beat_{utils::GetTime() + interval}, interval_{interval} {}

  bool Hit() {
    if (utils::GetTime() >= next_beat_) {
      while (utils::GetTime() >= next_beat_) next_beat_ += interval_;
      return true;
    } else {
      return false;
    }
  }

 private:
  double next_beat_;
  const double interval_;
};

}  // namespace gf3::utils
