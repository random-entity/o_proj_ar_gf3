#pragma once

#include "utils.h"

namespace gf3 {

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
  uint32_t next_beat_;
  const uint32_t interval_;
};

}  // namespace gf3