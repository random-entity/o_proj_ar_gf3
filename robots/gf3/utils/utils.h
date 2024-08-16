#pragma once

#include <time.h>

namespace gf3::utils {

double GetTime() {
  struct timespec ts;
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) * 1e-9;
}

}  // namespace gf3::utils
