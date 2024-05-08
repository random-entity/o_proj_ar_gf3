#include <time.h>

namespace som {
namespace helpers {
static double GetNow() {  // Get precise current time in seconds
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) * 1e-9;
}
}  // namespace helpers
}  // namespace som