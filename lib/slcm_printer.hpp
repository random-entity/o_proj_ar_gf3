#pragma once

#include <iostream>

#include "moteus_multiplex.h"

namespace som {
using namespace mjbots;

inline std::ostream& operator<<(std::ostream& os,
                                const moteus::Resolution& resolution) {
  switch (resolution) {
    case moteus::Resolution::kInt8:
      os << "kInt8";
      break;
    case moteus::Resolution::kInt16:
      os << "kInt16";
      break;
    case moteus::Resolution::kInt32:
      os << "kInt32";
      break;
    case moteus::Resolution::kFloat:
      os << "kFloat";
      break;
    case moteus::Resolution::kIgnore:
      os << "kIgnore";
      break;
    default:
      os << "<Unknown>";
      break;
  }
  return os;
}

}  // namespace som
