#include <string>

#include "moteus_multiplex.h"

using namespace mjbots;

namespace som {
struct Printer {
  static const std::string Resolution(moteus::Resolution resolution) {
    switch (resolution) {
      case moteus::Resolution::kInt8:
        return "kInt8";
      case moteus::Resolution::kInt16:
        return "kInt16";
      case moteus::Resolution::kInt32:
        return "kInt32";
      case moteus::Resolution::kFloat:
        return "kFloat";
      case moteus::Resolution::kIgnore:
        return "kIgnore";
      default:
        return "<unknown>";
    }
  }
};
}  // namespace som