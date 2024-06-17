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

inline std::ostream& operator<<(std::ostream& os, const CommandItem& item) {
  switch (item) {
    case CommandItem::position:
      os << "position";
      break;
    case CommandItem::velocity:
      os << "velocity";
      break;
    case CommandItem::feedforward_torque:
      os << "feedforward_torque";
      break;
    case CommandItem::kp_scale:
      os << "kp_scale";
      break;
    case CommandItem::kd_scale:
      os << "kd_scale";
      break;
    case CommandItem::maximum_torque:
      os << "maximum_torque";
      break;
    /* Command Item stop_position is omitted to prevent faults since
       using it with velocity_limit or accel_limit causes fault. */
    case CommandItem::watchdog_timeout:
      os << "watchdog_timeout";
      break;
    case CommandItem::velocity_limit:
      os << "velocity_limit";
      break;
    case CommandItem::accel_limit:
      os << "accel_limit";
      break;
    case CommandItem::fixed_voltage_override:
      os << "fixed_voltage_override";
      break;
    case CommandItem::ilimit_scale:
      os << "ilimit_scale";
      break;
    default:
      os << "<Unknown>";
      break;
  }
  return os;
}

}  // namespace som
