#pragma once

#include <stdint.h>

#include <iostream>

#include "moteus_multiplex.h"

using namespace mjbots;

namespace som {

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
      os << "<Unknown Resolution>";
      break;
  }
  return os;
}

enum CommandType : size_t {
  POSITION = 0,
  VELOCITY = 1,
  FEEDFORWARD_TORQUE = 2,
  KP_SCALE = 3,
  KD_SCALE = 4,
  MAXIMUM_TORQUE = 5,
  STOP_POSITION = 6,
  WATCHDOG_TIMEOUT = 7,
  VELOCITY_LIMIT = 8,
  ACCEL_LIMIT = 9,
  FIXED_VOLTAGE_OVERRIDE = 10
};

inline std::ostream& operator<<(std::ostream& os, const CommandType& type) {
  switch (type) {
    case POSITION:
      os << "Position";
      break;
    case VELOCITY:
      os << "Velocity";
      break;
    case FEEDFORWARD_TORQUE:
      os << "Feedforward Torque";
      break;
    case KP_SCALE:
      os << "Kp Scale";
      break;
    case KD_SCALE:
      os << "Kd Scale";
      break;
    case MAXIMUM_TORQUE:
      os << "Maximum Torque";
      break;
    case STOP_POSITION:
      os << "Stop Position";
      break;
    case WATCHDOG_TIMEOUT:
      os << "Watchdog Timeout";
      break;
    case VELOCITY_LIMIT:
      os << "Velocity Limit";
      break;
    case ACCEL_LIMIT:
      os << "Accel Limit";
      break;
    case FIXED_VOLTAGE_OVERRIDE:
      os << "Fixed Voltage Override";
      break;
    default:
      os << "<Unknown CommandType>";
      break;
  }
  return os;
}

enum ListeningMode { EXTERNAL, INTERNAL };

inline std::ostream& operator<<(std::ostream& os, const ListeningMode& mode) {
  switch (mode) {
    case EXTERNAL:
      os << "External";
      break;
    case INTERNAL:
      os << "Internal";
      break;
    default:
      os << "<Unknown ListeningMode>";
      break;
  }
  return os;
}

}  // namespace som
