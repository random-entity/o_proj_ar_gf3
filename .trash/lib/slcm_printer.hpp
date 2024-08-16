#pragma once

#include <iostream>

#include "moteus_multiplex.h"
#include "slcm_multiplex.hpp"

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
  os << CommandItemsManager::ItemToName(item);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const ReplyItem& item) {
  os << ReplyItemsManager::ItemToName(item);
  return os;
}

inline std::ostream& operator<<(std::ostream& os,
                                const moteus::PositionMode::Command& cmd) {
  for (const auto& item : CommandItemsManager::items_) {
    os << item << ": " << *CommandItemsManager::ItemToPtr(item, cmd)
       << std::endl;
  }
  return os;
}

inline std::ostream& operator<<(std::ostream& os,
                                const moteus::PositionMode::Format& fmt) {
  for (const auto& item : CommandItemsManager::items_) {
    os << item << ": " << *CommandItemsManager::ItemToPtr(item, fmt)
       << std::endl;
  }
  return os;
}

inline std::ostream& operator<<(std::ostream& os,
                                const moteus::Query::Result& rpl) {
  for (const auto& item : ReplyItemsManager::items_) {
    os << item << ": " << ReplyItemsManager::ItemToFloat(item, rpl)
       << std::endl;
  }
  return os;
}

inline std::ostream& operator<<(std::ostream& os,
                                const moteus::Query::Format& fmt) {
  for (const auto& item : ReplyItemsManager::items_) {
    os << item << ": " << *ReplyItemsManager::ItemToPtr(item, fmt) << std::endl;
  }
  return os;
}

}  // namespace som
