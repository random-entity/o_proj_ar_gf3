#pragma once

#include <algorithm>
#include <iostream>
#include <optional>
#include <set>
#include <string>
#include <typeindex>
#include <vector>

#include "moteus_protocol.h"
#include "slcm_utils.hpp"

namespace som {
using namespace mjbots;

enum class CommandPositionRelativeTo { Absolute, Base, Recent };
enum class ReplyPositionRelativeTo { Absolute, Base };
enum class WhichPosition { Internal, Aux2 };

enum class CommandItem {
  position,
  velocity,
  feedforward_torque,
  kp_scale,
  kd_scale,
  maximum_torque,
  // Command Item stop_position is deprecated, and causes fault
  // if used along with velocity or acceleration limit. See fault code 44 at:
  // https://github.com/mjbots/moteus/blob/main/docs/reference.md#0x00f---fault-code
  stop_position,
  watchdog_timeout,
  velocity_limit,
  accel_limit,
  fixed_voltage_override,
  ilimit_scale
};

enum class ReplyItem {
  mode,
  position,
  velocity,
  torque,
  q_current,
  d_current,
  abs_position,
  power,
  motor_temperature,
  trajectory_complete,
  home_state,
  voltage,
  temperature,
  fault
};

class ItemMetadata {
 protected:
  ItemMetadata(const std::ptrdiff_t& offset_cmdrpl,
               const std::ptrdiff_t& offset_fmt,
               const std::vector<std::string>& aliases)
      : offset_cmdrpl_{offset_cmdrpl},
        offset_fmt_{offset_fmt},
        aliases_{aliases} {}

  // Address offset of corresponding member in
  // struct PositionMode::Command or Query::Result.
  const std::ptrdiff_t offset_cmdrpl_;

  // Address offset of corresponding member in
  // struct PositionMode::Format or Query::Format.
  const std::ptrdiff_t offset_fmt_;

  // Vector of aliases. The first element should be its full name.
  // The second element should be its 3-letter abbreviation.
  const std::vector<std::string> aliases_;
};

class CommandItemMetadata : public ItemMetadata {
  friend class CommandItemsManager;

 private:
  CommandItemMetadata(const std::ptrdiff_t& offset_cmdrpl,
                      const std::ptrdiff_t& offset_fmt,
                      const std::vector<std::string>& aliases)
      : ItemMetadata{offset_cmdrpl, offset_fmt, aliases} {}
};

class ReplyItemMetadata : public ItemMetadata {
  friend class ReplyItemsManager;

 private:
  ReplyItemMetadata(const std::ptrdiff_t& offset_cmdrpl,
                    const std::ptrdiff_t& offset_fmt,
                    const std::vector<std::string>& aliases,
                    const std::type_index& type)
      : ItemMetadata{offset_cmdrpl, offset_fmt, aliases}, type_{type} {}

  // Runtime type ID of corresponding member in struct Query::Result.
  const std::type_index type_;
};

class CommandItemsManager {
 private:
  inline static const moteus::PositionMode::Command c_;
  inline static const moteus::PositionMode::Format f_;

 public:
  inline static const std::map<CommandItem, CommandItemMetadata>
      item_to_metadata_ = {
          {CommandItem::position,
           {Utils::GetAddrOffset(c_, c_.position),
            Utils::GetAddrOffset(f_, f_.position),
            {"position", "pos"}}},
          {CommandItem::velocity,
           {Utils::GetAddrOffset(c_, c_.velocity),
            Utils::GetAddrOffset(f_, f_.velocity),
            {"velocity", "vel"}}},
          {CommandItem::feedforward_torque,
           {Utils::GetAddrOffset(c_, c_.feedforward_torque),
            Utils::GetAddrOffset(f_, f_.feedforward_torque),
            {"feedforward_torque", "fft"}}},
          {CommandItem::kp_scale,
           {Utils::GetAddrOffset(c_, c_.kp_scale),
            Utils::GetAddrOffset(f_, f_.kp_scale),
            {"kp_scale", "kps"}}},
          {CommandItem::kd_scale,
           {Utils::GetAddrOffset(c_, c_.kd_scale),
            Utils::GetAddrOffset(f_, f_.kd_scale),
            {"kd_scale", "kds"}}},
          {CommandItem::maximum_torque,
           {Utils::GetAddrOffset(c_, c_.maximum_torque),
            Utils::GetAddrOffset(f_, f_.maximum_torque),
            {"maximum_torque", "mxt"}}},
          {CommandItem::stop_position,
           {Utils::GetAddrOffset(c_, c_.stop_position),
            Utils::GetAddrOffset(f_, f_.stop_position),
            {"stop_position", "spo"}}},
          {CommandItem::watchdog_timeout,
           {Utils::GetAddrOffset(c_, c_.watchdog_timeout),
            Utils::GetAddrOffset(f_, f_.watchdog_timeout),
            {"watchdog_timeout", "wto"}}},
          {CommandItem::velocity_limit,
           {Utils::GetAddrOffset(c_, c_.velocity_limit),
            Utils::GetAddrOffset(f_, f_.velocity_limit),
            {"velocity_limit", "vlm"}}},
          {CommandItem::accel_limit,
           {Utils::GetAddrOffset(c_, c_.accel_limit),
            Utils::GetAddrOffset(f_, f_.accel_limit),
            {"accel_limit", "alm"}}},
          {CommandItem::fixed_voltage_override,
           {Utils::GetAddrOffset(c_, c_.fixed_voltage_override),
            Utils::GetAddrOffset(f_, f_.fixed_voltage_override),
            {"fixed_voltage_override", "fvo"}}},
          {CommandItem::ilimit_scale,
           {Utils::GetAddrOffset(c_, c_.ilimit_scale),
            Utils::GetAddrOffset(f_, f_.ilimit_scale),
            {"ilimit_scale", "ils"}}}};

  inline static const std::vector<CommandItem> items_ = []() {
    std::vector<CommandItem> items;
    for (const auto& item_metadata : CommandItemsManager::item_to_metadata_) {
      items.push_back(item_metadata.first);
    }
    return items;
  }();

  inline static const std::map<std::string, CommandItem> alias_to_item_ = []() {
    std::map<std::string, CommandItem> alias_to_item;
    for (const auto& item_metadata : item_to_metadata_) {
      const auto& item = item_metadata.first;
      const auto& metadata = item_metadata.second;
      for (const auto& alias : metadata.aliases_) {
        alias_to_item.insert({alias, item});
      }
    }
    return alias_to_item;
  }();

  static double* ItemToPtr(const CommandItem item,
                           const moteus::PositionMode::Command& cmd) {
    auto* cmd_non_const = const_cast<moteus::PositionMode::Command*>(&cmd);
    return reinterpret_cast<double*>(reinterpret_cast<char*>(cmd_non_const) +
                                     item_to_metadata_.at(item).offset_cmdrpl_);
  }

  static moteus::Resolution* ItemToPtr(
      const CommandItem item, const moteus::PositionMode::Format& fmt) {
    auto* fmt_non_const = const_cast<moteus::PositionMode::Format*>(&fmt);
    return reinterpret_cast<moteus::Resolution*>(
        reinterpret_cast<char*>(fmt_non_const) +
        item_to_metadata_.at(item).offset_fmt_);
  }

  static std::string ItemToName(const CommandItem item) {
    return item_to_metadata_.at(item).aliases_.at(0);
  }

  static std::optional<CommandItem> AliasToItem(const std::string& alias) {
    return Utils::SafeAt(alias_to_item_, alias);
  }
};

class ReplyItemsManager {
 private:
  inline static const moteus::Query::Result r_;
  inline static const moteus::Query::Format f_;

 public:
  inline static const std::map<ReplyItem, ReplyItemMetadata> item_to_metadata_ =
      {{ReplyItem::mode,
        {Utils::GetAddrOffset(r_, r_.mode),
         Utils::GetAddrOffset(f_, f_.mode),
         {"mode", "mod"},
         typeid(r_.mode)}},
       {ReplyItem::position,
        {Utils::GetAddrOffset(r_, r_.position),
         Utils::GetAddrOffset(f_, f_.position),
         {"position", "pos"},
         typeid(r_.position)}},
       {ReplyItem::velocity,
        {Utils::GetAddrOffset(r_, r_.velocity),
         Utils::GetAddrOffset(f_, f_.velocity),
         {"velocity", "vel"},
         typeid(r_.velocity)}},
       {ReplyItem::torque,
        {Utils::GetAddrOffset(r_, r_.torque),
         Utils::GetAddrOffset(f_, f_.torque),
         {"torque", "trq"},
         typeid(r_.torque)}},
       {ReplyItem::q_current,
        {Utils::GetAddrOffset(r_, r_.q_current),
         Utils::GetAddrOffset(f_, f_.q_current),
         {"q_current", "qcr"},
         typeid(r_.q_current)}},
       {ReplyItem::d_current,
        {Utils::GetAddrOffset(r_, r_.d_current),
         Utils::GetAddrOffset(f_, f_.d_current),
         {"d_current", "dcr"},
         typeid(r_.d_current)}},
       {ReplyItem::abs_position,
        {Utils::GetAddrOffset(r_, r_.abs_position),
         Utils::GetAddrOffset(f_, f_.abs_position),
         {"abs_position", "apo"},
         typeid(r_.abs_position)}},
       {ReplyItem::power,
        {Utils::GetAddrOffset(r_, r_.power),
         Utils::GetAddrOffset(f_, f_.power),
         {"power", "pwr"},
         typeid(r_.power)}},
       {ReplyItem::motor_temperature,
        {Utils::GetAddrOffset(r_, r_.motor_temperature),
         Utils::GetAddrOffset(f_, f_.motor_temperature),
         {"motor_temperature", "mtp"},
         typeid(r_.motor_temperature)}},
       {ReplyItem::trajectory_complete,
        {Utils::GetAddrOffset(r_, r_.trajectory_complete),
         Utils::GetAddrOffset(f_, f_.trajectory_complete),
         {"trajectory_complete", "tjc"},
         typeid(r_.trajectory_complete)}},
       {ReplyItem::home_state,
        {Utils::GetAddrOffset(r_, r_.home_state),
         Utils::GetAddrOffset(f_, f_.home_state),
         {"home_state", "hom"},
         typeid(r_.home_state)}},
       {ReplyItem::voltage,
        {Utils::GetAddrOffset(r_, r_.voltage),
         Utils::GetAddrOffset(f_, f_.voltage),
         {"voltage", "vlt"},
         typeid(r_.voltage)}},
       {ReplyItem::temperature,
        {Utils::GetAddrOffset(r_, r_.temperature),
         Utils::GetAddrOffset(f_, f_.temperature),
         {"temperature", "tem"},
         typeid(r_.temperature)}},
       {ReplyItem::fault,
        {Utils::GetAddrOffset(r_, r_.fault),
         Utils::GetAddrOffset(f_, f_.fault),
         {"fault", "flt"},
         typeid(r_.fault)}}};

  inline static const std::vector<ReplyItem> items_ = []() {
    std::vector<ReplyItem> items;
    for (const auto& item_metadata : ReplyItemsManager::item_to_metadata_) {
      items.push_back(item_metadata.first);
    }
    return items;
  }();

  inline static const std::map<std::string, ReplyItem> alias_to_item_ = []() {
    std::map<std::string, ReplyItem> alias_to_item;
    for (const auto& item_metadata : item_to_metadata_) {
      const auto& item = item_metadata.first;
      const auto& metadata = item_metadata.second;
      for (const auto& alias : metadata.aliases_) {
        alias_to_item.insert({alias, item});
      }
    }
    return alias_to_item;
  }();

  template <typename ItemType>
  static ItemType* ItemToPtr(const ReplyItem item,
                             const moteus::Query::Result& rpl) {
    auto* rpl_non_const = const_cast<moteus::Query::Result*>(&rpl);
    return reinterpret_cast<ItemType*>(
        reinterpret_cast<char*>(rpl_non_const) +
        item_to_metadata_.at(item).offset_cmdrpl_);
  }

  static float ItemToFloat(const ReplyItem item,
                           const moteus::Query::Result& rpl) {
    auto* rpl_non_const = const_cast<moteus::Query::Result*>(&rpl);
    auto metadata = item_to_metadata_.at(item);
    if (metadata.type_ == typeid(double)) {
      return static_cast<float>(*reinterpret_cast<double*>(
          reinterpret_cast<char*>(rpl_non_const) + metadata.offset_cmdrpl_));
    } else if (metadata.type_ == typeid(moteus::Mode)) {
      return static_cast<float>(*reinterpret_cast<moteus::Mode*>(
          reinterpret_cast<char*>(rpl_non_const) + metadata.offset_cmdrpl_));
    } else if (metadata.type_ == typeid(moteus::HomeState)) {
      return static_cast<float>(*reinterpret_cast<moteus::HomeState*>(
          reinterpret_cast<char*>(rpl_non_const) + metadata.offset_cmdrpl_));
    } else {
      std::cout << "Reply Item " << ItemToName(item)
                << "is of unsupported type." << std::endl;
      return 1234.5678f;
    }
  }

  static moteus::Resolution* ItemToPtr(const ReplyItem item,
                                       const moteus::Query::Format& fmt) {
    auto* fmt_non_const = const_cast<moteus::Query::Format*>(&fmt);
    return reinterpret_cast<moteus::Resolution*>(
        reinterpret_cast<char*>(fmt_non_const) +
        item_to_metadata_.at(item).offset_fmt_);
  }

  static std::string ItemToName(const ReplyItem item) {
    return item_to_metadata_.at(item).aliases_.at(0);
  }

  static std::optional<ReplyItem> AliasToItem(const std::string& str) {
    const auto& maybe_item = Utils::SafeAt(alias_to_item_, str);
    if (maybe_item) {
      return maybe_item.value();
    } else {
      return std::nullopt;
    }
  }
};

}  // namespace som
