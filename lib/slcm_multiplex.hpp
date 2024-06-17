#pragma once

#include <algorithm>
#include <iostream>
#include <optional>
#include <set>
#include <string>
#include <typeindex>
#include <vector>

#include "moteus_protocol.h"
#include "slcm_printer.hpp"
#include "slcm_utils.hpp"

namespace som {
using namespace mjbots;

enum class CmdPosRelTo { Absolute, Base, Recent };
enum class RplPosRelTo { Absolute, Base };
enum class WhichPos { Internal, Aux2 };

enum class CommandItem {
  position,
  velocity,
  feedforward_torque,
  kp_scale,
  kd_scale,
  maximum_torque,
  stop_position, /* Command Item stop_position causes fault
                    if used with velocity_limit or accel_limit */
  watchdog_timeout,
  velocity_limit,
  accel_limit,
  fixed_voltage_override,
  ilimit_scale
};

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
    case CommandItem::stop_position:
      os << "stop_position";
      break;
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
               const std::ptrdiff_t& offset_fmt)
      : offset_cmdrpl_{offset_cmdrpl}, offset_fmt_{offset_fmt} {}

  const std::ptrdiff_t
      offset_cmdrpl_;  // Address offset of corresponding member in struct
                       // PositionMode::Command or Query::Result.
  const std::ptrdiff_t
      offset_fmt_;  // Address offset of corresponding member
                    // in struct PositionMode::Format or Query::Format.
};

class CmdItemMetadata : public ItemMetadata {
  friend class CmdItemsMgr;

 private:
  CmdItemMetadata(const std::ptrdiff_t& offset_cmdrpl,
                  const std::ptrdiff_t& offset_fmt)
      : ItemMetadata{offset_cmdrpl, offset_fmt} {}
};

class RplItemMetadata : public ItemMetadata {
  friend class RplItemsMgr;

 private:
  RplItemMetadata(const std::ptrdiff_t& offset_cmdrpl,
                  const std::ptrdiff_t& offset_fmt, const std::type_index& type)
      : ItemMetadata{offset_cmdrpl, offset_fmt}, type_{type} {}

  const std::type_index type_;  // Runtime type ID of corresponding member
};

class CmdItemsMgr {
 private:
  inline static const moteus::PositionMode::Command c_;
  inline static const moteus::PositionMode::Format f_;

  inline static const CmdItemMetadata position_ = {
      Utils::GetAddrOffset(c_, c_.position),  //
      Utils::GetAddrOffset(f_, f_.position)   //
  };
  inline static const CmdItemMetadata velocity_ = {
      Utils::GetAddrOffset(c_, c_.velocity),  //
      Utils::GetAddrOffset(f_, f_.velocity)   //
  };
  inline static const CmdItemMetadata feedforward_torque_ = {
      Utils::GetAddrOffset(c_, c_.feedforward_torque),  //
      Utils::GetAddrOffset(f_, f_.feedforward_torque)   //
  };
  inline static const CmdItemMetadata kp_scale_ = {
      Utils::GetAddrOffset(c_, c_.kp_scale),  //
      Utils::GetAddrOffset(f_, f_.kp_scale)   //
  };
  inline static const CmdItemMetadata kd_scale_ = {
      Utils::GetAddrOffset(c_, c_.kd_scale),  //
      Utils::GetAddrOffset(f_, f_.kd_scale)   //
  };
  inline static const CmdItemMetadata maximum_torque_ = {
      Utils::GetAddrOffset(c_, c_.maximum_torque),  //
      Utils::GetAddrOffset(f_, f_.maximum_torque)   //
  };
  /* Command Item stop_position causes fault
     if used with velocity_limit or accel_limit */
  inline static const CmdItemMetadata stop_position_ = {
      Utils::GetAddrOffset(c_, c_.stop_position),  //
      Utils::GetAddrOffset(f_, f_.stop_position)   //
  };
  inline static const CmdItemMetadata watchdog_timeout_ = {
      Utils::GetAddrOffset(c_, c_.watchdog_timeout),  //
      Utils::GetAddrOffset(f_, f_.watchdog_timeout)   //
  };
  inline static const CmdItemMetadata velocity_limit_ = {
      Utils::GetAddrOffset(c_, c_.velocity_limit),  //
      Utils::GetAddrOffset(f_, f_.velocity_limit)   //
  };
  inline static const CmdItemMetadata accel_limit_ = {
      Utils::GetAddrOffset(c_, c_.accel_limit),  //
      Utils::GetAddrOffset(f_, f_.accel_limit)   //
  };
  inline static const CmdItemMetadata fixed_voltage_override_ = {
      Utils::GetAddrOffset(c_, c_.fixed_voltage_override),  //
      Utils::GetAddrOffset(f_, f_.fixed_voltage_override)   //
  };
  inline static const CmdItemMetadata ilimit_scale_ = {
      Utils::GetAddrOffset(c_, c_.ilimit_scale),  //
      Utils::GetAddrOffset(f_, f_.ilimit_scale)   //
  };

  inline static const std::map<CommandItem, CmdItemMetadata> metadata_ = {
      {CommandItem::position, position_},                      //
      {CommandItem::velocity, velocity_},                      //
      {CommandItem::feedforward_torque, feedforward_torque_},  //
      {CommandItem::kp_scale, kp_scale_},                      //
      {CommandItem::kd_scale, kd_scale_},                      //
      {CommandItem::maximum_torque, maximum_torque_},          //
      /* Command Item stop_position causes fault
         if used with velocity_limit or accel_limit */
      {CommandItem::stop_position, stop_position_},
      {CommandItem::watchdog_timeout, watchdog_timeout_},              //
      {CommandItem::velocity_limit, velocity_limit_},                  //
      {CommandItem::accel_limit, accel_limit_},                        //
      {CommandItem::fixed_voltage_override, fixed_voltage_override_},  //
      {CommandItem::ilimit_scale, ilimit_scale_}                       //
  };

  inline static const std::map<std::string, CommandItem> aliases_ = {
      {"position", CommandItem::position},
      {"pos", CommandItem::position},
      {"velocity", CommandItem::velocity},
      {"vel", CommandItem::velocity},
      {"feedforward_torque", CommandItem::feedforward_torque},
      {"fft", CommandItem::feedforward_torque},
      {"kp_scale", CommandItem::kp_scale},
      {"kps", CommandItem::kp_scale},
      {"kd_scale", CommandItem::kd_scale},
      {"kds", CommandItem::kd_scale},
      {"maximum_torque", CommandItem::maximum_torque},
      {"mxt", CommandItem::maximum_torque},
      /* Command Item stop_position causes fault
         if used with velocity_limit or accel_limit */
      {"stop_position", CommandItem::stop_position},
      {"spo", CommandItem::stop_position},
      {"watchdog_timeout", CommandItem::watchdog_timeout},
      {"wto", CommandItem::watchdog_timeout},
      {"velocity_limit", CommandItem::velocity_limit},
      {"vlm", CommandItem::velocity_limit},
      {"accel_limit", CommandItem::accel_limit},
      {"alm", CommandItem::accel_limit},
      {"fixed_voltage_override", CommandItem::fixed_voltage_override},
      {"fvo", CommandItem::fixed_voltage_override},
      {"ilimit_scale", CommandItem::ilimit_scale},
      {"ils", CommandItem::ilimit_scale},
  };

 public:
  static double* ItemToPtr(const CommandItem item,
                           const moteus::PositionMode::Command& cmd) {
    auto* non_const = const_cast<moteus::PositionMode::Command*>(&cmd);
    return reinterpret_cast<double*>(reinterpret_cast<char*>(non_const) +
                                     metadata_.at(item).offset_cmdrpl_);
  }

  static moteus::Resolution* ItemToPtr(
      const CommandItem item, const moteus::PositionMode::Format& fmt) {
    auto* non_const = const_cast<moteus::PositionMode::Format*>(&fmt);
    return reinterpret_cast<moteus::Resolution*>(
        reinterpret_cast<char*>(non_const) + metadata_.at(item).offset_fmt_);
  }

  static std::optional<CommandItem> StrToItem(const std::string& str) {
    const auto& maybe_item = Utils::SafeAt(aliases_, str);
    if (maybe_item) {
      return maybe_item.value();
    } else {
      return std::nullopt;
    }
  }
};

class RplItemsMgr {
 private:
  inline static const moteus::Query::Result r_;
  inline static const moteus::Query::Format f_;

  inline static const RplItemMetadata mode_ = {
      Utils::GetAddrOffset(r_, r_.mode),  //
      Utils::GetAddrOffset(f_, f_.mode),  //
      typeid(r_.mode)                     //
  };
  inline static const RplItemMetadata position_ = {
      Utils::GetAddrOffset(r_, r_.position),  //
      Utils::GetAddrOffset(f_, f_.position),  //
      typeid(r_.position)                     //
  };
  inline static const RplItemMetadata velocity_ = {
      Utils::GetAddrOffset(r_, r_.velocity),  //
      Utils::GetAddrOffset(f_, f_.velocity),  //
      typeid(r_.velocity)                     //
  };
  inline static const RplItemMetadata torque_ = {
      Utils::GetAddrOffset(r_, r_.torque),  //
      Utils::GetAddrOffset(f_, f_.torque),  //
      typeid(r_.torque)                     //
  };
  inline static const RplItemMetadata q_current_ = {
      Utils::GetAddrOffset(r_, r_.q_current),  //
      Utils::GetAddrOffset(f_, f_.q_current),  //
      typeid(r_.q_current)                     //
  };
  inline static const RplItemMetadata d_current_ = {
      Utils::GetAddrOffset(r_, r_.d_current),  //
      Utils::GetAddrOffset(f_, f_.d_current),  //
      typeid(r_.d_current)                     //
  };
  inline static const RplItemMetadata abs_position_ = {
      Utils::GetAddrOffset(r_, r_.abs_position),  //
      Utils::GetAddrOffset(f_, f_.abs_position),  //
      typeid(r_.abs_position)                     //
  };
  inline static const RplItemMetadata power_ = {
      Utils::GetAddrOffset(r_, r_.power),  //
      Utils::GetAddrOffset(f_, f_.power),  //
      typeid(r_.power)                     //
  };
  inline static const RplItemMetadata motor_temperature_ = {
      Utils::GetAddrOffset(r_, r_.motor_temperature),  //
      Utils::GetAddrOffset(f_, f_.motor_temperature),  //
      typeid(r_.motor_temperature)                     //
  };
  inline static const RplItemMetadata trajectory_complete_ = {
      Utils::GetAddrOffset(r_, r_.trajectory_complete),  //
      Utils::GetAddrOffset(f_, f_.trajectory_complete),  //
      typeid(r_.trajectory_complete)                     //
  };
  inline static const RplItemMetadata home_state_ = {
      Utils::GetAddrOffset(r_, r_.home_state),  //
      Utils::GetAddrOffset(f_, f_.home_state),  //
      typeid(r_.home_state)                     //
  };
  inline static const RplItemMetadata voltage_ = {
      Utils::GetAddrOffset(r_, r_.voltage),  //
      Utils::GetAddrOffset(f_, f_.voltage),  //
      typeid(r_.voltage)                     //
  };
  inline static const RplItemMetadata temperature_ = {
      Utils::GetAddrOffset(r_, r_.temperature),  //
      Utils::GetAddrOffset(f_, f_.temperature),  //
      typeid(r_.temperature)                     //
  };
  inline static const RplItemMetadata fault_ = {
      Utils::GetAddrOffset(r_, r_.fault),  //
      Utils::GetAddrOffset(f_, f_.fault),  //
      typeid(r_.fault)                     //
  };

  inline static const std::map<ReplyItem, RplItemMetadata> metadata_ = {
      {ReplyItem::mode, mode_},                                //
      {ReplyItem::position, position_},                        //
      {ReplyItem::velocity, velocity_},                        //
      {ReplyItem::torque, torque_},                            //
      {ReplyItem::q_current, q_current_},                      //
      {ReplyItem::d_current, d_current_},                      //
      {ReplyItem::abs_position, abs_position_},                //
      {ReplyItem::power, power_},                              //
      {ReplyItem::motor_temperature, motor_temperature_},      //
      {ReplyItem::trajectory_complete, trajectory_complete_},  //
      {ReplyItem::home_state, home_state_},                    //
      {ReplyItem::voltage, voltage_},                          //
      {ReplyItem::temperature, temperature_},                  //
      {ReplyItem::fault, fault_}                               //
  };

  inline static const std::map<std::string, ReplyItem> aliases_ = {
      {"mode", ReplyItem::mode},                                //
      {"mod", ReplyItem::mode},                                 //
      {"position", ReplyItem::position},                        //
      {"pos", ReplyItem::position},                             //
      {"velocity", ReplyItem::velocity},                        //
      {"vel", ReplyItem::velocity},                             //
      {"torque", ReplyItem::torque},                            //
      {"trq", ReplyItem::torque},                               //
      {"q_current", ReplyItem::q_current},                      //
      {"qcr", ReplyItem::q_current},                            //
      {"d_current", ReplyItem::d_current},                      //
      {"dcr", ReplyItem::d_current},                            //
      {"abs_position", ReplyItem::abs_position},                //
      {"apo", ReplyItem::abs_position},                         //
      {"power", ReplyItem::power},                              //
      {"pwr", ReplyItem::power},                                //
      {"motor_temperature", ReplyItem::motor_temperature},      //
      {"mtp", ReplyItem::motor_temperature},                    //
      {"trajectory_complete", ReplyItem::trajectory_complete},  //
      {"tjc", ReplyItem::trajectory_complete},                  //
      {"home_state", ReplyItem::home_state},                    //
      {"hom", ReplyItem::home_state},                           //
      {"voltage", ReplyItem::voltage},                          //
      {"vlt", ReplyItem::voltage},                              //
      {"temperature", ReplyItem::temperature},                  //
      {"tem", ReplyItem::temperature},                          //
      {"fault", ReplyItem::fault},                              //
      {"flt", ReplyItem::fault}                                 //
  };

 public:
  template <typename ItemType>
  static ItemType* ItemToPtr(const ReplyItem item,
                             const moteus::Query::Result& rpl) {
    auto* non_const = const_cast<moteus::Query::Result*>(&rpl);
    return reinterpret_cast<ItemType*>(reinterpret_cast<char*>(non_const) +
                                       metadata_.at(item).offset_cmdrpl_);
  }

  static float ItemToFloat(const ReplyItem item,
                           const moteus::Query::Result& rpl) {
    auto* non_const = const_cast<moteus::Query::Result*>(&rpl);
    auto metadata = metadata_.at(item);
    if (metadata.type_ == typeid(double)) {
      return static_cast<float>(*reinterpret_cast<double*>(
          reinterpret_cast<char*>(non_const) + metadata.offset_cmdrpl_));
    } else if (metadata.type_ == typeid(moteus::Mode)) {
      return static_cast<float>(*reinterpret_cast<moteus::Mode*>(
          reinterpret_cast<char*>(non_const) + metadata.offset_cmdrpl_));
    } else if (metadata.type_ == typeid(moteus::HomeState)) {
      return static_cast<float>(*reinterpret_cast<moteus::HomeState*>(
          reinterpret_cast<char*>(non_const) + metadata.offset_cmdrpl_));
    } else {
      std::cout << "Reply Item is of unsupported type." << std::endl;
      return 1234.5678f;
    }
  }

  static moteus::Resolution* ItemToPtr(const ReplyItem item,
                                       const moteus::Query::Format& fmt) {
    auto* non_const = const_cast<moteus::Query::Format*>(&fmt);
    return reinterpret_cast<moteus::Resolution*>(
        reinterpret_cast<char*>(non_const) + metadata_.at(item).offset_fmt_);
  }

  static std::optional<ReplyItem> StrToItem(const std::string& str) {
    const auto& maybe_item = Utils::SafeAt(aliases_, str);
    if (maybe_item) {
      return maybe_item.value();
    } else {
      return std::nullopt;
    }
  }
};

}  // namespace som
