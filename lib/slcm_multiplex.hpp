#pragma once

#include <algorithm>
#include <iostream>
#include <set>
#include <string>
#include <typeindex>
#include <vector>

#include "moteus_protocol.h"
#include "slcm_utils.hpp"

namespace som {
using namespace mjbots;

enum CmdPosRelTo { cABSOLUTE, cBASE, cRECENT };
enum RplPosRelTo { rABSOLUTE, rBASE };
enum WhichPos { pINTERNAL, pAUX2 };

class Item {
 protected:
  Item(const std::ptrdiff_t& offset_cmdrpl, const std::ptrdiff_t& offset_fmt,
       const std::type_index& type, const std::vector<std::string>& aliases)
      : offset_cmdrpl_{offset_cmdrpl},
        offset_fmt_{offset_fmt},
        type_{type},
        aliases_{aliases} {}

  const std::ptrdiff_t
      offset_cmdrpl_;  // Address offset of corresponding member in struct
                       // PositionMode::Command or Query::Result.
  const std::ptrdiff_t
      offset_fmt_;  // Address offset of corresponding member
                    // in struct PositionMode::Format or Query::Format.
  const std::type_index type_;  // Runtime type ID of corresponding member
  const std::vector<std::string> aliases_;  // String aliases

 public:
  std::string GetName() const {
    if (aliases_.empty()) {
      return "";
    } else {
      return aliases_.at(0);
    }
  }

  bool operator<(const Item& other) const {
    return this->offset_fmt_ < other.offset_fmt_;
  }
};

class CmdItem : public Item {
  friend class CmdItems;

 private:
  CmdItem(const std::ptrdiff_t& offset_cmdrpl, const std::ptrdiff_t& offset_fmt,
          const std::type_index& type, const std::vector<std::string>& aliases)
      : Item{offset_cmdrpl, offset_fmt, type, aliases} {}
};

class RplItem : public Item {
  friend class RplItems;

 private:
  RplItem(const std::ptrdiff_t& offset_cmdrpl, const std::ptrdiff_t& offset_fmt,
          const std::type_index& type, const std::vector<std::string>& aliases)
      : Item{offset_cmdrpl, offset_fmt, type, aliases} {}
};

class CmdItems {
 private:
  inline static moteus::PositionMode::Command c_;
  inline static moteus::PositionMode::Format f_;

 public:
  inline static const CmdItem position = {
      Utils::GetAddrOffset(c_, c_.position),  //
      Utils::GetAddrOffset(f_, f_.position),  //
      typeid(c_.position),                    //
      {"position", "pos"}                     //
  };
  inline static const CmdItem velocity = {
      Utils::GetAddrOffset(c_, c_.velocity),  //
      Utils::GetAddrOffset(f_, f_.velocity),  //
      typeid(c_.velocity),                    //
      {"velocity", "vel"}                     //
  };
  inline static const CmdItem feedforward_torque = {
      Utils::GetAddrOffset(c_, c_.feedforward_torque),  //
      Utils::GetAddrOffset(f_, f_.feedforward_torque),  //
      typeid(c_.feedforward_torque),                    //
      {"feedforward_torque", "fft"}                     //
  };
  inline static const CmdItem kp_scale = {
      Utils::GetAddrOffset(c_, c_.kp_scale),  //
      Utils::GetAddrOffset(f_, f_.kp_scale),  //
      typeid(c_.kp_scale),                    //
      {"kp_scale", "kps"}                     //
  };
  inline static const CmdItem kd_scale = {
      Utils::GetAddrOffset(c_, c_.kd_scale),  //
      Utils::GetAddrOffset(f_, f_.kd_scale),  //
      typeid(c_.kd_scale),                    //
      {"kd_scale", "kds"}                     //
  };
  inline static const CmdItem maximum_torque = {
      Utils::GetAddrOffset(c_, c_.maximum_torque),  //
      Utils::GetAddrOffset(f_, f_.maximum_torque),  //
      typeid(c_.maximum_torque),                    //
      {"maximum_torque", "mxt"}                     //
  };
  /* Command Item stop_position is omitted to prevent faults since
     using it with velocity_limit or accel_limit causes fault. */
  inline static const CmdItem watchdog_timeout = {
      Utils::GetAddrOffset(c_, c_.watchdog_timeout),  //
      Utils::GetAddrOffset(f_, f_.watchdog_timeout),  //
      typeid(c_.watchdog_timeout),                    //
      {"watchdog_timeout", "wto"}                     //
  };
  inline static const CmdItem velocity_limit = {
      Utils::GetAddrOffset(c_, c_.velocity_limit),  //
      Utils::GetAddrOffset(f_, f_.velocity_limit),  //
      typeid(c_.velocity_limit),                    //
      {"velocity_limit", "vlm"}                     //
  };
  inline static const CmdItem accel_limit = {
      Utils::GetAddrOffset(c_, c_.accel_limit),  //
      Utils::GetAddrOffset(f_, f_.accel_limit),  //
      typeid(c_.accel_limit),                    //
      {"accel_limit", "alm"}                     //
  };
  inline static const CmdItem fixed_voltage_override = {
      Utils::GetAddrOffset(c_, c_.fixed_voltage_override),  //
      Utils::GetAddrOffset(f_, f_.fixed_voltage_override),  //
      typeid(c_.fixed_voltage_override),                    //
      {"fixed_voltage_override", "fvo"}                     //
  };
  inline static const CmdItem ilimit_scale = {
      Utils::GetAddrOffset(c_, c_.ilimit_scale),  //
      Utils::GetAddrOffset(f_, f_.ilimit_scale),  //
      typeid(c_.ilimit_scale),                    //
      {"ilimit_scale", "ils"}                     //
  };

  inline static const std::set<const CmdItem*> set_ = {
      &position,           &velocity,
      &feedforward_torque, &kp_scale,
      &kd_scale,           &maximum_torque,
      &watchdog_timeout,   &velocity_limit,
      &accel_limit,        &fixed_voltage_override,
      &ilimit_scale};

  static const CmdItem* const Find(const std::string& alias) {
    auto it =
        std::find_if(set_.begin(), set_.end(), [&alias](const CmdItem* item) {
          const auto& aliases = item->aliases_;
          return std::find(aliases.begin(), aliases.end(), alias) !=
                 aliases.end();
        });
    if (it == set_.end()) {
      return nullptr;
    } else {
      std::cout << "Command Item " << alias << " not found." << std::endl;
      return *it;
    }
  }

  static double* Get(const CmdItem& item,
                     const moteus::PositionMode::Command& cmd) {
    if (set_.find(&item) == set_.end()) {
      std::cout << "Command Item not found." << std::endl;
      return nullptr;
    } else if (item.type_ == typeid(double)) {
      auto* non_const = const_cast<moteus::PositionMode::Command*>(&cmd);
      return reinterpret_cast<double*>(reinterpret_cast<char*>(non_const) +
                                       item.offset_cmdrpl_);
    } else {
      std::cout << "Command Item is not of type double" << std::endl;
      return nullptr;
    }
  }

  static double* Get(const std::string& alias,
                     const moteus::PositionMode::Command& cmd) {
    const auto* item = Find(alias);
    if (item) {
      return Get(*item, cmd);
    } else {
      std::cout << "Command Item " << alias << " not found." << std::endl;
      return nullptr;
    }
  }

  static moteus::Resolution* Get(const CmdItem& item,
                                 const moteus::PositionMode::Format& fmt) {
    if (set_.find(&item) == set_.end()) {
      std::cout << "Command Item not found." << std::endl;
      return nullptr;
    } else {
      auto* non_const = const_cast<moteus::PositionMode::Format*>(&fmt);
      return reinterpret_cast<moteus::Resolution*>(
          reinterpret_cast<char*>(non_const) + item.offset_fmt_);
    }
  }

  static moteus::Resolution* Get(const std::string& alias,
                                 const moteus::PositionMode::Format& fmt) {
    const auto* item = Find(alias);
    if (item) {
      return Get(*item, fmt);
    } else {
      std::cout << "Command Item " << alias << " not found." << std::endl;
      return nullptr;
    }
  }
};

class RplItems {
 private:
  inline static moteus::Query::Result r_;
  inline static moteus::Query::Format f_;

 public:
  inline static const RplItem mode = {
      Utils::GetAddrOffset(r_, r_.mode),  //
      Utils::GetAddrOffset(f_, f_.mode),  //
      typeid(r_.mode),                    //
      {"mode", "mod"}                     //
  };
  inline static const RplItem position = {
      Utils::GetAddrOffset(r_, r_.position),  //
      Utils::GetAddrOffset(f_, f_.position),  //
      typeid(r_.position),                    //
      {"position", "pos"}                     //
  };
  inline static const RplItem velocity = {
      Utils::GetAddrOffset(r_, r_.velocity),  //
      Utils::GetAddrOffset(f_, f_.velocity),  //
      typeid(r_.velocity),                    //
      {"velocity", "vel"}                     //
  };
  inline static const RplItem torque = {
      Utils::GetAddrOffset(r_, r_.torque),  //
      Utils::GetAddrOffset(f_, f_.torque),  //
      typeid(r_.torque),                    //
      {"torque", "trq"}                     //
  };
  inline static const RplItem q_current = {
      Utils::GetAddrOffset(r_, r_.q_current),  //
      Utils::GetAddrOffset(f_, f_.q_current),  //
      typeid(r_.q_current),                    //
      {"q_current", "qcr"}                     //
  };
  inline static const RplItem d_current = {
      Utils::GetAddrOffset(r_, r_.d_current),  //
      Utils::GetAddrOffset(f_, f_.d_current),  //
      typeid(r_.d_current),                    //
      {"d_current", "dcr"}                     //
  };
  inline static const RplItem abs_position = {
      Utils::GetAddrOffset(r_, r_.abs_position),  //
      Utils::GetAddrOffset(f_, f_.abs_position),  //
      typeid(r_.abs_position),                    //
      {"abs_position", "apo"}                     //
  };
  inline static const RplItem power = {
      Utils::GetAddrOffset(r_, r_.power),  //
      Utils::GetAddrOffset(f_, f_.power),  //
      typeid(r_.power),                    //
      {"power", "pwr"}                     //
  };
  inline static const RplItem motor_temperature = {
      Utils::GetAddrOffset(r_, r_.motor_temperature),  //
      Utils::GetAddrOffset(f_, f_.motor_temperature),  //
      typeid(r_.motor_temperature),                    //
      {"motor_temperature", "mtp"}                     //
  };
  inline static const RplItem trajectory_complete = {
      Utils::GetAddrOffset(r_, r_.trajectory_complete),  //
      Utils::GetAddrOffset(f_, f_.trajectory_complete),  //
      typeid(r_.trajectory_complete),                    //
      {"trajectory_complete", "tjc"}                     //
  };
  inline static const RplItem home_state = {
      Utils::GetAddrOffset(r_, r_.home_state),  //
      Utils::GetAddrOffset(f_, f_.home_state),  //
      typeid(r_.home_state),                    //
      {"home_state", "hom"}                     //
  };
  inline static const RplItem voltage = {
      Utils::GetAddrOffset(r_, r_.voltage),  //
      Utils::GetAddrOffset(f_, f_.voltage),  //
      typeid(r_.voltage),                    //
      {"voltage", "vlt"}                     //
  };
  inline static const RplItem temperature = {
      Utils::GetAddrOffset(r_, r_.temperature),  //
      Utils::GetAddrOffset(f_, f_.temperature),  //
      typeid(r_.temperature),                    //
      {"temperature", "tem"}                     //
  };
  inline static const RplItem fault = {
      Utils::GetAddrOffset(r_, r_.fault),  //
      Utils::GetAddrOffset(f_, f_.fault),  //
      typeid(r_.fault),                    //
      {"fault", "flt"}                     //
  };

  inline static const std::set<const RplItem*> set_ = {&mode,
                                                       &position,
                                                       &velocity,
                                                       &torque,
                                                       &q_current,
                                                       &d_current,
                                                       &abs_position,
                                                       &power,
                                                       &motor_temperature,
                                                       &trajectory_complete,
                                                       &home_state,
                                                       &voltage,
                                                       &temperature,
                                                       &fault};

  static const RplItem* const Find(const std::string& alias) {
    auto it =
        std::find_if(set_.begin(), set_.end(), [&alias](const RplItem* item) {
          const auto& aliases = item->aliases_;
          return std::find(aliases.begin(), aliases.end(), alias) !=
                 aliases.end();
        });
    if (it == set_.end()) {
      return nullptr;
    } else {
      std::cout << "Reply Item " << alias << " not found." << std::endl;
      return *it;
    }
  }

  static float GetAsFloat(const RplItem& item,
                          const moteus::Query::Result& rpl) {
    if (set_.find(&item) == set_.end()) {
      std::cout << "Reply Item not found." << std::endl;
      return 8765.4321f;
    } else {
      auto* non_const = const_cast<moteus::Query::Result*>(&rpl);
      if (item.type_ == typeid(double)) {
        return static_cast<float>(*reinterpret_cast<double*>(
            reinterpret_cast<char*>(non_const) + item.offset_cmdrpl_));
      } else if (item.type_ == typeid(moteus::Mode)) {
        return static_cast<float>(*reinterpret_cast<moteus::Mode*>(
            reinterpret_cast<char*>(non_const) + item.offset_cmdrpl_));
      } else if (item.type_ == typeid(moteus::HomeState)) {
        return static_cast<float>(*reinterpret_cast<moteus::HomeState*>(
            reinterpret_cast<char*>(non_const) + item.offset_cmdrpl_));
      } else {
        std::cout << "Reply Item is of unsupported type." << std::endl;
        return 8765.4321f;
      }
    }
  }

  static float GetAsFloat(const std::string& alias,
                          const moteus::Query::Result& rpl) {
    const auto* item = Find(alias);
    if (item) {
      return GetAsFloat(*item, rpl);
    } else {
      std::cout << "Reply Item " << alias << " not found." << std::endl;
      return 8765.4321f;
    }
  }

  static moteus::Resolution* Get(const RplItem& item,
                                 const moteus::Query::Format& fmt) {
    if (set_.find(&item) == set_.end()) {
      std::cout << "Reply Item not found." << std::endl;
      return nullptr;
    } else {
      auto* non_const = const_cast<moteus::Query::Format*>(&fmt);
      return reinterpret_cast<moteus::Resolution*>(
          reinterpret_cast<char*>(non_const) + item.offset_fmt_);
    }
  }

  static moteus::Resolution* Get(const std::string& alias,
                                 const moteus::Query::Format& fmt) {
    const auto* item = Find(alias);
    if (item) {
      return Get(*item, fmt);
    } else {
      std::cout << "Reply Item " << alias << " not found." << std::endl;
      return nullptr;
    }
  }
};

}  // namespace som
