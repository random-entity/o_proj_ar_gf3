#pragma once

#include <fstream>
#include <iostream>

#include "moteus_protocol.h"
#include "nlohmann/json.hpp"
#include "slcm_enums.hpp"

#ifdef NaN
#undef NaN
#endif
#define NaN (std::numeric_limits<double>::quiet_NaN())

using namespace mjbots;
using json = nlohmann::json;

namespace som {

/// @brief A collection of functions used for parsing.
///        All methods are declared as static.
struct Parser {
  /// @brief Split a string at all splitter chars.
  /// @return A vector of split up substrings, excluding the splitters.
  static std::vector<std::string> Split(const std::string& str, char splitter) {
    std::vector<std::string> result;
    size_t start = 0;
    auto pos = str.find(splitter);
    while (pos != std::string::npos) {
      result.push_back(str.substr(start, pos - start));
      start = pos + 1;
      pos = str.find(splitter, start);
    }
    result.push_back(str.substr(start));
    return result;
  }

  /// @brief Parse string input of specific format described below
  ///        to form a command map: ID -> CmdItem -> value
  /// @param input The format of string input is as follows:
  ///              <id>=<3-letter-cmd-item-abbr><value>[,...][;...]
  ///              For example,
  ///              1=pos1.0,vel0.1,mtq1.0;2=pos-1.0,vel-0.1,vlm0.5
  static std::map<int, std::map<CmdItem, double>> ParseStrInput(
      std::string input) {
    std::map<int, std::map<CmdItem, double>> cmds;
    auto sentence = Split(input, ';');
    for (auto& id_cmd : sentence) {
      auto id_and_cmd = Split(id_cmd, '=');
      try {
        if (id_and_cmd.size() != 2) throw std::exception();
        auto id = std::stoi(id_and_cmd[0]);
        auto cmd = id_and_cmd[1];
        auto fields = Split(cmd, ',');
        for (const auto& field : fields) {
          auto cmd_item_str = field.substr(0, 3);
          CmdItem cmd_item;
          {
            if (cmd_item_str == "pos") {
              cmd_item = CmdItem::POSITION;
            } else if (cmd_item_str == "vel") {
              cmd_item = CmdItem::VELOCITY;
            } else if (cmd_item_str == "fft") {
              cmd_item = CmdItem::FEEDFORWARD_TORQUE;
            } else if (cmd_item_str == "kps") {
              cmd_item = CmdItem::KP_SCALE;
            } else if (cmd_item_str == "kds") {
              cmd_item = CmdItem::KD_SCALE;
            } else if (cmd_item_str == "mtq") {
              cmd_item = CmdItem::MAXIMUM_TORQUE;
            } else if (cmd_item_str == "stp") {
              cmd_item = CmdItem::STOP_POSITION;
            } else if (cmd_item_str == "wtl") {
              cmd_item = CmdItem::WATCHDOG_TIMEOUT;
            } else if (cmd_item_str == "vlm") {
              cmd_item = CmdItem::VELOCITY_LIMIT;
            } else if (cmd_item_str == "alm") {
              cmd_item = CmdItem::ACCEL_LIMIT;
            } else if (cmd_item_str == "fvo") {
              cmd_item = CmdItem::FIXED_VOLTAGE_OVERRIDE;
            } else if (cmd_item_str == "ils") {
              cmd_item = CmdItem::ILIMIT_SCALE;
            } else {
              throw std::exception();
            }
          }
          if (field.substr(3) == "nan") {
            cmds[id][cmd_item] = NaN;
          } else {
            auto cmd_val = stod(field.substr(3));
            cmds[id][cmd_item] = cmd_val;
          }
        }
      } catch (...) {
        std::cout << "Ignoring wrong command: "
                  << (id_cmd.empty() ? "<empty>" : id_cmd) << std::endl;
      }
    }
    return cmds;
  }

  /// @brief Parse the (PositionMode) command config JSON file.
  /// @param conf_dir The path to the directory where the
  ///        config file is located at, relative to the working directory
  ///        within the terminal where the binary is executed, if given as
  ///        relative.  The config file must be named
  ///        "slcm.cmd.conf.json".  If the file is not found, the
  ///        default PositionMode Format and Command which can be found at
  ///        https://github.com/mjbots/moteus/blob/main/lib/cpp/mjbots/moteus/moteus_protocol.h
  ///        (Look for structs PositionMode::Format and PositionMode::Command.)
  ///        will be returned.  It is always recommended to use manual
  ///        configuration since the default Format ignores maximum torque and
  ///        velocity limit, which can cause your ServoSystem to shut down.
  /// @return A pair of PositionMode Format and Command, which will be used for
  ///         controller options configuration and initial command respectively.
  static std::pair<moteus::PositionMode::Format, moteus::PositionMode::Command>
  ParseCmdConf(const std::string& conf_dir) {
    moteus::PositionMode::Format fmt;
    moteus::PositionMode::Command cmd;

    char* conf_dir_abs = (char*)(malloc(512));
    realpath(conf_dir.c_str(), conf_dir_abs);
    std::cout << "Looking for command config file at: " << conf_dir_abs
              << std::endl;
    free(conf_dir_abs);

    const std::string conf_file_path = conf_dir + "/slcm.cmd.conf.json";
    std::ifstream conf_file(conf_file_path);
    if (!conf_file) {
      std::cout << "Command config file " << conf_file_path
                << " not found.  Using default values from moteus_protocol.h, "
                   "which is not recommended."
                << std::endl;
    } else {
      std::cout << "Command config file " << conf_file_path << " found."
                << std::endl;

      json conf_json;
      conf_file >> conf_json;
      conf_file.close();

      const std::map<std::string, std::pair<moteus::Resolution*, double*>>
          member_map{
              {"position", {&fmt.position, &cmd.position}},
              {"velocity", {&fmt.velocity, &cmd.velocity}},
              {"feedforward_torque",
               {&fmt.feedforward_torque, &cmd.feedforward_torque}},
              {"kp_scale", {&fmt.kp_scale, &cmd.kp_scale}},
              {"kd_scale", {&fmt.kd_scale, &cmd.kd_scale}},
              {"maximum_torque", {&fmt.maximum_torque, &cmd.maximum_torque}},
              {"stop_position", {&fmt.stop_position, &cmd.stop_position}},
              {"watchdog_timeout",
               {&fmt.watchdog_timeout, &cmd.watchdog_timeout}},
              {"velocity_limit", {&fmt.velocity_limit, &cmd.velocity_limit}},
              {"accel_limit", {&fmt.accel_limit, &cmd.accel_limit}},
              {"fixed_voltage_override",
               {&fmt.fixed_voltage_override, &cmd.fixed_voltage_override}},
              {"ilimit_scale", {&fmt.ilimit_scale, &cmd.ilimit_scale}},
          };

      bool default_used = false;
      for (auto& item : member_map) {
        const auto& item_name = item.first;
        auto& res = item.second.first;
        auto& init_val = item.second.second;

        std::cout << "Configuring command resolution and initial command "
                     "value for "
                  << item_name << "..." << std::endl;

        if (!conf_json.contains(item_name)) {
          if (!default_used) {
            std::cout << "Default values can be found at moteus_protocol.h "
                         "of the mjbots/moteus library."
                      << std::endl;
            default_used = true;
          }

          std::cout << "Config JSON does not contain key " << item_name
                    << ".  Using default values " << *res << ", " << *init_val
                    << " and skipping to the next key." << std::endl;
          continue;
        }

        const auto& inner_json = conf_json[item_name];

        if (!inner_json.contains("resolution") ||
            !inner_json["resolution"].is_string()) {
          std::cout << "Config JSON does not contain key " << item_name
                    << "/resolution, or the value is not a string.  "
                       "Using default value: "
                    << *res << std::endl;
        } else {
          if (inner_json["resolution"] == "kInt8") {
            *res = moteus::Resolution::kInt8;
          } else if (inner_json["resolution"] == "kInt16") {
            *res = moteus::Resolution::kInt16;
          } else if (inner_json["resolution"] == "kInt32") {
            *res = moteus::Resolution::kInt32;
          } else if (inner_json["resolution"] == "kFloat") {
            *res = moteus::Resolution::kFloat;
          } else if (inner_json["resolution"] == "kIgnore") {
            *res = moteus::Resolution::kIgnore;
          } else {
            std::cout << "Unknown resolution.  Using default value: " << *res
                      << std::endl;
          }
        }

        std::cout << "Command resolution for " << item_name
                  << " is set to: " << *res << std::endl;

        if (*res == moteus::Resolution::kIgnore) {
          std::cout << "Skipping initial value setting of " << item_name
                    << " since its resolution is set to kIgnore." << std::endl;
          continue;
        }

        if (!inner_json.contains("initial_value")) {
          std::cout << "JSON does not contain key " << item_name
                    << "/initial_value.  Using default value: " << *init_val
                    << std::endl;
        } else if (!inner_json["initial_value"].is_number()) {
          if (inner_json["initial_value"].is_string() &&
              inner_json["initial_value"] == "NaN") {
            *init_val = NaN;
          } else {
            std::cout << "Value for key " << item_name
                      << "/initial_value is not a number nor NaN.  "
                         "Using default value: "
                      << *init_val << std::endl;
          }
        } else {
          *init_val = static_cast<double>(inner_json["initial_value"]);
        }

        std::cout << "Initial command value for " << item_name
                  << " is set to: " << *init_val << std::endl;
      }
    }

    return std::make_pair(fmt, cmd);
  }

  static moteus::Query::Format ParseRplConf(const std::string& conf_dir) {
    moteus::Query::Format fmt;

    char* conf_dir_abs = (char*)(malloc(512));
    realpath(conf_dir.c_str(), conf_dir_abs);
    std::cout << "Looking for reply config file at: " << conf_dir_abs
              << std::endl;
    free(conf_dir_abs);

    const std::string conf_file_path = conf_dir + "/slcm.rpl.conf.json";
    std::ifstream conf_file(conf_file_path);
    if (!conf_file) {
      std::cout << "Reply config file " << conf_file_path
                << " not found.  Using default values from moteus_protocol.h, "
                   "which is not suitable when using an external encoder."
                << std::endl;
    } else {
      std::cout << "Reply config file " << conf_file_path << " found."
                << std::endl;

      json conf_json;
      conf_file >> conf_json;
      conf_file.close();

      const std::map<std::string, moteus::Resolution*> member_map{
          {"mode", &fmt.mode},
          {"position", &fmt.position},
          {"velocity", &fmt.velocity},
          {"torque", &fmt.torque},
          {"q_current", &fmt.q_current},
          {"d_current", &fmt.d_current},
          {"abs_position", &fmt.abs_position},
          {"power", &fmt.power},
          {"motor_temperature", &fmt.motor_temperature},
          {"trajectory_complete", &fmt.trajectory_complete},
          {"home_state", &fmt.home_state},
          {"voltage", &fmt.voltage},
          {"temperature", &fmt.temperature},
          {"fault", &fmt.fault},
      };

      bool default_used = false;
      for (auto& item : member_map) {
        auto& item_name = item.first;
        auto& res = item.second;

        std::cout << "Configuring reply resolution for " << item_name << "..."
                  << std::endl;

        if (!conf_json.contains(item_name)) {
          if (!default_used) {
            std::cout << "Default values can be found at moteus_protocol.h "
                         "of the mjbots/moteus library."
                      << std::endl;
            default_used = true;
          }

          std::cout << "Config JSON does not contain key " << item_name
                    << ".  Using default resolution " << *res
                    << " and skipping to the next key." << std::endl;
          continue;
        }

        if (conf_json[item_name].is_string()) {
          if (conf_json[item_name] == "kInt8") {
            *res = moteus::Resolution::kInt8;
          } else if (conf_json[item_name] == "kInt16") {
            *res = moteus::Resolution::kInt16;
          } else if (conf_json[item_name] == "kInt32") {
            *res = moteus::Resolution::kInt32;
          } else if (conf_json[item_name] == "kFloat") {
            *res = moteus::Resolution::kFloat;
          } else if (conf_json[item_name] == "kIgnore") {
            *res = moteus::Resolution::kIgnore;
          } else {
            std::cout << "Unknown resolution.  Using default value: " << *res
                      << std::endl;
          }
        } else {
          std::cout << "Resolution value is not a string.  "
                       "Using default value: "
                    << *res << std::endl;
        }

        std::cout << "Reply resolution for " << item_name
                  << " is set to: " << *res << std::endl;
      }
    }

    return fmt;
  }
};

}  // namespace som
