#pragma once

#include <fstream>
#include <iostream>

#include "enums.hpp"
#include "moteus_protocol.h"
#include "nlohmann/json.hpp"

#ifdef NaN
#define NaN_backup NaN
#undef NaN
#endif
#define NaN (std::numeric_limits<double>::quiet_NaN())

using namespace mjbots;
using json = nlohmann::json;

namespace som {

struct Parser {
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

  static const std::map<int, std::map<CommandType, double>>
  ParseCommandLineInput(std::string input) {
    std::map<int, std::map<CommandType, double>> result;
    auto split_by_comma = Split(input, ',');
    for (auto& id_pos : split_by_comma) {
      auto split_by_equals = Split(id_pos, '=');
      try {
        if (split_by_equals.size() != 2) throw std::exception();
        auto id = std::stoi(split_by_equals[0]);
        auto position = std::stod(split_by_equals[1]);
        result[id][CommandType::POSITION] = position;
      } catch (...) {
        std::cout << "Ignoring wrong command: "
                  << (id_pos.empty() ? "<empty>" : id_pos) << std::endl;
      }
    }
    return result;
  }

  static std::pair<moteus::PositionMode::Format, moteus::PositionMode::Command>
  ParsePositionModeConfig(const std::string& config_dir_path) {
    moteus::PositionMode::Format format{.maximum_torque =
                                            moteus::Resolution::kFloat};
    moteus::PositionMode::Command command{.position = NaN};

    const std::string config_file_path =
        config_dir_path + "/slcm.positionmode.config.json";
    std::ifstream config_file(config_file_path);
    if (!config_file) {
      std::cout
          << "PositionMode config file `" << config_file_path
          << "` not found. Using default values from `moteus_protocol.h`, "
             "except `Format::maximun_torque = kFloat` "
             "and `Command::position = NaN(stay in current position)`."
          << std::endl;
    } else {
      json config_json;
      config_file >> config_json;
      config_file.close();

      const std::map<std::string, std::pair<moteus::Resolution*, double*>>
          member_map{
              {"position", {&format.position, &command.position}},
              {"velocity", {&format.velocity, &command.velocity}},
              {"feedforward_torque",
               {&format.feedforward_torque, &command.feedforward_torque}},
              {"kp_scale", {&format.kp_scale, &command.kp_scale}},
              {"kd_scale", {&format.kd_scale, &command.kd_scale}},
              {"maximum_torque",
               {&format.maximum_torque, &command.maximum_torque}},
              {"stop_position",
               {&format.stop_position, &command.stop_position}},
              {"watchdog_timeout",
               {&format.watchdog_timeout, &command.watchdog_timeout}},
              {"velocity_limit",
               {&format.velocity_limit, &command.velocity_limit}},
              {"accel_limit", {&format.accel_limit, &command.accel_limit}},
              {"fixed_voltage_override",
               {&format.fixed_voltage_override,
                &command.fixed_voltage_override}},
          };

      bool default_used = false;
      for (auto& item : member_map) {
        auto& member_name = item.first;
        auto& resolution = item.second.first;
        auto& initial_value = item.second.second;

        std::cout << "Configuring `" << member_name << "`..." << std::endl;

        if (!config_json.contains(member_name)) {
          if (!default_used) {
            std::cout
                << "Default values are unchanged from those in "
                   "`moteus_protocol.h` "
                   "except `Format::maximun_torque = kFloat` "
                   "and `Command::position = NaN(stay in current position)`."
                << std::endl;
            default_used = true;
          }

          std::cout << "Config JSON does not contain key <" << member_name
                    << ">. Using default values " << (*resolution) << ", "
                    << *initial_value << " and skipping to the next key."
                    << std::endl;
          continue;
        }

        const auto& inner_json = config_json[member_name];

        if (!inner_json.contains("using")) {
          std::cout << "JSON does not contain key <" << member_name
                    << "/using>. Using default value: " << (*resolution)
                    << std::endl;
        } else {
          if (!inner_json["using"].is_boolean()) {
            std::cout << "Value for key <" << member_name
                      << "/using> is not boolean. Using default value: "
                      << (*resolution) << std::endl;
          } else {
            *resolution = static_cast<bool>(inner_json["using"])
                              ? moteus::Resolution::kFloat
                              : moteus::Resolution::kIgnore;
            std::cout << "Successfully set resolution for " << member_name
                      << " to " << (*resolution) << std::endl;
          }
        }

        if (!inner_json.contains("initial_value")) {
          std::cout << "JSON does not contain key <" << member_name
                    << "/initial_value>. Using default value: "
                    << *initial_value << std::endl;
        } else {
          if (!inner_json["initial_value"].is_number()) {
            if (inner_json["initial_value"].is_string() &&
                inner_json["initial_value"] == "NaN") {
              *initial_value = NaN;
            } else {
              std::cout << "Value for key <" << member_name
                        << "/initial_value> is not a number nor NaN. "
                           "Using default value: "
                        << *initial_value << std::endl;
            }
          } else {
            *initial_value = static_cast<double>(inner_json["initial_value"]);
            std::cout << "Successfully set initial value for " << member_name
                      << " to " << *initial_value << std::endl;
          }
        }
      }
    }

    return std::pair{format, command};
  }
};

}  // namespace som

#ifdef NaN_backup
#undef NaN
#define NaN NaN_backup
#undef NaN_backup
#endif
