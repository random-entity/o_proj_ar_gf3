#pragma once

#include <fstream>
#include <iostream>

#include "moteus_protocol.h"
#include "nlohmann/json.hpp"
#include "slcm_enums.hpp"

#ifdef NaN
#define NaN_backup NaN
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
  ///        to form a command map.
  /// @param input The format of string input is as follows:
  ///              <id>=<3-letter-command-type-abbr><value>[,...][;...]
  ///              For example,
  ///              1=pos1.0,vel0.1,mtq1.0;2=pos-1.0,vel-0.1,vlm0.5
  static std::map<int, std::map<CommandType, double>> ParseStringInput(
      std::string input) {
    std::map<int, std::map<CommandType, double>> commands;
    auto sentence = Split(input, ';');
    for (auto& id_command : sentence) {
      auto id_and_command = Split(id_command, '=');
      try {
        if (id_and_command.size() != 2) throw std::exception();
        auto id = std::stoi(id_and_command[0]);
        auto command = id_and_command[1];
        auto fields = Split(command, ',');
        for (const auto& field : fields) {
          auto command_type_string = field.substr(0, 3);
          CommandType command_type;
          {
            if (command_type_string == "pos") {
              command_type = CommandType::POSITION;
            } else if (command_type_string == "vel") {
              command_type = CommandType::VELOCITY;
            } else if (command_type_string == "fft") {
              command_type = CommandType::FEEDFORWARD_TORQUE;
            } else if (command_type_string == "kps") {
              command_type = CommandType::KP_SCALE;
            } else if (command_type_string == "kds") {
              command_type = CommandType::KD_SCALE;
            } else if (command_type_string == "mtq") {
              command_type = CommandType::MAXIMUM_TORQUE;
            } else if (command_type_string == "stp") {
              command_type = CommandType::STOP_POSITION;
            } else if (command_type_string == "wtl") {
              command_type = CommandType::WATCHDOG_TIMEOUT;
            } else if (command_type_string == "vlm") {
              command_type = CommandType::VELOCITY_LIMIT;
            } else if (command_type_string == "alm") {
              command_type = CommandType::ACCEL_LIMIT;
            } else if (command_type_string == "fvo") {
              command_type = CommandType::FIXED_VOLTAGE_OVERRIDE;
            } else {
              throw std::exception();
            }
          }
          if (field.substr(3) == "nan") {
            commands[id][command_type] = NaN;
          } else {
            auto command_value = stod(field.substr(3));
            commands[id][command_type] = command_value;
          }
        }
      } catch (...) {
        std::cout << "Ignoring wrong command: "
                  << (id_command.empty() ? "<empty>" : id_command) << std::endl;
      }
    }
    return commands;
  }

  /// @brief Parse the PositionMode config JSON file.
  /// @param config_dir_path The path to the directory where the
  ///        config file is located at, relative to the working directory
  ///        within the terminal where the binary is executed, if relative.
  ///        The config file must be named "slcm.positionmode.config.json".
  ///        If the file is not found, the default Format and Command
  ///        which can be found at
  ///        https://github.com/mjbots/moteus/blob/main/lib/cpp/mjbots/moteus/moteus_protocol.h
  ///        (Look for structs PositionMode::Format and PositionMode::Command.)
  ///        will be returned.  It is always recommended to use manual
  ///        configuration since the default Format ignores maximum torque and
  ///        velocity limit, which can cause your servo system to shut down.
  /// @return A pair of PositionMode Format and Command, which will be used for
  ///         controller options configuration and initial command respectively.
  static std::pair<moteus::PositionMode::Format, moteus::PositionMode::Command>
  ParsePositionModeConfig(const std::string& config_dir_path) {
    moteus::PositionMode::Format format;
    moteus::PositionMode::Command command;

    char* config_dir_path_absolute = (char*)(malloc(512));
    realpath(config_dir_path.c_str(), config_dir_path_absolute);
    std::cout << "Looking for config file at: " << config_dir_path_absolute
              << std::endl;
    free(config_dir_path_absolute);

    const std::string config_file_path =
        config_dir_path + "/slcm.positionmode.config.json";
    std::ifstream config_file(config_file_path);
    if (!config_file) {
      std::cout << "PositionMode config file " << config_file_path
                << " not found.  Using default values from moteus_protocol.h, "
                   "which can be dangerous."
                << std::endl;
    } else {
      std::cout << "PositionMode config file " << config_file_path << " found."
                << std::endl;

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

        std::cout << "Configuring " << member_name << "..." << std::endl;

        if (!config_json.contains(member_name)) {
          if (!default_used) {
            std::cout << "Default values can be found at moteus_protocol.h "
                         "of the mjbots/moteus library."
                      << std::endl;
            default_used = true;
          }

          std::cout << "Config JSON does not contain key " << member_name
                    << ".  Using default values " << *resolution << ", "
                    << *initial_value << " and skipping to the next key."
                    << std::endl;
          continue;
        }

        const auto& inner_json = config_json[member_name];

        if (!inner_json.contains("using")) {
          std::cout << "JSON does not contain key " << member_name
                    << "/using.  Using default value: " << *resolution
                    << std::endl;
        } else if (!inner_json["using"].is_boolean()) {
          std::cout << "Value for key " << member_name
                    << "/using is not boolean.  Using default value: "
                    << *resolution << std::endl;
        } else {
          *resolution = static_cast<bool>(inner_json["using"])
                            ? moteus::Resolution::kFloat
                            : moteus::Resolution::kIgnore;
          std::cout << "Successfully set resolution for " << member_name << ": "
                    << *resolution << std::endl;
        }

        if (*resolution == moteus::Resolution::kIgnore) {
          std::cout << "Skipping initial value setting " << member_name
                    << " since resolution is set to kIgnore." << std::endl;
          continue;
        }

        if (!inner_json.contains("initial_value")) {
          std::cout << "JSON does not contain key " << member_name
                    << "/initial_value.  Using default value: "
                    << *initial_value << std::endl;
        } else if (!inner_json["initial_value"].is_number()) {
          if (inner_json["initial_value"].is_string() &&
              inner_json["initial_value"] == "NaN") {
            *initial_value = NaN;
          } else {
            std::cout << "Value for key " << member_name
                      << "/initial_value is not a number nor NaN.  "
                         "Using default value: "
                      << *initial_value << std::endl;
          }
        } else {
          *initial_value = static_cast<double>(inner_json["initial_value"]);
          std::cout << "Successfully set initial value for " << member_name
                    << ": " << *initial_value << std::endl;
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
