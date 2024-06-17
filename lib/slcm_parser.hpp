#pragma once

#include <fstream>
#include <iostream>

#include "moteus_protocol.h"
#include "nlohmann/json.hpp"
#include "slcm_multiplex.hpp"
#include "slcm_printer.hpp"

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
  ///        to form a command map: ID -> Item -> value
  /// @param input The format of string input is as follows:
  ///              <id>=<3-letter-item-abbr><value>[,...][;...]
  ///              For example,
  ///              1=pos1.0,vel0.1,mtq1.0;2=pos-1.0,vel-0.1,vlm0.5
  static std::map<int, std::map<CommandItem, double>> ParseStrInput(
      std::string input) {
    std::map<int, std::map<CommandItem, double>> cmds;
    auto sentence = Split(input, ';');
    for (auto& id_cmd : sentence) {
      auto id_and_cmd = Split(id_cmd, '=');
      try {
        if (id_and_cmd.size() != 2) throw std::exception();
        auto id = std::stoi(id_and_cmd[0]);
        auto cmd = id_and_cmd[1];
        auto fields = Split(cmd, ',');
        for (const auto& field : fields) {
          auto item_str = field.substr(0, 3);
          const auto maybe_item = CmdItemsMgr::StrToItem(item_str);
          if (!maybe_item) {
            throw std::exception();
          }
          const auto item = maybe_item.value();
          if (field.substr(3) == "nan") {
            cmds[id][item] = NaN;
          } else {
            auto cmd_val = stod(field.substr(3));
            cmds[id][item] = cmd_val;
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
    std::cout << "Looking for Command config file at: " << conf_dir_abs
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

      for (const auto& element : conf_json.items()) {
        const auto& item_str = element.key();
        const auto maybe_item = CmdItemsMgr::StrToItem(item_str);
        if (!maybe_item) {
          std::cout << "Command Item of given alias not found: " << item_str
                    << std::endl;
          continue;
        }
        const auto item = maybe_item.value();

        std::cout << "Configuring Command resolution and initial value for "
                  << item << "..." << std::endl;

        const auto& inner_json = element.value();
        auto* res = CmdItemsMgr::ItemToPtr(item, fmt);

        if (!inner_json.contains("resolution") ||
            !inner_json["resolution"].is_string()) {
          std::cout << "Config JSON does not contain key " << item_str
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
        std::cout << "Command resolution for " << item_str
                  << " is set to: " << *res << std::endl;
        if (*res == moteus::Resolution::kIgnore) {
          std::cout << "Skipping initial value setting of " << item_str
                    << " since its resolution is set to kIgnore." << std::endl;
          continue;
        }
        auto* init_val = CmdItemsMgr::ItemToPtr(item, cmd);
        if (!inner_json.contains("initial_value")) {
          std::cout << "JSON does not contain key " << item_str
                    << "/initial_value.  Using default value: " << *init_val
                    << std::endl;
        } else if (!inner_json["initial_value"].is_number()) {
          if (inner_json["initial_value"].is_string() &&
              inner_json["initial_value"] == "NaN") {
            *init_val = NaN;
          } else {
            std::cout << "Value for key " << item_str
                      << "/initial_value is not a number nor NaN.  "
                         "Using default value: "
                      << *init_val << std::endl;
          }
        } else {
          *init_val = static_cast<double>(inner_json["initial_value"]);
        }

        std::cout << "Initial command value for " << item_str
                  << " is set to: " << *init_val << std::endl;
      }
    }

    std::cout << "Default values can be found at moteus_protocol.h "
                 "of the mjbots/moteus library."
              << std::endl;

    return std::make_pair(fmt, cmd);
  }

  static moteus::Query::Format ParseRplConf(const std::string& conf_dir) {
    moteus::Query::Format fmt;

    char* conf_dir_abs = (char*)(malloc(512));
    realpath(conf_dir.c_str(), conf_dir_abs);
    std::cout << "Looking for Reply config file at: " << conf_dir_abs
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

      for (const auto& element : conf_json.items()) {
        const auto& item_str = element.key();
        const auto maybe_item = RplItemsMgr::StrToItem(item_str);
        if (!maybe_item) {
          std::cout << "Reply Item of given alias not found: " << item_str
                    << std::endl;
          continue;
        }
        const auto item = maybe_item.value();

        std::cout << "Configuring Reply resolution for " << item_str << "..."
                  << std::endl;

        const auto& val = element.value();
        auto* res = RplItemsMgr::ItemToPtr(item, fmt);

        if (val.is_string()) {
          if (val == "kInt8") {
            *res = moteus::Resolution::kInt8;
          } else if (val == "kInt16") {
            *res = moteus::Resolution::kInt16;
          } else if (val == "kInt32") {
            *res = moteus::Resolution::kInt32;
          } else if (val == "kFloat") {
            *res = moteus::Resolution::kFloat;
          } else if (val == "kIgnore") {
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
        std::cout << "Reply resolution for " << item_str
                  << " is set to: " << *res << std::endl;
      }

      std::cout << "Default values can be found at moteus_protocol.h "
                   "of the mjbots/moteus library."
                << std::endl;
    }

    return fmt;
  }
};

}  // namespace som
