#pragma once

#include <cxxabi.h>

#include <map>
#include <memory>
#include <typeinfo>

#include "moteus.h"
#ifdef __RASPBERRY_PI__
#include "pi3hat.h"
#endif

#include "enums.hpp"
#include "helpers.hpp"
#include "parser.hpp"

#ifdef NaN
#define NaN_backup NaN
#undef NaN
#endif
#define NaN (std::numeric_limits<double>::quiet_NaN())

using namespace mjbots;

namespace som {

class ServoSystem {
 private:
  class Servo {
    friend ServoSystem;

   public:
    Servo(const int id, const int bus,
          const std::shared_ptr<moteus::Transport> transport,
          const moteus::PositionMode::Format& format,
          const moteus::PositionMode::Command& initial_command)
        : command_(initial_command) {
      // Initialize a moteus controller interface
      controller_ = std::make_shared<moteus::Controller>([&, id]() {
        moteus::Controller::Options options;
        options.id = id;
        options.bus = bus;
        options.transport = transport;
        options.position_format = format;
        return options;
      }());

      // Initial stop command required before any operation
      auto maybe_reply = controller_->SetStop();
      std::cout << (maybe_reply ? "Got" : "Failed to get")
                << " reply from controller ID " << id
                << " for initial SetStop command." << std::endl;

      // Get initial motor position
      SetBasePosition(3.0);
    }

   private:
    void SetBasePosition(double time_limit) {
      int id = controller_->options().id;
      std::cout << "Attempting to get current position from controller ID "
                << id << "..." << std::endl;
      const double time_init = helpers::GetNow();
      while (true) {
        const auto& maybe_reply = controller_->SetQuery();
        if (!maybe_reply) {
          const double time_passed = helpers::GetNow() - time_init;
          const double time_left = time_limit - time_passed;
          if (time_left > 0.0) {
            std::cout << "Not getting replies from controller ID " << id
                      << " but will try " << time_left << " more seconds...\r";
            ::usleep(10000);
            continue;
          } else {
            std::cout << "Failed to get initial position from controller ID "
                      << id << " for " << time_limit
                      << " seconds. Skipping this ID." << std::endl;
            break;
          }
        } else {
          state_.recent_reply = maybe_reply->values;
          base_position_ = state_.recent_reply.position;
          std::cout << "Got base position from controller ID " << id << ": "
                    << base_position_
                    << (init_suceeded()
                            ? ""
                            : ", but is NaN. Base position setting failed.")
                    << std::endl;
          break;
        }
      }
    }

    std::shared_ptr<moteus::Controller> controller_;
    double base_position_ = NaN;
    bool init_suceeded() { return std::isfinite(base_position_); }
    moteus::PositionMode::Command command_;
    struct State {
      moteus::Query::Result recent_reply;
      bool updated_this_cycle = false;
    } state_;
  };

 public:
  ServoSystem(const std::map<int, int>& id_bus_map = {{1, 0}},
              const std::string& config_dir_path = "../config",
              ListeningMode initial_listening_mode = ListeningMode::EXTERNAL,
              bool enable_external_input = true) {
    transport_ = moteus::Controller::MakeSingletonTransport({});
    if (transport_) {
      int status;
      char* class_name = abi::__cxa_demangle(typeid(*transport_).name(),
                                             nullptr, nullptr, &status);
      if (status == 0) {
        std::cout << "Default transport found: " << class_name << std::endl;
        free(class_name);
      } else {
        std::cout
            << "Default transport found, but failed to demangle its class name."
            << std::endl;
      }
    } else {
      std::cout << "Default transport not found. Returning from "
                   "`ServoSystem` constructor without any initialization."
                << std::endl;
      return;
    }

    const auto& format_command =
        Parser::ParsePositionModeConfig(config_dir_path);
    const auto& format = format_command.first;
    const auto& initial_command = format_command.second;

    std::vector<int> init_failed_ids;
    for (auto& id_bus : id_bus_map) {
      auto id = id_bus.first;
      auto bus = id_bus.second;
      std::cout << "Initializing servo for ID " << id << " on bus " << bus
                << "..." << std::endl;
      auto servo =
          std::make_shared<Servo>(id, bus, transport_, format, initial_command);
      if (servo->init_suceeded()) {
        servos_[id] = servo;
        ids_.push_back(id);
        std::cout << "Initialization succeeded for ID " << id << std::endl;
      } else {
        init_failed_ids.push_back(id);
        std::cout << "Initialization failed for ID " << id << std::endl;
      }
    }
    if (!init_failed_ids.empty()) {
      std::cout
          << "Excluded servos with the following IDs from the servo list: ";
      for (auto id : init_failed_ids) {
        std::cout << id << " ";
      }
      std::cout << std::endl;
    }
    if (!servos_.empty()) {
      std::cout << "Ready to control servos with the following IDs: ";
      for (const auto& id_servo : servos_) {
        std::cout << id_servo.first << " ";
      }
      std::cout << std::endl;
    } else {
      std::cout << "No available servos found." << std::endl;
    }

    SetListeningMode(initial_listening_mode);

    runner_thread_ = std::thread(&ServoSystem::Run, this);
    runner_thread_.detach();
    std::cout << "ServoSystem Runner thread started running..." << std::endl;

    if (enable_external_input) {
      external_input_getter_thread_ =
          std::thread(&ServoSystem::ExternalInputGetter, this);
      external_input_getter_thread_.detach();
      std::cout << "ServoSystem ExternalInputGetter thread started running..."
                << std::endl;
    } else {
      std::cout << "ExternalInputGetter thread is not running." << std::endl;
    }
  }

  void Input(const std::map<int, std::map<CommandType, double>> commands) {
    InternalInputGetter(commands);
  }

  void InputAll(std::map<CommandType, double> command) {
    std::map<int, std::map<CommandType, double>> commands;
    for (const auto id : ids_) {
      commands[id] = command;
    }
    InternalInputGetter(commands);
  }

  void SetBasePositions(std::vector<int> ids) {
    for (const auto id : ids) {
      const auto& servo = helpers::SafeAt(servos_, id);
      if (servo) {
        servo.value()->SetBasePosition(0.0);
      }
    }
  }

  void SetBasePositionsAll() { SetBasePositions(ids_); }

  void Stop(const std::vector<int> ids) {
    for (const auto id : ids) {
      const auto& servo = helpers::SafeAt(servos_, id);
      if (servo) {
        servo.value()->controller_->SetStop();
      }
    }
  }

  void StopAll() { Stop(ids_); }

  void Terminate() { terminated_ = true; }

  std::vector<int> GetIds() { return ids_; }

  void SetListeningMode(ListeningMode listening_mode) {
    listening_mode_ = listening_mode;
    std::cout << "Listening mode set to " << listening_mode_ << std::endl;
  }

 protected:
  const unsigned int cycle_period_us_ = 0.01 * 1e6;

  virtual void ExternalInputGetter() {
    std::string input;
    while (!terminated_) {
      std::getline(std::cin, input);
      if (listening_mode_ != ListeningMode::EXTERNAL) {
        std::cout << "Rejecting external input since listening mode is set to "
                     "INTERNAL."
                  << std::endl;
        continue;
      }

      InputGetter(Parser::ParseCommandLineInput(input));
    }
  }

 private:
  void InternalInputGetter(
      const std::map<int, std::map<CommandType, double>>& commands) {
    if (listening_mode_ != ListeningMode::INTERNAL) {
      std::cout
          << "Rejecting internal input since listening mode is set to EXTERNAL."
          << std::endl;
      return;
    }

    InputGetter(commands);
  }

  void InputGetter(
      const std::map<int, std::map<CommandType, double>>& commands) {
    std::cout << "Parsing input..." << std::endl;
    for (const auto& id_cmd : commands) {
      const auto id = id_cmd.first;
      const auto& cmd = id_cmd.second;
      const auto& maybe_servo = helpers::SafeAt(servos_, id);
      if (!maybe_servo) {
        std::cout << "No servo with ID " << id
                  << " found on the servo list. Ignoring command for this ID."
                  << std::endl;
        continue;
      }
      const auto& servo = maybe_servo.value();

      std::cout << "Parsing input for ID " << id << "..." << std::endl;
      for (const auto& type_value : cmd) {
        const auto type = type_value.first;
        const auto value = type_value.second;
        std::cout << "Parsing input of CommandType " << type << "..."
                  << std::endl;
        if (((moteus::Resolution*)(&(
                servo->controller_->options().position_format)))[type] ==
            moteus::Resolution::kIgnore) {
          std::cout << "ServoSystem PositionMode Format is set to " << type
                    << " = " << "kIgnore. Ignoring command." << std::endl;
          continue;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          ((double*)(&(servo->command_)))[type] = value;
        }
        std::cout << "Successfully set command: " << type << " = " << value
                  << std::endl;
      }
    }
  }

  void Run() {
    if (servos_.empty()) {
      std::cout << "No servos found. Exiting `Run` for this ServoSystem."
                << std::endl;
      return;
    }

    while (!terminated_) {
      ::usleep(cycle_period_us_);

      std::vector<moteus::CanFdFrame> command_frames;
      for (auto& id_servo : servos_) {
        auto id = id_servo.first;
        auto& servo = id_servo.second;
        servo->state_.updated_this_cycle = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          command_frames.push_back(
              servo->controller_->MakePosition(servo->command_));
        }
      }

      std::vector<moteus::CanFdFrame> reply_frames;
      transport_->BlockingCycle(&command_frames[0], command_frames.size(),
                                &reply_frames);

      for (const auto& frame : reply_frames) {
        int id = frame.source;
        const auto& maybe_servo = helpers::SafeAt(servos_, id);
        if (maybe_servo) {
          auto& state = maybe_servo.value()->state_;
          state.recent_reply = moteus::Query::Parse(frame.data, frame.size);
          state.updated_this_cycle = true;
        }
      }
    }
  }

  std::vector<int> ids_;
  std::map<int, std::shared_ptr<Servo>> servos_;  // ID -> Servo
  std::shared_ptr<moteus::Transport> transport_;
  ListeningMode listening_mode_;
  std::mutex mutex_;
  std::thread runner_thread_;
  std::thread external_input_getter_thread_;
  bool terminated_;
};

}  // namespace som
