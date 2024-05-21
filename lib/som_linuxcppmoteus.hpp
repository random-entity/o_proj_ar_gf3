#pragma once

#include <condition_variable>
#include <map>
#include <memory>

#include "moteus.h"
#ifdef __RASPBERRY_PI__
#include "pi3hat.h"
#endif

#include "slcm_enums.hpp"
#include "slcm_parser.hpp"
#include "slcm_utils.hpp"

#ifdef NaN
#undef NaN
#endif
#define NaN (std::numeric_limits<double>::quiet_NaN())

using namespace mjbots;

namespace som {

class ServoSystem {
 protected:
  class Servo {
   public:
    Servo(const int id, const int bus,
          const std::shared_ptr<moteus::Transport>& transport,
          const moteus::PositionMode::Format& format,
          const moteus::PositionMode::Command& initial_command)
        : command_(initial_command) {
      controller_ = std::make_shared<moteus::Controller>([&, id, bus]() {
        moteus::Controller::Options options;
        options.id = id;
        options.bus = bus;
        options.transport = transport;
        options.position_format = format;
        return options;
      }());

      auto maybe_reply = controller_->SetStop();
      std::cout << (maybe_reply ? "Got" : "Failed to get")
                << " reply for initial SetStop command from Servo ID " << id
                << "." << std::endl;

      SetBasePosition(1.0);
    }

    bool SetBasePosition(double time_limit) {
      const int id = controller_->options().id;
      std::cout << "Attempting to get current position from Servo ID " << id
                << "..." << std::endl;
      const double time_init = Utils::GetNow();
      while (true) {
        const auto& maybe_reply = controller_->SetQuery();
        if (!maybe_reply) {
          const double time_passed = Utils::GetNow() - time_init;
          const double time_left = time_limit - time_passed;
          if (time_left > 0.0) {
            std::cout << "Not getting reply from Servo ID " << id
                      << ", but will try " << time_left << " more seconds...\r";
            ::usleep(0.01 * 1e6);
            continue;
          } else {
            std::cout << "Failed to get reply from Servo ID " << id << " for "
                      << time_limit << " seconds.  "
                      << "Base position setting failed." << std::endl;
            return false;
          }
        } else {
          const double current_position = maybe_reply->values.position;
          if (std::isfinite(current_position)) {
            base_position_ = current_position;
            std::cout << "Successfully set base position to current position "
                         "for Servo ID "
                      << id << ": " << base_position_ << std::endl;
            state_.recent_reply = maybe_reply->values;
            return true;
          } else {
            const double time_passed = Utils::GetNow() - time_init;
            const double time_left = time_limit - time_passed;
            if (time_left > 0.0) {
              std::cout << "Current position for Servo ID " << id
                        << " is NaN, but will try " << time_left
                        << " more seconds...\r";
              ::usleep(0.01 * 1e6);
              continue;
            } else {
              std::cout << "Failed to get finite value as the current position "
                           "for Servo ID "
                        << id << " for " << time_limit << " seconds.  "
                        << "Base position setting failed." << std::endl;
              return false;
            }
          }
        }
      }
    }

    std::shared_ptr<moteus::Controller> controller_;
    moteus::PositionMode::Command command_;
    struct State {
      moteus::Query::Result recent_reply;
      bool updated_this_cycle = false;
    } state_;
    double base_position_ = NaN;
    bool init_suceeded() { return std::isfinite(base_position_); }
  };

  friend class LoopingThreadManager;
  class LoopingThreadManager {
    friend class ServoSystem;

   public:
    LoopingThreadManager(
        ServoSystem* servo_system,
        std::function<void(ServoSystem*, std::atomic_bool*)> task,
        const std::vector<std::string>& aliases)
        : servo_system_(servo_system),
          task_(task),
          aliases_(aliases),
          thread_(nullptr) {
      std::cout << "LTM constructor begin" << std::endl;

      if (aliases_.empty()) {
        std::cout << "ThreadManager initialization failed since "
                     "no names are given for this thread."
                  << std::endl;
        return;
      }
      for (const auto& name : aliases_) {
        std::cout << "for 1" << std::endl;
        servo_system_->ThreadsManager[name] = this;
        std::cout << "for 2" << std::endl;
      }
    }

    void Start() {
      if (!thread_) {
        terminated_ = false;
        thread_ = new std::thread(task_, servo_system_, &terminated_);
        std::cout << aliases_[0]
                  << " thread successfully initialized and detached."
                  << std::endl;
      } else {
        std::cout << aliases_[0] << " thread is already running." << std::endl;
      }
    }

    void Terminate() {
      if (thread_) {
        terminated_ = true;
        std::cout << "Waiting for " << aliases_[0] << " thread to join..."
                  << std::endl;
        thread_->join();
        delete thread_;
        thread_ = nullptr;
        std::cout << aliases_[0] << " thread successfully terminated."
                  << std::endl;
      } else {
        std::cout << aliases_[0] << " thread has not been running."
                  << std::endl;
      }
    }

    bool IsOn() { return !!thread_; }

   private:
    const std::vector<std::string> aliases_;
    ServoSystem* const servo_system_;
    std::thread* thread_;
    const std::function<void(ServoSystem*, std::atomic_bool*)> task_;
    std::atomic_bool terminated_;
  };

  std::map<std::string, LoopingThreadManager*> ThreadsManager;

 public:
  ServoSystem(const std::map<int, int>& id_bus_map = {{1, 1}},
              const std::string& config_dir_path = "../config")
      : runner_thread_manager_{this, &ServoSystem::Run, {"Runner", "run", "r"}},
        external_input_getter_manager_{this,
                                       &ServoSystem::ExternalInputGetter,
                                       {"ExternalInputGetter", "exin", "ei"}},
        external_output_sender_manager_{
            this,
            &ServoSystem::ExternalOutputSender,
            {"ExternalOutputSender", "exout", "eo"}} {
    std::cout << "constructor begin" << std::endl;

    /// Get default transport. Priority order: pi3hat > fdcanusb > socketcan
    transport_ = moteus::Controller::MakeSingletonTransport({});
    /*[ Print transport status and handle failure. ]*/ {
      if (transport_) {
        std::string transport_name = Utils::GetClassName(transport_.get());
        if (!transport_name.empty()) {
          std::cout << "Default transport found: " << transport_name
                    << std::endl;
        } else {
          std::cout
              << "Default transport found, but failed to get its class "
                 "name.  Returning from ServoSystem constructor without any "
                 "initialization for safety."
              << std::endl;
        }
      } else {
        std::cout << "Default transport not found.  Returning from "
                     "ServoSystem constructor without any initialization."
                  << std::endl;
        return;
      }
    }

    std::cout << "got transport" << std::endl;

    /// Parse the PositionMode config file.
    const auto format_command =
        Parser::ParsePositionModeConfig(config_dir_path);
    const auto& format = format_command.first;
    const auto& initial_command = format_command.second;

    /// Initialize Servos.
    std::set<int> init_failed_ids;
    for (const auto& id_bus : id_bus_map) {
      const auto id = id_bus.first;
      const auto bus = id_bus.second;
      std::cout << "Initializing Servo ID " << id << " on bus " << bus << "..."
                << std::endl;
      const auto servo =
          std::make_shared<Servo>(id, bus, transport_, format, initial_command);
      if (servo->init_suceeded()) {
        servos_[id] = servo;
        ids_.insert(id);
        std::cout << "Initialization succeeded for Servo ID " << id << "."
                  << std::endl;
      } else {
        init_failed_ids.insert(id);
        std::cout << "Initialization failed for Servo ID " << id << "."
                  << std::endl;
      }
    }

    /*[ Print Servo initialization result ]*/ {
      if (!init_failed_ids.empty()) {
        std::cout << "Excluded Servos with the following IDs "
                     "from the Servo list: ";
        for (auto id : init_failed_ids) {
          std::cout << id << " ";
        }
        std::cout << std::endl;
      }

      if (!servos_.empty()) {
        std::cout << "Ready to control Servos of the following IDs: ";
        for (const auto& id_servo : servos_) {
          std::cout << id_servo.first << " ";
        }
        std::cout << std::endl;
      } else {
        std::cout << "No available Servos found." << std::endl;
      }
    }
  }

  ///////////////////////////////////////////////////////////////
  ///           Commands you can send to the Servos.          ///

  void Input(const std::map<int, std::map<CommandType, double>> commands) {
    InputGetter(commands);
  }

  void InputAll(const std::map<CommandType, double> command) {
    std::map<int, std::map<CommandType, double>> commands;
    for (const auto id : ids_) {
      commands[id] = command;
    }
    InputGetter(commands);
  }

  void Stop(const std::set<int> ids) {
    std::map<int, std::map<CommandType, double>> commands;
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        servo.value()->controller_->SetStop();
        commands[id][CommandType::POSITION] = NaN;
        commands[id][CommandType::VELOCITY] = 0.0;
        commands[id][CommandType::FEEDFORWARD_TORQUE] = 0.0;
      }
    }
    InputGetter(commands);
  }

  void StopAll() { Stop(ids_); }

  void SetBasePositions(const std::set<int> ids) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        if (!servo.value()->SetBasePosition(0.0)) {
          std::cout << "Base position setting failed for ID " << id << "."
                    << std::endl;
        } else {
          std::cout << "Base position setting succeeded for ID " << id << "."
                    << std::endl;
        }
      } else {
        std::cout << "No Servo of ID " << id
                  << " found.  Base position setting failed," << std::endl;
      }
    }
  }

  void SetBasePositionsAll() { SetBasePositions(ids_); }

  void GetOutput(const void* output, size_t size) {
    InternalOutputSender(output, size);
  }

  std::set<int> GetIds() { return ids_; }

  void StartThread(const std::string& name) {
    const auto& maybe_thread_manager = Utils::SafeAt(ThreadsManager, name);
    if (maybe_thread_manager) {
      maybe_thread_manager.value()->Start();
    } else {
      std::cout << "Cannot find thread of name " << name << "." << std::endl;
    }
  }

  void StartThreadAll() {
    std::set<LoopingThreadManager*> threads;
    for (const auto& pair : ThreadsManager) {
      threads.insert(pair.second);
    }
    for (const auto& thread : threads) {
      StartThread(thread);
    }
  }

  void TerminateThread(const std::string& name) {
    const auto& maybe_thread_manager = Utils::SafeAt(ThreadsManager, name);
    if (maybe_thread_manager) {
      maybe_thread_manager.value()->Terminate();
    } else {
      std::cout << "Cannot find thread of name " << name << "." << std::endl;
    }
  }

  void TerminateThreadAll() {
    std::set<LoopingThreadManager*> threads;
    for (const auto& pair : ThreadsManager) {
      threads.insert(pair.second);
    }
    for (const auto& thread : threads) {
      TerminateThread(thread);
    }
  }

  void PrintThreads() {
    std::set<LoopingThreadManager*> threads;
    for (const auto& pair : ThreadsManager) {
      threads.insert(pair.second);
    }
    int i = 1;
    for (const auto& thread : threads) {
      std::cout << "Thread #" << i++ << ":" << std::endl;
      std::cout << "    Aliases: ";
      for (const auto& alias : thread->aliases_) {
        std::cout << alias << ", ";
      }
      std::cout << std::endl
                << "    Is On?: " << (thread->IsOn() ? "True" : "False")
                << std::endl;
    }
  }

  ///           Commands you can send to the Servos.          ///
  ///////////////////////////////////////////////////////////////

 protected:
  std::set<int> ids_;
  std::map<int, std::shared_ptr<Servo>> servos_;  // ID -> Servo
  std::shared_ptr<moteus::Transport> transport_;
  const unsigned int cycle_period_us_ = 0.01 * 1e6;
  std::mutex servo_access_mutex_;
  LoopingThreadManager runner_thread_manager_;
  LoopingThreadManager external_input_getter_manager_;
  LoopingThreadManager external_output_sender_manager_;

  virtual void InternalOutputSender(const void* output, size_t size) {
    char* output_charptr = (char*)output;
    int written;
    for (const auto& id_servo : servos_) {
      const auto id = id_servo.first;
      const auto& servo = id_servo.second;
      moteus::Query::Result recent_state;
      bool updated_this_cycle;
      {
        std::lock_guard<std::mutex> lock(servo_access_mutex_);
        recent_state = servo->state_.recent_reply;
        updated_this_cycle = servo->state_.updated_this_cycle;
      }

      if (updated_this_cycle) {
        written = ::snprintf(output_charptr, size,
                             "id=%2d/mode=%2d/pos=%3.3f/vel=%3.3f/tor=%3.3f/"
                             "volt=%3.2f/temp=%2.1f/fault=%2d\n",
                             id, static_cast<int>(recent_state.mode),
                             recent_state.position - servo->base_position_,
                             recent_state.velocity, recent_state.torque,
                             recent_state.voltage, recent_state.temperature,
                             recent_state.fault);
      } else {
        written = ::snprintf(output_charptr, size,
                             "id=%2d/NOT UPDATED THIS CYCLE\n", id);
      }

      if (written < 0) {
        std::cout
            << "Error occured during printing output to designated pointer."
            << std::endl;
        return;
      } else if (written >= size) {
        std::cout << "Insufficient size to accommodate output." << std::endl;
        return;
      }

      output_charptr += written;
      size -= written;
    }
  }

  virtual void ExternalOutputSender(std::atomic_bool* terminated) {
    std::cout << "ExternalOutputSender thread is running..." << std::endl;

    /// Probably let a ROS2 publisher publish servo states
    /// or send encoded servo states as UDP packets.
  }

  virtual void ExternalInputGetter(std::atomic_bool* terminated) {
    std::cout << "ExternalInputGetter thread is running..." << std::endl;

    while (!(*terminated).load()) {
      std::string input;
      std::getline(std::cin, input);
      if (input.empty()) continue;
      InputGetter(Parser::ParseStringInput(input));
    }
  }

  void InputGetter(
      const std::map<int, std::map<CommandType, double>>& commands) {
    std::cout << "Parsing input..." << std::endl;
    for (const auto& id_cmd : commands) {
      const auto id = id_cmd.first;
      const auto& cmd = id_cmd.second;
      const auto& maybe_servo = Utils::SafeAt(servos_, id);
      if (!maybe_servo) {
        std::cout << "No Servo of ID " << id
                  << " found on the Servo list.  Ignoring command for this ID."
                  << std::endl;
        continue;
      }
      const auto& servo = maybe_servo.value();

      std::cout << "Parsing input for Servo ID " << id << "..." << std::endl;
      for (const auto& type_value : cmd) {
        const auto type = type_value.first;
        auto value = type_value.second;
        std::cout << "Parsing input of CommandType " << type << "..."
                  << std::endl;
        {
          std::lock_guard<std::mutex> lock(servo_access_mutex_);
          if (((moteus::Resolution*)(&(
                  servo->controller_->options().position_format)))[type] ==
              moteus::Resolution::kIgnore) {
            std::cout << "ServoSystem PositionMode Format is set to " << type
                      << " = " << "kIgnore.  Ignoring command." << std::endl;
            continue;
          }
          if (type == CommandType::POSITION && std::isfinite(value)) {
            value += servo->base_position_;
          }
          ((double*)(&(servo->command_)))[type] = value;
        }
        std::cout << "Successfully set command " << type << " = " << value
                  << " for Servo ID " << id << std::endl;
      }
    }
  }

 private:
  void Run(std::atomic_bool* terminated) {
    std::cout << "Runner thread is running..." << std::endl;

    if (servos_.empty()) {
      std::cout << "No Servos found. Runner thread terminating." << std::endl;
      return;
    }

    while (!(*terminated).load()) {
      ::usleep(cycle_period_us_);

      std::vector<moteus::CanFdFrame> command_frames;
      for (auto& id_servo : servos_) {
        auto id = id_servo.first;
        auto& servo = id_servo.second;
        servo->state_.updated_this_cycle = false;
        {
          std::lock_guard<std::mutex> lock(servo_access_mutex_);
          command_frames.push_back(
              servo->controller_->MakePosition(servo->command_));
        }
      }

      std::vector<moteus::CanFdFrame> reply_frames;
      transport_->BlockingCycle(&command_frames[0], command_frames.size(),
                                &reply_frames);

      for (const auto& frame : reply_frames) {
        int id = frame.source;
        const auto& maybe_servo = Utils::SafeAt(servos_, id);
        if (maybe_servo) {
          auto& state = maybe_servo.value()->state_;
          state.recent_reply = moteus::Query::Parse(frame.data, frame.size);
          state.updated_this_cycle = true;
        }
      }
    }
  }

  void StartThread(LoopingThreadManager* thread) {
    if (thread) thread->Start();
  }

  void TerminateThread(LoopingThreadManager* thread) {
    if (thread) thread->Terminate();
  }
};

}  // namespace som
