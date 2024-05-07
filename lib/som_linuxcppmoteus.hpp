#include <map>

#include "moteus.h"
#ifdef __RASPBERRY_PI__
#include "pi3hat.h"
#endif

#ifdef NaN
#define NaN_backup NaN
#undef NaN
#endif
#define NaN (std::numeric_limits<double>::quiet_NaN())

namespace som {
using namespace mjbots;

namespace helpers {
static double GetNow() {  // Get precise current time in seconds
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) * 1e-9;
}
}  // namespace helpers

class ServoSystem {
 public:
  ServoSystem(const std::map<int, int>& id_bus_map = {{1, 0}},
              const double maximum_torque = 1.0,
              const moteus::PositionMode::Format& position_format =
                  moteus::PositionMode::Format{
                      .maximum_torque = moteus::kFloat}) {
    transport_ = moteus::Controller::MakeSingletonTransport({});
    if (transport_) {
      std::cout << "Default transport found: " << transport_ << std::endl;
    } else {
      std::cout << "Default transport not found. Returning from "
                   "`ServoSystem` constructor without any initialization."
                << std::endl;
      return;
    }

    std::vector<int> init_failed_ids;
    for (auto& id_bus : id_bus_map) {
      auto id = id_bus.first;
      auto bus = id_bus.second;
      std::cout << "Initializing servo for ID: " << id << ", bus: " << bus
                << "..." << std::endl;
      auto servo =
          std::make_shared<Servo>(id, bus, maximum_torque, position_format);
      if (servo->init_suceeded_()) {
        servos_[id] = servo;
        std::cout << "Initialization succeeded for ID: " << id << std::endl;
      } else {
        init_failed_ids.push_back(id);
        std::cout << "Initialization failed for ID: " << id << std::endl;
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
  }

  void Run() {
    if (servos_.empty()) {
      std::cout
          << "No servos found. Exiting `Run` for this `ServoSystem` instance."
          << std::endl;
      return;
    }

    input_getter_thread_ = std::thread(&ServoSystem::InputGetter, this);

    while (true) {
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
        if (servos_.find(id) != servos_.end()) {
          auto& state = servos_[id]->state_;
          state.recent_state = moteus::Query::Parse(frame.data, frame.size);
          state.updated_this_cycle = true;
        }
      }
    }

    input_getter_thread_.join();
  }

 protected:
  class Servo {
    friend ServoSystem;

   public:
    Servo(const int id, const int bus, const double maximum_torque,
          const moteus::PositionMode::Format& position_format) {
      // Initialize moteus controller
      controller_ = std::make_shared<moteus::Controller>([&, id]() {
        moteus::Controller::Options options;
        options.id = id;
        options.position_format = position_format;
        return options;
      }());

      // Initial stop command required before any operation
      auto maybe_reply = controller_->SetStop();
      std::cout << (maybe_reply ? "Got" : "Failed to get")
                << " reply from controller ID " << id
                << " for initial SetStop command." << std::endl;

      // Get initial motor position
      std::cout << "Attempting to get initial position from controller ID "
                << id << "..." << std::endl;
      const double time_init = helpers::GetNow();
      while (true) {
        maybe_reply = controller_->SetQuery();
        if (!maybe_reply) {
          if (helpers::GetNow() - time_init < 3.0) {
            std::cout << "Not getting replies from controller ID " << id
                      << "...\r";
            ::usleep(10000);
            continue;
          } else {
            std::cout << "Failed to get initial position from controller ID "
                      << id << " for 3 seconds. Skip this ID." << std::endl;
            break;
          }
        } else {
          state_.recent_state = maybe_reply->values;
          initial_position_ = state_.recent_state.position;
          std::cout << "Got initial position from controller ID " << id << ": "
                    << initial_position_
                    << (init_suceeded_()
                            ? ""
                            : ", but is NaN. Initialization failed.")
                    << std::endl;
          break;
        }
      }

      // Setup command values that will be constant over program lifetime
      command_.maximum_torque = maximum_torque;
    }

   private:
    std::shared_ptr<moteus::Controller> controller_;
    double initial_position_ = NaN;
    bool init_suceeded_() { return std::isfinite(initial_position_); }
    moteus::PositionMode::Command command_{.position = NaN};
    struct State {
      moteus::Query::Result recent_state;
      bool updated_this_cycle = false;
    } state_;
  };

  virtual void InputGetter() {
    std::string input;
    while (true) {
      std::getline(std::cin, input);
      for (auto id_pos : Parser::ParseInput(input)) {
        auto id = id_pos.first;
        auto position = id_pos.second;
        if (servos_.find(id) == servos_.end()) {
          std::cout << "No servo with ID " << id
                    << " found on the servo list. Ignoring command."
                    << std::endl;
          continue;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          servos_[id]->command_.position =
              servos_[id]->initial_position_ + position;
        }
      }
    }
  }

  std::map<int, std::shared_ptr<Servo>> servos_;  // ID -> Servo
  std::shared_ptr<moteus::Transport> transport_;
  const unsigned int cycle_period_us_ = 10000;

 private:
  struct Parser {
    static std::vector<std::string> Split(const std::string str,
                                          char splitter) {
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

    static std::map<int, double> ParseInput(std::string input) {
      std::map<int, double> result;
      auto split_by_comma = Split(input, ',');
      for (auto& id_pos : split_by_comma) {
        auto split_by_equals = Split(id_pos, '=');
        try {
          if (split_by_equals.size() != 2) throw std::exception();
          auto id = std::stoi(split_by_equals[0]);
          auto position = std::stod(split_by_equals[1]);
          result[id] = position;
        } catch (std::exception& e) {
          std::cout << "Ignoring wrong command: "
                    << (id_pos.empty() ? "<empty>" : id_pos) << std::endl;
        }
      }
      return result;
    }
  };

  std::mutex mutex_;
  std::thread input_getter_thread_;
};

}  // namespace som

#ifdef NaN_backup
#undef NaN
#define NaN NaN_backup
#undef NaN_backup
#endif
