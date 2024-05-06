#include <map>

#include "moteus.h"
#ifdef __RASPBERRY_PI__
#include "pi3hat.h"
#endif

#define NaN (std::numeric_limits<double>::quiet_NaN())

using namespace mjbots;

namespace som {

class ServoSystem {
 public:
  ServoSystem(std::map<int, int> id_bus_map, double maximum_torque = 1,
              moteus::PositionMode::Format position_format =
                  moteus::PositionMode::Format{
                      .maximum_torque = moteus::kFloat}) {
    for (auto id_bus : id_bus_map) {
      int id = id_bus.first;
      int bus = id_bus.second;
      servos_.emplace(id, std::make_shared<Servo>(id, bus, maximum_torque,
                                                  position_format));
    }
  }

  void Run() {
    input_getter_thread = std::thread(&ServoSystem::InputGetter, this);

    while (true) {
      ::usleep(10000);

      std::vector<moteus::CanFdFrame> command_frames;
      for (auto& id_servo : servos_) {
        auto id = id_servo.first;
        auto& servo = id_servo.second;
        servo->state_.updated_this_cycle = false;
        {
          std::lock_guard<std::mutex> lock(mutex);
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

    input_getter_thread.join();
  }

 protected:
  class Servo {
    friend ServoSystem;

   public:
    Servo(int id, int bus, double maximum_torque,
          moteus::PositionMode::Format position_format) {
      moteus::Controller::Options options;
      options.id = id;
      options.position_format = position_format;
      controller_ = std::make_shared<moteus::Controller>(options);

      auto maybe_reply = controller_->SetStop();
      if (maybe_reply) {
        state_.recent_state = maybe_reply->values;
      } else {
        std::cout << "Cannot get reply from controller ID " << id
                  << " after SetStop command" << std::endl;
      }

      while (!std::isfinite(initial_position_)) {
        maybe_reply = controller_->SetQuery();
        if (!maybe_reply) {
          std::cout << "Cannot get initial position from controller ID " << id
                    << "\r";
          ::usleep(10000);
          continue;
        }
        std::cout << std::endl << std::flush;
        initial_position_ = maybe_reply->values.position;
      }

      command_.maximum_torque = maximum_torque;
    }

   private:
    std::shared_ptr<moteus::Controller> controller_;
    double initial_position_ = NaN;
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
        {
          std::lock_guard<std::mutex> lock(mutex);
          servos_[id]->command_.position =
              servos_[id]->initial_position_ + position;
        }
      }
    }
  }

  std::map<int, std::shared_ptr<Servo>> servos_;  // ID -> Servo
  std::mutex mutex;
  std::thread input_getter_thread;
  std::shared_ptr<moteus::Transport> transport_ =
      moteus::Controller::MakeSingletonTransport({});

 private:
  struct Parser {
    static std::vector<std::string> Split(const std::string str,
                                          char splitter) {
      std::vector<std::string> result;
      size_t start = 0;
      auto pos = str.find(splitter);
      while (pos != std::string::npos) {
        result.push_back(str.substr(start, pos - start + 1));
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
        auto id = std::stoi(split_by_equals[0]);
        auto position = std::stod(split_by_equals[1]);
        result[id] = position;
      }
      return result;
    }
  };
};

}  // namespace som
