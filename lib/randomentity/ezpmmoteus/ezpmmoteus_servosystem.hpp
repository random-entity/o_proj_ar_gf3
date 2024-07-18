#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "moteus.h"

namespace random_entity::ezpmmoteus {
using namespace mjbots;

class ICommandGetter {
 public:
  ICommandGetter(IServoSystem* servosystem)
      : servosystem_{servosystem}, thread_{nullptr} {};

  IServoSystem* servosystem_;
  std::function<void(IServoSystem*, std::atomic_bool*)> task_;
  std::atomic_bool terminated_;
  std::thread* thread_;
  std::vector<std::string> aliases_;

  void Start() {
    if (!thread_) {
      terminated_ = false;
      thread_ = new std::thread(task_, servosystem_, &terminated_);
      std::cout << aliases_[0]
                << " thread successfully initialized and detached."
                << std::endl;
    } else {
      std::cout << aliases_[0] << " thread is already running." << std::endl;
    }
  }
};

class IServoSystem {
 public:
  struct ICommand {};
  std::map<std::string, ICommandGetter> cmd_getters_;
};

class DifferentialJoint : public IServoSystem {
 public:
  struct Command : IServoSystem::ICommand {
    double avg_axis;
    double diff_axis;
  };
};

class ServoGroup {
  std::map<std::string, IServoSystem> servosystems_;
};

}  // namespace random_entity::ezpmmoteus