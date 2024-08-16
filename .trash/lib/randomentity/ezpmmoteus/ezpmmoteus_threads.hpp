#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>

#include "moteus_protocol.h"

namespace random_entity::ezpmmoteus {
using namespace mjbots;

class ICommandGetter {
 public:
  std::atomic_bool terminated{false};
  virtual void get_cmd(std::queue<moteus::CanFdFrame>& cmd_q,
                       std::mutex& q_mutex, std::condition_variable& cv) = 0;
};

std::map<std::string, std::function<void(ServoSystem*, std::atomic_bool*)>>
    threads = {{"dummy", [](ServoSystem* ss, std::atomic_bool* terminated) {
                  while (!terminated->load()) {
                    /* code */
                  }
                }}};

}  // namespace random_entity::ezpmmoteus
