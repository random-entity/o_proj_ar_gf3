#pragma once

#include "slcm_base.hpp"

namespace som {
class VanillaServoSystem : virtual public ServoSystem {
 public:
  VanillaServoSystem()
      : stdin_ecg_{this,
                   [](ServoSystem* servosystem, std::atomic_bool* terminated) {
                     dynamic_cast<VanillaServoSystem*>(servosystem)
                         ->StdinECG(terminated);
                   },
                   {"Standard Input ExternalCommandGetter", "stdin_ecg"}} {}

 protected:
  LoopingThreadManager stdin_ecg_;

  virtual void StdinECG(std::atomic_bool* terminated) {
    std::cout << "Standard Input ExternalCommandGetter thread is running..."
              << std::endl;

    while (!terminated->load()) {
      std::string input;
      std::getline(std::cin, input);
      if (!listen_.external || input.empty()) continue;
      EmplaceCommands(Parser::ParseCommandString(input));
    }
  }
};
}  // namespace som
