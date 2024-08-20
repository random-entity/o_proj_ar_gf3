// Do NOT use this often!

#pragma once

#include "../executer.h"

namespace gf3::utils {

void Zero(GF3& gf3) {
  // Zero.
  for (const auto& s : gf3.servos_set_) {
    std::cout << "Zeroing aux2 position for Servo ID " << s->GetId()
              << std::endl;

    s->SetStop();
    s->DiagnosticWrite("tel stop\n");
    s->DiagnosticFlush();

    const std::string reset_offset =
        "conf set motor_position.sources.1.offset 0";
    std::cout << reset_offset << s->DiagnosticCommand(reset_offset)
              << std::endl;

    std::cout << "Query result: ";
    const auto maybe_reply = s->SetQuery(&global_q_fmt);
    if (!maybe_reply) {
      std::cout << "NO Reply! Skip to next Servo." << std::endl;
      continue;
    }
    const auto cur_filtered = 4096.0 * maybe_reply->values.extra[0].value;
    std::cout << cur_filtered << std::endl;

    std::cout << s->DiagnosticCommand(
                     "tel get motor_position",
                     Controller::DiagnosticReplyMode::kExpectSingleLine)
              << std::endl;

    std::ostringstream ostr;
    ostr << "conf set motor_position.sources.1.offset " << (-cur_filtered);
    const auto d_result = s->DiagnosticCommand(ostr.str());
    std::cout << ostr.str() << ": " << d_result << std::endl;

    // s->DiagnosticCommand("conf write");
  }
}

}  // namespace gf3::utils
