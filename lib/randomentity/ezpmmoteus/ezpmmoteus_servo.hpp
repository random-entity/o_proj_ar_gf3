#pragma once

#include "ezpmmoteus_multiplex.hpp"
#include "moteus.h"

namespace randomentity::ezpmmoteus {
using namespace mjbots;

class Servo {
 public:
  Servo(const int id, const int bus,
        const std::shared_ptr<moteus::Transport>& transport,
        const moteus::PositionMode::Format& cmd_fmt,
        const moteus::PositionMode::Command& init_cmd,
        const CommandPositionRelativeTo usr_cmd_pos_rel_to,
        const moteus::Query::Format& rpl_fmt,
        const ReplyPositionRelativeTo usr_rpl_pos_rel_to,  //
        const bool use_aux2,
        const ReplyPositionRelativeTo usr_rpl_aux2_pos_rel_to)
      : controller_{std::make_unique<moteus::Controller>([&]() {
          moteus::Controller::Options options;
          options.id = id;
          options.bus = bus;
          options.transport = transport;
          options.position_format = cmd_fmt;
          options.query_format = rpl_fmt;
          return options;
        }())},
        usr_cmd_{init_cmd},
        usr_cmd_pos_rel_to_{usr_cmd_pos_rel_to},
        usr_rpl_pos_rel_to_{usr_rpl_pos_rel_to},
        use_aux2_{use_aux2},
        usr_rpl_aux2_pos_rel_to_{usr_rpl_aux2_pos_rel_to} {
    /* TODO: moteus firmware-API compatibility check */

    /// Execute Stop Command to clears all faults.
    const auto maybe_reply = controller_->SetStop();
    std::cout << (maybe_reply ? "Got" : "Failed to get")
              << " reply from initial Stop Command for Servo ID " << id << "."
              << std::endl;
  }

  const bool InitSucceeded() {
    return use_aux2_ ? std::isfinite(base_pos_) && std::isfinite(base_aux2_pos_)
                     : std::isfinite(base_pos_);
  }

  const int GetId() {
    static const auto id = controller_->options().id;
    return id;
  }

  moteus::Query::Result GetReply() { return usr_rpl(); }

  void SetReply(const moteus::Query::Result& new_sys_rpl) {
    /// Track aux2 revolution by checking impossible velocity.
    /// This method won't work if Replies are not received
    /// in fast enough frequency continuously.
    const auto delta = new_sys_rpl.abs_position - sys_rpl_.abs_position;
    if (delta > 0.5) {
      aux2_revs_--;
    } else if (delta < -0.5) {
      aux2_revs_++;
    }

    /// sys_rpl_ contains coiled position in order to check
    /// impossible velocity by comparing with the next Reply.
    sys_rpl_ = new_sys_rpl;
  }

  void SetCommandPositionRelativeTo(CommandPositionRelativeTo new_val) {
    usr_cmd_pos_rel_to_ = new_val;
  }

  void SetReplyPositionRelativeTo(ReplyPositionRelativeTo new_val) {
    usr_rpl_pos_rel_to_ = new_val;
  }

  void SetReplyAux2PositionRelativeTo(ReplyPositionRelativeTo new_val) {
    usr_rpl_aux2_pos_rel_to_ = new_val;
  }

  enum class WhichPosition { Internal, Aux2 };

  bool SetBasePosition(const double time_limit) {
    return SetBase(time_limit, WhichPosition::Internal);
  }

  bool SetBaseAux2Position(const double time_limit) {
    return SetBase(time_limit, WhichPosition::Aux2);
  }

  /// At least one attempt is guaranteed even if @a time_limit is non-positive.
  bool SetBase(const double time_limit, WhichPosition which_pos) {
    const std::string which_str =
        (which_pos == WhichPosition::Internal) ? "" : "aux2 ";
    const int id = GetId();
    std::cout << "Attempting to get current " << which_str
              << "position from Servo ID " << id << "..." << std::endl;
    const double time_init = Utils::GetTime();
    while (true) {
      const auto& maybe_reply = controller_->SetQuery();
      if (!maybe_reply) {
        const double time_passed = Utils::GetTime() - time_init;
        const double time_left = time_limit - time_passed;
        if (time_left > 0.0) {
          std::cout << "Not getting reply from Servo ID " << id
                    << ", but will try " << time_left << " more seconds...\r";
          ::usleep(1e6 / 100.0);
          continue;
        } else {
          std::cout << "Failed to get reply from Servo ID " << id << " for "
                    << time_limit << " seconds.  " << "Base " << which_str
                    << "position setting failed." << std::endl;
          return false;
        }
      } else {
        const double cur_val = (which_pos == WhichPosition::Internal)
                                   ? maybe_reply->values.position
                                   : maybe_reply->values.abs_position;
        if (std::isfinite(cur_val)) {
          auto& base = (which_pos == WhichPosition::Internal) ? base_pos_
                                                              : base_aux2_pos_;
          base = cur_val;
          std::cout << "Successfully set base " << which_str
                    << "position to current " << which_str
                    << "position for Servo ID " << id << ": " << base
                    << std::endl;
          SetReply(maybe_reply->values);
          return true;
        } else {
          const double time_passed = Utils::GetTime() - time_init;
          const double time_left = time_limit - time_passed;
          if (time_left > 0.0) {
            std::cout << "Current " << which_str << "position for Servo ID "
                      << id << " is NaN, but will try " << time_left
                      << " more seconds...\r";
            ::usleep(1e6 / 100.0);
            continue;
          } else {
            std::cout << "Failed to get finite value as the current "
                      << which_str << "position for Servo ID " << id << " for "
                      << time_limit << " seconds.  " << "Base " << which_str
                      << "position setting failed." << std::endl;
            return false;
          }
        }
      }
    }
  }

  const std::unique_ptr<moteus::Controller> controller_;
  CommandPositionRelativeTo usr_cmd_pos_rel_to_;
  moteus::PositionMode::Command usr_cmd_;
  moteus::PositionMode::Command sys_cmd() {
    auto cmd = usr_cmd_;
    switch (usr_cmd_pos_rel_to_) {
      case CommandPositionRelativeTo::Base:
        if (std::isnan(base_pos_) || std::isnan(cmd.position)) break;
        cmd.position += base_pos_;
        break;
      case CommandPositionRelativeTo::Recent:
        if (std::isnan(cmd.position)) break;
        cmd.position += sys_rpl_.position;
        break;
      default:  // case CommandPositionRelativeTo::Absolute:
        break;
    }
    return cmd;
  }
  ReplyPositionRelativeTo usr_rpl_pos_rel_to_;
  ReplyPositionRelativeTo usr_rpl_aux2_pos_rel_to_;
  moteus::Query::Result sys_rpl_;  /// aux2 position coiled.
                                   /// Must update via dedicated setter
                                   /// to track aux2 revolutions.
  moteus::Query::Result usr_rpl() {
    auto rpl = sys_rpl_;
    rpl.abs_position += aux2_revs_;  // Uncoil aux2 position.
    if (usr_rpl_pos_rel_to_ == ReplyPositionRelativeTo::Base) {
      rpl.position -= base_pos_;
    }
    if (usr_rpl_aux2_pos_rel_to_ == ReplyPositionRelativeTo::Base) {
      rpl.abs_position -= base_aux2_pos_;
    }
    return rpl;
  }
  bool updated_last_cycle_ = false;
  double base_pos_ = NaN;
  double base_aux2_pos_ = NaN;
  int aux2_revs_ = 0;
  bool use_aux2_;
};

}  // namespace randomentity::ezpmmoteus
