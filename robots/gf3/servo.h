#pragma once

#include <cmath>

#include "utils/imports.h"
#include "utils/utils.h"

namespace gf3 {

enum class PosRelTo : uint8_t { Absolute = 0, Base = 1, Recent = 2 };

class Servo : public Controller {
 public:
  Servo(const int id, const uint8_t bus,
        const std::shared_ptr<Transport>& transport,
        const PmFmt* pm_fmt = nullptr, const QFmt* q_fmt = nullptr)
      : Controller{[&]() {
          Options options;
          options.id = id;
          options.bus = bus;
          options.transport = transport;
          options.position_format = pm_fmt ? *pm_fmt : PositionMode::Format{};
          options.query_format = q_fmt ? *q_fmt : Query::Format{};
          options.default_query = true;
          return options;
        }()},
        id_{id} {
    SetStop();
  }

  int GetId() { return id_; }

  CanFdFrame GetFrame(PmCmd cmd, PosRelTo pos_rel) {
    switch (pos_rel) {
      case PosRelTo::Base: {
        if (std::isnan(base_pos_) || std::isnan(cmd.position)) break;
        cmd.position += base_pos_;
      } break;
      case PosRelTo::Recent: {
        if (std::isnan(cmd.position)) break;
        if (utils::GetTime() - last_rpl_time_ < 1.0) {
          cmd.position += last_rpl_.position;
        } else {
          std::cout << "More that 1s has past since last Reply. "
                       "Setting Command position to NaN."
                    << std::endl;
          cmd.position = NaN;
        }
      } break;
      default:
        break;
    }

    return MakePosition(cmd);
  }

  // Does NOT handle PosRelTo::Recent.
  QRpl GetRpl(PosRelTo pos_rel = PosRelTo::Absolute,
              PosRelTo aux2_pos_rel = PosRelTo::Absolute) {
    auto rpl = last_rpl_;
    rpl.abs_position += aux2_revs_;  // Uncoil aux2 position.
    if (pos_rel == PosRelTo::Base && std::isfinite(base_pos_)) {
      rpl.position -= base_pos_;
    }
    if (aux2_pos_rel == PosRelTo::Base && std::isfinite(base_aux2_pos_)) {
      rpl.abs_position -= base_aux2_pos_;
    }
    return rpl;
  }

  void SetRpl(const QRpl& new_sys_rpl) {
    const auto delta_aux2_pos =
        new_sys_rpl.abs_position - last_rpl_.abs_position;
    if (delta_aux2_pos > 0.5) {
      aux2_revs_--;
    } else if (delta_aux2_pos < -0.5) {
      aux2_revs_++;
    }

    last_rpl_ = new_sys_rpl;
    last_rpl_time_ = utils::GetTime();
  }

  double GetBaseAux2Pos() { return base_aux2_pos_; }

  void SetBaseAux2PosFromCurrent(const double& offset = 0.0) {
    const auto maybe_rpl = SetQuery();
    if (maybe_rpl) SetRpl(maybe_rpl->values);
    base_aux2_pos_ = GetRpl().abs_position - offset;
  }

  void SetBaseAux2PosToValue(const double& base_aux2_pos) {
    base_aux2_pos_ = base_aux2_pos;
  }

 private:
  const int id_;

  double last_rpl_time_;
  QRpl last_rpl_;  // Raw system Reply where aux2 position is coiled.
                   // Should be updated by `SetRpl()` to track aux2 revolutions.
                   // This field necessary in order to compare with a new Reply
                   // to update aux2 revolutions.
  int aux2_revs_ = 0;

  double base_pos_ = NaN;
  double base_aux2_pos_ = NaN;
};

}  // namespace gf3
