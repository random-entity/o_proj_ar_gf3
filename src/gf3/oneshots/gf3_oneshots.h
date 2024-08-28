#pragma once

#include "../servo_units/gf3.h"

namespace gf3 {

struct GF3Oneshots {
  static void Read(GF3*);
  static void Write(GF3*);

  static inline const std::map<uint8_t, void (*)(GF3*)> oneshots = {
      {0, &Read}, {1, &Write}};

  static void Shoot(GF3* gf3) {
    for (uint8_t oneshot = 0; oneshot < 8; oneshot++) {
      if (gf3->cmd_.oneshots & (1 << oneshot)) {
        const auto maybe_method = utils::SafeAt(oneshots, oneshot);
        if (maybe_method) {
          maybe_method.value()(gf3);
        }
      }
    }
    gf3->cmd_.oneshots = 0;
  }
};

}  // namespace gf3
