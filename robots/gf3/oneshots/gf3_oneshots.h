#pragma once

#include "../servo_units/gf3.h"

namespace gf3 {

struct GF3Oneshots {
  static void Read(GF3*);
  static void Write(GF3*);

  static inline const std::map<uint8_t, void (*)(GF3*)> methods = {{0, &Read},
                                                                   {1, &Write}};

  static void Shoot(GF3* gf3) {
    for (uint8_t shot = 0; shot < 8; shot++) {
      if (gf3->cmd_.shots & (1 << shot)) {
        const auto maybe_method = utils::SafeAt(methods, shot);
        if (maybe_method) {
          maybe_method.value()(gf3);
        }
        gf3->cmd_.shots &= ~(1 << shot);
      }
    }
  }
};

}  // namespace gf3
