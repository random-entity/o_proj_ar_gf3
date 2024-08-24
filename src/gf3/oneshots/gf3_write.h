#pragma once

#include "gf3_oneshots.h"

namespace gf3 {

void GF3Oneshots::Write(GF3* gf3) {
  json j = *gf3;
  std::ofstream outfile{"../poses/gf3-pose_" +
                        std::to_string(gf3->cmd_.write.fileindex) + ".json"};
  if (outfile.is_open()) {
    outfile << j.dump(4);
    outfile.close();
  } else {
    std::cerr << "Error opening file for writing." << std::endl;
  }
}

}  // namespace gf3
