#pragma once

#include "gf3_oneshots.h"

namespace gf3 {

void GF3Oneshots::Read(GF3* gf3) {
  std::ifstream infile{"../poses/gf3-pose_" +
                       std::to_string(gf3->cmd_.read.fileindex) + ".json"};
  if (infile.is_open()) {
    json j;
    infile >> j;
    from_json(j, *gf3);
    infile.close();
  } else {
    std::cerr << "Error opening file for reading." << std::endl;
  }
}

}  // namespace gf3
