#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "moteus.h"

#ifdef __RASPBERRY_PI__
#include "pi3hat.h"
#include "pi3hat_moteus_transport.h"
#endif

namespace gf3 {
using namespace mjbots::moteus;
using PmCmd = PositionMode::Command;
using PmFmt = PositionMode::Format;
using QRpl = Query::Result;
using QFmt = Query::Format;
using json = nlohmann::json;
}  // namespace gf3
