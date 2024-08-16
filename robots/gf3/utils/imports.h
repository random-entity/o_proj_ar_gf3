#pragma once

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
}  // namespace gf3
