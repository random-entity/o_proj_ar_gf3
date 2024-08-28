#pragma once

#include "helpers/helpers.h"

namespace gf3::globals {

std::shared_ptr<Transport> transport{[] {
#ifdef __RASPBERRY_PI__
  using namespace mjbots::pi3hat;

  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
    throw std::runtime_error("Error locking memory");
  }
  mjbots::pi3hat::Pi3HatMoteusFactory::Register();
  return Controller::MakeSingletonTransport(
      {"--pi3hat-cpu", "2",  //
       "--pi3hat-cfg", "1=1,2,3,4,5,6;2=7,8,9,10,11,12;3=13,14",
       "--pi3hat-disable-aux"});
#else
  return Controller::MakeSingletonTransport({});
#endif
}()};

const PmFmt pm_fmt{.position = kFloat,
                   .velocity = kFloat,
                   .feedforward_torque = kIgnore,
                   .kp_scale = kIgnore,
                   .kd_scale = kIgnore,
                   .maximum_torque = kFloat,
                   .stop_position = kIgnore,  // Do NOT use!
                   .watchdog_timeout = kFloat,
                   .velocity_limit = kFloat,
                   .accel_limit = kFloat,
                   .fixed_voltage_override = kIgnore,
                   .ilimit_scale = kIgnore};

const PmCmd pm_cmd_template{.position = NaN,
                            .velocity = 0.0,
                            .feedforward_torque = 0.0,
                            .kp_scale = 1.0,
                            .kd_scale = 1.0,
                            .maximum_torque = 32.0,
                            .stop_position = NaN,  // Do NOT use!
                            .watchdog_timeout = NaN,
                            .velocity_limit = 32.0,
                            .accel_limit = 32.0,
                            .fixed_voltage_override = NaN,
                            .ilimit_scale = 1.0};

const QFmt q_fmt{[] {
  QFmt fmt;
  fmt.mode = kInt8;
  fmt.position = kFloat;
  fmt.velocity = kFloat;
  fmt.torque = kFloat;
  fmt.q_current = kIgnore;
  fmt.d_current = kIgnore;
  fmt.abs_position = kFloat;
  fmt.power = kFloat;
  fmt.motor_temperature = kFloat;
  fmt.trajectory_complete = kInt8;
  fmt.home_state = kIgnore;
  fmt.voltage = kFloat;
  fmt.temperature = kFloat;
  fmt.fault = kInt8;
  fmt.extra[0] = {.register_number = kEncoder1Velocity, .resolution = kFloat};
  fmt.extra[1] = {.register_number = kEncoderValidity, .resolution = kInt8};
  return fmt;
}()};

}  // namespace gf3::globals
