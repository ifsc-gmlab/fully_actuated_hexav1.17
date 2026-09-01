#pragma once

#include "actuator.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

// A FramePreset describes everything a vehicle needs OTHER than identity
// (model name, sysid, transport). It returns a fresh list of ActuatorOutputs
// each time so multiple vehicles can share the same preset without aliasing.
struct FramePreset {
    std::string name;
    std::string description;
    std::string sensor_link = "base_link";   // gz topic link name for IMU/mag/baro/gps
    std::vector<std::function<std::unique_ptr<ActuatorOutput>()>> outputs;
};

// Look up a built-in preset by name. Returns nullptr if not found.
// Built-in preset names:
//   x500_quad             — 4 motors (default x500)
//   x500_hex              — 6 motors
//   fully_actuated_hexa   — 6 tilted motors (max 1600 rad/s)
//   x500_octo             — 8 motors
//   standard_vtol         — 4 lift motors + 1 pusher + 4 servos (elevon L/R + rudder + ?)
//   rc_cessna             — 1 throttle + aileron + elevator + rudder
//   r1_rover              — 2 wheels (diff drive)
//   rover_ackermann       — 2 wheels + 1 steering servo
const FramePreset *findBuiltinPreset(const std::string &name);

// List all built-in preset names (for --list-presets).
std::vector<std::string> listBuiltinPresets();
