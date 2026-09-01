#pragma once

#include "vehicle.hpp"

#include <string>
#include <vector>

// Parse a YAML file into a list of Vehicle::Config ready to instantiate.
// Throws std::runtime_error on parse failure or unknown preset.
//
// Schema (see configs/swarm_example.yaml):
//
//   defaults:                       # optional, applied to each vehicle
//     world: default
//     baud: 921600
//
//   vehicles:
//     - name: drone_0
//       sysid: 1
//       transport: serial:///dev/ttyACM0      # or udp://host:port[@local]
//       baud: 921600                          # optional, default 921600 (serial only)
//       model: x500_0                         # gz model name (with _N suffix if needed)
//       world: default                        # optional, inherits from defaults
//       preset: x500_quad                     # built-in preset name
//
//     - name: vtol_0
//       sysid: 5
//       transport: udp://127.0.0.1:14550
//       model: standard_vtol_0
//       preset: standard_vtol
//
//     - name: custom_quad
//       sysid: 8
//       transport: serial:///dev/ttyUSB0
//       model: my_quad
//       outputs:                              # inline override (skip preset)
//         - type: motor
//           channels: [0, 1, 2, 3]
//           min: 100
//           max: 1100
std::vector<Vehicle::Config> loadConfigYaml(const std::string &path);
