# Flying-car reference difference manifest

## Scope and method

Implementation baseline: `D:/flying_car/fully_actuated_hexav1.17-fullycontrol-1.17.0/fully_actuated_hexav1.17-fullycontrol-1.17.0`.

Read-only reference: `D:/flying_car/PX4-Autopilot/PX4-Autopilot`.

This is a feature manifest, not a wholesale tree merge. The trees contain substantial unrelated upstream/configuration drift. The dispositions below cover the flying-car feature and map accepted work to the isolated-integration design and its later tasks.

Excluded from comparison: `build/`, generated metadata, vendored submodule contents (including MAVLink and Zenoh), documentation translations, binary/mesh incidental matches, and line-ending-only differences. Unrelated PX4 version/configuration changes are rejected rather than implicitly merged.

Disposition meanings:

- `copy`: copy the resource as a review patch, then normalize it for this baseline.
- `rewrite`: retain the intent but implement it through the isolated design.
- `reject`: do not import the reference change.

## ROMFS

| Reference file(s) | Disposition | Concrete reason / destination |
| --- | --- | --- |
| `ROMFS/px4fmu_common/init.d/airframes/80003_flying_car` | copy | Copy as a review patch, then normalize to `SYS_FC_TYPE=1`, four rotors at Motor1-4, wheels at Motor5-6, `CA_R_REV=48`, and target output defaults (Task 6). |
| `ROMFS/px4fmu_common/init.d-posix/airframes/80003_flying_car` | copy | Copy only the airframe registration/default intent, then normalize it for the real six-actuator GZ model rather than the reference Iris layout (Tasks 6 and 8). |
| `ROMFS/px4fmu_common/init.d/rc.flying_car_defaults` | copy | Copy then replace reference rover/`MAV_TYPE` identity defaults with the isolated `SYS_FC_TYPE` and `FC_*` contract (Task 6). |
| `ROMFS/px4fmu_common/init.d/rc.flying_car_apps` | copy | Copy startup structure, but start the dedicated `flying_car` module without controlling generic modules from Commander (Task 6). |
| `ROMFS/px4fmu_common/init.d/rc.flying_car_sitl_defaults` | copy | Copy then normalize geometry/output defaults for six independently observable SITL actuators (Tasks 6 and 8). |
| `ROMFS/px4fmu_common/init.d/CMakeLists.txt` | rewrite | Register flying-car scripts under the dedicated module/configuration condition; do not couple them to generic `rover_differential` availability (Task 6). |
| `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` | rewrite | Add only the `80003` registration while retaining the fully-actuated `6003`/`6004` registrations omitted by the reference tree (Task 6). |
| `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` | rewrite | Register `80003` without importing unrelated reference airframe deletions/renumbering (Task 6). |
| `ROMFS/px4fmu_common/init.d/rc.vehicle_setup` | rewrite | Add one narrow `VEHICLE_TYPE=flying_car` branch; retain existing spacecraft and all other vehicle branches (Task 6). |

## Messages

| Reference file(s) | Disposition | Concrete reason / destination |
| --- | --- | --- |
| No flying-car status message exists in `msg/` | rewrite | Create `msg/FlyingCarStatus.msg` and register it. The reference uses direct Commander state/parameter changes, which cannot represent safe transition status (Task 2). |

## Modules

| Reference file(s) | Disposition | Concrete reason / destination |
| --- | --- | --- |
| `src/modules/commander/Commander.cpp` flying-car externs, switch functions, RC block | reject | The reference directly starts/stops modules and writes `MAV_TYPE`; replace only with a narrow `flying_car_status` safety/arming and stable-type integration (Task 7). |
| `src/modules/rover_differential/RoverDifferential.cpp` reads/writes `actuator_motors.control[4]` and `[5]` | reject | Those indices belong only to 80003. Preserve normal rover Motor1/2 semantics and implement dedicated wheel mixing/gating in `src/modules/flying_car/` (Tasks 3-5). |
| New `src/modules/flying_car/` | rewrite | The reference has no isolated module; implement the specified state manager, differential control, output gate, and runtime module (Tasks 2-5). |

## Board configuration

| Reference file(s) | Disposition | Concrete reason / destination |
| --- | --- | --- |
| `boards/px4/fmu-v6x/default.px4board` | rewrite | Merge only `CONFIG_MODULES_FLYING_CAR=y` and the required rover-differential selection. Retain fully-actuated baseline board choices and reject unrelated reference driver/module substitutions (Task 5). |
| `boards/px4/fmu-v6c*` | reject | The design assigns flying-car hardware to FMUv6X; do not add it to the dedicated fully-actuated/arm FMUv6C variants. |

## Simulation resources

| Reference file(s) | Disposition | Concrete reason / destination |
| --- | --- | --- |
| Reference `80003_flying_car` POSIX airframe / Iris-compatible SITL layout | reject | It only exercises the quadrotor/Iris path and cannot test Motor5/6 or wheel physics. It is not a six-actuator model. |
| Missing `Tools/simulation/gz/models/flying_car/` | rewrite | Create a dedicated model with four rotor joints, two wheel joints, sensors, friction/collisions, and six addressed actuator topics (Task 8). |
| `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim` | rewrite | Add only the new model selector after the model exists; do not alias `gz_flying_car` to Iris (Task 8). |

## Required search coverage

The following command was run exactly against the read-only reference (excluding MAVLink):

```powershell
rg -l 'Flying Car|flying_car|actuator_motors.control\[4\]' D:\flying_car\PX4-Autopilot\PX4-Autopilot\src D:\flying_car\PX4-Autopilot\PX4-Autopilot\ROMFS --glob '!**/mavlink/**'
```

Every product-relevant result is classified above: the three `rc.flying_car_*` scripts, both `80003_flying_car` airframes, both airframe CMake registries, init.d `CMakeLists.txt`, `rc.vehicle_setup`, `Commander.cpp`, and `RoverDifferential.cpp`. The search's `init.d-posix/airframes/CMakeLists.txt` result is covered by the POSIX registry row. No result is left unclassified.

## Risks retained for later tasks

- FMUv6X AUX timer-group support for four DShot600 rotors plus two 50 Hz reversible PWM wheels still needs a board build and propeller-off bench verification.
- A clean GZ build/model smoke test is required to prove six-channel routing; script registration alone is insufficient.
- Commander status staleness, transition arming denial, and generic-controller isolation require implementation tests before flight use.
