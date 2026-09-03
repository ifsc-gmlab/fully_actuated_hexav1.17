# Flying Car Isolated Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate airframe `80003` as an isolated quadrotor-plus-differential-rover configuration into the fully-actuated PX4 v1.17 tree without changing native multicopter, native rover, airframe `6003`, or airframe `6004` behavior.

**Architecture:** Airframe `80003` sets a permanent physical-configuration flag and starts one dedicated `flying_car` module. The module owns the flight/ground state machine and wheel outputs at actuator motor indices 4 and 5; Commander receives only a compact status used to block arming during transitions and report the active vehicle type. Native multicopter, native rover, and fully-actuated control implementations remain unaware of flying-car actuator routing.

**Tech Stack:** PX4 v1.17, C++17 PX4 modules, uORB messages, PX4 parameters, NuttX FMUv6X, POSIX SITL, Gazebo GZ, GoogleTest/CMake.

**Spec:** `docs/superpowers/specs/2026-09-01-flying-car-isolated-integration-design.md`

## Global Constraints

- The fully-actuated source tree is the only implementation base; `D:/flying_car/PX4-Autopilot/PX4-Autopilot` remains read-only reference material.
- Native multicopter behavior must remain unchanged when `SYS_FC_TYPE=0`.
- Airframes `6003` and `6004` must retain their current controllers, geometry, parameters, and Motor1–6 semantics.
- Native rover must retain Motor1/Motor2 semantics; flying-car wheel indices must not be added to `rover_differential`.
- Only airframe `80003` uses Motor1–4 for rotors and Motor5–6 for wheels.
- The first release permits transitions only while disarmed, landed, and below the configured horizontal-speed threshold.
- Every completed task updates `docs/integration/flying-car-integration-log.md` with files, commands, results, and risks.
- The source directory has no Git metadata at plan creation time; initialize a local repository before product edits so every task can end in a recoverable commit.

---

### Task 1: Create a Recoverable Baseline and Difference Manifest

**Files:**
- Create: `.git/` through `git init`
- Create: `docs/integration/flying-car-reference-manifest.md`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: the fully-actuated source tree and the read-only flying-car reference tree.
- Produces: a baseline commit and a functional manifest that maps every accepted reference change to a later task.

- [ ] **Step 1: Record both tree roots and exclude generated content**

The manifest must classify differences under `ROMFS`, `msg`, `src/modules`, `boards/px4`, and simulation resources. It must explicitly exclude `build/`, generated metadata, vendored submodule contents, documentation translations, and line-ending-only differences.

- [ ] **Step 2: Initialize Git and commit the untouched implementation baseline**

Run:

```sh
git init
git add -A
git commit -m "chore: snapshot fully actuated PX4 v1.17 baseline"
```

Expected: `git status --short` prints no product-code changes after the commit. If Git identity is missing, configure repository-local `user.name` and `user.email`, not global values.

- [ ] **Step 3: Write the functional difference manifest**

For each reference file, record one disposition: `copy`, `rewrite`, or `reject`. At minimum include:

```text
80003 airframes and rc scripts        copy then normalize
Commander flying-car block            reject; replace with narrow status integration
RoverDifferential indices 4/5         reject; implement in dedicated module
FMUv6X module selection                merge
SITL model and target                  reject Iris alias; create a six-actuator model
```

- [ ] **Step 4: Verify the manifest has no unclassified flying-car product files**

Run:

```sh
rg -l "Flying Car|flying_car|actuator_motors.control\[4\]" D:/flying_car/PX4-Autopilot/PX4-Autopilot/src D:/flying_car/PX4-Autopilot/PX4-Autopilot/ROMFS --glob '!**/mavlink/**'
```

Expected: every relevant result is present in the manifest.

- [ ] **Step 5: Update the integration log and commit**

```sh
git add docs/integration
git commit -m "docs: map flying car reference changes"
```

---

### Task 2: Add Configuration Identity, Parameters, and Status Message

**Files:**
- Create: `msg/FlyingCarStatus.msg`
- Modify: `msg/CMakeLists.txt`
- Create: `src/modules/flying_car/module.yaml`
- Create: `src/modules/flying_car/flying_car_params.c`
- Create: `src/modules/flying_car/FlyingCarTypes.hpp`
- Create: `src/modules/flying_car/FlyingCarModeManagerTest.cpp`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: `arming_state`, `vehicle_land_detected.landed`, horizontal speed, and a requested mode.
- Produces: parameter `SYS_FC_TYPE`, `FC_MODE_CH`, `FC_BOOT_MODE`, `FC_SW_VEL_MAX`, `FC_SW_DELAY`, `FC_WHEEL_TRACK`, `FC_WHEEL_SPD_MAX`, `FC_WHEEL_THR_MAX`, and `FC_WHEEL_REV`; uORB topic `flying_car_status`.

- [ ] **Step 1: Define the status message**

Create `FlyingCarStatus.msg` with stable numeric constants:

```text
uint64 timestamp

uint8 MODE_FLIGHT = 0
uint8 MODE_TRANSITION_TO_GROUND = 1
uint8 MODE_GROUND = 2
uint8 MODE_TRANSITION_TO_FLIGHT = 3
uint8 MODE_FAULT = 4

uint8 REJECT_NONE = 0
uint8 REJECT_NOT_FLYING_CAR = 1
uint8 REJECT_ARMED = 2
uint8 REJECT_NOT_LANDED = 3
uint8 REJECT_MOVING = 4
uint8 REJECT_CHAIN_UNHEALTHY = 5
uint8 REJECT_TIMEOUT = 6

uint8 mode
uint8 requested_mode
uint8 rejection_reason
bool transition_allowed
bool flight_chain_ready
bool ground_chain_ready
```

- [ ] **Step 2: Register the message and verify generation fails before registration**

Before editing `msg/CMakeLists.txt`, run the SITL configure/build and confirm code including `uORB/topics/flying_car_status.h` cannot compile. Then add `FlyingCarStatus.msg` to `msg_files`.

- [ ] **Step 3: Define configuration and state types**

Create `FlyingCarTypes.hpp`:

```cpp
enum class FlyingCarMode : uint8_t {
	Flight = 0,
	TransitionToGround = 1,
	Ground = 2,
	TransitionToFlight = 3,
	Fault = 4,
};

enum class FlyingCarRejection : uint8_t {
	None = 0,
	NotFlyingCar = 1,
	Armed = 2,
	NotLanded = 3,
	Moving = 4,
	ChainUnhealthy = 5,
	Timeout = 6,
};
```

- [ ] **Step 4: Add parameter metadata with safe defaults**

Use defaults: `SYS_FC_TYPE=0`, `FC_MODE_CH=0`, `FC_BOOT_MODE=0`, `FC_SW_VEL_MAX=0.2 m/s`, `FC_SW_DELAY=0.5 s`, `FC_WHEEL_TRACK=0.5 m`, `FC_WHEEL_SPD_MAX=2.0 m/s`, `FC_WHEEL_THR_MAX=0.5`, and `FC_WHEEL_REV=0`.

- [ ] **Step 5: Add compile-time consistency tests**

Write GoogleTest assertions that each C++ enum value equals the generated uORB constant. This catches later message/type drift.

- [ ] **Step 6: Build message and parameter metadata**

Run:

```sh
make px4_sitl_default
make parameters_metadata
```

Expected: generated `flying_car_status.h` exists and all new parameters appear in metadata.

- [ ] **Step 7: Update the integration log and commit**

```sh
git add msg src/modules/flying_car docs/integration/flying-car-integration-log.md
git commit -m "feat: define isolated flying car configuration"
```

---

### Task 3: Implement and Unit-Test the Pure Mode State Machine

**Files:**
- Create: `src/modules/flying_car/FlyingCarModeManager.hpp`
- Create: `src/modules/flying_car/FlyingCarModeManager.cpp`
- Modify: `src/modules/flying_car/FlyingCarModeManagerTest.cpp`
- Create: `src/modules/flying_car/CMakeLists.txt`
- Create: `src/modules/flying_car/Kconfig`
- Modify: `src/modules/Kconfig`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: `FlyingCarTransitionInput` and elapsed time.
- Produces: deterministic `FlyingCarTransitionResult` with mode, output ownership, arming lock, and rejection reason.

- [ ] **Step 1: Write the state-machine interface**

```cpp
struct FlyingCarTransitionInput {
	bool configuration_enabled;
	bool armed;
	bool landed;
	bool flight_chain_ready;
	bool ground_chain_ready;
	float horizontal_speed_m_s;
	FlyingCarMode requested_mode;
	uint64_t now_us;
};

struct FlyingCarTransitionResult {
	FlyingCarMode mode;
	FlyingCarRejection rejection;
	bool transition_allowed;
	bool arming_locked;
	bool flight_output_enabled;
	bool ground_output_enabled;
};
```

- [ ] **Step 2: Write failing tests for every safety guard**

Tests must cover disabled configuration, armed, not landed, excessive speed, unhealthy target chain, request debounce, transition delay, successful Flight-to-Ground, successful Ground-to-Flight, timeout-to-Fault, and Fault output ownership.

- [ ] **Step 3: Run the focused test and verify failure**

Run:

```sh
make px4_sitl_default tests TESTFILTER=FlyingCarModeManagerTest
```

Expected: failure because the state-machine implementation is absent.

- [ ] **Step 4: Implement the minimal deterministic state machine**

The implementation must contain no uORB, parameters, module start/stop calls, or Commander dependencies. `Flight` owns only flight output, `Ground` owns only ground output, and transition/fault states own neither.

- [ ] **Step 5: Run focused and full unit tests**

```sh
make px4_sitl_default tests TESTFILTER=FlyingCarModeManagerTest
make tests
```

Expected: focused test and existing PX4 unit tests pass.

- [ ] **Step 6: Update the integration log and commit**

```sh
git add src/modules/flying_car src/modules/Kconfig docs/integration/flying-car-integration-log.md
git commit -m "feat: add flying car safety state machine"
```

---

### Task 4: Implement Dedicated Differential Wheel Control and Output Gating

**Files:**
- Create: `src/modules/flying_car/FlyingCarDifferentialControl.hpp`
- Create: `src/modules/flying_car/FlyingCarDifferentialControl.cpp`
- Create: `src/modules/flying_car/FlyingCarDifferentialControlTest.cpp`
- Create: `src/modules/flying_car/FlyingCarActuatorGate.hpp`
- Create: `src/modules/flying_car/FlyingCarActuatorGate.cpp`
- Create: `src/modules/flying_car/FlyingCarActuatorGateTest.cpp`
- Modify: `src/modules/flying_car/CMakeLists.txt`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: normalized forward throttle, normalized yaw command, `FC_WHEEL_THR_MAX`, and current `FlyingCarMode`.
- Produces: wheel values at actuator indices 4 and 5 and an ownership decision for rotor indices 0–3 versus wheel indices 4–5.

- [ ] **Step 1: Write failing differential-control tests**

Use the explicit mixer contract:

```cpp
left = math::constrain(throttle - yaw, -limit, limit);
right = math::constrain(throttle + yaw, -limit, limit);
```

Test straight motion, left/right turn, pivot, saturation, direction reversal, non-finite input, and `limit=0`.

- [ ] **Step 2: Implement the pure differential mixer**

Expose:

```cpp
matrix::Vector2f mix(float throttle, float yaw, float limit, uint8_t reversal_mask);
```

Bit 0 reverses the left wheel and bit 1 reverses the right wheel. Invalid input returns `{0.f, 0.f}`.

- [ ] **Step 3: Write failing output-gate tests**

Verify:

```text
Flight: rotor[0..3] pass through; wheel[4..5] are zero
Ground: rotor[0..3] are NaN/disabled; wheel[4..5] pass through
Transition/Fault: rotor[0..3] disabled; wheel[4..5] zero
SYS_FC_TYPE=0: gate reports bypass and does not republish data
```

- [ ] **Step 4: Implement the gate without changing generic RoverDifferential**

Expose a pure transform that fills one `actuator_motors_s` sample and sets `reversible_flags` bits 4 and 5 (`48`) only for `80003` wheel operation.

- [ ] **Step 5: Run tests and inspect the generic rover tree**

```sh
make px4_sitl_default tests TESTFILTER=FlyingCar
git diff -- src/modules/rover_differential
```

Expected: all flying-car tests pass and the generic rover directory has no diff.

- [ ] **Step 6: Update the integration log and commit**

```sh
git add src/modules/flying_car docs/integration/flying-car-integration-log.md
git commit -m "feat: isolate flying car wheel actuation"
```

---

### Task 5: Build the Runtime Flying-Car Module

**Files:**
- Create: `msg/FlyingCarActuatorMotors.msg`
- Modify: `msg/CMakeLists.txt`
- Create: `src/modules/flying_car/FlyingCar.hpp`
- Create: `src/modules/flying_car/FlyingCar.cpp`
- Modify: `src/modules/flying_car/CMakeLists.txt`
- Modify: `src/modules/flying_car/Kconfig`
- Modify: `src/lib/mixer_module/functions/FunctionMotors.hpp`
- Modify: `src/lib/mixer_module/mixer_module_tests.cpp`
- Modify: `boards/px4/fmu-v6x/default.px4board`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: `vehicle_status`, `vehicle_land_detected`, `vehicle_local_position`, `manual_control_setpoint`, `rover_throttle_setpoint`, `rover_steering_setpoint`, and parameters.
- Produces: `flying_car_status` and `flying_car_actuator_motors`; Motor output providers lock to the dedicated gated topic after its first sample, while configurations that never publish it retain original `actuator_motors` behavior.

- [ ] **Step 1: Add a module-start isolation test**

Start with `SYS_FC_TYPE=0` and assert the command returns a clear refusal without advertising `flying_car_status`. Set `SYS_FC_TYPE=1`, start again, and assert one status publisher appears in `Flight` mode.

- [ ] **Step 2: Implement module startup and subscriptions**

The start path must check `SYS_FC_TYPE` before scheduling work. It must not start/stop Commander, control allocator, flight mode manager, land detector, or generic rover modules. It must publish a safe gated output before reporting ready.

- [ ] **Step 3: Translate RC input into explicit requests**

Use `FC_MODE_CH` to select an RC auxiliary function and apply hysteresis: values below `-0.5` request Flight, values above `0.5` request Ground, values in between preserve the last request. Ignore RC requests when the message is invalid or not sourced from RC.

- [ ] **Step 4: Publish status and gate actuator ownership**

Publish status on every state change and at 2 Hz while stable. Publish gated actuator output at 100 Hz and on new allocator data. During transition or fault, publish no active rotor values and neutral wheel values.

- [ ] **Step 5: Add FMUv6X build selection**

Add:

```text
CONFIG_MODULES_FLYING_CAR=y
CONFIG_MODULES_ROVER_DIFFERENTIAL=y
```

to `boards/px4/fmu-v6x/default.px4board`. Do not add flying-car configuration to the dedicated FMUv6C arm variant.

- [ ] **Step 6: Build SITL and FMUv6X**

```sh
make px4_sitl_default
make px4_fmu-v6x_default
```

Expected: both builds complete and the FMUv6X firmware includes the `flying_car` command.

- [ ] **Step 7: Update the integration log and commit**

```sh
git add src/modules/flying_car boards/px4/fmu-v6x/default.px4board docs/integration/flying-car-integration-log.md
git commit -m "feat: add isolated flying car runtime"
```

---

### Task 6: Add Airframe 80003 and Startup Isolation

**Files:**
- Create: `ROMFS/px4fmu_common/init.d/airframes/80003_flying_car`
- Create: `ROMFS/px4fmu_common/init.d-posix/airframes/80003_flying_car`
- Create: `ROMFS/px4fmu_common/init.d/rc.flying_car_defaults`
- Create: `ROMFS/px4fmu_common/init.d/rc.flying_car_apps`
- Create: `ROMFS/px4fmu_common/init.d/rc.flying_car_sitl_defaults`
- Modify: `ROMFS/px4fmu_common/init.d/CMakeLists.txt`
- Modify: `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt`
- Modify: `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`
- Modify: `ROMFS/px4fmu_common/init.d/rc.vehicle_setup`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: `SYS_AUTOSTART=80003`.
- Produces: `SYS_FC_TYPE=1`, four-rotor geometry, Motor1–6 output mapping, MC startup plus `flying_car start`; all other airframes preserve existing startup branches.

- [ ] **Step 1: Copy the reference scripts into a review patch**

Copy content manually through `apply_patch`, preserving the fully-actuated tree's formatting and headers. Do not copy the reference `rc.vehicle_setup` wholesale.

- [ ] **Step 2: Normalize the true-hardware airframe defaults**

Set `CA_AIRFRAME=0`, `CA_ROTOR_COUNT=4`, rotor geometry, `SYS_FC_TYPE=1`, Motor1–4 mappings `101..104`, Motor5–6 mappings `105..106`, wheel neutral PWM `1500`, wheel range `1100..1900`, and `CA_R_REV=48`.

- [ ] **Step 3: Add one isolated startup branch**

In `rc.vehicle_setup`, add a `VEHICLE_TYPE=flying_car` branch that sources `rc.flying_car_apps`. It must not alter the `mc`, `rover`, `vtol`, or fully-actuated branches.

- [ ] **Step 4: Verify generated airframe metadata**

```sh
make px4_sitl_default metadata_airframes
rg -n '80003|Flying Car' build/px4_sitl_default/airframes.xml
```

Expected: exactly one hardware and one POSIX registration resolve to airframe 80003 without duplicate metadata errors.

- [ ] **Step 5: Run startup isolation smoke tests**

Boot a native quad airframe and verify `flying_car status` reports not running. Boot `80003` and verify `SYS_FC_TYPE=1`, `flying_car status` is running, and MC modules remain available.

- [ ] **Step 6: Update the integration log and commit**

```sh
git add ROMFS docs/integration/flying-car-integration-log.md
git commit -m "feat: register isolated flying car airframe"
```

---

### Task 7: Add Narrow Commander Safety Integration

**Files:**
- Modify: `src/modules/commander/Commander.hpp`
- Modify: `src/modules/commander/Commander.cpp`
- Modify: `src/modules/commander/HealthAndArmingChecks/checks/systemCheck.cpp`
- Modify: `src/modules/commander/HealthAndArmingChecks/HealthAndArmingChecksTest.cpp`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: `flying_car_status` only when `SYS_FC_TYPE=1`.
- Produces: arming denial during transitions, fault, or stale status; externally reported Quadrotor/Rover system type based on stable flying-car mode.

- [ ] **Step 1: Write failing arming-check tests**

Test that native configurations ignore missing flying-car status, while `SYS_FC_TYPE=1` rejects arming for transition, fault, or status older than one second and permits evaluation to continue for stable Flight/Ground status.

- [ ] **Step 2: Add a single flying-car status subscription**

Do not include `flying_car` module headers or call module main functions. Commander uses only the generated uORB message and `SYS_FC_TYPE` parameter.

- [ ] **Step 3: Implement vehicle-type reporting**

Stable Flight maps to rotary wing and `MAV_TYPE_QUADROTOR`; stable Ground maps to rover and `MAV_TYPE_GROUND_ROVER`. Transition and Fault retain the last stable type but deny arming.

- [ ] **Step 4: Confirm removal of the rejected reference pattern**

Run:

```sh
rg -n 'flying_car_switch_to_|rover_differential_main|control_allocator_main' src/modules/commander
```

Expected: no flying-car code directly starts or stops those modules.

- [ ] **Step 5: Run Commander and SITL tests**

```sh
make px4_sitl_default tests TESTFILTER=Commander
make px4_sitl_default
```

Expected: tests and SITL build pass.

- [ ] **Step 6: Update the integration log and commit**

```sh
git add src/modules/commander docs/integration/flying-car-integration-log.md
git commit -m "feat: enforce flying car arming isolation"
```

---

### Task 8: Integrate and Validate the Gazebo Model

**Files:**
- Create: `Tools/simulation/gz/models/flying_car/model.config`
- Create: `Tools/simulation/gz/models/flying_car/model.sdf`
- Create: `Tools/simulation/gz/models/flying_car/model_test.py`
- Modify: `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: Motor1–6 outputs for `80003`.
- Produces: four rotor joints and two wheel joints with observable, separately addressed commands.

- [ ] **Step 1: Record why the reference SITL model is rejected**

The reference tree has no dedicated `Tools/simulation/gz/models/flying_car` asset and its POSIX airframe uses an Iris-compatible rotor layout. Record this as insufficient because it cannot exercise Motor5/6 or wheel physics.

- [ ] **Step 2: Add a model-structure test**

Create `model_test.py` using Python `xml.etree.ElementTree`. It must assert that `model.sdf` contains four rotor joints, `left_wheel_joint`, `right_wheel_joint`, two wheel collision elements with friction, IMU and navsat sensors, four rotor actuator topics, and two wheel actuator topics.

- [ ] **Step 3: Create and register the model**

Build the model from existing PX4 GZ conventions. Map Motor1–4 to rotor speed and configure the existing `GZMixingInterfaceWheel` path for Motor5–6 as left/right wheel velocity. Add only the model selection needed in `px4-rc.gzsim`; the baseline already contains `GZMixingInterfaceWheel`, so do not modify the bridge unless the model smoke test demonstrates a concrete interface mismatch.

- [ ] **Step 4: Build and run the SITL smoke test**

```sh
make px4_sitl gz_flying_car
```

Expected: PX4 boots with `SYS_AUTOSTART=80003`, all required sensors are healthy, and six actuator channels are visible.

- [ ] **Step 5: Exercise both stable modes**

While disarmed, switch Flight to Ground and back. Confirm four rotor channels are disabled in Ground, two wheel channels are neutral in Flight, and no transition succeeds while armed.

- [ ] **Step 6: Update the integration log and commit**

```sh
git add ROMFS src/modules/simulation docs/integration/flying-car-integration-log.md
git commit -m "feat: simulate isolated flying car actuators"
```

---

### Task 9: Full Regression and Hardware-Build Gate

**Files:**
- Modify: `docs/integration/flying-car-integration-log.md`
- Create: `docs/integration/flying-car-validation-report.md`

**Interfaces:**
- Consumes: all preceding implementation tasks.
- Produces: reproducible evidence that the four supported configuration families build and retain isolated behavior.

- [ ] **Step 1: Run targeted unit tests**

```sh
make px4_sitl_default tests TESTFILTER=FlyingCar
make px4_sitl_default tests TESTFILTER=FullyActuated
make px4_sitl_default tests TESTFILTER=Rover
```

Expected: all targeted suites pass.

- [ ] **Step 2: Run the full SITL unit-test suite**

```sh
make tests
```

Expected: no regression relative to the baseline. Record any baseline-existing failure separately with its exact output.

- [ ] **Step 3: Build supported firmware targets**

```sh
make px4_sitl_default
make px4_fmu-v6c_default
make px4_fmu-v6c_arm
make px4_fmu-v6x_default
```

Expected: all four targets build. Record firmware sizes and paths.

- [ ] **Step 4: Run configuration-isolation checks**

For native quad, `6003`, `6004`, and `80003`, record `SYS_AUTOSTART`, `SYS_FC_TYPE`, running control modules, actuator-function mapping, and the response to a flying-car switch request.

- [ ] **Step 5: Write the validation report**

The report must contain exact commands, timestamps, pass/fail results, firmware hashes, unresolved hardware-only checks, and a clear statement that build/SITL success is not equivalent to a completed real-flight test.

- [ ] **Step 6: Verify no unintended reference-tree or generic-controller changes**

```sh
git status --short
git diff HEAD~8 -- src/modules/mc_pos_control src/modules/mc_att_control src/modules/mc_rate_control src/modules/rover_differential src/modules/control_allocator
```

Expected: no flying-car-specific edits in the listed generic controllers except separately justified Commander integration outside this command.

- [ ] **Step 7: Commit the validation evidence**

```sh
git add docs/integration
git commit -m "test: document flying car integration validation"
```

---

### Task 10: Prepare the Real-Hardware Bench Procedure

**Files:**
- Create: `docs/integration/flying-car-pixhawk6x-bench-test.md`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: the validated `px4_fmu-v6x_default.px4` firmware.
- Produces: a propeller-off, wheel-off-ground procedure with explicit abort criteria and ULog evidence requirements.

- [ ] **Step 1: Document wiring and power separation**

Specify AUX1–4 DShot rotor ESC connections, AUX5–6 reversible wheel ESC connections, common ground, external motor power, emergency stop, and removal of propellers.

- [ ] **Step 2: Document output verification**

Verify one channel at a time with actuator test: Motor1–4 must address only rotors; Motor5–6 must address only wheels; wheel neutral must be 1500 microseconds; no timer group may silently inherit the wrong protocol.

- [ ] **Step 3: Document transition and failure cases**

Include armed-switch rejection, RC loss, Commander restart, flying-car module stop, stale status, reboot in each stable mode, and restoration to a safe boot state.

- [ ] **Step 4: Define pass criteria**

No rotor motion in Ground, no wheel motion in Flight, no active output during transition/fault, no armed transition, no unexpected reboot, and a saved ULog plus parameter snapshot for every run.

- [ ] **Step 5: Update the integration log and commit**

```sh
git add docs/integration
git commit -m "docs: add Pixhawk 6X flying car bench procedure"
```

---

### Final review remediation: Provide isolated 80003 Ground input

- Add a pure, testable source selector: complete fresh Rover pair first; otherwise valid fresh RC manual input only while Ground is requested.
- Map the documented PX4 manual fields `throttle` and `roll`, without starting the full Rover chain.
- Feed readiness, wheel mixing and output sample time from the single selected source; never mix axes from different sources.
- Validate absent publishers, source loss, stale/future/NaN data, precedence, Flight isolation and non-80003 bypass in the tracked suite.
- Update the bench procedure to observe the actual selected input path and RC-loss neutralization.
