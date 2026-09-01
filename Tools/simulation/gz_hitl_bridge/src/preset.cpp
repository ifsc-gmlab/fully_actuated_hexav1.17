#include "preset.hpp"

#include <cmath>
#include <unordered_map>

namespace {

// Convention for HIL_ACTUATOR_CONTROLS slots used by PX4 control allocator
// (output function values 101..116 → motors, 201..216 → servos).
// HIL_ACT_FUNC1=101 puts Motor1 onto controls[0]; we follow that 1:1 mapping
// throughout. So:
//   Motors are at controls[0..N-1]
//   Servos start at controls[N] for VTOL/fixed-wing
// This matches every default PX4 airframe we've inspected.

// Build N identical motors min=150, max=1000 rad/s starting at HIL slot 0.
std::vector<ChannelMap> nMotors(int n, float min = 150.f, float max = 1000.f) {
    std::vector<ChannelMap> v;
    v.reserve(n);
    for (int i = 0; i < n; ++i) v.push_back({i, min, max, false});
    return v;
}

constexpr float kPi = 3.14159265358979323846f;

const std::vector<FramePreset> kBuiltins = {
    // ---------- multirotor ----------
    {
        "x500_quad",
        "PX4 x500 quadrotor (default)",
        "base_link",
        { []{ return std::make_unique<MotorGroup>(nMotors(4)); } }
    },
    {
        "x500_hex",
        "Generic hexrotor (6 motors on /<model>/command/motor_speed)",
        "base_link",
        { []{ return std::make_unique<MotorGroup>(nMotors(6)); } }
    },
    {
        // Matches Tools/simulation/gz/models/fully_actuated_hexa (maxRotVelocity=1600)
        // and ROMFS airframe 6003_fully_actuated_hexa (CA_ROTOR_COUNT=6).
        "fully_actuated_hexa",
        "Fully actuated hex — 6 tilted motors, maxRotVelocity 1600 rad/s",
        "base_link",
        { []{ return std::make_unique<MotorGroup>(nMotors(6, 150.f, 1600.f)); } }
    },
    {
        "x500_octo",
        "Generic octorotor (8 motors)",
        "base_link",
        { []{ return std::make_unique<MotorGroup>(nMotors(8)); } }
    },

    // ---------- VTOL ----------
    // standard_vtol model has 4 lift rotors + 1 pusher motor + 4 control surfaces.
    // PX4 mixer: motors 1..4 = lift, motor 5 = pusher, then 4 servos.
    {
        "standard_vtol",
        "Standard VTOL — 4 lift + 1 pusher + 4 surfaces (aileron L/R, elevator, rudder)",
        "base_link",
        {
            // 5 motors at controls[0..4]
            []{ return std::make_unique<MotorGroup>(nMotors(5)); },
            // 4 servos at controls[5..8], each ±45 deg
            []{
                std::vector<ServoGroup::ServoChannel> v{
                    {5, 0,  -kPi/4, kPi/4, true},   // aileron L
                    {6, 1,  -kPi/4, kPi/4, true},   // aileron R
                    {7, 2,  -kPi/4, kPi/4, true},   // elevator
                    {8, 3,  -kPi/4, kPi/4, true},   // rudder
                };
                return std::make_unique<ServoGroup>(std::move(v));
            },
        }
    },

    // ---------- fixed-wing ----------
    {
        "rc_cessna",
        "RC Cessna — 1 throttle + 3 surfaces (aileron, elevator, rudder)",
        "base_link",
        {
            // single motor at controls[0]
            []{ return std::make_unique<MotorGroup>(nMotors(1)); },
            // 3 servos at controls[1..3]
            []{
                std::vector<ServoGroup::ServoChannel> v{
                    {1, 0,  -kPi/4, kPi/4, true},   // aileron
                    {2, 1,  -kPi/4, kPi/4, true},   // elevator
                    {3, 2,  -kPi/4, kPi/4, true},   // rudder
                };
                return std::make_unique<ServoGroup>(std::move(v));
            },
        }
    },

    // ---------- rover ----------
    {
        "r1_rover",
        "Differential-drive rover (2 wheels)",
        "base_link",
        {
            []{
                std::vector<ChannelMap> v{
                    {0, -10.f, 10.f, true},   // left wheel rad/s, reversible
                    {1, -10.f, 10.f, true},   // right wheel
                };
                return std::make_unique<WheelGroup>(std::move(v));
            },
        }
    },
    {
        "rover_ackermann",
        "Ackermann rover — 1 drive wheel + 1 steering servo",
        "base_link",
        {
            // drive on controls[0]
            []{
                std::vector<ChannelMap> v{ {0, -10.f, 10.f, true} };
                return std::make_unique<WheelGroup>(std::move(v));
            },
            // steering servo on controls[1], ±0.6 rad
            []{
                std::vector<ServoGroup::ServoChannel> v{
                    {1, 0, -0.6f, 0.6f, true},
                };
                return std::make_unique<ServoGroup>(std::move(v));
            },
        }
    },
};

}  // namespace

const FramePreset *findBuiltinPreset(const std::string &name) {
    for (const auto &p : kBuiltins) if (p.name == name) return &p;
    return nullptr;
}

std::vector<std::string> listBuiltinPresets() {
    std::vector<std::string> v;
    v.reserve(kBuiltins.size());
    for (const auto &p : kBuiltins) v.push_back(p.name + " — " + p.description);
    return v;
}
