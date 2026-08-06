#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gz/transport/Node.hh>

// One channel = one HIL_ACTUATOR_CONTROLS.controls[index] slot.
// Maps to a single physical actuator in the simulator.
struct ChannelMap {
    int   index;     // 0..15 — which HIL_ACTUATOR_CONTROLS slot to read
    float min;       // value at controls[i] = 0
    float max;       // value at controls[i] = 1
    // For reversible / surfaces with range [-1, 1] (set is_bidirectional=true,
    // the bridge maps -1 → min, +1 → max).
    bool  is_bidirectional = false;
};

// Abstract base for one group of actuators that share a Gz topic / message type.
// Each group owns a set of ChannelMaps and writes them out on each apply().
class ActuatorOutput {
public:
    virtual ~ActuatorOutput() = default;

    // One-time setup: create the Gz publisher. Returns false on failure.
    // `gz_node` is the parent Vehicle's node (we don't own it).
    virtual bool init(gz::transport::Node &gz_node, const std::string &model_name) = 0;

    // Push the latest controls[16] to Gz. `armed` gates motors so they don't
    // spin when the FC isn't armed (servos/wheels typically still respond
    // because they have a defined neutral).
    virtual void apply(const float controls[16], bool armed) = 0;

    virtual const char *kind() const = 0;
    virtual int num_channels() const = 0;
};

// ---------------------------------------------------------------------------
// Motors via gz::msgs::Actuators (the multicopter motor model plugin).
// Topic: /<model>/command/motor_speed
// One Actuators message carries velocity[] for ALL motors in this group.
// ---------------------------------------------------------------------------
class MotorGroup : public ActuatorOutput {
public:
    explicit MotorGroup(std::vector<ChannelMap> channels);
    bool init(gz::transport::Node &node, const std::string &model_name) override;
    void apply(const float controls[16], bool armed) override;
    const char *kind() const override { return "motor"; }
    int num_channels() const override { return static_cast<int>(_channels.size()); }

private:
    std::vector<ChannelMap> _channels;
    gz::transport::Node::Publisher _pub;
    std::string _topic;
};

// ---------------------------------------------------------------------------
// Wheels via gz::msgs::Actuators (DiffDrive / Ackermann motor model).
// Topic: /model/<model>/command/motor_speed
// ---------------------------------------------------------------------------
class WheelGroup : public ActuatorOutput {
public:
    explicit WheelGroup(std::vector<ChannelMap> channels);
    bool init(gz::transport::Node &node, const std::string &model_name) override;
    void apply(const float controls[16], bool armed) override;
    const char *kind() const override { return "wheel"; }
    int num_channels() const override { return static_cast<int>(_channels.size()); }

private:
    std::vector<ChannelMap> _channels;
    gz::transport::Node::Publisher _pub;
    std::string _topic;
};

// ---------------------------------------------------------------------------
// Servos via per-servo gz::msgs::Double messages (JointPositionController).
// Topic per channel: /model/<model>/servo_<n>
// Each ChannelMap.index here is BOTH the HIL_ACT slot AND the servo number.
// To decouple them, pass `gz_index_offset`: gz topic n = channel_index + offset.
// ---------------------------------------------------------------------------
class ServoGroup : public ActuatorOutput {
public:
    struct ServoChannel {
        int   hil_index;       // HIL_ACTUATOR_CONTROLS slot to read
        int   gz_servo_num;    // /model/<model>/servo_<this>
        float min_rad;         // controls = 0  (or -1 if bidirectional)
        float max_rad;         // controls = 1
        bool  is_bidirectional;
    };

    explicit ServoGroup(std::vector<ServoChannel> channels);
    bool init(gz::transport::Node &node, const std::string &model_name) override;
    void apply(const float controls[16], bool armed) override;
    const char *kind() const override { return "servo"; }
    int num_channels() const override { return static_cast<int>(_channels.size()); }

private:
    std::vector<ServoChannel> _channels;
    std::vector<gz::transport::Node::Publisher> _pubs;
};
