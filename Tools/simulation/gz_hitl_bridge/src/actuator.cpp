#include "actuator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>

namespace {
// Map a raw controls[i] value to physical units.
//   For "unidirectional" channels (e.g. motors): clip to [0,1], lerp [min,max].
//   For "bidirectional" channels (e.g. surfaces, reversible motors):
//     clip to [-1,1], lerp [-1..+1] → [min..max] so 0 maps to midpoint.
float mapChannel(float c, float min, float max, bool bidir) {
    if (std::isnan(c)) c = bidir ? 0.f : 0.f;
    if (bidir) {
        if (c < -1.f) c = -1.f;
        if (c >  1.f) c =  1.f;
        return min + (c + 1.f) * 0.5f * (max - min);
    }
    if (c < 0.f) c = 0.f;
    if (c > 1.f) c = 1.f;
    return min + c * (max - min);
}
} // namespace

// ===========================================================================
// MotorGroup
// ===========================================================================
MotorGroup::MotorGroup(std::vector<ChannelMap> channels)
    : _channels(std::move(channels)) {}

bool MotorGroup::init(gz::transport::Node &node, const std::string &model_name) {
    _topic = "/" + model_name + "/command/motor_speed";
    _pub = node.Advertise<gz::msgs::Actuators>(_topic);
    if (!_pub) {
        std::fprintf(stderr, "[actuator/motor] advertise failed: %s\n", _topic.c_str());
        return false;
    }
    std::printf("[actuator/motor] %s, %zu channels\n", _topic.c_str(), _channels.size());
    return true;
}

void MotorGroup::apply(const float controls[16], bool armed) {
    gz::msgs::Actuators out;
    for (const auto &ch : _channels) {
        float vel = 0.f;
        if (armed) {
            const float c = controls[ch.index];
            vel = mapChannel(c, ch.min, ch.max, ch.is_bidirectional);
        }
        out.add_velocity(vel);
    }
    _pub.Publish(out);
}

// ===========================================================================
// WheelGroup — same payload type, different topic prefix and not gated on armed
// (wheels typically idle at 0 either way, but a rover can be commanded while
// "disarmed" in some PX4 flows; mirror motor gating for safety).
// ===========================================================================
WheelGroup::WheelGroup(std::vector<ChannelMap> channels)
    : _channels(std::move(channels)) {}

bool WheelGroup::init(gz::transport::Node &node, const std::string &model_name) {
    _topic = "/model/" + model_name + "/command/motor_speed";
    _pub = node.Advertise<gz::msgs::Actuators>(_topic);
    if (!_pub) {
        std::fprintf(stderr, "[actuator/wheel] advertise failed: %s\n", _topic.c_str());
        return false;
    }
    std::printf("[actuator/wheel] %s, %zu channels\n", _topic.c_str(), _channels.size());
    return true;
}

void WheelGroup::apply(const float controls[16], bool armed) {
    gz::msgs::Actuators out;
    for (const auto &ch : _channels) {
        float vel = 0.f;
        if (armed) {
            const float c = controls[ch.index];
            vel = mapChannel(c, ch.min, ch.max, ch.is_bidirectional);
        }
        out.add_velocity(vel);
    }
    _pub.Publish(out);
}

// ===========================================================================
// ServoGroup — one Double publisher per servo. Servos always respond, even
// when disarmed (so e.g. VTOL transition surfaces hold center pre-arm).
// ===========================================================================
ServoGroup::ServoGroup(std::vector<ServoChannel> channels)
    : _channels(std::move(channels)) {}

bool ServoGroup::init(gz::transport::Node &node, const std::string &model_name) {
    _pubs.reserve(_channels.size());
    for (const auto &ch : _channels) {
        std::string topic = "/model/" + model_name + "/servo_" +
                            std::to_string(ch.gz_servo_num);
        auto pub = node.Advertise<gz::msgs::Double>(topic);
        if (!pub) {
            std::fprintf(stderr, "[actuator/servo] advertise failed: %s\n", topic.c_str());
            return false;
        }
        std::printf("[actuator/servo] %s (HIL ch=%d, range=[%.2f,%.2f] rad%s)\n",
                    topic.c_str(), ch.hil_index, ch.min_rad, ch.max_rad,
                    ch.is_bidirectional ? ", bidir" : "");
        _pubs.push_back(pub);
    }
    return true;
}

void ServoGroup::apply(const float controls[16], bool /*armed*/) {
    for (size_t i = 0; i < _channels.size(); ++i) {
        const auto &ch = _channels[i];
        const float c = controls[ch.hil_index];
        const float angle = mapChannel(c, ch.min_rad, ch.max_rad, ch.is_bidirectional);
        gz::msgs::Double msg;
        msg.set_data(angle);
        _pubs[i].Publish(msg);
    }
}
