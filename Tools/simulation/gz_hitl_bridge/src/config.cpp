#include "config.hpp"

#include "preset.hpp"
#include "transport.hpp"

#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace {

// Get a value from `node` or `defaults` or use `fallback`.
template <typename T>
T get(const YAML::Node &node, const YAML::Node &defaults,
      const std::string &key, const T &fallback) {
    if (node[key]) return node[key].as<T>();
    if (defaults && defaults[key]) return defaults[key].as<T>();
    return fallback;
}

std::unique_ptr<ActuatorOutput> parseInlineOutput(const YAML::Node &n) {
    const std::string type = n["type"].as<std::string>();
    auto chans_node = n["channels"];
    if (!chans_node || !chans_node.IsSequence()) {
        throw std::runtime_error("inline output requires `channels: [..]` list");
    }
    const float min_v = n["min"] ? n["min"].as<float>() : 0.f;
    const float max_v = n["max"] ? n["max"].as<float>() : 1.f;
    const bool bidir  = n["bidirectional"] ? n["bidirectional"].as<bool>() : false;

    if (type == "motor" || type == "wheel") {
        std::vector<ChannelMap> cm;
        for (auto c : chans_node) cm.push_back({c.as<int>(), min_v, max_v, bidir});
        if (type == "motor") return std::make_unique<MotorGroup>(std::move(cm));
        return std::make_unique<WheelGroup>(std::move(cm));
    }
    if (type == "servo") {
        // For servos, channels: [{hil: 5, gz: 0, min: -0.78, max: 0.78}, ...]
        std::vector<ServoGroup::ServoChannel> sc;
        for (auto c : chans_node) {
            ServoGroup::ServoChannel s{};
            s.hil_index        = c["hil"].as<int>();
            s.gz_servo_num     = c["gz"]  ? c["gz"].as<int>()  : s.hil_index;
            s.min_rad          = c["min"] ? c["min"].as<float>() : min_v;
            s.max_rad          = c["max"] ? c["max"].as<float>() : max_v;
            s.is_bidirectional = c["bidirectional"] ? c["bidirectional"].as<bool>() : bidir;
            sc.push_back(s);
        }
        return std::make_unique<ServoGroup>(std::move(sc));
    }
    throw std::runtime_error("unknown output type: " + type);
}

} // namespace

std::vector<Vehicle::Config> loadConfigYaml(const std::string &path) {
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node defaults = root["defaults"];
    YAML::Node vehicles = root["vehicles"];
    if (!vehicles || !vehicles.IsSequence()) {
        throw std::runtime_error("config file missing `vehicles:` list");
    }

    std::vector<Vehicle::Config> out;
    int auto_sysid = 1;
    for (auto v : vehicles) {
        // `sdf_model:` and `pose:` are launcher-only fields, silently skipped here.
        (void)v["sdf_model"];
        (void)v["pose"];
        Vehicle::Config c;
        c.name        = get<std::string>(v, defaults, "name",  "drone_" + std::to_string(out.size()));
        c.world_name  = get<std::string>(v, defaults, "world", "default");
        c.model_name  = v["model"] ? v["model"].as<std::string>() : c.name;
        c.sensor_link = get<std::string>(v, defaults, "sensor_link", "base_link");

        const int sysid  = v["sysid"]  ? v["sysid"].as<int>()  : auto_sysid++;
        const int compid = v["compid"] ? v["compid"].as<int>() : 1;

        // Transport
        std::string uri = v["transport"] ? v["transport"].as<std::string>() : "";
        if (uri.empty()) throw std::runtime_error("vehicle '" + c.name + "': missing `transport`");
        // Append optional baud for serial:// without explicit ":baud"
        if (uri.rfind("serial://", 0) == 0 && uri.find(':', 9) == std::string::npos) {
            int baud = get<int>(v, defaults, "baud", 921600);
            uri += ":" + std::to_string(baud);
        }
        c.transport = Transport::create(uri, static_cast<uint8_t>(sysid),
                                              static_cast<uint8_t>(compid));
        if (!c.transport) {
            throw std::runtime_error("vehicle '" + c.name + "': bad transport: " + uri);
        }

        // Outputs: either `preset:` name, or `outputs:` inline list
        if (v["preset"]) {
            const std::string pname = v["preset"].as<std::string>();
            const FramePreset *p = findBuiltinPreset(pname);
            if (!p) throw std::runtime_error("unknown preset: " + pname);
            for (const auto &factory : p->outputs) c.outputs.push_back(factory());
            if (!v["sensor_link"]) c.sensor_link = p->sensor_link;
        } else if (v["outputs"] && v["outputs"].IsSequence()) {
            for (auto n : v["outputs"]) c.outputs.push_back(parseInlineOutput(n));
        } else {
            throw std::runtime_error("vehicle '" + c.name + "': need `preset:` or `outputs:`");
        }

        out.push_back(std::move(c));
    }
    return out;
}
