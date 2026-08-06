#include "config.hpp"
#include "preset.hpp"
#include "transport.hpp"
#include "vehicle.hpp"
#include "vehicle_manager.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_run{true};
void on_signal(int) { g_run = false; }

void print_usage(const char *prog) {
    std::printf(
        "Usage:\n"
        "  Single vehicle (CLI):\n"
        "    %s --device URI [--preset NAME] [--model M] [--world W] [--sysid N]\n"
        "    %s --device URI [--motors N] [--motor-min] [--motor-max]\n"
        "  Multi vehicle (swarm):\n"
        "    %s --config swarm.yaml\n"
        "\n"
        "Options:\n"
        "  --config PATH       YAML config file (multi-vehicle mode)\n"
        "  --device URI        single-vehicle transport (see formats below)\n"
        "  --baud RATE         serial baudrate (default 921600, ignored for udp)\n"
        "  --preset NAME       built-in frame preset (--list-presets to see all)\n"
        "  --world NAME        gz world (default \"default\")\n"
        "  --model NAME        gz model name (default \"x500\")\n"
        "  --sysid N           MAVLink system id   (default 1)\n"
        "  --compid N          MAVLink component id(default 1)\n"
        "  --motors N          quick preset: N-motor quad/hex/octo (overrides --preset)\n"
        "  --motor-min RAD/S   motor min vel       (default 150)\n"
        "  --motor-max RAD/S   motor max vel       (default 1000)\n"
        "  --list-presets      print built-in presets and exit\n"
        "  -h, --help          this message\n"
        "\n"
        "Transport URIs:\n"
        "  serial:///dev/ttyACM0:921600     serial / USB CDC ACM\n"
        "  /dev/ttyACM0                     same (shorthand)\n"
        "  udp://127.0.0.1:14550            udp endpoint (send + recv)\n"
        "  udp://0.0.0.0:14550              udp server (learns peer on first packet)\n"
        "  udp://127.0.0.1:14550@14600      udp, bind local port :14600 explicitly\n",
        prog, prog, prog);
}

void print_presets() {
    std::printf("Built-in frame presets:\n");
    for (const auto &p : listBuiltinPresets()) std::printf("  %s\n", p.c_str());
}
} // namespace

int main(int argc, char **argv) {
    // Multi-vehicle path
    std::string config_path;

    // Single-vehicle path
    std::string device;
    int baud = 921600;
    int sysid = 1, compid = 1;
    std::string preset_name;
    std::string world = "default", model = "x500";
    int    motors_n = 0;
    float  motor_min = 150.f, motor_max = 1000.f;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char *name) -> const char * {
            if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", name); std::exit(1); }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") { print_usage(argv[0]); return 0; }
        else if (a == "--list-presets") { print_presets(); return 0; }
        else if (a == "--config")    config_path  = need("--config");
        else if (a == "--device")    device       = need("--device");
        else if (a == "--baud")      baud         = std::atoi(need("--baud"));
        else if (a == "--preset")    preset_name  = need("--preset");
        else if (a == "--world")     world        = need("--world");
        else if (a == "--model")     model        = need("--model");
        else if (a == "--sysid")     sysid        = std::atoi(need("--sysid"));
        else if (a == "--compid")    compid       = std::atoi(need("--compid"));
        else if (a == "--motors")    motors_n     = std::atoi(need("--motors"));
        else if (a == "--motor-min") motor_min    = std::atof(need("--motor-min"));
        else if (a == "--motor-max") motor_max    = std::atof(need("--motor-max"));
        else { std::fprintf(stderr, "unknown arg: %s\n", a.c_str()); print_usage(argv[0]); return 1; }
    }

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    VehicleManager mgr;

    if (!config_path.empty()) {
        // ---- Multi-vehicle mode ----
        try {
            auto configs = loadConfigYaml(config_path);
            for (auto &c : configs) mgr.add(std::move(c));
            std::printf("[main] loaded %zu vehicles from %s\n", mgr.size(), config_path.c_str());
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Config error: %s\n", e.what());
            return 2;
        }
    } else {
        // ---- Single-vehicle CLI mode ----
        if (device.empty()) {
            std::fprintf(stderr, "Need either --device or --config\n");
            print_usage(argv[0]);
            return 1;
        }

        // If --device is a bare path, prepend serial:// and attach baud
        std::string uri = device;
        if (uri.rfind("serial://", 0) != 0 && uri.rfind("udp://", 0) != 0) {
            uri = "serial://" + uri + ":" + std::to_string(baud);
        } else if (uri.rfind("serial://", 0) == 0 &&
                   uri.find(':', std::string("serial://").size()) == std::string::npos) {
            uri += ":" + std::to_string(baud);
        }

        Vehicle::Config c;
        c.name        = "drone";
        c.world_name  = world;
        c.model_name  = model;
        c.transport   = Transport::create(uri, static_cast<uint8_t>(sysid),
                                                static_cast<uint8_t>(compid));
        if (!c.transport) {
            std::fprintf(stderr, "Bad transport: %s\n", uri.c_str());
            return 2;
        }

        // Output selection precedence: --motors > --preset > default x500_quad
        if (motors_n > 0) {
            std::vector<ChannelMap> ch;
            ch.reserve(motors_n);
            for (int i = 0; i < motors_n; ++i) ch.push_back({i, motor_min, motor_max, false});
            c.outputs.push_back(std::make_unique<MotorGroup>(std::move(ch)));
        } else {
            if (preset_name.empty()) preset_name = "x500_quad";
            const FramePreset *p = findBuiltinPreset(preset_name);
            if (!p) {
                std::fprintf(stderr, "Unknown preset '%s'. --list-presets to see all.\n",
                             preset_name.c_str());
                return 2;
            }
            for (const auto &factory : p->outputs) c.outputs.push_back(factory());
            c.sensor_link = p->sensor_link;
        }

        mgr.add(std::move(c));
    }

    if (!mgr.initAll()) return 3;

    std::printf("[main] %zu vehicle(s) running. Ctrl-C to stop.\n", mgr.size());
    auto t_last = std::chrono::steady_clock::now();
    while (g_run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto now = std::chrono::steady_clock::now();
        if (now - t_last >= std::chrono::seconds(1)) {
            t_last = now;
            mgr.printStatus();
        }
    }

    std::printf("[main] shutting down\n");
    mgr.shutdownAll();
    return 0;
}
