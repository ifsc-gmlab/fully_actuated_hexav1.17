#include "vehicle.hpp"

#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

extern "C" {
#include <mavlink.h>
}

namespace {
constexpr uint32_t HIL_SENSOR_FIELDS =
    (1u<<0) | (1u<<1) | (1u<<2) |   // accel xyz
    (1u<<3) | (1u<<4) | (1u<<5) |   // gyro xyz
    (1u<<6) | (1u<<7) | (1u<<8) |   // mag xyz
    (1u<<9) | (1u<<11) | (1u<<12);  // abs_p, press_alt, temp

struct SensorBit {
    uint32_t bit;
    const char *name;
};

constexpr SensorBit kSensorBits[] = {
    {MAV_SYS_STATUS_SENSOR_3D_GYRO, "gyro"},
    {MAV_SYS_STATUS_SENSOR_3D_ACCEL, "accel"},
    {MAV_SYS_STATUS_SENSOR_3D_MAG, "mag"},
    {MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, "baro"},
    {MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, "diff_pressure"},
    {MAV_SYS_STATUS_SENSOR_GPS, "gps"},
    {MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, "optical_flow"},
    {MAV_SYS_STATUS_SENSOR_VISION_POSITION, "vision"},
    {MAV_SYS_STATUS_SENSOR_LASER_POSITION, "laser"},
    {MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH, "ext_truth"},
    {MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL, "rate_ctrl"},
    {MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION, "att"},
    {MAV_SYS_STATUS_SENSOR_YAW_POSITION, "yaw"},
    {MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL, "alt"},
    {MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL, "pos_xy"},
    {MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, "motors"},
    {MAV_SYS_STATUS_SENSOR_RC_RECEIVER, "rc"},
    {MAV_SYS_STATUS_SENSOR_3D_GYRO2, "gyro2"},
    {MAV_SYS_STATUS_SENSOR_3D_ACCEL2, "accel2"},
    {MAV_SYS_STATUS_SENSOR_3D_MAG2, "mag2"},
    {MAV_SYS_STATUS_GEOFENCE, "geofence"},
    {MAV_SYS_STATUS_AHRS, "ahrs"},
    {MAV_SYS_STATUS_TERRAIN, "terrain"},
    {MAV_SYS_STATUS_REVERSE_MOTOR, "rev_motor"},
    {MAV_SYS_STATUS_LOGGING, "logging"},
    {MAV_SYS_STATUS_SENSOR_BATTERY, "battery"},
    {MAV_SYS_STATUS_SENSOR_PROXIMITY, "proximity"},
    {MAV_SYS_STATUS_SENSOR_SATCOM, "satcom"},
    {MAV_SYS_STATUS_PREARM_CHECK, "prearm"},
    {MAV_SYS_STATUS_OBSTACLE_AVOIDANCE, "obstacle"},
    {MAV_SYS_STATUS_SENSOR_PROPULSION, "propulsion"},
};

bool looksLikeArmingOrPreflight(const std::string &text) {
    auto lower = text;
    for (char &c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return lower.find("arm") != std::string::npos
        || lower.find("preflight") != std::string::npos
        || lower.find("pre-arm") != std::string::npos
        || lower.find("prearm") != std::string::npos
        || lower.find("fail") != std::string::npos
        || lower.find("denied") != std::string::npos
        || lower.find("reject") != std::string::npos
        || lower.find("check") != std::string::npos
        || lower.find("calib") != std::string::npos
        || lower.find("safety") != std::string::npos
        || lower.find("ekf") != std::string::npos
        || lower.find("gps") != std::string::npos
        || lower.find("mag") != std::string::npos
        || lower.find("battery") != std::string::npos
        || lower.find("esc") != std::string::npos
        || lower.find("motor") != std::string::npos
        || lower.find("rc ") != std::string::npos
        || lower.find("home") != std::string::npos;
}
}

Vehicle::Vehicle(Config cfg) : _cfg(std::move(cfg)) {}
Vehicle::~Vehicle() { shutdown(); }

uint64_t Vehicle::now_usec() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

bool Vehicle::init() {
    if (!_cfg.transport) {
        std::fprintf(stderr, "[%s] no transport configured\n", _cfg.name.c_str());
        return false;
    }
    if (!_cfg.transport->open()) return false;

    const std::string base = "/world/" + _cfg.world_name + "/model/" + _cfg.model_name
                             + "/link/" + _cfg.sensor_link + "/sensor/";

    if (!_node.Subscribe(base + "imu_sensor/imu",  &Vehicle::imuCallback,  this)) {
        std::fprintf(stderr, "[%s] imu subscribe failed\n", _cfg.name.c_str());
        return false;
    }
    if (!_node.Subscribe(base + "magnetometer_sensor/magnetometer",
                         &Vehicle::magCallback, this)) {
        std::fprintf(stderr, "[%s] mag subscribe failed\n", _cfg.name.c_str());
        return false;
    }
    if (!_node.Subscribe(base + "air_pressure_sensor/air_pressure",
                         &Vehicle::baroCallback, this)) {
        std::fprintf(stderr, "[%s] baro subscribe failed\n", _cfg.name.c_str());
        return false;
    }
    if (!_node.Subscribe(base + "navsat_sensor/navsat",
                         &Vehicle::navSatCallback, this)) {
        std::fprintf(stderr, "[%s] navsat subscribe failed\n", _cfg.name.c_str());
        return false;
    }

    for (auto &out : _cfg.outputs) {
        if (!out->init(_node, _cfg.model_name)) {
            std::fprintf(stderr, "[%s] actuator init failed\n", _cfg.name.c_str());
            return false;
        }
    }

    _cfg.transport->start([this](const mavlink_message_t &m){ onMavlinkMessage(m); });

    std::printf("[%s] up: model=%s, transport=%s, outputs:",
                _cfg.name.c_str(), _cfg.model_name.c_str(),
                _cfg.transport->description());
    for (const auto &o : _cfg.outputs) {
        std::printf(" %s(%d)", o->kind(), o->num_channels());
    }
    std::printf("\n");
    std::printf("[%s] printing FC STATUSTEXT / prearm health (arming errors)\n",
                _cfg.name.c_str());

    _initialized = true;
    return true;
}

void Vehicle::shutdown() {
    if (!_initialized) return;
    if (_cfg.transport) _cfg.transport->stop();
    _initialized = false;
}

// ===========================================================================
// Gz callbacks
// ===========================================================================
void Vehicle::imuCallback(const gz::msgs::IMU &msg) {
    // FLU → FRD: negate Y, Z on both accel and gyro
    const float ax =  static_cast<float>(msg.linear_acceleration().x());
    const float ay = -static_cast<float>(msg.linear_acceleration().y());
    const float az = -static_cast<float>(msg.linear_acceleration().z());
    const float gx =  static_cast<float>(msg.angular_velocity().x());
    const float gy = -static_cast<float>(msg.angular_velocity().y());
    const float gz = -static_cast<float>(msg.angular_velocity().z());

    _imu_cb.fetch_add(1);
    {
        std::lock_guard<std::mutex> lk(_state_mutex);
        _acc_x = ax;  _acc_y = ay;  _acc_z = az;
        _gyro_x = gx; _gyro_y = gy; _gyro_z = gz;
    }
    sendHilSensor(now_usec());

    const uint64_t t = now_usec();
    if (t - _last_heartbeat_us > 1'000'000ULL) {
        _last_heartbeat_us = t;
        sendHeartbeat();
        sendSystemTime(t);
    }
}

void Vehicle::magCallback(const gz::msgs::Magnetometer &msg) {
    // Gz mag publishes Gauss in left-handed frame (field named "field_tesla"
    // is a misnomer per gz-sim#2460). Frame: x_frd = -y_gz, y = -x_gz, z = z_gz.
    _mag_cb.fetch_add(1);
    std::lock_guard<std::mutex> lk(_state_mutex);
    _mag_x = -static_cast<float>(msg.field_tesla().y());
    _mag_y = -static_cast<float>(msg.field_tesla().x());
    _mag_z =  static_cast<float>(msg.field_tesla().z());
}

void Vehicle::baroCallback(const gz::msgs::FluidPressure &msg) {
    _baro_cb.fetch_add(1);
    const float pa = static_cast<float>(msg.pressure());
    constexpr float P0 = 101325.0f;
    const float alt = 44330.0f * (1.0f - std::pow(pa / P0, 1.0f / 5.255f));

    std::lock_guard<std::mutex> lk(_state_mutex);
    _abs_pressure_hpa = pa / 100.0f;
    _pressure_alt_m   = alt;
}

void Vehicle::navSatCallback(const gz::msgs::NavSat &msg) {
    _gps_cb.fetch_add(1);
    {
        std::lock_guard<std::mutex> lk(_state_mutex);
        _gps_lat = msg.latitude_deg();
        _gps_lon = msg.longitude_deg();
        _gps_alt = msg.altitude();
        _gps_vn  = static_cast<float>(msg.velocity_north());
        _gps_ve  = static_cast<float>(msg.velocity_east());
        _gps_vd  = -static_cast<float>(msg.velocity_up());
        _gps_valid = true;
    }
    sendHilGps(now_usec());
}

// ===========================================================================
// MAVLink senders
// ===========================================================================
void Vehicle::sendHilSensor(uint64_t time_usec) {
    mavlink_hil_sensor_t hil{};
    {
        std::lock_guard<std::mutex> lk(_state_mutex);
        hil.xacc = _acc_x; hil.yacc = _acc_y; hil.zacc = _acc_z;
        hil.xgyro = _gyro_x; hil.ygyro = _gyro_y; hil.zgyro = _gyro_z;
        hil.xmag = _mag_x; hil.ymag = _mag_y; hil.zmag = _mag_z;
        hil.abs_pressure = _abs_pressure_hpa;
        hil.pressure_alt = _pressure_alt_m;
        hil.temperature  = _temperature_c;
    }
    hil.diff_pressure  = 0.0f;
    hil.time_usec      = time_usec;
    hil.fields_updated = HIL_SENSOR_FIELDS;

    mavlink_message_t msg;
    mavlink_msg_hil_sensor_encode(_cfg.transport->system_id(),
                                  _cfg.transport->component_id(), &msg, &hil);
    if (_cfg.transport->send(msg)) _hil_sensor_sent.fetch_add(1);
}

void Vehicle::sendHilGps(uint64_t time_usec) {
    mavlink_hil_gps_t gps{};
    {
        std::lock_guard<std::mutex> lk(_state_mutex);
        gps.lat = static_cast<int32_t>(_gps_lat * 1e7);
        gps.lon = static_cast<int32_t>(_gps_lon * 1e7);
        gps.alt = static_cast<int32_t>(_gps_alt * 1000.0);
        gps.vn  = static_cast<int16_t>(_gps_vn  * 100.0f);
        gps.ve  = static_cast<int16_t>(_gps_ve  * 100.0f);
        gps.vd  = static_cast<int16_t>(_gps_vd  * 100.0f);
        float vel = std::sqrt(_gps_vn*_gps_vn + _gps_ve*_gps_ve + _gps_vd*_gps_vd);
        gps.vel = static_cast<uint16_t>(vel * 100.0f);
        float cog = std::atan2(_gps_ve, _gps_vn) * 180.0f / static_cast<float>(M_PI);
        if (cog < 0) cog += 360.0f;
        gps.cog = static_cast<uint16_t>(cog * 100.0f);
    }
    gps.time_usec          = time_usec;
    gps.fix_type           = 3;
    gps.eph                = 100;
    gps.epv                = 150;
    gps.satellites_visible = 12;

    mavlink_message_t msg;
    mavlink_msg_hil_gps_encode(_cfg.transport->system_id(),
                               _cfg.transport->component_id(), &msg, &gps);
    _cfg.transport->send(msg);
}

void Vehicle::sendSystemTime(uint64_t time_usec) {
    mavlink_system_time_t st{};
    st.time_unix_usec = time_usec;
    st.time_boot_ms   = static_cast<uint32_t>(time_usec / 1000);
    mavlink_message_t msg;
    mavlink_msg_system_time_encode(_cfg.transport->system_id(),
                                   _cfg.transport->component_id(), &msg, &st);
    _cfg.transport->send(msg);
}

void Vehicle::sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        _cfg.transport->system_id(), _cfg.transport->component_id(), &msg,
        MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    _cfg.transport->send(msg);

    // Once HIL traffic is flowing, ask the FC for SYS_STATUS so we can print
    // prearm / sensor health without needing QGC on the same port.
    if (!_streams_requested && _actuators_received.load() > 0) {
        requestFcStreams();
        _streams_requested = true;
    }
}

void Vehicle::requestFcStreams() {
    // Target the FC on this link. Use broadcast component so USB mavlink picks it up.
    const uint8_t target_sys = _cfg.transport->system_id();
    const uint8_t target_comp = 0; // all components
    const uint8_t our_sys = _cfg.transport->system_id();
    const uint8_t our_comp = _cfg.transport->component_id();

    auto set_interval = [&](uint32_t msgid, float hz) {
        mavlink_message_t msg;
        mavlink_msg_command_long_pack(
            our_sys, our_comp, &msg,
            target_sys, target_comp,
            MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            static_cast<float>(msgid),
            (hz > 0.f) ? (1.0e6f / hz) : -1.0f,
            0, 0, 0, 0, 0);
        _cfg.transport->send(msg);
    };

    set_interval(MAVLINK_MSG_ID_SYS_STATUS, 1.0f);
    set_interval(MAVLINK_MSG_ID_STATUSTEXT, 10.0f);
    set_interval(MAVLINK_MSG_ID_HEARTBEAT, 1.0f);
    std::printf("[%s] requested FC streams: SYS_STATUS/STATUSTEXT/HEARTBEAT\n",
                _cfg.name.c_str());
}

const char *Vehicle::severityName(uint8_t severity) {
    switch (severity) {
        case MAV_SEVERITY_EMERGENCY: return "EMERGENCY";
        case MAV_SEVERITY_ALERT:     return "ALERT";
        case MAV_SEVERITY_CRITICAL:  return "CRITICAL";
        case MAV_SEVERITY_ERROR:     return "ERROR";
        case MAV_SEVERITY_WARNING:   return "WARNING";
        case MAV_SEVERITY_NOTICE:    return "NOTICE";
        case MAV_SEVERITY_INFO:      return "INFO";
        case MAV_SEVERITY_DEBUG:     return "DEBUG";
        default:                     return "UNKNOWN";
    }
}

std::string Vehicle::describeUnhealthy(uint32_t present, uint32_t enabled, uint32_t health) {
    // Unhealthy = required (enabled) but not healthy. Prefer bits that are present
    // or explicitly enabled — matches how QGC surfaces sensor fails.
    const uint32_t bad = enabled & ~health;
    if (bad == 0) return {};

    std::ostringstream oss;
    bool first = true;
    for (const auto &s : kSensorBits) {
        if (bad & s.bit) {
            if (!first) oss << ',';
            first = false;
            oss << s.name;
            if (!(present & s.bit) && s.bit != MAV_SYS_STATUS_PREARM_CHECK) {
                oss << "(missing)";
            }
        }
    }
    if (first) {
        char hex[16];
        std::snprintf(hex, sizeof(hex), "0x%08x", bad);
        return hex;
    }
    return oss.str();
}

// ===========================================================================
// MAVLink RX → actuator dispatch + FC diagnostics
// ===========================================================================
void Vehicle::onMavlinkMessage(const mavlink_message_t &msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS: {
            mavlink_hil_actuator_controls_t act;
            mavlink_msg_hil_actuator_controls_decode(&msg, &act);
            _actuators_received.fetch_add(1);

            const bool armed = (act.mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            _fc_armed.store(armed);

            for (auto &out : _cfg.outputs) {
                out->apply(act.controls, armed);
            }
            break;
        }
        case MAVLINK_MSG_ID_STATUSTEXT:
            handleStatustext(msg);
            break;
        case MAVLINK_MSG_ID_SYS_STATUS:
            handleSysStatus(msg);
            break;
        case MAVLINK_MSG_ID_HEARTBEAT:
            handleHeartbeat(msg);
            break;
        case MAVLINK_MSG_ID_COMMAND_ACK:
            handleCommandAck(msg);
            break;
        default:
            break;
    }
}

void Vehicle::handleStatustext(const mavlink_message_t &msg) {
    mavlink_statustext_t st{};
    mavlink_msg_statustext_decode(&msg, &st);

    char text[sizeof(st.text) + 1];
    std::memcpy(text, st.text, sizeof(st.text));
    text[sizeof(st.text)] = '\0';
    // Trim trailing NULs / spaces for cleaner logs.
    for (int i = static_cast<int>(std::strlen(text)) - 1; i >= 0; --i) {
        if (text[i] == '\0' || text[i] == ' ' || text[i] == '\n' || text[i] == '\r') {
            text[i] = '\0';
        } else {
            break;
        }
    }
    if (text[0] == '\0') return;

    const std::string body(text);
    // Always print arming/preflight related text; otherwise WARNING+.
    const bool interesting = looksLikeArmingOrPreflight(body) || st.severity <= MAV_SEVERITY_WARNING;
    if (!interesting) return;

    std::printf("[%s][FC %s] %s\n", _cfg.name.c_str(), severityName(st.severity), text);
    std::fflush(stdout);
}

void Vehicle::handleSysStatus(const mavlink_message_t &msg) {
    mavlink_sys_status_t sys{};
    mavlink_msg_sys_status_decode(&msg, &sys);

    const bool prearm_ok = (sys.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0;
    const uint32_t unhealthy = sys.onboard_control_sensors_enabled & ~sys.onboard_control_sensors_health;

    bool changed = false;
    {
        std::lock_guard<std::mutex> lk(_state_mutex);
        changed = !_have_sys_status
               || prearm_ok != _prearm_ok
               || unhealthy != _last_unhealthy_mask;
        _have_sys_status = true;
        _prearm_ok = prearm_ok;
        _sensors_present = sys.onboard_control_sensors_present;
        _sensors_enabled = sys.onboard_control_sensors_enabled;
        _sensors_health = sys.onboard_control_sensors_health;
        _last_unhealthy_mask = unhealthy;
    }

    if (!changed) return;

    const std::string bad = describeUnhealthy(sys.onboard_control_sensors_present,
                                              sys.onboard_control_sensors_enabled,
                                              sys.onboard_control_sensors_health);
    if (prearm_ok) {
        std::printf("[%s][FC PREARM] OK\n", _cfg.name.c_str());
    } else {
        std::printf("[%s][FC PREARM] FAIL%s%s\n",
                    _cfg.name.c_str(),
                    bad.empty() ? "" : " unhealthy=",
                    bad.empty() ? "" : bad.c_str());
    }
    std::fflush(stdout);
}

void Vehicle::handleHeartbeat(const mavlink_message_t &msg) {
    // Ignore our own / non-autopilot heartbeats.
    mavlink_heartbeat_t hb{};
    mavlink_msg_heartbeat_decode(&msg, &hb);
    if (hb.autopilot == MAV_AUTOPILOT_INVALID) return;

    const bool armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    bool changed = false;
    {
        std::lock_guard<std::mutex> lk(_state_mutex);
        changed = !_have_fc_heartbeat
               || hb.base_mode != _fc_base_mode
               || hb.system_status != _fc_system_status
               || armed != _fc_armed.load();
        _have_fc_heartbeat = true;
        _fc_base_mode = hb.base_mode;
        _fc_system_status = hb.system_status;
    }
    _fc_armed.store(armed);

    if (!changed) return;

    std::printf("[%s][FC HB] armed=%d base_mode=0x%02x custom_mode=%u sys_status=%u hil=%d\n",
                _cfg.name.c_str(),
                (int)armed,
                (unsigned)hb.base_mode,
                (unsigned)hb.custom_mode,
                (unsigned)hb.system_status,
                (hb.base_mode & MAV_MODE_FLAG_HIL_ENABLED) ? 1 : 0);
    std::fflush(stdout);
}

void Vehicle::handleCommandAck(const mavlink_message_t &msg) {
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&msg, &ack);
    if (ack.command != MAV_CMD_COMPONENT_ARM_DISARM) return;

    const char *result = "UNKNOWN";
    switch (ack.result) {
        case MAV_RESULT_ACCEPTED: result = "ACCEPTED"; break;
        case MAV_RESULT_TEMPORARY_REJECTED: result = "TEMPORARY_REJECTED"; break;
        case MAV_RESULT_DENIED: result = "DENIED"; break;
        case MAV_RESULT_UNSUPPORTED: result = "UNSUPPORTED"; break;
        case MAV_RESULT_FAILED: result = "FAILED"; break;
        case MAV_RESULT_IN_PROGRESS: result = "IN_PROGRESS"; break;
        case MAV_RESULT_CANCELLED: result = "CANCELLED"; break;
        default: break;
    }
    std::printf("[%s][FC ARM_ACK] result=%s (%u)\n",
                _cfg.name.c_str(), result, (unsigned)ack.result);
    std::fflush(stdout);
}

// ===========================================================================
std::string Vehicle::statusLine() {
    float az, p_hpa;
    double lat, lon;
    bool gps_ok;
    bool prearm_ok = false;
    bool have_sys = false;
    uint32_t present = 0, enabled = 0, health = 0;
    {
        std::lock_guard<std::mutex> lk(_state_mutex);
        az = _acc_z;
        p_hpa = _abs_pressure_hpa;
        lat = _gps_lat; lon = _gps_lon;
        gps_ok = _gps_valid;
        prearm_ok = _prearm_ok;
        have_sys = _have_sys_status;
        present = _sensors_present;
        enabled = _sensors_enabled;
        health = _sensors_health;
    }

    std::string unhealthy;
    if (have_sys && !prearm_ok) {
        unhealthy = describeUnhealthy(present, enabled, health);
    }

    char buf[768];
    std::snprintf(buf, sizeof(buf),
        "[%s] sysid=%d sent:hil_s=%lu act_rx=%lu armed=%d prearm=%s%s%s | "
        "cb: imu=%lu mag=%lu baro=%lu gps=%lu | "
        "az=%+5.2f baro=%.1fhPa gps=%s lat=%.5f lon=%.5f",
        _cfg.name.c_str(),
        (int)_cfg.transport->system_id(),
        (unsigned long)_hil_sensor_sent.load(),
        (unsigned long)_actuators_received.load(),
        (int)_fc_armed.load(),
        have_sys ? (prearm_ok ? "OK" : "FAIL") : "?",
        unhealthy.empty() ? "" : " bad=",
        unhealthy.empty() ? "" : unhealthy.c_str(),
        (unsigned long)_imu_cb.load(),
        (unsigned long)_mag_cb.load(),
        (unsigned long)_baro_cb.load(),
        (unsigned long)_gps_cb.load(),
        az, p_hpa, gps_ok ? "OK" : "NO", lat, lon);
    return buf;
}
