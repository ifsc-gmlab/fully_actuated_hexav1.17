#include "vehicle.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <sstream>

extern "C" {
#include <mavlink.h>
}

namespace {
constexpr uint32_t HIL_SENSOR_FIELDS =
    (1u<<0) | (1u<<1) | (1u<<2) |   // accel xyz
    (1u<<3) | (1u<<4) | (1u<<5) |   // gyro xyz
    (1u<<6) | (1u<<7) | (1u<<8) |   // mag xyz
    (1u<<9) | (1u<<11) | (1u<<12);  // abs_p, press_alt, temp
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
}

// ===========================================================================
// MAVLink RX → actuator dispatch
// ===========================================================================
void Vehicle::onMavlinkMessage(const mavlink_message_t &msg) {
    if (msg.msgid != MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) return;

    mavlink_hil_actuator_controls_t act;
    mavlink_msg_hil_actuator_controls_decode(&msg, &act);
    _actuators_received.fetch_add(1);

    const bool armed = (act.mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    _fc_armed.store(armed);

    for (auto &out : _cfg.outputs) {
        out->apply(act.controls, armed);
    }
}

// ===========================================================================
std::string Vehicle::statusLine() {
    float az, p_hpa;
    double lat, lon;
    bool gps_ok;
    {
        std::lock_guard<std::mutex> lk(_state_mutex);
        az = _acc_z;
        p_hpa = _abs_pressure_hpa;
        lat = _gps_lat; lon = _gps_lon;
        gps_ok = _gps_valid;
    }
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "[%s] sysid=%d sent:hil_s=%lu act_rx=%lu armed=%d | "
        "cb: imu=%lu mag=%lu baro=%lu gps=%lu | "
        "az=%+5.2f baro=%.1fhPa gps=%s lat=%.5f lon=%.5f",
        _cfg.name.c_str(),
        (int)_cfg.transport->system_id(),
        (unsigned long)_hil_sensor_sent.load(),
        (unsigned long)_actuators_received.load(),
        (int)_fc_armed.load(),
        (unsigned long)_imu_cb.load(),
        (unsigned long)_mag_cb.load(),
        (unsigned long)_baro_cb.load(),
        (unsigned long)_gps_cb.load(),
        az, p_hpa, gps_ok ? "OK" : "NO", lat, lon);
    return buf;
}
