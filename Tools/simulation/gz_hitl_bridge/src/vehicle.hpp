#pragma once

#include "actuator.hpp"
#include "transport.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/transport/Node.hh>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/navsat.pb.h>

// Per-vehicle bridge. One Vehicle owns:
//   - one Transport (talks to one FC)
//   - one set of Gz sensor subscriptions (IMU, mag, baro, gps) on a model
//   - one set of ActuatorOutputs that publish back to Gz
//
// VehicleManager runs N Vehicles in the same process for swarms.
class Vehicle {
public:
    struct Config {
        std::string name = "drone";           // log prefix
        std::string world_name = "default";
        std::string model_name = "x500";
        std::string sensor_link = "base_link";
        std::vector<std::unique_ptr<ActuatorOutput>> outputs;
        std::unique_ptr<Transport> transport;
    };

    explicit Vehicle(Config cfg);
    ~Vehicle();

    bool init();
    void shutdown();

    // Called by VehicleManager every second to print stats. Returns the line
    // to print (no trailing newline) so the manager can prefix with vehicle id.
    std::string statusLine();

    const std::string &name() const { return _cfg.name; }

private:
    // Gz callbacks
    void imuCallback(const gz::msgs::IMU &msg);
    void magCallback(const gz::msgs::Magnetometer &msg);
    void baroCallback(const gz::msgs::FluidPressure &msg);
    void navSatCallback(const gz::msgs::NavSat &msg);

    // MAVLink RX
    void onMavlinkMessage(const mavlink_message_t &msg);

    // MAVLink TX
    void sendHilSensor(uint64_t time_usec);
    void sendHilGps(uint64_t time_usec);
    void sendSystemTime(uint64_t time_usec);
    void sendHeartbeat();

    static uint64_t now_usec();

    Config _cfg;
    gz::transport::Node _node;

    // Latched sensor state
    std::mutex _state_mutex;
    float _acc_x{0}, _acc_y{0}, _acc_z{-9.81f};
    float _gyro_x{0}, _gyro_y{0}, _gyro_z{0};
    float _mag_x{0.21f}, _mag_y{0.0f}, _mag_z{0.43f};
    float _abs_pressure_hpa{1013.25f};
    float _pressure_alt_m{0.0f};
    float _temperature_c{25.0f};
    bool   _gps_valid{false};
    double _gps_lat{0}, _gps_lon{0}, _gps_alt{0};
    float  _gps_vn{0}, _gps_ve{0}, _gps_vd{0};

    // Stats
    std::atomic<uint64_t> _last_heartbeat_us{0};
    std::atomic<uint64_t> _hil_sensor_sent{0};
    std::atomic<uint64_t> _actuators_received{0};
    std::atomic<uint64_t> _imu_cb{0};
    std::atomic<uint64_t> _mag_cb{0};
    std::atomic<uint64_t> _baro_cb{0};
    std::atomic<uint64_t> _gps_cb{0};
    std::atomic<bool>     _fc_armed{false};

    bool _initialized{false};
};
