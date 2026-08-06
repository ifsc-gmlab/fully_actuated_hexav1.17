#pragma once

#include "vehicle.hpp"

#include <memory>
#include <vector>

// Runs N Vehicles in the same process. Each Vehicle is independent:
// own transport, own MAVLink RX thread, own Gz subscriptions. The Manager
// only orchestrates init/shutdown and prints a periodic combined status.
class VehicleManager {
public:
    VehicleManager() = default;
    ~VehicleManager();

    // Take ownership of a configured vehicle (call before initAll).
    void add(Vehicle::Config cfg);

    // Init all vehicles. Returns false if ANY vehicle failed (and stops the rest).
    bool initAll();
    void shutdownAll();

    // Print one combined status block (one line per vehicle). Cheap, call ~1Hz.
    void printStatus();

    size_t size() const { return _vehicles.size(); }

private:
    std::vector<std::unique_ptr<Vehicle>> _vehicles;
};
