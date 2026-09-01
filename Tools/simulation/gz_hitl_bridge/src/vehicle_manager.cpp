#include "vehicle_manager.hpp"

#include <cstdio>

VehicleManager::~VehicleManager() { shutdownAll(); }

void VehicleManager::add(Vehicle::Config cfg) {
    _vehicles.push_back(std::make_unique<Vehicle>(std::move(cfg)));
}

bool VehicleManager::initAll() {
    for (auto &v : _vehicles) {
        if (!v->init()) {
            std::fprintf(stderr, "[manager] init failed for vehicle '%s'\n",
                         v->name().c_str());
            shutdownAll();
            return false;
        }
    }
    return true;
}

void VehicleManager::shutdownAll() {
    for (auto &v : _vehicles) v->shutdown();
}

void VehicleManager::printStatus() {
    std::printf("--- status (%zu vehicles) ---\n", _vehicles.size());
    for (auto &v : _vehicles) {
        std::printf("%s\n", v->statusLine().c_str());
    }
}
