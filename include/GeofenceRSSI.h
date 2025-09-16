#pragma once
#include <Arduino.h>

class ComManager;
class SensorManager;
class FlightController;

enum GeofenceState {
    GFS_OK = 0,
    GFS_WARN,
    GFS_BLOCK,
    GFS_LOST
};

class GeofenceRSSI {
public:
    GeofenceRSSI();

    // Call every loop with pointers to your managers (non-owning)
    void update(ComManager* com, SensorManager* sensors, FlightController* fc);

    // Tuning parameters (public for easy tuning)
    float d0 = 1.0f;          // reference distance (m)
    float RSSI0 = -40.0f;     // reference RSSI at d0 (dBm) — calibrate!
    float pathLossExp = 2.2f; // path-loss exponent (n)
    float alpha = 0.15f;      // EMA alpha for RSSI smoothing
    float R_block = 1.2f;     // blocking radius (m)
    float R_warn  = 1.8f;     // warning radius (m)
    float R_exit  = 1.6f;     // exit hysteresis radius (m)
    uint32_t t_confirm_ms = 200; // require stable breach for this time
    uint32_t t_lost_ms    = 500; // treat RSSI loss as "lost" after this

    // Current outputs / readbacks
    GeofenceState state() const { return _state; }
    float lastFilteredRSSI() const { return _rssi_f; }
    float lastDistance() const { return _lastDistance; } // meters (slant)
    uint32_t lastUpdateMs() const { return _lastUpdateMs; }

private:
    // internal state
    float _rssi_f;
    float _lastDistance; // slant distance estimate in meters
    GeofenceState _state;
    uint32_t _stateTransitionMs;
    uint32_t _lastUpdateMs;

    // Helpers
    float rssiToDistance(float rssi_dbm) const;
};
