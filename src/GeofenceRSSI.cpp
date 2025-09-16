#include "GeofenceRSSI.h"
#include "ComManager.h"
#include "SensorManager.h"
#include "FlightController.h"
#include <math.h>

// tuning safety factors
#define MIN_VALID_RSSI -120

GeofenceRSSI::GeofenceRSSI()
: _rssi_f(MIN_VALID_RSSI), _lastDistance(1e6), _state(GFS_LOST),
  _stateTransitionMs(0), _lastUpdateMs(0)
{}

float GeofenceRSSI::rssiToDistance(float rssi_dbm) const {
    // d = d0 * 10^((RSSI0 - RSSI) / (10 * n))
    float exponent = (RSSI0 - rssi_dbm) / (10.0f * pathLossExp);
    return d0 * powf(10.0f, exponent);
}

void GeofenceRSSI::update(ComManager* com, SensorManager* sensors, FlightController* fc) {
    uint32_t now = millis();
    _lastUpdateMs = now;

    // Read latest RSSI from com manager (dBm)
    int8_t raw_rssi = com->getRSSI(); // -127 means no or very weak signal
    bool havePacket = (raw_rssi > MIN_VALID_RSSI + 1); // simple check

    if (!havePacket) {
        // Count as "lost" after t_lost_ms
        if (_state != GFS_LOST && now - _stateTransitionMs > t_lost_ms) {
            _state = GFS_LOST;
            _stateTransitionMs = now;
            Serial.println("[GEOFENCE] RSSI lost -> GFS_LOST");
        }
        // do conservative behavior: prevent aggressive motion
        // Set safe altitude target and reduce lateral by scaling to 0.3
        if (fc) {
            // prevent horizontal commands by scaling down setpoints
            fc->roll_set *= 0.3f;
            fc->pitch_set *= 0.3f;
            // keep altitude
            fc->alt_set  = sensors->alt_est;
        }
        return;
    }

    float rssi = (float)raw_rssi;

    // Exponential smoothing (EMA)
    if (_lastUpdateMs == 0) {
        _rssi_f = rssi;
    } else {
        _rssi_f = alpha * rssi + (1.0f - alpha) * _rssi_f;
    }

    // Convert to slant distance (meters)
    float slantD = rssiToDistance(_rssi_f);

    // convert to horizontal distance using barometer altitude
    // assume pilot height = 1.2m by default (you can change)
    float z_pilot = 1.2f;
    float z_drone = sensors->alt_est;
    float dz = z_drone - z_pilot;
    float horizontalD = slantD;
    float dz2 = dz * dz;
    if (slantD * slantD > dz2) horizontalD = sqrtf(slantD * slantD - dz2);
    else horizontalD = 0.0f;

    _lastDistance = horizontalD;

    // Decide geofence state with hysteresis + confirmation time
    GeofenceState wanted = GFS_OK;
    if (horizontalD <= R_block) wanted = GFS_BLOCK;
    else if (horizontalD <= R_warn) wanted = GFS_WARN;
    else wanted = GFS_OK;

    if (wanted != _state) {
        // If state changed, require confirmation time
        if (_stateTransitionMs == 0) {
            _stateTransitionMs = now;
        } else if (now - _stateTransitionMs >= t_confirm_ms) {
            // Confirmed
            _state = wanted;
            _stateTransitionMs = now;
            Serial.printf("[GEOFENCE] State -> %d  D=%.2f m  RSSI_f=%.1fdBm\n",
                          (int)_state, horizontalD, _rssi_f);
        }
        // don't apply enforcement until confirmed
    } else {
        // keep transition ms fresh
        _stateTransitionMs = now;
    }

    // Enforcement actions (soft-block behavior)
    if (fc) {
        switch (_state) {
            case GFS_OK:
                // normal: no intervention
                break;
            case GFS_WARN:
                // reduce lateral responsiveness (scale roll/pitch)
                fc->roll_set  *= 0.5f;
                fc->pitch_set *= 0.5f;
                // keep altitude target
                fc->alt_set = sensors->alt_est;
                Serial.println("[GEOFENCE] WARN: reduced lateral authority.");
                break;
            case GFS_BLOCK:
                // Block horizontal motion entirely; hover at current altitude
                fc->roll_set  = 0.0f;
                fc->pitch_set = 0.0f;
                fc->alt_set = sensors->alt_est;
                // optional: if your FlightController has a means to reduce base_throttle or tilt, do it elsewhere
                Serial.println("[GEOFENCE] BLOCK: preventing horizontal motion, hovering.");
                break;
            case GFS_LOST:
                // treat as conservative: scale lateral down
                fc->roll_set  *= 0.3f;
                fc->pitch_set *= 0.3f;
                fc->alt_set = sensors->alt_est;
                Serial.println("[GEOFENCE] LOST: RSSI missing, conservative mode.");
                break;
        }
    }
}
