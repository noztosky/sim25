#pragma once
#include <cstdint>

// GNSS Data Structure
struct GNSSData {
    double lat;            // Latitude in degrees (0.0 if local-only)
    double lon;            // Longitude in degrees (0.0 if local-only)
    double alt;            // Altitude in meters
    double pos_ned[3];     // Local NED position in meters
    double vel_ned[3];     // Local NED velocity in m/s
    uint64_t timestamp_ns;
};

// Abstract Interface for GNSS Sensors
class GNSS_IF {
public:
    virtual ~GNSS_IF() = default;

    // Initialize sensor
    virtual bool init() = 0;

    // Read latest data
    // Returns true if new data is available
    virtual bool read(GNSSData& out_data) = 0;

    // Check health status
    virtual bool is_healthy() const = 0;
};
