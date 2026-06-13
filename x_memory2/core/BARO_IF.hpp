#pragma once
#include <cstdint>

// Barometer Data Structure
struct BAROData {
    double altitude;       // Pressure altitude in meters
    double pressure;       // Pressure in Pascals (Pa)
    double temperature;    // Temperature in Celsius
    uint64_t timestamp_ns;
};

// Abstract Interface for Barometer Sensors
class BARO_IF {
public:
    virtual ~BARO_IF() = default;

    // Initialize sensor
    virtual bool init() = 0;

    // Read latest data
    // Returns true if new data is available
    virtual bool read(BAROData& out_data) = 0;

    // Check health status
    virtual bool is_healthy() const = 0;
};
