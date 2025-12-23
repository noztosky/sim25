#pragma once
#include <cstdint>

// IMU Raw Data Structure
struct IMUData {
    double accel[3]; // m/s^2 (Body frame)
    double gyro[3];  // rad/s (Body frame)
    double mag[3];   // uT (Optional)
    uint64_t timestamp_ns;
};

// Abstract Interface for IMU Sensors
class IMU_IF {
public:
    virtual ~IMU_IF() = default;

    // Initialize sensor connection
    virtual bool init() = 0;

    // Read latest data
    // Returns true if new data is available
    virtual bool read(IMUData& out_data) = 0;

    // Check health status
    virtual bool is_healthy() const = 0;
};
