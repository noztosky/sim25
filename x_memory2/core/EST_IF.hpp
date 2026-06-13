#pragma once
#include <cstdint>
#include "IMU_IF.hpp"
#include "GNSS_IF.hpp"
#include "BARO_IF.hpp"

// Unified State Estimation Structure
struct EstimatedState {
    float quat[4];       // w, x, y, z (Attitude Quaternion)
    float vel_ned[3];    // m/s (NED Velocity)
    float pos_ned[3];    // m (NED Position relative to local origin)
    float gyro_bias[3];  // rad/s (Estimated Gyroscope Biases)
    float accel_bias[3]; // m/s^2 (Estimated Accelerometer Biases)
    uint64_t timestamp_ns;
    bool is_healthy;
};

// Abstract State Estimator Interface
class EST_IF {
public:
    virtual ~EST_IF() = default;

    // Initialize estimator states and covariances
    virtual void init() = 0;

    // Reset/Reinitialize state estimation
    virtual void reset() = 0;

    // Update with high-rate IMU data (Prediction step)
    virtual void update_imu(const IMUData& imu_data) = 0;

    // Update with GNSS/GPS data (Correction step)
    virtual void update_gnss(const GNSSData& gnss_data) = 0;

    // Update with Barometer data (Correction step)
    virtual void update_baro(const BAROData& baro_data) = 0;

    // Update with Magnetometer data (Correction step)
    virtual void update_mag(const double mag[3]) {}

    // Retrieve the latest estimated state
    virtual bool get_estimated_state(EstimatedState& out_state) const = 0;
};
