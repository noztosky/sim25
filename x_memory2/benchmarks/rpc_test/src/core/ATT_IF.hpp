#pragma once

// Attitude Data Structure (Quaternion)
struct AttitudeData {
    float w, x, y, z;
    float roll, pitch, yaw; // Optional, for convenience debug
};

// Abstract Interface for Attitude Estimators
class ATT_IF {
public:
    virtual ~ATT_IF() = default;

    // Initialize estimator
    virtual void init() = 0;

    // Update estimator with new IMU data (and dt if managed internally, or pass dt)
    // Assuming IMUData already contains timestamp for dt calculation
    virtual void update(const struct IMUData& imu_data) = 0;

    // Get current attitude
    virtual void get_quaternion(float& w, float& x, float& y, float& z) const = 0;
    
    // Reset/Re-initialize estimator state
    virtual void reset() = 0;
};
