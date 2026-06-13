#pragma once
#include "../../core/EST_IF.hpp"
#include <iostream>

#ifdef USE_PX4_ECL
// Include real PX4 ECL headers when compiling inside PX4/Firmware environment
#include <ecl/EKF/ekf.h>
#else
// Define stub structures to allow clean local Windows compilation without ECL source files
namespace ecl {
    struct Vector3f {
        float x, y, z;
        Vector3f(float x_ = 0.0f, float y_ = 0.0f, float z_ = 0.0f) : x(x_), y(y_), z(z_) {}
    };

    struct Quaternion {
        float w, x, y, z;
        Quaternion(float w_ = 1.0f, float x_ = 0.0f, float y_ = 0.0f, float z_ = 0.0f) : w(w_), x(x_), y(y_), z(z_) {}
    };

    struct imuSample {
        uint64_t time_us;
        Vector3f delta_ang;
        Vector3f delta_vel;
    };

    struct gpsSample {
        uint64_t time_us;
        double lat;
        double lon;
        float alt;
        Vector3f vel_ned;
        float hacc;
        float vacc;
        float sacc;
        uint8_t fix_type;
        uint8_t nsats;
    };

    struct baroSample {
        uint64_t time_us;
        float hgt;
    };

    class Ekf {
    public:
        Ekf() : initialized_(false), last_time_us_(0) {}
        bool init(uint64_t time_us) { initialized_ = true; last_time_us_ = time_us; return true; }
        void reset() { initialized_ = false; }
        void setIMUData(const imuSample& imu) { last_time_us_ = imu.time_us; }
        void setGpsData(const gpsSample& gps) {}
        void setBaroData(const baroSample& baro) {}
        bool update() { return initialized_; }

        // Mock states
        Quaternion getQuaternion() const { return Quaternion(1.0f, 0.0f, 0.0f, 0.0f); }
        Vector3f getVelocity() const { return Vector3f(0.0f, 0.0f, 0.0f); }
        Vector3f getPosition() const { return Vector3f(0.0f, 0.0f, 0.0f); }
        Vector3f getGyroBias() const { return Vector3f(0.0f, 0.0f, 0.0f); }
        Vector3f getAccelBias() const { return Vector3f(0.0f, 0.0f, 0.0f); }
        bool isHealthy() const { return initialized_; }

    private:
        bool initialized_;
        uint64_t last_time_us_;
    };
}
#endif

class EST_ECL : public EST_IF {
public:
    EST_ECL() : last_timestamp_ns_(0) {
        init();
    }

    void init() override {
        ekf_.init(0);
        last_timestamp_ns_ = 0;
    }

    void reset() override {
        ekf_.reset();
        last_timestamp_ns_ = 0;
    }

    void update_imu(const IMUData& imu_data) override {
        uint64_t time_us = imu_data.timestamp_ns / 1000;
        
        float dt = 0.001f;
        if (last_timestamp_ns_ != 0) {
            long long diff = imu_data.timestamp_ns - last_timestamp_ns_;
            if (diff > 0) {
                dt = static_cast<float>(diff * 1e-9);
            }
        }
        last_timestamp_ns_ = imu_data.timestamp_ns;

        // Populate and push IMU data to ECL
        ecl::imuSample sample;
        sample.time_us = time_us;
        
        // ECL expects integrated delta angles and delta velocities over the dt interval
        sample.delta_ang.x = static_cast<float>(imu_data.gyro[0]) * dt;
        sample.delta_ang.y = static_cast<float>(imu_data.gyro[1]) * dt;
        sample.delta_ang.z = static_cast<float>(imu_data.gyro[2]) * dt;
        
        sample.delta_vel.x = static_cast<float>(imu_data.accel[0]) * dt;
        sample.delta_vel.y = static_cast<float>(imu_data.accel[1]) * dt;
        sample.delta_vel.z = static_cast<float>(imu_data.accel[2]) * dt;

        ekf_.setIMUData(sample);
        ekf_.update();
    }

    void update_gnss(const GNSSData& gnss_data) override {
        ecl::gpsSample sample;
        sample.time_us = gnss_data.timestamp_ns / 1000;
        sample.lat = gnss_data.lat;
        sample.lon = gnss_data.lon;
        sample.alt = static_cast<float>(gnss_data.alt);
        
        sample.vel_ned.x = static_cast<float>(gnss_data.vel_ned[0]);
        sample.vel_ned.y = static_cast<float>(gnss_data.vel_ned[1]);
        sample.vel_ned.z = static_cast<float>(gnss_data.vel_ned[2]);
        
        sample.hacc = 0.5f; // Standard clean GPS values
        sample.vacc = 1.0f;
        sample.sacc = 0.1f;
        sample.fix_type = 3; // 3D Fix
        sample.nsats = 10;

        ekf_.setGpsData(sample);
    }

    void update_baro(const BAROData& baro_data) override {
        ecl::baroSample sample;
        sample.time_us = baro_data.timestamp_ns / 1000;
        sample.hgt = static_cast<float>(baro_data.altitude);

        ekf_.setBaroData(sample);
    }

    bool get_estimated_state(EstimatedState& out_state) const override {
        auto q = ekf_.getQuaternion();
        auto v = ekf_.getVelocity();
        auto p = ekf_.getPosition();
        auto gb = ekf_.getGyroBias();
        auto ab = ekf_.getAccelBias();

        out_state.quat[0] = q.w;
        out_state.quat[1] = q.x;
        out_state.quat[2] = q.y;
        out_state.quat[3] = q.z;

        out_state.vel_ned[0] = v.x;
        out_state.vel_ned[1] = v.y;
        out_state.vel_ned[2] = v.z;

        out_state.pos_ned[0] = p.x;
        out_state.pos_ned[1] = p.y;
        out_state.pos_ned[2] = p.z;

        out_state.gyro_bias[0] = gb.x;
        out_state.gyro_bias[1] = gb.y;
        out_state.gyro_bias[2] = gb.z;

        out_state.accel_bias[0] = ab.x;
        out_state.accel_bias[1] = ab.y;
        out_state.accel_bias[2] = ab.z;

        out_state.timestamp_ns = last_timestamp_ns_;
#ifdef USE_PX4_ECL
        out_state.is_healthy = ekf_.isHealthy();
#else
        out_state.is_healthy = true; // stub is always healthy
#endif
        return true;
    }

private:
    ecl::Ekf ekf_;
    uint64_t last_timestamp_ns_;
};
