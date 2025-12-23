#pragma once
#include "../../core/ATT_IF.hpp"
#include "../../core/IMU_IF.hpp"
#include <cmath>

class ATT_Mahony : public ATT_IF {
public:
    ATT_Mahony(float kp = 0.5f, float ki = 0.005f) 
        : kp_(kp), ki_(ki) 
    {
        reset();
    }

    void init() override {
        reset();
    }

    void reset() override {
        qw_ = 1.0f; qx_ = 0.0f; qy_ = 0.0f; qz_ = 0.0f;
        ex_i_ = 0.0f; ey_i_ = 0.0f; ez_i_ = 0.0f;
        last_timestamp_ns_ = 0;
    }

    void update(const IMUData& imu_data) override {
        float dt = 0.001f; // default fall-back
        if (last_timestamp_ns_ != 0) {
            long long diff = imu_data.timestamp_ns - last_timestamp_ns_;
            if (diff > 0) {
                dt = static_cast<float>(diff * 1e-9);
            }
        }
        last_timestamp_ns_ = imu_data.timestamp_ns;

        // Safety clamp for dt
        if (dt < 0.0001f) dt = 0.0001f;
        else if (dt > 0.02f) dt = 0.02f;

        float ax = static_cast<float>(imu_data.accel[0]);
        float ay = static_cast<float>(imu_data.accel[1]);
        float az = static_cast<float>(imu_data.accel[2]);
        
        float gx = static_cast<float>(imu_data.gyro[0]);
        float gy = static_cast<float>(imu_data.gyro[1]);
        float gz = static_cast<float>(imu_data.gyro[2]);

        updateMahony(gx, gy, gz, ax, ay, az, dt);
    }

    void get_quaternion(float& w, float& x, float& y, float& z) const override {
        w = qw_; x = qx_; y = qy_; z = qz_;
    }

private:
    void updateMahony(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        float an = std::sqrt(ax*ax + ay*ay + az*az);
        const float g = 9.80665f;
        
        // Use accel only if close to 1g (simple gating)
        bool use_acc = (std::fabs(an - g) < 2.0f);
        if (use_acc && an > 0.0f) {
            ax /= an; ay /= an; az /= an;
        } else {
            ax = 0; ay = 0; az = 0; // effectively disable accel correction
            use_acc = false;
        }

        // Estimated gravity direction from quaternion
        float q0 = qw_, q1 = qx_, q2 = qy_, q3 = qz_;
        float gxb = 2.0f*(q1*q3 - q0*q2);
        float gyb = 2.0f*(q0*q1 + q2*q3);
        float gzb = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // Error
        float ex = 0.0f, ey = 0.0f, ez = 0.0f;
        if (use_acc) {
            ex = gyb*az - gzb*ay;
            ey = gzb*ax - gxb*az;
            ez = gxb*ay - gyb*ax;
            
            ex_i_ += ex * dt;
            ey_i_ += ey * dt;
            ez_i_ += ez * dt;
        }

        // Apply feedback
        gx += kp_ * ex + ki_ * ex_i_;
        gy += kp_ * ey + ki_ * ey_i_;
        gz += kp_ * ez + ki_ * ez_i_;

        // Quat integration
        float dq0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
        float dq1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
        float dq2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
        float dq3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

        qw_ += dq0 * dt;
        qx_ += dq1 * dt;
        qy_ += dq2 * dt;
        qz_ += dq3 * dt;

        // Normalize
        float n = std::sqrt(qw_*qw_ + qx_*qx_ + qy_*qy_ + qz_*qz_);
        if (n > 0.0f) {
            qw_ /= n; qx_ /= n; qy_ /= n; qz_ /= n;
        }
    }

private:
    float qw_, qx_, qy_, qz_;
    float ex_i_, ey_i_, ez_i_;
    float kp_, ki_;
    uint64_t last_timestamp_ns_;
};
