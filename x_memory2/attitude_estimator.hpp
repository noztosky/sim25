#pragma once

#include <cmath>

// Simple Mahony-style attitude estimator using gyro (rad/s) and accel (m/s^2)
// Quaternion is w,x,y,z (float). Gravity reference is [0,0,-1] in body when level.
class AttitudeEstimator
{
public:
    AttitudeEstimator(float kp = 2.0f, float ki = 0.0f)
        : kp_(kp), ki_(ki)
    {
        reset(1.0f, 0.0f, 0.0f, 0.0f);
        ex_i_ = ey_i_ = ez_i_ = 0.0f;
    }

    void reset(float w, float x, float y, float z)
    {
        qw_ = w; qx_ = x; qy_ = y; qz_ = z; normalizeQuat();
        ex_i_ = ey_i_ = ez_i_ = 0.0f;
    }

    void setGains(float kp, float ki)
    {
        kp_ = kp; ki_ = ki;
    }

    // Update with gyro (rad/s), accel (m/s^2), dt (s)
    void update(const float gyro[3], const float accel[3], float dt)
    {
        if (dt <= 0.0f) return;

        float ax = accel[0], ay = accel[1], az = accel[2];
        float an = std::sqrt(ax*ax + ay*ay + az*az);
        // gate accel correction near 1g (within ~0.2g)
        const float g = 9.80665f;
        bool use_acc = (std::fabs(an - g) < 2.0f);
        if (use_acc) { ax /= an; ay /= an; az /= an; }

        // Estimated gravity direction from current quaternion (body frame)
        // Using q to rotate world [0,0,-1] into body: g_b = R(q)*[0,0,-1]
        // For efficiency, derive directly:
        const float qw = qw_, qx = qx_, qy = qy_, qz = qz_;
        float gxb = 2.0f*(qx*qz - qw*qy);         // x component
        float gyb = 2.0f*(qy*qz + qw*qx);         // y component
        float gzb = qw*qw - qx*qx - qy*qy + qz*qz; // z component (points roughly -1 at level)

        // Error between measured and estimated gravity
        float ex = 0.0f, ey = 0.0f, ez = 0.0f;
        if (use_acc) {
            // e = (g_est x a_meas)
            ex = gyb*az - gzb*ay;
            ey = gzb*ax - gxb*az;
            ez = gxb*ay - gyb*ax;

            // Integrator
            ex_i_ += ex * dt;
            ey_i_ += ey * dt;
            ez_i_ += ez * dt;
        }

        // Apply proportional + integral correction to gyro
        float gx = gyro[0] + kp_*ex + ki_*ex_i_;
        float gy = gyro[1] + kp_*ey + ki_*ey_i_;
        float gz = gyro[2] + kp_*ez + ki_*ez_i_;

        // Quaternion derivative: q_dot = 0.5 * q ⊗ [0,g]
        float dq_w = 0.5f * (-qx*gx - qy*gy - qz*gz);
        float dq_x = 0.5f * ( qw*gx + qy*gz - qz*gy);
        float dq_y = 0.5f * ( qw*gy - qx*gz + qz*gx);
        float dq_z = 0.5f * ( qw*gz + qx*gy - qy*gx);

        qw_ += dq_w * dt;
        qx_ += dq_x * dt;
        qy_ += dq_y * dt;
        qz_ += dq_z * dt;
        normalizeQuat();
    }

    void getQuaternion(float& w, float& x, float& y, float& z) const
    {
        w = qw_; x = qx_; y = qy_; z = qz_;
    }

private:
    void normalizeQuat()
    {
        float n = std::sqrt(qw_*qw_ + qx_*qx_ + qy_*qy_ + qz_*qz_);
        if (n > 0.0f) { qw_ /= n; qx_ /= n; qy_ /= n; qz_ /= n; }
    }

private:
    // state
    float qw_{1.0f}, qx_{0.0f}, qy_{0.0f}, qz_{0.0f};
    float ex_i_{0.0f}, ey_i_{0.0f}, ez_i_{0.0f};
    float kp_{2.0f}, ki_{0.0f};
};


