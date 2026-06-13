#pragma once
#include "../../core/EST_IF.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

class EST_EKF15 : public EST_IF {
public:
    EST_EKF15() {
        reset();
    }

    void init() override {
        reset();
    }

    void reset() override {
        // Initialize nominal state
        pos_.setZero();
        vel_.setZero();
        quat_.setIdentity();
        bg_.setZero();
        ba_.setZero();

        // Initialize covariance P
        P_.setZero();
        P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.1;    // Position variance
        P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.1;    // Velocity variance
        P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.01;   // Attitude variance (rad^2)
        P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-6;   // Gyro bias variance
        P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-4; // Accel bias variance

        // Process noise tuning parameters
        double sigma_acc = 0.15;      // m/s^2/sqrt(Hz)
        double sigma_gyro = 0.005;    // rad/s/sqrt(Hz)
        double sigma_bias_gyro = 1e-5;// rad/s^2/sqrt(Hz)
        double sigma_bias_acc = 1e-3; // m/s^3/sqrt(Hz)

        Q_.setZero();
        Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (sigma_acc * sigma_acc);
        Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (sigma_gyro * sigma_gyro);
        Q_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (sigma_bias_gyro * sigma_bias_gyro);
        Q_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (sigma_bias_acc * sigma_bias_acc);

        // Measurement noises
        R_gps_pos_ = 0.25; // m^2 (0.5m std)
        R_gps_vel_ = 0.04; // (m/s)^2 (0.2m/s std)
        R_baro_ = 0.64;    // m^2 (0.8m std)

        last_timestamp_ns_ = 0;
        initialized_ = false;
    }

    void update_imu(const IMUData& imu_data) override {
        if (last_timestamp_ns_ == 0) {
            last_timestamp_ns_ = imu_data.timestamp_ns;
            initialized_ = true;
            return;
        }

        double dt = static_cast<double>(imu_data.timestamp_ns - last_timestamp_ns_) * 1e-9;
        last_timestamp_ns_ = imu_data.timestamp_ns;

        // Guard against unreasonable dt (e.g. pauses or anomalies)
        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.000125; // Default 8kHz fallback
        }

        // Unbias IMU measurements
        Eigen::Vector3d f_m(imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
        Eigen::Vector3d w_m(imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);

        Eigen::Vector3d f = f_m - ba_;
        Eigen::Vector3d w = w_m - bg_;

        // Gravitational acceleration in NED (m/s^2)
        const Eigen::Vector3d g(0.0, 0.0, 9.80665);

        // Rotate body accel to world NED
        Eigen::Matrix3d R = quat_.toRotationMatrix();
        Eigen::Vector3d a_ned = R * f + g;

        // 1. Nominal State Propagation
        pos_ += vel_ * dt + 0.5 * a_ned * dt * dt;
        vel_ += a_ned * dt;

        // Quaternion propagation
        Eigen::Vector3d d_theta = w * dt;
        double angle = d_theta.norm();
        if (angle > 1e-8) {
            Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, d_theta / angle));
            quat_ = (quat_ * dq).normalized();
        }

        // 2. Covariance Propagation: P = F * P * F^T + Q * dt
        Eigen::Matrix<double, 15, 15> F_sys = Eigen::Matrix<double, 15, 15>::Identity();
        
        // F_sys(0..2, 3..5) = I * dt
        F_sys.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        
        // F_sys(3..5, 6..8) = -R * [f]x * dt
        F_sys.block<3, 3>(3, 6) = -R * skew(f) * dt;
        
        // F_sys(3..5, 12..14) = -R * dt
        F_sys.block<3, 3>(3, 12) = -R * dt;
        
        // F_sys(6..8, 6..8) = R_dtheta^T = I - [w]x * dt
        F_sys.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew(w) * dt;
        
        // F_sys(6..8, 9..11) = -I * dt
        F_sys.block<3, 3>(6, 9) = Eigen::Matrix3d::Identity() * (-dt);

        P_ = F_sys * P_ * F_sys.transpose() + Q_ * dt;
    }

    void update_gnss(const GNSSData& gnss_data) override {
        if (!initialized_) return;

        // Measurement vector: z = [p_gps_ned, v_gps_ned]^T (6x1)
        Eigen::Matrix<double, 6, 1> z;
        z << gnss_data.pos_ned[0], gnss_data.pos_ned[1], gnss_data.pos_ned[2],
             gnss_data.vel_ned[0], gnss_data.vel_ned[1], gnss_data.vel_ned[2];

        // Predicted measurement: h(x) = [pos, vel]^T (6x1)
        Eigen::Matrix<double, 6, 1> h_x;
        h_x << pos_, vel_;

        // Innovation: d = z - h(x)
        Eigen::Matrix<double, 6, 1> innovation = z - h_x;

        // Measurement Jacobian H (6x15)
        Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // H_pos
        H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(); // H_vel

        // Measurement Noise Covariance R (6x6)
        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
        R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * R_gps_pos_;
        R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * R_gps_vel_;

        // Kalman Gain: K = P * H^T * (H * P * H^T + R)^-1
        Eigen::Matrix<double, 15, 6> PH_T = P_ * H.transpose();
        Eigen::Matrix<double, 6, 6> S = H * PH_T + R;
        Eigen::Matrix<double, 15, 6> K = PH_T * S.inverse();

        // Update Error State: dx (15x1)
        Eigen::Matrix<double, 15, 1> dx = K * innovation;

        // Update Covariance: P = (I - K * H) * P
        P_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P_;

        // Inject Error State into Nominal State
        inject_error_state(dx);
    }

    void update_baro(const BAROData& baro_data) override {
        if (!initialized_) return;

        // Measurement: z = pressure altitude (1x1)
        double z = baro_data.altitude;

        // Predicted measurement: h(x) = -pos_z (downwards is positive in NED, alt is positive upwards)
        double h_x = -pos_.z();

        double innovation = z - h_x;

        // Measurement Jacobian H (1x15)
        Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
        H(0, 2) = -1.0; // alt is -pos_z

        // Measurement Noise Covariance R (1x1)
        double R = R_baro_;

        // Kalman Gain
        Eigen::Matrix<double, 15, 1> PH_T = P_ * H.transpose();
        double S = H * PH_T + R;
        Eigen::Matrix<double, 15, 1> K = PH_T / S;

        // Update Error State
        Eigen::Matrix<double, 15, 1> dx = K * innovation;

        // Update Covariance
        P_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P_;

        // Inject Error State
        inject_error_state(dx);
    }

    bool get_estimated_state(EstimatedState& out_state) const override {
        if (!initialized_) return false;

        out_state.quat[0] = static_cast<float>(quat_.w());
        out_state.quat[1] = static_cast<float>(quat_.x());
        out_state.quat[2] = static_cast<float>(quat_.y());
        out_state.quat[3] = static_cast<float>(quat_.z());

        out_state.vel_ned[0] = static_cast<float>(vel_.x());
        out_state.vel_ned[1] = static_cast<float>(vel_.y());
        out_state.vel_ned[2] = static_cast<float>(vel_.z());

        out_state.pos_ned[0] = static_cast<float>(pos_.x());
        out_state.pos_ned[1] = static_cast<float>(pos_.y());
        out_state.pos_ned[2] = static_cast<float>(pos_.z());

        out_state.gyro_bias[0] = static_cast<float>(bg_.x());
        out_state.gyro_bias[1] = static_cast<float>(bg_.y());
        out_state.gyro_bias[2] = static_cast<float>(bg_.z());

        out_state.accel_bias[0] = static_cast<float>(ba_.x());
        out_state.accel_bias[1] = static_cast<float>(ba_.y());
        out_state.accel_bias[2] = static_cast<float>(ba_.z());

        out_state.timestamp_ns = last_timestamp_ns_;
        out_state.is_healthy = true;

        return true;
    }

private:
    // Skew-symmetric matrix from vector
    Eigen::Matrix3d skew(const Eigen::Vector3d& v) const {
        Eigen::Matrix3d s;
        s << 0.0, -v.z(), v.y(),
             v.z(), 0.0, -v.x(),
             -v.y(), v.x(), 0.0;
        return s;
    }

    // Inject 15-state error state dx into nominal state variables
    void inject_error_state(const Eigen::Matrix<double, 15, 1>& dx) {
        pos_ += dx.block<3, 1>(0, 0);
        vel_ += dx.block<3, 1>(3, 0);

        // Attitude error injection using small-angle approximation
        Eigen::Vector3d d_theta = dx.block<3, 1>(6, 0);
        double angle = d_theta.norm();
        if (angle > 1e-8) {
            Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, d_theta / angle));
            quat_ = (quat_ * dq).normalized();
        }

        bg_ += dx.block<3, 1>(9, 0);
        ba_ += dx.block<3, 1>(12, 0);
    }

private:
    // Nominal states
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Quaterniond quat_;
    Eigen::Vector3d bg_;
    Eigen::Vector3d ba_;

    // Covariances
    Eigen::Matrix<double, 15, 15> P_;
    Eigen::Matrix<double, 15, 15> Q_;

    // Measurement noise std dev (variances)
    double R_gps_pos_;
    double R_gps_vel_;
    double R_baro_;

    uint64_t last_timestamp_ns_ = 0;
    bool initialized_ = false;
};
