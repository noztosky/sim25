#pragma once
#include "../../core/EST_IF.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

class EST_EKF24 : public EST_IF {
public:
    EST_EKF24() {
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
        mag_earth_.setZero();
        mag_bias_.setZero();
        wind_.setZero();
        terrain_ = 0.0;

        // Initialize covariance P_ (24x24)
        P_.setZero();
        P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.1;    // Position variance
        P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.1;    // Velocity variance
        P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.01;   // Attitude variance (rad^2)
        P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-6;   // Gyro bias variance
        P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-4; // Accel bias variance
        P_.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() * 1.0;  // Earth magnetic field variance
        P_.block<3, 3>(18, 18) = Eigen::Matrix3d::Identity() * 0.1;  // Body magnetic bias variance
        P_.block<2, 2>(21, 21) = Eigen::Matrix2d::Identity() * 0.25; // Wind velocity variance
        P_(23, 23) = 1.0;                                            // Terrain offset variance

        // Process noise Q_ (24x24)
        double sigma_acc = 0.15;        // m/s^2/sqrt(Hz)
        double sigma_gyro = 0.005;      // rad/s/sqrt(Hz)
        double sigma_bias_gyro = 1e-5;  // rad/s^2/sqrt(Hz)
        double sigma_bias_acc = 1e-3;   // m/s^3/sqrt(Hz)
        double sigma_mag_earth = 1e-4;  // uT/sqrt(Hz)
        double sigma_mag_bias = 1e-4;   // uT/sqrt(Hz)
        double sigma_wind = 1e-2;       // m/s/sqrt(Hz)
        double sigma_terrain = 1e-2;    // m/sqrt(Hz)

        Q_.setZero();
        Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (sigma_acc * sigma_acc);
        Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (sigma_gyro * sigma_gyro);
        Q_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (sigma_bias_gyro * sigma_bias_gyro);
        Q_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (sigma_bias_acc * sigma_bias_acc);
        Q_.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() * (sigma_mag_earth * sigma_mag_earth);
        Q_.block<3, 3>(18, 18) = Eigen::Matrix3d::Identity() * (sigma_mag_bias * sigma_mag_bias);
        Q_.block<2, 2>(21, 21) = Eigen::Matrix2d::Identity() * (sigma_wind * sigma_wind);
        Q_(23, 23) = sigma_terrain * sigma_terrain;

        // Measurement noises
        R_gps_pos_ = 0.25; // m^2
        R_gps_vel_ = 0.04; // (m/s)^2
        R_baro_ = 0.64;    // m^2
        R_mag_ = 2.25;     // uT^2

        last_timestamp_ns_ = 0;
        initialized_ = false;
        mag_initialized_ = false;
    }

    void update_imu(const IMUData& imu_data) override {
        if (last_timestamp_ns_ == 0) {
            last_timestamp_ns_ = imu_data.timestamp_ns;
            initialized_ = true;
            return;
        }

        double dt = static_cast<double>(imu_data.timestamp_ns - last_timestamp_ns_) * 1e-9;
        last_timestamp_ns_ = imu_data.timestamp_ns;

        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.000125;
        }

        // Unbias IMU measurements
        Eigen::Vector3d f_m(imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
        Eigen::Vector3d w_m(imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);

        Eigen::Vector3d f = f_m - ba_;
        Eigen::Vector3d w = w_m - bg_;

        const Eigen::Vector3d g(0.0, 0.0, 9.80665);
        Eigen::Matrix3d R = quat_.toRotationMatrix();
        Eigen::Vector3d a_ned = R * f + g;

        // 1. Nominal State Propagation
        pos_ += vel_ * dt + 0.5 * a_ned * dt * dt;
        vel_ += a_ned * dt;

        Eigen::Vector3d d_theta = w * dt;
        double angle = d_theta.norm();
        if (angle > 1e-8) {
            Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, d_theta / angle));
            quat_ = (quat_ * dq).normalized();
        }

        // Initialize Earth magnetic field if not done yet
        if (!mag_initialized_ && (imu_data.mag[0] != 0.0 || imu_data.mag[1] != 0.0 || imu_data.mag[2] != 0.0)) {
            Eigen::Vector3d mag_m(imu_data.mag[0], imu_data.mag[1], imu_data.mag[2]);
            mag_earth_ = R * mag_m;
            mag_initialized_ = true;
        }

        // 2. Covariance Propagation (24x24)
        Eigen::Matrix<double, 24, 24> F_sys = Eigen::Matrix<double, 24, 24>::Identity();
        
        F_sys.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F_sys.block<3, 3>(3, 6) = -R * skew(f) * dt;
        F_sys.block<3, 3>(3, 12) = -R * dt;
        F_sys.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew(w) * dt;
        F_sys.block<3, 3>(6, 9) = Eigen::Matrix3d::Identity() * (-dt);

        P_ = F_sys * P_ * F_sys.transpose() + Q_ * dt;
    }

    void update_gnss(const GNSSData& gnss_data) override {
        if (!initialized_) return;

        Eigen::Matrix<double, 6, 1> z;
        z << gnss_data.pos_ned[0], gnss_data.pos_ned[1], gnss_data.pos_ned[2],
             gnss_data.vel_ned[0], gnss_data.vel_ned[1], gnss_data.vel_ned[2];

        Eigen::Matrix<double, 6, 1> h_x;
        h_x << pos_, vel_;

        Eigen::Matrix<double, 6, 1> innovation = z - h_x;

        Eigen::Matrix<double, 6, 24> H = Eigen::Matrix<double, 6, 24>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
        R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * R_gps_pos_;
        R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * R_gps_vel_;

        Eigen::Matrix<double, 24, 6> PH_T = P_ * H.transpose();
        Eigen::Matrix<double, 6, 6> S = H * PH_T + R;
        Eigen::Matrix<double, 24, 6> K = PH_T * S.inverse();

        Eigen::Matrix<double, 24, 1> dx = K * innovation;

        P_ = (Eigen::Matrix<double, 24, 24>::Identity() - K * H) * P_;

        inject_error_state(dx);
    }

    void update_baro(const BAROData& baro_data) override {
        if (!initialized_) return;

        double z = baro_data.altitude;
        double h_x = -pos_.z();
        double innovation = z - h_x;

        Eigen::Matrix<double, 1, 24> H = Eigen::Matrix<double, 1, 24>::Zero();
        H(0, 2) = -1.0;

        double R = R_baro_;

        Eigen::Matrix<double, 24, 1> PH_T = P_ * H.transpose();
        double S = H * PH_T + R;
        Eigen::Matrix<double, 24, 1> K = PH_T / S;

        Eigen::Matrix<double, 24, 1> dx = K * innovation;

        P_ = (Eigen::Matrix<double, 24, 24>::Identity() - K * H) * P_;

        inject_error_state(dx);
    }

    void update_mag(const double mag[3]) override {
        if (!initialized_ || !mag_initialized_) return;

        // Measurement z = raw magnetometer values (3x1)
        Eigen::Vector3d z(mag[0], mag[1], mag[2]);

        // Predicted measurement h(x) = R^T * mag_earth_ + mag_bias_
        Eigen::Matrix3d R = quat_.toRotationMatrix();
        Eigen::Vector3d mag_body_pred = R.transpose() * mag_earth_;
        Eigen::Vector3d h_x = mag_body_pred + mag_bias_;

        Eigen::Vector3d innovation = z - h_x;

        // Measurement Jacobian H (3x24)
        Eigen::Matrix<double, 3, 24> H = Eigen::Matrix<double, 3, 24>::Zero();
        
        // H w.r.t attitude error (states 6..8) = skew(mag_body_pred)
        H.block<3, 3>(0, 6) = skew(mag_body_pred);
        
        // H w.r.t Earth magnetic field (states 15..17) = R.transpose()
        H.block<3, 3>(0, 15) = R.transpose();
        
        // H w.r.t Body magnetic bias (states 18..20) = I
        H.block<3, 3>(0, 18) = Eigen::Matrix3d::Identity();

        // Measurement noise covariance
        Eigen::Matrix3d R_cov = Eigen::Matrix3d::Identity() * R_mag_;

        // Kalman update
        Eigen::Matrix<double, 24, 3> PH_T = P_ * H.transpose();
        Eigen::Matrix3d S = H * PH_T + R_cov;
        Eigen::Matrix<double, 24, 3> K = PH_T * S.inverse();

        Eigen::Matrix<double, 24, 1> dx = K * innovation;

        // Decouple magnetometer updates from roll and pitch errors (states 6 and 7)
        dx(6) = 0.0;
        dx(7) = 0.0;

        P_ = (Eigen::Matrix<double, 24, 24>::Identity() - K * H) * P_;

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
    Eigen::Matrix3d skew(const Eigen::Vector3d& v) const {
        Eigen::Matrix3d s;
        s << 0.0, -v.z(), v.y(),
             v.z(), 0.0, -v.x(),
             -v.y(), v.x(), 0.0;
        return s;
    }

    void inject_error_state(const Eigen::Matrix<double, 24, 1>& dx) {
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
        mag_earth_ += dx.block<3, 1>(15, 0);
        mag_bias_ += dx.block<3, 1>(18, 0);
        wind_ += dx.block<2, 1>(21, 0);
        terrain_ += dx(23);
    }

private:
    // Nominal states
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Quaterniond quat_;
    Eigen::Vector3d bg_;
    Eigen::Vector3d ba_;
    Eigen::Vector3d mag_earth_;
    Eigen::Vector3d mag_bias_;
    Eigen::Vector2d wind_;
    double terrain_;

    // Covariances
    Eigen::Matrix<double, 24, 24> P_;
    Eigen::Matrix<double, 24, 24> Q_;

    // Measurement noise parameters
    double R_gps_pos_;
    double R_gps_vel_;
    double R_baro_;
    double R_mag_;

    uint64_t last_timestamp_ns_ = 0;
    bool initialized_ = false;
    bool mag_initialized_ = false;
};
