#pragma once
#include <iostream>
#include <chrono>
#include <vector>
#include <cstring>
#include <cmath>

class PerfStats {
public:
    PerfStats() {
        reset();
        last_report_time_ = std::chrono::steady_clock::now();
    }

    void setTargetHz(double hz) {
        target_period_ms_ = 1000.0 / hz;
    }

    void recordImuCall(double duration_ms) {
        imu_calls_++;
        updateHist(imu_hist_, duration_ms, target_period_ms_);
    }

    void recordValidImu(double dt_ms) {
        valid_imu_count_++;
        sum_imu_dt_ += dt_ms;
    }

    void recordPwmCall(double duration_ms) {
        pwm_calls_++;
        // PWM is fixed at 400Hz (2.5ms target)
        const double pwm_target_ms = 2.5; 
        updateHist(pwm_hist_, duration_ms, pwm_target_ms);
        pwm_send_count_++;
    }

    // 10초 주기로 통계 리포트 출력 및 리셋 (편차 감소를 위해)
    void updateAndReport() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report_time_).count();
        
        if (elapsed_ms >= 10000) {
            double rpc_hz = (double)imu_calls_ / (elapsed_ms / 1000.0);
            double valid_hz = (double)valid_imu_count_ / (elapsed_ms / 1000.0);
            double pwm_hz = (double)pwm_send_count_ / (elapsed_ms / 1000.0);
            double rtf = sum_imu_dt_ / (double)elapsed_ms;

            printf("[10s Stats] IMU: %.0f Hz(Valid:%.0f) | PWM: %.0f Hz | RTF: %.2f | IMU_Hist: [%d,%d,%d,%d,%d,%d] | PWM_Hist: [%d,%d,%d,%d,%d,%d]\n",
                   rpc_hz, valid_hz, pwm_hz, rtf,
                   imu_hist_[0], imu_hist_[1], imu_hist_[2], imu_hist_[3], imu_hist_[4], imu_hist_[5],
                   pwm_hist_[0], pwm_hist_[1], pwm_hist_[2], pwm_hist_[3], pwm_hist_[4], pwm_hist_[5]);

            reset();
            last_report_time_ = now;
        }
    }

protected:
    void reset() {
        imu_calls_ = 0;
        valid_imu_count_ = 0;
        pwm_send_count_ = 0;
        pwm_calls_ = 0;
        sum_imu_dt_ = 0;
        memset(imu_hist_, 0, sizeof(imu_hist_));
        memset(pwm_hist_, 0, sizeof(pwm_hist_));
    }

    void updateHist(int* hist, double duration_ms, double target_p_ms) {
        double p = target_p_ms;
        // Stats based on % delay of current target period
        if (duration_ms < p * 1.05) hist[0]++;      // < 5% Jitter
        else if (duration_ms < p * 1.10) hist[1]++; // < 10%
        else if (duration_ms < p * 1.15) hist[2]++; // < 15%
        else if (duration_ms < p * 1.20) hist[3]++; // < 20%
        else if (duration_ms < p * 1.30) hist[4]++; // < 30%
        else hist[5]++;                            // >= 30%
    }

    uint64_t imu_calls_ = 0;
    uint64_t valid_imu_count_ = 0;
    uint64_t pwm_send_count_ = 0;
    uint64_t pwm_calls_ = 0;
    double sum_imu_dt_ = 0;
    double target_period_ms_ = 1.0; // Default 1ms (1kHz)
    int imu_hist_[6];
    int pwm_hist_[6];
    std::chrono::steady_clock::time_point last_report_time_;
};
