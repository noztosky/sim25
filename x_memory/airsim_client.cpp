#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include "fc_ring.h"

namespace {
static const int IMU_TARGET_HZ = 1000; // 100MHz 테스트
static const int PWM_TARGET_HZ = 400;
static const int BARO_TARGET_HZ = 100;
}



class AirSimClient {
private:
    fc_ring fc_;
    bool is_connected_;
    int imu_count_;
    int pwm_count_;
    int baro_count_;
    std::atomic<int> pwm_tx_count_{0};
    // latest received values
    double last_roll_ = 0.0;
    double last_pitch_ = 0.0;
    double last_yaw_ = 0.0;
    int last_baro_cm_ = 0;
    std::atomic<int> imu_changed_count_{0};
    std::atomic<int> imu_total_count_{0};
    std::atomic<int> baro_changed_count_{0};
    int last_baro_frequency_ = -1;
    int last_pwm_r1_ = 1000;
    int last_pwm_r2_ = 1000;
    int last_pwm_r3_ = 1000;
    int last_pwm_r4_ = 1000;
    int last_imu_frequency_ = -1;

public:
    AirSimClient() : is_connected_(false), imu_count_(0), pwm_count_(0) {}
    
    ~AirSimClient() {
        disconnect();
    }

    bool connect() {
        // 연결
        std::cout << "Connecting..." << std::endl;
        if (!fc_.client_connect("AirSimSharedMemory")) {
            std::cerr << "Failed to connect to shared memory." << std::endl;
            return false;
        }
        std::cout << "Connected." << std::endl;

        is_connected_ = true;
        // Configure and start background workers inside library
        fc_.setup_rates(IMU_TARGET_HZ, PWM_TARGET_HZ, BARO_TARGET_HZ);
        fc_.set_client_callbacks(
            [this](const ImuData& d){
                this->imu_count_++;
                this->imu_total_count_.fetch_add(1, std::memory_order_relaxed);
                // Use frequency field as sequence counter for change detection
                if (d.frequency != this->last_imu_frequency_) {
                    this->imu_changed_count_.fetch_add(1, std::memory_order_relaxed);
                    this->last_imu_frequency_ = d.frequency;
                }
                this->last_roll_ = d.roll;
                this->last_pitch_ = d.pitch;
                this->last_yaw_ = d.yaw;
            },
            [this](const BaroData& b){ 
                this->last_baro_cm_ = static_cast<int>(std::lround(b.altitude * 100.0)); 
                this->baro_count_++;
                // Use frequency field as sequence counter for change detection
                if (b.frequency != this->last_baro_frequency_) {
                    this->baro_changed_count_.fetch_add(1, std::memory_order_relaxed);
                    this->last_baro_frequency_ = b.frequency;
                }
            }
        );
        fc_.start_client_workers();
        std::cout << "Connected to AirSim server successfully!" << std::endl;
        return true;
    }

    void disconnect() {
        is_connected_ = false;

        fc_.close();

        std::cout << "Disconnected from AirSim server." << std::endl;
    }

    // Shared Memory 방식에서는 직접 읽기/쓰기 메서드가 불필요
    // 데이터는 콜백을 통해 자동으로 처리됨

    PWMData generatePWMData() {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        return fc_ring::make_fixed_pwm(nanoseconds, pwm_count_);
    }

    

    void run() {
        if (!is_connected_) return;

        std::cout << "Start." << std::endl;

        auto start_time = std::chrono::high_resolution_clock::now();
        auto last_print_time = start_time;
        int print_count = 0;

        const auto pwm_interval = std::chrono::microseconds(1000000 / PWM_TARGET_HZ); // 400 Hz => 2500 us
        auto next_pwm_time = start_time + pwm_interval;
        auto last_pwm_bump = start_time;
        int base_pwm = 1000;

        while (is_connected_) {
            auto now = std::chrono::high_resolution_clock::now();
            
            // PWM 데이터 전송 (400Hz) - uniform pacing with sleep_until
            if (now >= next_pwm_time) {
                PWMData pwm_data = generatePWMData();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_pwm_bump).count() >= 1) {
                    if (base_pwm < 2000) base_pwm += 5;
                    last_pwm_bump = now;
                }
                int sec_elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count());
                int r1 = base_pwm, r2 = base_pwm, r3 = base_pwm, r4 = base_pwm;
                const int d_us = 20;
                if ((sec_elapsed % 2) == 0) {
                    // roll test: left vs right
                    r1 += d_us; r4 += d_us; // right side up
                    r2 -= d_us; r3 -= d_us; // left side down
                } else {
                    // pitch test: front vs rear
                    r1 += d_us; r3 += d_us; // front up
                    r2 -= d_us; r4 -= d_us; // rear down
                }
                pwm_data.rotor1 = r1;
                pwm_data.rotor2 = r2;
                pwm_data.rotor3 = r3;
                pwm_data.rotor4 = r4;
                fc_.client_submit_pwm(pwm_data);
                last_pwm_r1_ = r1; last_pwm_r2_ = r2; last_pwm_r3_ = r3; last_pwm_r4_ = r4;
                pwm_count_++;
                pwm_tx_count_.fetch_add(1, std::memory_order_relaxed);
                // cumulative advance; skip-late without burst
                next_pwm_time += pwm_interval;
                while (next_pwm_time + pwm_interval < now) next_pwm_time += pwm_interval;
            }

            // 1초 주기 정밀 계산(정확한 dt로 정규화)
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= 1) {
                print_count++;
                double dt_sec = std::chrono::duration<double>(now - last_print_time).count();
                int imu_samples = imu_changed_count_.exchange(0);
                int baro_samples = baro_changed_count_.exchange(0);
                int pwm_samples = pwm_tx_count_.exchange(0);
                double imu_frequency = dt_sec > 0 ? imu_samples / dt_sec : 0.0;
                double baro_frequency = dt_sec > 0 ? baro_samples / dt_sec : 0.0;
                double pwm_frequency = dt_sec > 0 ? pwm_samples / dt_sec : 0.0;
                
                auto norm360 = [](double a){ while (a < 0) a += 360.0; while (a >= 360.0) a -= 360.0; return a; };
                int r = static_cast<int>(std::lround(norm360(last_roll_)));
                int p = static_cast<int>(std::lround(norm360(last_pitch_)));
                int y = static_cast<int>(std::lround(norm360(last_yaw_)));
                std::cout << "[" << print_count << "s] IMU: " << std::fixed << std::setprecision(0) << imu_frequency
                          << " Hz (" << std::setw(3) << std::setfill('0') << r << " "
                          << std::setw(3) << std::setfill('0') << p << " "
                          << std::setw(3) << std::setfill('0') << y << "), BARO: "
                          << std::fixed << std::setprecision(0) << baro_frequency
                          << " Hz, PWM: " << std::fixed << std::setprecision(0) << pwm_frequency
                          << " Hz (" << last_pwm_r1_ << " " << last_pwm_r2_ << " " << last_pwm_r3_ << " " << last_pwm_r4_ << ")" << std::endl;
                
                last_print_time = now;
            }

            // 다음 이벤트까지 정확 대기 (min without macro issues)
            auto print_deadline = last_print_time + std::chrono::seconds(1);
            auto next_time = (next_pwm_time < print_deadline) ? next_pwm_time : print_deadline;
            std::this_thread::sleep_until(next_time);
        }
    }
private:
};

int main() {
    std::cout << "AirSim Client (Bidirectional Communication)" << std::endl;
    std::cout << "Version: 0.0.1" << std::endl;
    std::cout << "Build Date: " << __DATE__ << " " << __TIME__ << std::endl;
    std::cout << "===========================================" << std::endl;

    AirSimClient client;
    
    if (!client.connect()) {
        std::cout << "Failed to connect to server. Exiting..." << std::endl;
        return 1;
    }

    try {
        client.run();
    }
    catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    client.disconnect();
    return 0;
}
