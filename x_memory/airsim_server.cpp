#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include "fc_memory.h"

namespace {
static const int IMU_TARGET_HZ = 1000; // 1000Hz 테스트
static const int PWM_TARGET_HZ = 400;
static const int BARO_TARGET_HZ = 100;
}

// ImuData, PWMData are provided by fc_pipeline.h

class AirSimServer {
private:
    fc_memory fc_;
    bool is_running_;
    int imu_count_;
    int pwm_count_;
    int baro_count_;
    bool metrics_started_;
    std::chrono::high_resolution_clock::time_point start_time_;
    // angle state updated every 1s
    double roll_deg_{0.0}, pitch_deg_{0.0}, yaw_deg_{0.0};
    std::chrono::high_resolution_clock::time_point last_angle_update_;
    // last received PWM snapshot
    PWMData last_pwm_{};
    bool last_pwm_valid_{false};

public:
    AirSimServer() : is_running_(false), imu_count_(0), pwm_count_(0), baro_count_(0), metrics_started_(false) {}
    
    ~AirSimServer() {
        stop();
    }

    bool start() {
        if (!fc_.server_create("AirSimSharedMemory")) {
            std::cerr << "Failed to create shared memory. Error: " << GetLastError() << std::endl;
            return false;
        }

        std::cout << "Server started. Shared memory ready." << std::endl;
        // improve timer granularity for tighter scheduling
        timeBeginPeriod(1);
        // Configure and start background workers inside library
        fc_.setup_rates(IMU_TARGET_HZ, PWM_TARGET_HZ, BARO_TARGET_HZ);
        fc_.start_server_workers();
        // Non-blocking; proceed to run loop
        is_running_ = true;
        start_time_ = std::chrono::high_resolution_clock::now();
        metrics_started_ = true; // Shared memory is always ready
        last_angle_update_ = start_time_;
        
        return true;
    }

    void stop() {
        is_running_ = false;
        cleanup();
        timeEndPeriod(1);
        std::cout << "AirSim Server stopped." << std::endl;
    }

    void cleanup() {
        fc_.close();
    }

    void run() {
        if (!is_running_) return;

        
        auto last_print_time = std::chrono::high_resolution_clock::now();
        int print_count = 0;

        // Publish intervals (workers handle actual I/O)
        const auto imu_interval = std::chrono::microseconds(1000000 / IMU_TARGET_HZ);
        auto next_imu_time = start_time_ + imu_interval;
        const auto pwm_poll_interval = std::chrono::microseconds(1000000 / PWM_TARGET_HZ);
        auto next_pwm_poll_time = start_time_;
        const auto baro_interval = std::chrono::microseconds(1000000 / BARO_TARGET_HZ);
        auto next_baro_time = start_time_;

        auto last_conn_dot = start_time_;
        bool newline_after_connect = false;
        
        while (is_running_) {
            auto now = std::chrono::high_resolution_clock::now();
            
            // IMU publish with catch-up scheduler
            {
                int safety = 0;
                while (now >= next_imu_time && safety < 5) {
                    double dt = std::chrono::duration<double>(now - last_angle_update_).count();
                    if (dt < 0.0) dt = 0.0;
                    if (dt > 0.05) dt = 0.05; // avoid huge steps when paused

                    // simple test increments
                    roll_deg_  += 0.01;
                    pitch_deg_ += 0.001;
                    yaw_deg_   += 0.0001;
                    if (roll_deg_  >= 360.0) roll_deg_  -= 360.0;
                    if (pitch_deg_ >= 360.0) pitch_deg_ -= 360.0;
                    if (yaw_deg_   >= 360.0) yaw_deg_   -= 360.0;
                    last_angle_update_ = now;

                    ImuData imu_data = generateImuData();
                    fc_.publish_imu(imu_data);
                    imu_count_++;

                    next_imu_time += imu_interval;
                    safety++;
                }
                // if we fell too far behind, re-sync to avoid long catch-up burst
                if (safety == 5 && now > next_imu_time + 10 * imu_interval) {
                    next_imu_time = now + imu_interval;
                }
            }

            // PWM 데이터 수신 폴링 (Shared Memory에서 직접 읽기)
            if (now >= next_pwm_poll_time) {
                PWMData pwm_data;
                if (fc_.try_get_pwm(pwm_data)) {
                    pwm_count_++;
                    last_pwm_ = pwm_data;
                    last_pwm_valid_ = true;
                    processPWMData(pwm_data);
                }
                next_pwm_poll_time += pwm_poll_interval;
            }

            // BARO 데이터 publish (100Hz) - Shared Memory 직접 쓰기
            maybeSendBaro(now, next_baro_time, baro_interval);

            // 1초마다 상태 출력 (라이브러리의 1초 윈도우 기반 값)
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= 1 && metrics_started_) {
                print_count++;
                double imu_frequency = fc_.get_imu_tx_hz();
                double pwm_frequency = fc_.get_pwm_rx_hz();
                double baro_frequency = fc_.get_baro_tx_hz();
                
                int r = static_cast<int>(std::round(roll_deg_)); if (r<0) r+=360; r%=360;
                int p = static_cast<int>(std::round(pitch_deg_)); if (p<0) p+=360; p%=360;
                int y = static_cast<int>(std::round(yaw_deg_)); if (y<0) y+=360; y%=360;
                std::cout << "[" << print_count << "s] ";
                std::cout << "IMU: " << std::fixed << std::setprecision(0) << imu_frequency << " Hz ("
                          << std::setw(3) << std::setfill('0') << r << " "
                          << std::setw(3) << std::setfill('0') << p << " "
                          << std::setw(3) << std::setfill('0') << y << ") [cnt:" << imu_count_ << "], ";
                std::cout << "BARO: " << std::fixed << std::setprecision(0) << baro_frequency << " Hz, ";
                std::cout << "PWM: " << std::fixed << std::setprecision(0) << pwm_frequency << " Hz ";
                if (last_pwm_valid_) {
                    std::cout << "(" << last_pwm_.rotor1 << " " << last_pwm_.rotor2 << " " << last_pwm_.rotor3 << " " << last_pwm_.rotor4 << ")";
                }
                std::cout << std::endl;
                
                last_print_time = now;
            }

            // 안전한 타이밍 제어
            auto next_time = next_imu_time;
            if (next_pwm_poll_time < next_time) next_time = next_pwm_poll_time;
            if (next_baro_time < next_time) next_time = next_baro_time;
            auto current_time = std::chrono::high_resolution_clock::now();
            
            if (current_time < next_time) {
                auto sleep_duration = next_time - current_time;
                if (sleep_duration > std::chrono::microseconds(0)) {
                    std::this_thread::sleep_for(sleep_duration);
                }
            }
        }
    }

private:
    ImuData generateImuData() {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        ImuData d{};
        d.roll = roll_deg_;
        d.pitch = pitch_deg_;
        d.yaw = yaw_deg_;
        d.timestamp = nanoseconds;
        d.frequency = imu_count_;
        d.is_valid = true;
        std::memset(d.padding, 0, sizeof d.padding);
        return d;
    }

    void maybeSendBaro(const std::chrono::high_resolution_clock::time_point& now,
                       std::chrono::high_resolution_clock::time_point& next_baro_time,
                       const std::chrono::microseconds& baro_interval) {
        if (now >= next_baro_time) {
            auto tp = std::chrono::high_resolution_clock::now();
            auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
            BaroData b = fc_memory::make_fixed_baro(ns, baro_count_);
            fc_.publish_baro(b);
            baro_count_++;
            next_baro_time += baro_interval;
        }
    }

    void processPWMData(const PWMData& pwm) {
        // 여기에 실제 드론 제어 로직 구현
        // 예: AirSim API 호출
        /*
        if (pwm.channel1 > 1100) {
            // 롤 오른쪽
        } else if (pwm.channel1 < 900) {
            // 롤 왼쪽
        }
        
        if (pwm.channel2 > 1100) {
            // 피치 앞으로
        } else if (pwm.channel2 < 900) {
            // 피치 뒤로
        }
        
        if (pwm.channel3 > 1100) {
            // 스로틀 증가
        } else if (pwm.channel3 < 900) {
            // 스로틀 감소
        }
        
        if (pwm.channel4 > 1100) {
            // 요 오른쪽
        } else if (pwm.channel4 < 900) {
            // 요 왼쪽
        }
        */
    }
};

int main() {
    std::cout << "AirSim Server (Bidirectional Communication)" << std::endl;
    std::cout << "Version: 0.0.1" << std::endl;
    std::cout << "Build Date: " << __DATE__ << " " << __TIME__ << std::endl;
    std::cout << "===========================================" << std::endl;

    AirSimServer server;
    
    if (!server.start()) {
        std::cout << "Failed to start server. Exiting..." << std::endl;
        return 1;
    }

    try {
        server.run();
    }
    catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    server.stop();
    return 0;
}
