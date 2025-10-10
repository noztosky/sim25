#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include "x_xsim.h"

static const int TELEM_HZ = 1000;
static const int PWM_POLL_HZ = 400;

int main(){
    std::cout << "xsim_server" << std::endl;
    timeBeginPeriod(1);

    x_xsim xs;
    if(!xs.server_create("AirSimXsim")){
        std::cerr << "xsim_server: create failed" << std::endl;
        timeEndPeriod(1);
        return 1;
    }

    std::atomic<bool> running{true};
    // per-category tx counters (actual updates)
    std::atomic<uint64_t> imu_tx_cnt{0}, mag_tx_cnt{0}, baro_tx_cnt{0}, loc_tx_cnt{0};
    XSimTelemetry last_telem{};

    std::thread telem_thread([&]{
        auto period = std::chrono::microseconds(1000000/TELEM_HZ);
        auto next_tp = std::chrono::high_resolution_clock::now() + period;
        int seq = 0;
        while(running.load()){
            std::this_thread::sleep_until(next_tp);
            next_tp += period;

            auto now = std::chrono::high_resolution_clock::now();
            long long ts = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

            // simple synthetic data
            double t = seq * (1.0/TELEM_HZ);
            XSimTelemetry d{};
            d.acc[0] = std::sin(t); d.acc[1] = std::cos(t*0.5); d.acc[2] = 9.81;
            d.gyro[0] = 0.01; d.gyro[1] = 0.002; d.gyro[2] = 0.0005;
            // unit quaternion (no rotation)
            d.quat[0] = 1.0f; d.quat[1] = 0.0f; d.quat[2] = 0.0f; d.quat[3] = 0.0f;
            // update loc/mag/alt at ~100 Hz (every 10 samples)
            if ((seq % 10) == 0) {
                double tt = seq * (1.0/TELEM_HZ);
                d.loc_ned[0] = 0.1*tt; d.loc_ned[1] = 0.0; d.loc_ned[2] = 0.0;
                d.alt = 0.0;
                d.mag[0] = 1.0; d.mag[1] = 0.0; d.mag[2] = 0.0;
            } else {
                // keep last values
                d.loc_ned[0] = last_telem.loc_ned[0]; d.loc_ned[1] = last_telem.loc_ned[1]; d.loc_ned[2] = last_telem.loc_ned[2];
                d.alt = last_telem.alt;
                d.mag[0] = last_telem.mag[0]; d.mag[1] = last_telem.mag[1]; d.mag[2] = last_telem.mag[2];
            }
            d.timestamp = ts;
            d.seq = ++seq;
            d.is_valid = true;
            std::memset(d.padding, 0, sizeof d.padding);
            xs.publish_telem(d);
            // counters
            imu_tx_cnt.fetch_add(1, std::memory_order_relaxed);
            if ((seq % 10) == 0) {
                mag_tx_cnt.fetch_add(1, std::memory_order_relaxed);
                baro_tx_cnt.fetch_add(1, std::memory_order_relaxed);
                loc_tx_cnt.fetch_add(1, std::memory_order_relaxed);
            }
            last_telem = d;
        }
    });

    uint32_t last_pwm_seq = 0;
    auto pwm_period = std::chrono::microseconds(1000000/PWM_POLL_HZ);
    auto last_print = std::chrono::high_resolution_clock::now();
    // window counters
    uint64_t imu_tx_win=0, mag_tx_win=0, baro_tx_win=0, loc_tx_win=0, pwm_rx_win=0;
    XSimPwm last_pwm{}; bool have_pwm=false;

    while(true){
        auto now = std::chrono::high_resolution_clock::now();
        static auto next_poll = now;
        if(now >= next_poll){
            XSimPwm p{};
            if(xs.try_get_pwm(last_pwm_seq, p)){
                last_pwm = p; have_pwm = true; ++pwm_rx_win;
            }
            next_poll += pwm_period;
        }

        if(std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 1){
            double dt = std::chrono::duration<double>(now - last_print).count();
            last_print = now;
            // snapshot and reset windows
            uint64_t imu_tx_now = imu_tx_cnt.exchange(0);
            uint64_t mag_tx_now = mag_tx_cnt.exchange(0);
            uint64_t baro_tx_now = baro_tx_cnt.exchange(0);
            uint64_t loc_tx_now = loc_tx_cnt.exchange(0);
            // use local per-second dequeued count
            int imu_hz = static_cast<int>(std::lround(imu_tx_now / dt));
            int mag_hz = static_cast<int>(std::lround(mag_tx_now / dt));
            int baro_hz = static_cast<int>(std::lround(baro_tx_now / dt));
            int loc_hz = static_cast<int>(std::lround(loc_tx_now / dt));
            int pwm_hz = static_cast<int>(std::lround(pwm_rx_win / dt));

            std::cout << std::fixed << std::setprecision(0)
                      << "imu: " << (last_telem.gyro[0]*57.3) << " " << (last_telem.gyro[1]*57.3) << " " << (last_telem.gyro[2]*57.3)
                      << " " << last_telem.acc[0] << " " << last_telem.acc[1] << " " << last_telem.acc[2] << "(" << imu_hz << "hz), "
                      << "mag: " << last_telem.mag[0] << " " << last_telem.mag[1] << " " << last_telem.mag[2] << " (" << mag_hz << "hz) "
                      << "baro: " << last_telem.alt << " (" << baro_hz << "hz) "
                      << "loc: " << last_telem.loc_ned[0] << " " << last_telem.loc_ned[1] << " " << last_telem.loc_ned[2] << " (" << loc_hz << "hz) "
                      << "pwm: " << (have_pwm? last_pwm.rotor1:0) << " " << (have_pwm? last_pwm.rotor2:0) << " " << (have_pwm? last_pwm.rotor3:0) << " " << (have_pwm? last_pwm.rotor4:0)
                      << " (" << pwm_hz << "hz)"
                      << std::endl;
            pwm_rx_win = 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    running.store(false);
    if(telem_thread.joinable()) telem_thread.join();
    timeEndPeriod(1);
    return 0;
}


