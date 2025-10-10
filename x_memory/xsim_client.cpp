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
#include <atomic>
#include "x_xsim.h"

static const int PWM_TX_HZ = 400;

int main(){
    std::cout << "xsim_client" << std::endl;
    timeBeginPeriod(1);

    x_xsim xs;
    if(!xs.client_connect("AirSimXsim")){
        std::cerr << "xsim_client: connect failed" << std::endl;
        timeEndPeriod(1);
        return 1;
    }

    // consume telemetry
    std::atomic<bool> running{true};
    // last received values and counters for Hz
    XSimTelemetry last_telem{}; bool have_telem=false;
    std::atomic<uint64_t> imu_rx_cnt{0}, mag_rx_cnt{0}, baro_rx_cnt{0}, loc_rx_cnt{0};
    std::thread rx_thread([&]{
        uint32_t last_seq = 0;
        int seq_local=0;
        while(running.load()){
            xs.consume_telem(last_seq, [&](const XSimTelemetry& d){ last_telem = d; have_telem = true; imu_rx_cnt.fetch_add(1,std::memory_order_relaxed); if(((++seq_local)%10)==0){ mag_rx_cnt.fetch_add(1,std::memory_order_relaxed); baro_rx_cnt.fetch_add(1,std::memory_order_relaxed); loc_rx_cnt.fetch_add(1,std::memory_order_relaxed);} });
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // transmit pwm at 400 Hz
    auto period = std::chrono::microseconds(1000000/PWM_TX_HZ);
    auto start_time = std::chrono::high_resolution_clock::now();
    auto liftoff_time = start_time + std::chrono::seconds(3);
    auto next_tp = start_time + period;
    int seq=0; int base=1000; bool armed=false;

    auto last_print = std::chrono::high_resolution_clock::now();
    XSimPwm last{}; bool have=false;

    while(true){
        auto now = std::chrono::high_resolution_clock::now();
        if(now >= next_tp){
            if(!armed && now >= liftoff_time){ base=1600; armed=true; }
            XSimPwm p{}; 
            p.rotor1=p.rotor2=p.rotor3=base;
            p.rotor4=base + 1;
            p.timestamp=std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(); p.seq=++seq; p.is_valid=true; std::memset(p.padding,0,sizeof p.padding);
            xs.submit_pwm(p);
            last = p; have=true;
            next_tp += period;
        }

        if(std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 1){
            double dt = std::chrono::duration<double>(now - last_print).count();
            last_print = now;
            uint64_t imu_now = imu_rx_cnt.exchange(0);
            uint64_t mag_now = mag_rx_cnt.exchange(0);
            uint64_t baro_now = baro_rx_cnt.exchange(0);
            uint64_t loc_now = loc_rx_cnt.exchange(0);
            int imu_hz = static_cast<int>(std::lround(imu_now / dt));
            int mag_hz = static_cast<int>(std::lround(mag_now / dt));
            int baro_hz = static_cast<int>(std::lround(baro_now / dt));
            int loc_hz = static_cast<int>(std::lround(loc_now / dt));
            int pwm_hz = static_cast<int>(std::lround(xs.get_pwm_tx_hz()));

            std::cout << std::fixed << std::setprecision(0)
                      << "imu: " << (have_telem? last_telem.gyro[0]*57.3:0) << " " << (have_telem? last_telem.gyro[1]*57.3:0) << " " << (have_telem? last_telem.gyro[2]*57.3:0)
                      << " " << (have_telem? last_telem.acc[0]:0) << " " << (have_telem? last_telem.acc[1]:0) << " " << (have_telem? last_telem.acc[2]:0) << "(" << imu_hz << "hz), "
                      << "mag: " << (have_telem? last_telem.mag[0]:0) << " " << (have_telem? last_telem.mag[1]:0) << " " << (have_telem? last_telem.mag[2]:0) << " (" << mag_hz << "hz) "
                      << "baro: " << (have_telem? last_telem.alt:0) << " (" << baro_hz << "hz) "
                      << "loc: " << (have_telem? last_telem.loc_ned[0]:0) << " " << (have_telem? last_telem.loc_ned[1]:0) << " " << (have_telem? last_telem.loc_ned[2]:0) << " (" << loc_hz << "hz) "
                      << "pwm: " << (have? last.rotor1:0) << " " << (have? last.rotor2:0) << " " << (have? last.rotor3:0) << " " << (have? last.rotor4:0) << "  (" << pwm_hz << "hz)"
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    running.store(false);
    if(rx_thread.joinable()) rx_thread.join();
    timeEndPeriod(1);
    return 0;
}


