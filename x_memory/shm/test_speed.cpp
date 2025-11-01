#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include "lib/XSimIo.hpp"

int main(int argc, char** argv)
{
    using clock = std::chrono::steady_clock;

    // Args: [seconds]
    int duration_sec = (argc >= 2) ? std::atoi(argv[1]) : 10;

    // Target rates
    const double pwm_hz  = 400.0;
    const double baro_hz = 100.0;
    const double mag_hz  = 100.0;
    const auto   pwm_period  = std::chrono::duration<double>(1.0 / pwm_hz);
    const auto   baro_period = std::chrono::duration<double>(1.0 / baro_hz);
    const auto   mag_period  = std::chrono::duration<double>(1.0 / mag_hz);

    timeBeginPeriod(1);

    XSimIo io;
    if (!io.connect("AirSimXsim")) {
        std::cerr << "[shm] connect failed" << std::endl;
        timeEndPeriod(1);
        return 1;
    }

    const auto t_begin = clock::now();
    auto last_report = t_begin;
    auto next_report = t_begin + std::chrono::seconds(1);

    // Next deadlines for periodic tasks
    auto next_baro = t_begin + std::chrono::duration_cast<clock::duration>(baro_period);
    auto next_mag  = t_begin + std::chrono::duration_cast<clock::duration>(mag_period);
    auto next_pwm  = t_begin + std::chrono::duration_cast<clock::duration>(pwm_period);

    // Per-second counters
    uint64_t imu_unique_cnt = 0;
    uint64_t baro_poll_cnt  = 0;
    uint64_t mag_poll_cnt   = 0;
    uint64_t pwm_tx_cnt     = 0;

    // Telemetry consume state
    uint32_t last_seq = 0;
    XSimTelemetry last_telem{}; bool have_telem = false;

    auto ns_now = [](){ return std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now().time_since_epoch()).count(); };

    while (std::chrono::duration_cast<std::chrono::seconds>(clock::now() - t_begin).count() < duration_sec) {
        auto now = clock::now();

        // 1) Run any due periodic tasks
        if (now >= next_baro) {
            // Read latest altitude from last telemetry snapshot (no RPC; SHM is push-based)
            ++baro_poll_cnt;
            next_baro += std::chrono::duration_cast<clock::duration>(baro_period);
            continue;
        }
        if (now >= next_mag) {
            // Read latest magnetometer from last telemetry snapshot
            ++mag_poll_cnt;
            next_mag += std::chrono::duration_cast<clock::duration>(mag_period);
            continue;
        }
        if (now >= next_pwm) {
            // Transmit safe PWM at 400 Hz
            XSimPwm p{}; p.rotor1 = p.rotor2 = p.rotor3 = p.rotor4 = 1000; p.timestamp = ns_now(); p.seq = 0; p.is_valid = true;
            io.submitPwm(p);
            ++pwm_tx_cnt;
            next_pwm += std::chrono::duration_cast<clock::duration>(pwm_period);
            continue;
        }

        // 2) Reporting (1 Hz)
        if (now >= next_report) {
            const double sec = std::chrono::duration<double>(now - last_report).count();
            last_report = now;
            next_report += std::chrono::seconds(1);

            double imu_uniq_hz = imu_unique_cnt / sec;
            double baro_poll_hz = baro_poll_cnt / sec;
            double mag_poll_hz  = mag_poll_cnt / sec;
            double pwm_tx_hz    = pwm_tx_cnt / sec;

            std::cout.setf(std::ios::fixed); std::cout.precision(3);
            std::cout << "SHM | IMU unique: " << imu_uniq_hz
                      << " Hz | BARO poll: " << baro_poll_hz
                      << " Hz | MAG poll: " << mag_poll_hz
                      << " Hz | PWM tx: " << pwm_tx_hz
                      << " Hz" << std::endl;

            imu_unique_cnt = baro_poll_cnt = mag_poll_cnt = pwm_tx_cnt = 0;
            continue;
        }

        // 3) Slack: drain incoming telemetry as fast as available
        size_t consumed = 0;
        io.handle().consume_telem(last_seq, [&](const XSimTelemetry& d){ last_telem = d; have_telem = true; ++imu_unique_cnt; ++consumed; });
        if (consumed == 0) {
            // Yield briefly to avoid busy spin if no new telemetry
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    }

    timeEndPeriod(1);
    return 0;
}


