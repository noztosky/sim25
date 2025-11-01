// RPC client (single-thread scheduler):
// - IMU: max-speed polling between deadlines
// - Baro/Mag/PWM: fixed rates (default 400 Hz)
// Reports per-second achieved rates for IMU (poll/unique), Baro/Mag (poll), PWM (tx).

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <algorithm>
#include <cstdint>

int main(int argc, char** argv)
{
    using namespace msr::airlib;
    using clock = std::chrono::steady_clock;

    // Optional args: [ip] [port] [seconds]
    std::string ip = (argc >= 2) ? argv[1] : std::string("127.0.0.1");
    int port = (argc >= 3) ? std::atoi(argv[2]) : 41451;
    int duration_sec = (argc >= 4) ? std::atoi(argv[3]) : 10;
    const double pwm_hz = 400.0;
    const double baro_hz = 100.0;
    const double mag_hz  = 100.0;
    const auto   pwm_period  = std::chrono::duration<double>(1.0 / pwm_hz);
    const auto   baro_period = std::chrono::duration<double>(1.0 / baro_hz);
    const auto   mag_period  = std::chrono::duration<double>(1.0 / mag_hz);

    try {
        // Single RPC client and scheduler
        MultirotorRpcLibClient client(ip, port, 0.5f);
        client.confirmConnection();
        client.enableApiControl(true);
        client.armDisarm(true);

        const auto t_begin = clock::now();
        auto last_report = t_begin;
        auto next_report = t_begin + std::chrono::seconds(1);

        // Next deadlines
        auto next_baro = t_begin + std::chrono::duration_cast<clock::duration>(baro_period);
        auto next_mag  = t_begin + std::chrono::duration_cast<clock::duration>(mag_period);
        auto next_pwm  = t_begin + std::chrono::duration_cast<clock::duration>(pwm_period);

        // Report-window counters
        uint64_t imu_poll_cnt = 0, imu_unique_cnt = 0;
        uint64_t baro_poll_cnt = 0;
        uint64_t mag_poll_cnt  = 0;
        uint64_t pwm_tx_cnt    = 0;
        uint64_t last_imu_ts   = 0;

        const float pwm_cmd_dur = static_cast<float>(1.0 / pwm_hz);

        while (std::chrono::duration_cast<std::chrono::seconds>(clock::now() - t_begin).count() < duration_sec) {
            auto now = clock::now();

            // 1) Run any due periodic tasks (at most one iteration per loop each)
            if (now >= next_baro) {
                (void)client.getBarometerData();
                ++baro_poll_cnt;
                next_baro += std::chrono::duration_cast<clock::duration>(baro_period);
                continue; // give chance to other due tasks in next loop
            }
            if (now >= next_mag) {
                (void)client.getMagnetometerData();
                ++mag_poll_cnt;
                next_mag += std::chrono::duration_cast<clock::duration>(mag_period);
                continue;
            }
            if (now >= next_pwm) {
                client.moveByMotorPWMsAsync(1000.0f, 1000.0f, 1000.0f, 1000.0f, pwm_cmd_dur);
                ++pwm_tx_cnt;
                next_pwm += std::chrono::duration_cast<clock::duration>(pwm_period);
                continue;
            }

            // 2) Reporting each second
            if (now >= next_report) {
                const double sec = std::chrono::duration<double>(now - last_report).count();
                last_report = now;
                next_report += std::chrono::seconds(1);

                double imu_poll_hz = imu_poll_cnt / sec;
                double imu_uniq_hz = imu_unique_cnt / sec;
                double baro_poll_hz = baro_poll_cnt / sec;
                double mag_poll_hz = mag_poll_cnt / sec;
                double pwm_tx_hz = pwm_tx_cnt / sec;

                std::cout << "IMU: poll " << imu_poll_hz << " Hz, unique " << imu_uniq_hz
                          << " Hz | BARO: poll " << baro_poll_hz
                          << " Hz | MAG: poll " << mag_poll_hz
                          << " Hz | PWM tx: " << pwm_tx_hz << " Hz"
                          << std::endl;

                imu_poll_cnt = imu_unique_cnt = 0;
                baro_poll_cnt = mag_poll_cnt = pwm_tx_cnt = 0;
                continue;
            }

            // 3) Otherwise, use available slack time to poll IMU as fast as possible
            //    until the earliest upcoming deadline arrives.
            auto next_soonest = std::min({next_baro, next_mag, next_pwm, next_report});
            // Do at least one IMU poll; break if we hit the deadline
            while (true) {
                auto imu = client.getImuData();
                ++imu_poll_cnt;
                uint64_t ts = static_cast<uint64_t>(imu.time_stamp);
                if (ts != last_imu_ts) { last_imu_ts = ts; ++imu_unique_cnt; }
                if (clock::now() >= next_soonest) break;
            }
        }

        std::cout << "---\nDone." << std::endl;
    }
    catch (rpc::rpc_error& e) {
        const auto msg = e.get_error().as<std::string>();
        std::cerr << "RPC exception: " << msg << std::endl;
        return 2;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
