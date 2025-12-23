#include <iostream>
#include <windows.h>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <cstring>

#include "core/Driver_IF.hpp"
#include "core/IMU_IF.hpp"
#include "core/ATT_IF.hpp"
#include "core/ACT_IF.hpp"

#include "drivers/rpc/RPC_Driver.hpp"
#include "modules/imu/IMU_SimRPC.hpp"
#include "modules/actuator/ACT_SimRPC.hpp"

#include "drivers/shm/SHM_Driver.hpp"
#include "modules/imu/IMU_SimSHM.hpp"
#include "modules/actuator/ACT_SimSHM.hpp"

#include "modules/attitude/ATT_Mahony.hpp"
#include "core/PerfStats.hpp"

#include <string>

#pragma comment(lib, "winmm.lib")

int main(int argc, char* argv[]) {
    timeBeginPeriod(1);

    double target_hz = 8000.0;
    if (argc > 1) {
        target_hz = std::stod(argv[1]);
    }
    
    // --- [Option 1] RPC Mode ---
    /*
    printf("[SIL_App] Mode: RPC\n");
    RPC_Driver driver("127.0.0.1", 41451);
    IMU_SimRPC imu(driver);
    ACT_SimRPC act(driver);
    auto& target_driver = driver;
    */

    // --- [Option 2] SHM Mode ---
    printf("[SIL_App] Mode: SHM\n");
    SHM_Driver driver("AirSimXsim");
    IMU_SimSHM imu(driver);
    ACT_SimSHM act(driver);
    auto& target_driver = driver;

    ATT_Mahony att((float)target_hz); 

    if (!target_driver.connect()) {
        std::cerr << "[SIL_App] Failed to connect!" << std::endl;
        return 1;
    }

    imu.init();
    att.init();
    act.init(); // This will enable API control and arm the drone

    // Target intervals in Nanoseconds
    const uint64_t IMU_TARGET_NS = (uint64_t)(1000000000.0 / target_hz);
    const uint64_t PWM_TARGET_NS = 2500000;   // 400 Hz (2.5ms)

    IMUData imu_data;
    PWMData pwm_data;
    pwm_data.pwm_values = { 0.5f, 0.5f, 0.5f, 0.5f }; // Neutral

    uint64_t last_imu_ts = 0;
    uint64_t last_pwm_send_ts = 0;

    // Statistics via PerfStats class
    PerfStats stats;
    stats.setTargetHz(target_hz);

    std::cout << "[SIL_App] Loop starting..." << std::endl;
    std::cout << "   Hist (Delay%): [0]<5%, [1]<10%, [2]<15%, [3]<20%, [4]<30%, [5]>=30%" << std::endl;

    while (true) {
        auto t_imu_start = std::chrono::steady_clock::now();
        bool success = imu.read(imu_data);
        auto t_imu_end = std::chrono::steady_clock::now();
        
        if (success) {
            if (last_imu_ts == 0) last_imu_ts = imu_data.timestamp_ns;

            uint64_t diff = imu_data.timestamp_ns - last_imu_ts;
            
            // Record only when we actually need and process the sample
            if (diff >= (IMU_TARGET_NS - 5000)) { 
                stats.recordImuCall(std::chrono::duration<double, std::milli>(t_imu_end - t_imu_start).count());
                stats.recordValidImu(diff / 1000000.0);
                last_imu_ts = imu_data.timestamp_ns;
                att.update(imu_data);
            }
        }

        // --- 2. Actuator Writing (400Hz) ---
        // Use IMU timestamp for synchronization
        if (last_pwm_send_ts == 0) last_pwm_send_ts = imu_data.timestamp_ns;

        if (imu_data.timestamp_ns - last_pwm_send_ts >= PWM_TARGET_NS) {
            auto t_pwm_start = std::chrono::steady_clock::now();
            act.write(pwm_data);
            auto t_pwm_end = std::chrono::steady_clock::now();
            
            stats.recordPwmCall(std::chrono::duration<double, std::milli>(t_pwm_end - t_pwm_start).count());

            // Update using target interval to maintain 400Hz on average (3ms, 2ms, 3ms...)
            last_pwm_send_ts += PWM_TARGET_NS;
        }

        // --- 3. Reporting ---
        stats.updateAndReport();
    }

    return 0;
}
