#include <iostream>
#include <windows.h>
#include <thread>
#include <chrono>

#include "../drivers/shm/SHM_Driver.hpp"
#include "../modules/imu/IMU_SimSHM.hpp"
#include "../modules/attitude/ATT_Mahony.hpp"

// Simple App Loop
int main() {
    std::cout << "[SIL_App] Starting..." << std::endl;

    // 1. Instantiate Drivers and concrete implementations
    SHM_Driver shm_driver("AirSimXsim");
    
    // 2. Dependency Injection into Modules
    IMU_SimSHM imu(shm_driver);
    ATT_Mahony att(0.8f, 0.005f); // kp, ki

    // 3. Initialization
    std::cout << "[SIL_App] Connecting to Shared Memory..." << std::endl;
    if (!shm_driver.connect()) {
        std::cerr << "[SIL_App] Failed to connect to SHM!" << std::endl;
        return 1;
    }
    std::cout << "[SIL_App] SHM Connected." << std::endl;

    imu.init();
    att.init();

    // 4. Main Loop
    std::cout << "[SIL_App] Entering Control Loop..." << std::endl;
    long long loop_count = 0;
    
    while (true) {
        IMUData imu_data;
        
        // Read IMU (Non-blocking or block depending on logic, here polling)
        if (imu.read(imu_data)) {
            // Run Attitude Estimator
            att.update(imu_data);

            // Control Logic placeholder ...
            
            // Logging (1Hz)
            if (loop_count % 1000 == 0) {
                float w, x, y, z;
                att.get_quaternion(w, x, y, z);
                std::cout << "[SIL_App] Att Quat: " << w << ", " << x << ", " << y << ", " << z 
                          << " | AccZ: " << imu_data.accel[2] << std::endl;
            }
            loop_count++;
        }
        else {
            // Yield if no data
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    shm_driver.close();
    return 0;
}
