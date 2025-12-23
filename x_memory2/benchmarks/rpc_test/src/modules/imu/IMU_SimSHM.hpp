#pragma once
#include "../../core/IMU_IF.hpp"
#include "../../drivers/shm/SHM_Driver.hpp"
#include <iostream>

class IMU_SimSHM : public IMU_IF {
public:
    IMU_SimSHM(SHM_Driver& driver) : driver_(driver) {}

    bool init() override {
        // Driver should be connected externally or we check here
        return driver_.is_connected();
    }

    bool read(IMUData& out_data) override {
        if (!driver_.is_connected()) return false;

        // Use x_xsim's consume_telem to get latest data
        // We only need the LATEST sample for control loop usually, 
        // or we queue them. Here we just get the latest one available.
        
        bool got_new_data = false;
        
        // Lambda to extract data
        auto on_sample = [&](const XSimTelemetry& d) {
            out_data.timestamp_ns = d.timestamp;
            out_data.accel[0] = d.acc[0];
            out_data.accel[1] = d.acc[1];
            out_data.accel[2] = d.acc[2];
            
            out_data.gyro[0] = d.gyro[0];
            out_data.gyro[1] = d.gyro[1];
            out_data.gyro[2] = d.gyro[2];
            
            out_data.mag[0] = d.mag[0];
            out_data.mag[1] = d.mag[1];
            out_data.mag[2] = d.mag[2];
            
            got_new_data = true;
        };

        // consume_telem updates last_seq internally
        driver_.get_xsim().consume_telem(last_seq_, on_sample);
        
        return got_new_data;
    }

    bool is_healthy() const override {
        return driver_.is_connected(); 
    }

private:
    SHM_Driver& driver_;
    uint32_t last_seq_ = 0;
};
