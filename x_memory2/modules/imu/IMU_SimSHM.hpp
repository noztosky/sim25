#pragma once
#include "../../core/IMU_IF.hpp"
#include "../../drivers/shm/SHM_Driver.hpp"
#include <iostream>
#include <queue>

class IMU_SimSHM : public IMU_IF {
public:
    IMU_SimSHM(SHM_Driver& driver) : driver_(driver) {}

    bool init() override {
        // Driver should be connected externally or we check here
        return driver_.is_connected();
    }

    bool read(IMUData& out_data) override {
        if (!driver_.is_connected()) return false;

        // Consume all new samples from SHM and push to buffer
        auto on_sample = [&](const XSimTelemetry& d) {
            IMUData sample;
            sample.timestamp_ns = d.timestamp;
            sample.accel[0] = d.acc[0];
            sample.accel[1] = d.acc[1];
            sample.accel[2] = d.acc[2];
            
            sample.gyro[0] = d.gyro[0];
            sample.gyro[1] = d.gyro[1];
            sample.gyro[2] = d.gyro[2];
            
            sample.mag[0] = d.mag[0];
            sample.mag[1] = d.mag[1];
            sample.mag[2] = d.mag[2];
            
            buffer_.push(sample);
        };

        driver_.get_xsim().consume_telem(last_seq_, on_sample);

        // Return the oldest sample from queue
        if (!buffer_.empty()) {
            out_data = buffer_.front();
            buffer_.pop();
            return true;
        }
        
        return false;
    }

    bool is_healthy() const override {
        return driver_.is_connected(); 
    }

private:
    SHM_Driver& driver_;
    uint32_t last_seq_ = 0;
    std::queue<IMUData> buffer_;
};
