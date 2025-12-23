#pragma once
#include "../../core/ACT_IF.hpp"
#include "../../drivers/rpc/RPC_Driver.hpp"

class ACT_SimRPC : public ACT_IF {
public:
    ACT_SimRPC(RPC_Driver& driver) : driver_(driver) {}

    bool init() override {
        if (!driver_.is_connected()) return false;
        
        // Ensure API control is enabled
        try {
            driver_.get_client()->enableApiControl(true);
            driver_.get_client()->armDisarm(true);
            return true;
        } catch (...) {
            return false;
        }
    }

    bool write(const PWMData& data) override {
        if (!driver_.is_connected() || data.pwm_values.size() < 4) return false;

        try {
            // AirSim moveByMotorPWMsAsync takes (front_right, rear_left, front_left, rear_right)
            // We assume data.pwm_values matches this order or at least has 4 values
            driver_.get_client()->moveByMotorPWMsAsync(
                data.pwm_values[0], 
                data.pwm_values[1], 
                data.pwm_values[2], 
                data.pwm_values[3], 
                1.0f // duration in seconds (irrelevant for high freq stream but needed)
            );
            return true;
        } catch (...) {
            return false;
        }
    }

private:
    RPC_Driver& driver_;
};
