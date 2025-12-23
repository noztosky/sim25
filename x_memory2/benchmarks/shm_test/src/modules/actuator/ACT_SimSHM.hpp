#pragma once
#include "../../core/ACT_IF.hpp"
#include "../../drivers/shm/SHM_Driver.hpp"

class ACT_SimSHM : public ACT_IF {
public:
    ACT_SimSHM(SHM_Driver& driver) : driver_(driver) {}

    bool init() override {
        // SimpleFlightApi in AirSim is already configured to auto-enable control 
        // when valid SHM PWM data is received.
        return driver_.is_connected();
    }

    bool write(const PWMData& data) override {
        if (!driver_.is_connected() || data.pwm_values.size() < 4) return false;

        XSimPwm p;
        // Convert unit (0.0~1.0) to PWM microseconds (1000~2000)
        p.rotor1 = static_cast<int>(data.pwm_values[0] * 1000.0f + 1000.0f);
        p.rotor2 = static_cast<int>(data.pwm_values[1] * 1000.0f + 1000.0f);
        p.rotor3 = static_cast<int>(data.pwm_values[2] * 1000.0f + 1000.0f);
        p.rotor4 = static_cast<int>(data.pwm_values[3] * 1000.0f + 1000.0f);
        
        // Use 0 as timestamp or actual ns if required? 
        // SimpleFlightApi uses its own timing, but we can pass 0 or a sequence.
        p.timestamp = 0; 
        p.seq = ++seq_;
        p.is_valid = true;

        driver_.get_xsim().submit_pwm(p);
        return true;
    }

private:
    SHM_Driver& driver_;
    int seq_ = 0;
};
