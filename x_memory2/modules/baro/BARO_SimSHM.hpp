#pragma once
#include "../../core/BARO_IF.hpp"
#include "../../drivers/shm/SHM_Driver.hpp"
#include <cmath>

class BARO_SimSHM : public BARO_IF {
public:
    BARO_SimSHM(SHM_Driver& driver) : driver_(driver) {}

    bool init() override {
        return driver_.is_connected();
    }

    bool read(BAROData& out_data) override {
        if (!driver_.is_connected()) return false;

        bool got_new_data = false;
        
        auto on_sample = [&](const XSimTelemetry& d) {
            out_data.timestamp_ns = d.timestamp;
            out_data.altitude = d.alt;
            out_data.pressure = d.pressure;
            out_data.temperature = d.temperature;
            
            got_new_data = true;
        };

        // Consume using a local sequence tracker
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
