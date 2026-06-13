#pragma once
#include "../../core/GNSS_IF.hpp"
#include "../../drivers/shm/SHM_Driver.hpp"

class GNSS_SimSHM : public GNSS_IF {
public:
    GNSS_SimSHM(SHM_Driver& driver) : driver_(driver) {}

    bool init() override {
        return driver_.is_connected();
    }

    bool read(GNSSData& out_data) override {
        if (!driver_.is_connected()) return false;

        bool got_new_data = false;
        
        auto on_sample = [&](const XSimTelemetry& d) {
            out_data.timestamp_ns = d.timestamp;
            out_data.alt = d.alt;
            
            out_data.pos_ned[0] = d.loc_ned[0];
            out_data.pos_ned[1] = d.loc_ned[1];
            out_data.pos_ned[2] = d.loc_ned[2];
            
            out_data.vel_ned[0] = d.vel_ned[0];
            out_data.vel_ned[1] = d.vel_ned[1];
            out_data.vel_ned[2] = d.vel_ned[2];

            // Convert local NED offsets (meters) to global Latitude/Longitude
            // Reference Home: AirSim default coordinates (Redmond, WA)
            const double HOME_LAT = 47.641468;
            const double HOME_LON = -122.140165;
            const double earth_radius_m = 6378137.0;
            const double PI_VAL = 3.14159265358979323846;

            double lat_rad = HOME_LAT * PI_VAL / 180.0;
            out_data.lat = HOME_LAT + (d.loc_ned[0] / earth_radius_m) * (180.0 / PI_VAL);
            out_data.lon = HOME_LON + (d.loc_ned[1] / (earth_radius_m * std::cos(lat_rad))) * (180.0 / PI_VAL);
            
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
