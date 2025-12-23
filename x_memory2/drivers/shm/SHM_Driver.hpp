#pragma once
#include "../../core/Driver_IF.hpp"
#include "../../shm/x_xsim.h"
#include <iostream>

class SHM_Driver : public Driver_IF {
public:
    SHM_Driver(const char* map_name = "AirSimXsim") : map_name_(map_name) {}
    
    virtual ~SHM_Driver() {
        close();
    }

    bool connect() override {
        if (xsim_.client_connect(map_name_)) {
            connected_ = true;
            return true;
        }
        return false;
    }

    void close() override {
        xsim_.close();
        connected_ = false;
    }

    bool is_connected() const override {
        return connected_;
    }

    // Specific to internal usage (friend or public accessor)
    x_xsim& get_xsim() {
        return xsim_;
    }

private:
    x_xsim xsim_;
    const char* map_name_;
    bool connected_ = false;
};
