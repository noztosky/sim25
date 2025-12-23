#pragma once
#include "../../core/Driver_IF.hpp"

// We need to disable some warnings for AirLib headers if MSVC
#pragma warning(push)
#pragma warning(disable: 4100) // unreferenced formal parameter
#pragma warning(disable: 4505) // unreferenced local function has been removed
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#pragma warning(pop)

#include <iostream>

class RPC_Driver : public Driver_IF {
public:
    RPC_Driver(const std::string& ip = "localhost", uint16_t port = 41451) 
        : ip_(ip), port_(port), client_(nullptr) {}

    virtual ~RPC_Driver() {
        close();
    }

    bool connect() override {
        try {
            client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(ip_, port_);
            client_->confirmConnection();
            connected_ = true;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[RPC_Driver] Connection failed: " << e.what() << std::endl;
            connected_ = false;
            return false;
        }
    }

    void close() override {
        client_.reset();
        connected_ = false;
    }

    bool is_connected() const override {
        return connected_ && client_ != nullptr;
    }

    msr::airlib::MultirotorRpcLibClient* get_client() {
        return client_.get();
    }

private:
    std::string ip_;
    uint16_t port_;
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client_;
    bool connected_ = false;
};
