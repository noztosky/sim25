#pragma once
#include "../../core/IMU_IF.hpp"
#include "../../drivers/rpc/RPC_Driver.hpp"

class IMU_SimRPC : public IMU_IF {
public:
    IMU_SimRPC(RPC_Driver& driver) : driver_(driver) {}

    bool init() override {
        return driver_.is_connected();
    }

    bool read(IMUData& out_data) override {
        if (!driver_.is_connected()) return false;

        try {
            auto client = driver_.get_client();
            // Get IMU data (default imu name is usually empty string or "imu")
            auto imu_out = client->getImuData("", ""); 
            
            // Ensure new data (filter duplicates)
            if (imu_out.time_stamp == last_ts_) {
                return false;
            }
            last_ts_ = imu_out.time_stamp;

            out_data.timestamp_ns = imu_out.time_stamp;
            
            out_data.accel[0] = imu_out.linear_acceleration.x();
            out_data.accel[1] = imu_out.linear_acceleration.y();
            out_data.accel[2] = imu_out.linear_acceleration.z();

            out_data.gyro[0] = imu_out.angular_velocity.x();
            out_data.gyro[1] = imu_out.angular_velocity.y();
            out_data.gyro[2] = imu_out.angular_velocity.z();

            // Mag might be separate sensor, but for now we leave optional
            out_data.mag[0] = 0;
            out_data.mag[1] = 0;
            out_data.mag[2] = 0;

            return true;
        } catch (const std::exception& e) {
            // handle errors (e.g. lost connection)
            return false;
        }
    }

    bool is_healthy() const override {
        return driver_.is_connected();
    }

private:
    RPC_Driver& driver_;
    uint64_t last_ts_ = 0;
};
