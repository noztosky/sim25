#pragma once

#include <atomic>

namespace msr {
namespace airlib {

struct XlabUeMetrics {
    static void setImuHz(int hz)
    {
        getImuHzRef().store(hz, std::memory_order_relaxed);
    }
    static int getImuHz()
    {
        return getImuHzRef().load(std::memory_order_relaxed);
    }
private:
    static std::atomic<int>& getImuHzRef()
    {
        static std::atomic<int> imu_hz_i(0);
        return imu_hz_i;
    }
};

}
}


