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
    static void setBaroHz(int hz)
    {
        getBaroHzRef().store(hz, std::memory_order_relaxed);
    }
    static int getBaroHz()
    {
        return getBaroHzRef().load(std::memory_order_relaxed);
    }
    static void setMagHz(int hz)
    {
        getMagHzRef().store(hz, std::memory_order_relaxed);
    }
    static int getMagHz()
    {
        return getMagHzRef().load(std::memory_order_relaxed);
    }
    static void setLocHz(int hz)
    {
        getLocHzRef().store(hz, std::memory_order_relaxed);
    }
    static int getLocHz()
    {
        return getLocHzRef().load(std::memory_order_relaxed);
    }
    static void setImuEmitHz(int hz)
    {
        getImuEmitHzRef().store(hz, std::memory_order_relaxed);
    }
    static int getImuEmitHz()
    {
        return getImuEmitHzRef().load(std::memory_order_relaxed);
    }
    static void setImuSkipHz(int hz)
    {
        getImuSkipHzRef().store(hz, std::memory_order_relaxed);
    }
    static int getImuSkipHz()
    {
        return getImuSkipHzRef().load(std::memory_order_relaxed);
    }
private:
    static std::atomic<int>& getImuHzRef()
    {
        static std::atomic<int> imu_hz_i(0);
        return imu_hz_i;
    }
    static std::atomic<int>& getBaroHzRef()
    {
        static std::atomic<int> baro_hz_i(0);
        return baro_hz_i;
    }
    static std::atomic<int>& getMagHzRef()
    {
        static std::atomic<int> mag_hz_i(0);
        return mag_hz_i;
    }
    static std::atomic<int>& getLocHzRef()
    {
        static std::atomic<int> loc_hz_i(0);
        return loc_hz_i;
    }
    static std::atomic<int>& getImuEmitHzRef()
    {
        static std::atomic<int> imu_emit_hz_i(0);
        return imu_emit_hz_i;
    }
    static std::atomic<int>& getImuSkipHzRef()
    {
        static std::atomic<int> imu_skip_hz_i(0);
        return imu_skip_hz_i;
    }
};

}
}


