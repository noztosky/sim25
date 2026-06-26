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
    // --- physics/SHM sampling loop jitter (us), measured in ImuXsim ---
    static void setLoopHz(int hz) { getLoopHzRef().store(hz, std::memory_order_relaxed); }
    static int  getLoopHz() { return getLoopHzRef().load(std::memory_order_relaxed); }
    static void setLoopJitMeanUs(int us) { getLoopJitMeanRef().store(us, std::memory_order_relaxed); }
    static int  getLoopJitMeanUs() { return getLoopJitMeanRef().load(std::memory_order_relaxed); }
    static void setLoopJitP99Us(int us) { getLoopJitP99Ref().store(us, std::memory_order_relaxed); }
    static int  getLoopJitP99Us() { return getLoopJitP99Ref().load(std::memory_order_relaxed); }
    static void setLoopJitMaxUs(int us) { getLoopJitMaxRef().store(us, std::memory_order_relaxed); }
    static int  getLoopJitMaxUs() { return getLoopJitMaxRef().load(std::memory_order_relaxed); }
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
    static std::atomic<int>& getLoopHzRef()
    {
        static std::atomic<int> loop_hz_i(0);
        return loop_hz_i;
    }
    static std::atomic<int>& getLoopJitMeanRef()
    {
        static std::atomic<int> loop_jit_mean_i(0);
        return loop_jit_mean_i;
    }
    static std::atomic<int>& getLoopJitP99Ref()
    {
        static std::atomic<int> loop_jit_p99_i(0);
        return loop_jit_p99_i;
    }
    static std::atomic<int>& getLoopJitMaxRef()
    {
        static std::atomic<int> loop_jit_max_i(0);
        return loop_jit_max_i;
    }
};

}
}


