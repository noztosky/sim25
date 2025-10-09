#ifndef msr_airlib_XlabXMemoryAdapter_hpp
#define msr_airlib_XlabXMemoryAdapter_hpp

#include "common/Common.hpp"
#include "common/XlabXMemoryConfig.hpp"
#include <string>

#if defined(XLAB_XMEMORY_HEADER)
#include XLAB_XMEMORY_HEADER
#endif

namespace msr
{
namespace airlib
{

class XlabXMemoryWriter
{
public:
    void initialize(const std::string& path)
    {
        path_ = path;
        ready_ = openInternal(path_);
        if (!ready_) {
            Utils::log("XlabXMemoryWriter: open failed (no-op mode)", Utils::kLogLevelWarn);
        }
    }

    bool write(const void* data, size_t size)
    {
        if (!ready_)
            return false;
        return writeInternal(data, size);
    }

    bool writeImuEuler(double roll, double pitch, double yaw, long long timestamp_ns, int frequency)
    {
        if (!ready_)
            return false;
        return writeImuEulerInternal(roll, pitch, yaw, timestamp_ns, frequency);
    }

private:
    bool openInternal(const std::string& path)
    {
#if defined(XLAB_XMEMORY_OPEN)
        return XLAB_XMEMORY_OPEN(impl_, path.c_str());
#elif defined(XLAB_XMEMORY_OPEN_C)
        return XLAB_XMEMORY_OPEN_C(path.c_str());
#else
        unused(path);
        return false;
#endif
    }

    bool writeInternal(const void* data, size_t size)
    {
#if defined(XLAB_XMEMORY_WRITE_IMU)
        return XLAB_XMEMORY_WRITE_IMU(impl_, data, size);
#elif defined(XLAB_XMEMORY_WRITE_IMU_C)
        return XLAB_XMEMORY_WRITE_IMU_C(data, size);
#else
        unused(data);
        unused(size);
        return false;
#endif
    }

    bool writeImuEulerInternal(double roll, double pitch, double yaw, long long timestamp_ns, int frequency)
    {
#if defined(XLAB_XMEMORY_WRITE_IMU_EULER)
        XLAB_XMEMORY_WRITE_IMU_EULER(impl_, roll, pitch, yaw, timestamp_ns, frequency);
        return true;
#else
        unused(roll); unused(pitch); unused(yaw); unused(timestamp_ns); unused(frequency);
        return false;
#endif
    }

private:
    std::string path_;
    bool ready_ = false;

#if defined(XLAB_XMEMORY_TYPE)
    XLAB_XMEMORY_TYPE impl_;
#endif
};

class XlabXMemoryReader
{
public:
    void initialize(const std::string& path)
    {
        path_ = path;
        ready_ = openInternal(path_);
        if (!ready_) {
            Utils::log("XlabXMemoryReader: open failed (no-op mode)", Utils::kLogLevelWarn);
        }
    }

    bool read(float* pwm_out, int max_count, int* out_count)
    {
        if (!ready_)
            return false;
        return readInternal(pwm_out, max_count, out_count);
    }

private:
    bool openInternal(const std::string& path)
    {
#if defined(XLAB_XMEMORY_OPEN_READER)
        return XLAB_XMEMORY_OPEN_READER(impl_, path.c_str());
#elif defined(XLAB_XMEMORY_OPEN_C)
        return XLAB_XMEMORY_OPEN_C(path.c_str());
#else
        unused(path);
        return false;
#endif
    }

    bool readInternal(float* pwm_out, int max_count, int* out_count)
    {
#if defined(XLAB_XMEMORY_TRY_GET_PWM)
        ::PWMData tmp{};
        bool ok = XLAB_XMEMORY_TRY_GET_PWM(impl_, tmp);
        if (!ok) { if (out_count) *out_count = 0; return false; }
        if (max_count >= 4) {
            pwm_out[0] = static_cast<float>(tmp.rotor1);
            pwm_out[1] = static_cast<float>(tmp.rotor2);
            pwm_out[2] = static_cast<float>(tmp.rotor3);
            pwm_out[3] = static_cast<float>(tmp.rotor4);
            if (out_count) *out_count = 4;
            return true;
        }
        if (out_count) *out_count = 0;
        return false;
#elif defined(XLAB_XMEMORY_READ_PWM_C)
        return XLAB_XMEMORY_READ_PWM_C(pwm_out, max_count, out_count);
#else
        unused(pwm_out);
        unused(max_count);
        if (out_count) *out_count = 0;
        return false;
#endif
    }

private:
    std::string path_;
    bool ready_ = false;

#if defined(XLAB_XMEMORY_TYPE)
    XLAB_XMEMORY_TYPE impl_;
#endif
};

}
}

#endif
