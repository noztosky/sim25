#ifndef msr_airlib_BaroXsim_hpp
#define msr_airlib_BaroXsim_hpp

#include "common/Common.hpp"
#include "BarometerBase.hpp"
#include "common/XlabXMemoryAdapter.hpp"

namespace msr
{
namespace airlib
{

class BaroXsim : public BarometerBase
{
public:
    BaroXsim(const AirSimSettings::BarometerSetting& setting = AirSimSettings::BarometerSetting())
        : BarometerBase(setting.sensor_name)
    {
        unused(setting);
    }

    virtual void resetImplementation() override
    {
        last_time_ = clock()->nowNanos();
        // initialize mem writer (share same name as IMU)
        if (!xmem_inited_) {
            const std::string path = std::string("AirSimSharedMemory");
            xmem_writer_.initialize(path);
            target_hz_ = 100.0f;
            period_ns_ = static_cast<TTimePoint>(1e9 / static_cast<double>(target_hz_));
            next_write_tp_ns_ = clock()->nowNanos() + period_ns_;
            xmem_inited_ = true;
        }
        updateOutput();
    }

    virtual void update() override
    {
        BarometerBase::update();
        updateOutput();

        if (xmem_inited_ && target_hz_ > 0) {
            const TTimePoint now_ns = clock()->nowNanos();
            if (static_cast<long long>(now_ns - next_write_tp_ns_) >= 0) {
                const auto& out = getOutput();
                // publish altitude to ring buffer via adapter macro
                long long ts_ns = static_cast<long long>(out.time_stamp * 1e9);
                int seq_counter = static_cast<int>(++baro_seq_);
                xmem_writer_.writeBaro(out.altitude, ts_ns, seq_counter);
                next_write_tp_ns_ += period_ns_;
                while (static_cast<long long>((next_write_tp_ns_ + period_ns_) - now_ns) < 0)
                    next_write_tp_ns_ += period_ns_;
            }
        }
    }

private:
    void updateOutput()
    {
        Output out;
        const auto& gt = getGroundTruth();
        // altitude: use -Z (NED up is -Z)
        out.altitude = -gt.kinematics->pose.position.z();
        out.pressure = 0; out.qnh = 0;
        out.time_stamp = clock()->nowNanos();
        setOutput(out);
    }

private:
    TTimePoint last_time_ = 0;
    XlabXMemoryWriter xmem_writer_;
    bool xmem_inited_ = false;
    float target_hz_ = 100.0f;
    TTimePoint period_ns_ = 0;
    TTimePoint next_write_tp_ns_ = 0;
    uint32_t baro_seq_ = 0;
};

}
}

#endif


