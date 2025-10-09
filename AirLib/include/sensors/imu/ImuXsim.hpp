#ifndef msr_airlib_ImuXsim_hpp
#define msr_airlib_ImuXsim_hpp

#include "common/Common.hpp"
#include "ImuBase.hpp"
#include "common/VectorMath.hpp"
#include "common/XlabXMemoryAdapter.hpp"

namespace msr
{
namespace airlib
{

class ImuXsim : public ImuBase
{
public:
    ImuXsim(const AirSimSettings::ImuSetting& setting = AirSimSettings::ImuSetting())
        : ImuBase(setting.sensor_name)
    {
        unused(setting);
    }

    virtual void resetImplementation() override
    {
        last_time_ = clock()->nowNanos();

        last_log_time_ = clock()->nowNanos();
        update_count_ = 0;

        const GroundTruth& ground_truth = getGroundTruth();
        real_T pitch_r = 0, roll_r = 0, yaw_r = 0;
        VectorMath::toEulerianAngle(ground_truth.kinematics->pose.orientation, pitch_r, roll_r, yaw_r);
        last_yaw_rad_ = yaw_r;
        yaw_changed_count_ = 0;

        updateOutput();

        if (!xmem_inited_) {
            const std::string default_path = std::string("AirSimSharedMemory");
            xmem_writer_.initialize(default_path);
            target_hz_ = 1000.0f;
            period_ns_ = static_cast<TTimePoint>(1e9 / static_cast<double>(target_hz_));
            next_write_tp_ns_ = clock()->nowNanos() + period_ns_;
            xmem_inited_ = true;
        }
    }

    virtual void update() override
    {
        ImuBase::update();

        const GroundTruth& ground_truth = getGroundTruth();
        real_T pitch_r = 0, roll_r = 0, yaw_r = 0;
        VectorMath::toEulerianAngle(ground_truth.kinematics->pose.orientation, pitch_r, roll_r, yaw_r);
        if (std::abs(yaw_r - last_yaw_rad_) > static_cast<real_T>(1e-6)) {
            ++yaw_changed_count_;
            last_yaw_rad_ = yaw_r;
        }

        updateOutput();

        // write to x_memory at ~1 kHz (Euler-based) using catch-up scheduler to avoid drift/under-rate
        if (xmem_inited_ && target_hz_ > 0) {
            const TTimePoint now_ns = clock()->nowNanos();
            int safety = 0;
            while ((static_cast<long long>(now_ns - next_write_tp_ns_) >= 0) && safety < 5) {
                const auto& out = getOutput();
                real_T pr = 0, rr = 0, yr = 0;
                VectorMath::toEulerianAngle(out.orientation, pr, rr, yr);
                const double rad2deg = 57.29577951308232;
                double rr_deg = static_cast<double>(rr) * rad2deg;
                double pr_deg = static_cast<double>(pr) * rad2deg;
                double yr_deg = static_cast<double>(yr) * rad2deg;
                long long ts_ns = static_cast<long long>(out.time_stamp * 1e9);
                int seq_counter = static_cast<int>(++imu_seq_);
                xmem_writer_.writeImuEuler(rr_deg, pr_deg, yr_deg, ts_ns, seq_counter);

                next_write_tp_ns_ += period_ns_;
                ++safety;
            }
        }

        ++update_count_;
        const TTimeDelta elapsed_sec = clock()->elapsedSince(last_log_time_);
        if (elapsed_sec >= 1.0) {
            const double hz = static_cast<double>(update_count_) / static_cast<double>(elapsed_sec);
            const double yaw_deg = static_cast<double>(last_yaw_rad_) * 57.29577951308232;
            Utils::log(Utils::stringf("ImuXsim.update(): %.1f Hz (%u calls), yaw_changed=%u, yaw=%.1f deg",
                hz, static_cast<unsigned>(update_count_), static_cast<unsigned>(yaw_changed_count_), yaw_deg));
            last_log_time_ = clock()->nowNanos();
            update_count_ = 0;
            yaw_changed_count_ = 0;
        }
    }

    virtual ~ImuXsim() = default;

private:
    void updateOutput()
    {
        Output output;
        const GroundTruth& ground_truth = getGroundTruth();

        output.angular_velocity = ground_truth.kinematics->twist.angular;
        output.linear_acceleration = ground_truth.kinematics->accelerations.linear - ground_truth.environment->getState().gravity;
        output.orientation = ground_truth.kinematics->pose.orientation;

        output.linear_acceleration = VectorMath::transformToBodyFrame(
            output.linear_acceleration,
            ground_truth.kinematics->pose.orientation,
            true);

        output.time_stamp = clock()->nowNanos();
        setOutput(output);
    }

private:
    TTimePoint last_time_ = 0;
    TTimePoint last_log_time_ = 0;
    uint32_t update_count_ = 0;
    uint32_t yaw_changed_count_ = 0;
    real_T last_yaw_rad_ = 0;

    XlabXMemoryWriter xmem_writer_;
    bool xmem_inited_ = false;
    float target_hz_ = 1000.0f;
    // scheduling for consistent rate
    TTimePoint next_write_tp_ns_ = 0;
    TTimePoint period_ns_ = 0;
    uint32_t imu_seq_ = 0;
};

}
}

#endif
