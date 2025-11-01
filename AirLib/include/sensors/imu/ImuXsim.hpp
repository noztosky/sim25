#ifndef msr_airlib_ImuXsim_hpp
#define msr_airlib_ImuXsim_hpp

#include "common/Common.hpp"
#include "ImuBase.hpp"
#include "common/VectorMath.hpp"
#include "common/XlabUeMetrics.hpp"
#include "common/EarthUtils.hpp"
// use new x_xsim shared memory for composite telemetry
#include "../../../../x_memory/shm/x_xsim.h"
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
            // init 1kHz pacing and x_xsim server mapping
            target_hz_ = 10000.0f;
            period_ns_ = static_cast<TTimePoint>(1e9 / static_cast<double>(target_hz_));
            next_write_tp_ns_ = clock()->nowNanos() + period_ns_;
            if (!xsim_) xsim_.reset(new x_xsim());
            xsim_->server_create("AirSimXsim");
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

        // publish to x_xsim at ~1 kHz (uniform pacing with cumulative schedule + skip-late)
        if (xmem_inited_ && target_hz_ > 0) {
            const TTimePoint now_ns = clock()->nowNanos();
            if (static_cast<long long>(now_ns - next_write_tp_ns_) >= 0) {
                const auto& out = getOutput();
                const GroundTruth& gt = getGroundTruth();
                long long ts_ns = static_cast<long long>(out.time_stamp * 1e9);
                int seq_counter = static_cast<int>(++imu_seq_);
                // fill telemetry
                XSimTelemetry d{};
                d.gyro[0] = static_cast<double>(gt.kinematics->twist.angular.x());
                d.gyro[1] = static_cast<double>(gt.kinematics->twist.angular.y());
                d.gyro[2] = static_cast<double>(gt.kinematics->twist.angular.z());
                // acc body (already computed in output)
                d.acc[0] = static_cast<double>(out.linear_acceleration.x());
                d.acc[1] = static_cast<double>(out.linear_acceleration.y());
                d.acc[2] = static_cast<double>(out.linear_acceleration.z());
                // quat wxyz (float)
                const auto& q = gt.kinematics->pose.orientation;
                d.quat[0] = q.w(); d.quat[1] = q.x(); d.quat[2] = q.y(); d.quat[3] = q.z();
                // position NED
                const auto& p = gt.kinematics->pose.position;
                d.loc_ned[0] = static_cast<double>(p.x());
                d.loc_ned[1] = static_cast<double>(p.y());
                d.loc_ned[2] = static_cast<double>(p.z());
                d.alt = -static_cast<double>(p.z());
                // magnetic field: world(NED) -> body; use Earth dipole model in Gauss
                Vector3r mag_world = EarthUtils::getMagField(gt.environment->getState().geo_point) * 1E4f; // Tesla->Gauss
                Vector3r mag_body = VectorMath::transformToBodyFrame(mag_world, gt.kinematics->pose.orientation, true);
                d.mag[0] = static_cast<double>(mag_body.x());
                d.mag[1] = static_cast<double>(mag_body.y());
                d.mag[2] = static_cast<double>(mag_body.z());
                d.timestamp = ts_ns;
                d.seq = seq_counter;
                d.is_valid = true;
                std::memset(d.padding, 0, sizeof d.padding);
                if (xsim_) xsim_->publish_telem(d);

                // cumulative advance to keep average exactly 1 kHz
                next_write_tp_ns_ += period_ns_;
                // if slightly behind, skip-late without burst (advance schedule only)
                while (static_cast<long long>((next_write_tp_ns_ + period_ns_) - now_ns) < 0) {
                    next_write_tp_ns_ += period_ns_;
                }
                // if far behind, resync to avoid long drift
                if (static_cast<long long>(now_ns - next_write_tp_ns_) > static_cast<long long>(2 * period_ns_)) {
                    next_write_tp_ns_ = now_ns + period_ns_;
                }
            }
        }

        ++update_count_;
        const TTimeDelta elapsed_sec = clock()->elapsedSince(last_log_time_);
        if (elapsed_sec >= 1.0) {
            const double hz = static_cast<double>(update_count_) / static_cast<double>(elapsed_sec);
            XlabUeMetrics::setImuHz(static_cast<int>(hz + 0.5));
            // keep counters but suppress per-line IMU log to avoid duplicate lines; PWM logger prints combined line
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

    std::unique_ptr<x_xsim> xsim_;
    bool xmem_inited_ = false;
    float target_hz_ = 10000.0f;
    // scheduling for consistent rate
    TTimePoint next_write_tp_ns_ = 0;
    TTimePoint period_ns_ = 0;
    uint32_t imu_seq_ = 0;
};

}
}

#endif
