#ifndef msr_airlib_ImuXsim_hpp
#define msr_airlib_ImuXsim_hpp

#include "common/Common.hpp"
#include "ImuBase.hpp"
#include "common/VectorMath.hpp"
#include "common/XlabUeMetrics.hpp"
#include "common/EarthUtils.hpp"
#include <chrono>

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
        Utils::log("ImuXsim::resetImplementation() activated!");
        last_time_ = clock()->nowNanos();
        last_log_time_ = clock()->nowNanos();
        update_count_ = 0;
        updateOutput();
    }

    virtual void update() override
    {
        ImuBase::update();

        updateOutput();

        ++update_count_;
        const TTimeDelta elapsed_sec = clock()->elapsedSince(last_log_time_);
        if (elapsed_sec >= 1.0) {
            const double hz = static_cast<double>(update_count_) / static_cast<double>(elapsed_sec);
            XlabUeMetrics::setImuHz(static_cast<int>(hz + 0.5));
            last_log_time_ = clock()->nowNanos();
            update_count_ = 0;
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
};

}
}

#endif
