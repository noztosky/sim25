// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_ImuXsim_hpp
#define msr_airlib_ImuXsim_hpp

#include "common/Common.hpp"
#include "ImuBase.hpp"
#include "common/VectorMath.hpp"

namespace msr
{
namespace airlib
{

    // Minimal custom IMU: reads ground truth and publishes IMU output at engine rate.
    // Optional noise/latency can be added later via settings if needed.
    class ImuXsim : public ImuBase
    {
    public:
        ImuXsim(const AirSimSettings::ImuSetting& setting = AirSimSettings::ImuSetting())
            : ImuBase(setting.sensor_name)
        {
            unused(setting);
        }

        // UpdatableState implementation
        virtual void resetImplementation() override
        {
            last_time_ = clock()->nowNanos();

            // init frequency logging
            last_log_time_ = clock()->nowNanos();
            update_count_ = 0;

            // init yaw change tracking
            const GroundTruth& ground_truth = getGroundTruth();
            real_T pitch_r = 0, roll_r = 0, yaw_r = 0;
            VectorMath::toEulerianAngle(ground_truth.kinematics->pose.orientation, pitch_r, roll_r, yaw_r);
            last_yaw_rad_ = yaw_r;
            yaw_changed_count_ = 0;

            updateOutput();
        }

        virtual void update() override
        {
            ImuBase::update();

            // track yaw changes before/after output update using current ground truth
            const GroundTruth& ground_truth = getGroundTruth();
            real_T pitch_r = 0, roll_r = 0, yaw_r = 0;
            VectorMath::toEulerianAngle(ground_truth.kinematics->pose.orientation, pitch_r, roll_r, yaw_r);
            if (std::abs(yaw_r - last_yaw_rad_) > static_cast<real_T>(1e-6)) {
                ++yaw_changed_count_;
                last_yaw_rad_ = yaw_r;
            }

            updateOutput();

            // frequency logging: print once per ~1s window
            ++update_count_;
            const TTimeDelta elapsed_sec = clock()->elapsedSince(last_log_time_); // seconds
            if (elapsed_sec >= 1.0) {
                const double hz = static_cast<double>(update_count_) / static_cast<double>(elapsed_sec);
                const double yaw_deg = static_cast<double>(last_yaw_rad_) * 57.29577951308232; // rad -> deg
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

            // transform linear acceleration to body frame
            output.linear_acceleration = VectorMath::transformToBodyFrame(
                output.linear_acceleration,
                ground_truth.kinematics->pose.orientation,
                true);

            output.time_stamp = clock()->nowNanos();
            setOutput(output);
        }

    private:
        TTimePoint last_time_ = 0;
        // logging counters
        TTimePoint last_log_time_ = 0;
        uint32_t update_count_ = 0;
        uint32_t yaw_changed_count_ = 0;
        real_T last_yaw_rad_ = 0;
    };
}
} //namespace

#endif


