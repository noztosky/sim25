// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SimpleFlightDroneController_hpp
#define msr_airlib_SimpleFlightDroneController_hpp

#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/Common.hpp"
#include "firmware/Firmware.hpp"
#include "AirSimSimpleFlightBoard.hpp"
#include "AirSimSimpleFlightCommLink.hpp"
#include "AirSimSimpleFlightEstimator.hpp"
#include "AirSimSimpleFlightCommon.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/AirSimSettings.hpp"
#include "common/XlabUeMetrics.hpp"
// switch to x_xsim for PWM/telemetry
#include "D:\\open\\airsim\\x_memory\\x_xsim.h"

//TODO: we need to protect contention between physics thread and API server thread
namespace msr
{
namespace airlib
{

    class SimpleFlightApi : public MultirotorApiBase
    {

    public:
        SimpleFlightApi(const MultiRotorParams* vehicle_params, const AirSimSettings::VehicleSetting* vehicle_setting)
            : vehicle_params_(vehicle_params)
        {
            readSettings(*vehicle_setting);

            //TODO: set below properly for better high speed safety
            safety_params_.vel_to_breaking_dist = safety_params_.min_breaking_dist = 0;

            //create sim implementations of board and commlink
            board_.reset(new AirSimSimpleFlightBoard(&params_));
            comm_link_.reset(new AirSimSimpleFlightCommLink());
            estimator_.reset(new AirSimSimpleFlightEstimator());

            //create firmware
            firmware_.reset(new simple_flight::Firmware(&params_, board_.get(), comm_link_.get(), estimator_.get()));
        }

    public: //VehicleApiBase implementation
        virtual void resetImplementation() override
        {
            MultirotorApiBase::resetImplementation();

            firmware_->reset();
        }
        virtual void update() override
        {
            MultirotorApiBase::update();

            //update controller which will update actuator control signal
            firmware_->update();

            // External PWM passthrough via x_xsim
            if (!pwm_reader_inited_) {
                if (!xsim_) xsim_.reset(new x_xsim());
                xsim_->client_connect("AirSimXsim");
                pwm_reader_inited_ = true;
                last_log_tp_ = std::chrono::steady_clock::now();
                pwm_read_count_sec_ = 0;
                // ensure API control and arming are enabled so motor PWMs take effect
                if (!isApiControlEnabled()) enableApiControl(true);
                armDisarm(true);
            }
            if (xsim_) {
                static uint32_t last_pwm_seq = 0;
                static uint32_t last_telem_seq = 0;
                XSimPwm in{};
                if (xsim_->try_get_pwm(last_pwm_seq, in)) {
                    last_pwm_r_[0] = in.rotor1; last_pwm_r_[1] = in.rotor2; last_pwm_r_[2] = in.rotor3; last_pwm_r_[3] = in.rotor4;
                    auto to_unit = [](float us) {
                        float v = (us - 1000.0f) / 1000.0f;
                        if (v < 0.0f) v = 0.0f;
                        if (v > 1.0f) v = 1.0f;
                        return v;
                    };
                    float u0 = to_unit(static_cast<float>(in.rotor1));
                    float u1 = to_unit(static_cast<float>(in.rotor2));
                    float u2 = to_unit(static_cast<float>(in.rotor3));
                    float u3 = to_unit(static_cast<float>(in.rotor4));
                    // Apply immediately in passthrough mode
                    commandMotorPWMs(u0, u1, u2, u3);
                    pwm_read_count_sec_++;
                }
                // consume latest telemetry to populate values
                xsim_->consume_telem(last_telem_seq, [this](const XSimTelemetry& d){ last_telem_ = d; have_telem_ = true; });
                // 1-second combined log (even if no new PWM in this tick)
                auto now = std::chrono::steady_clock::now();
                auto dt_sec = std::chrono::duration<double>(now - last_log_tp_).count();
                if (dt_sec >= 1.0) {
                    int imu_hz = XlabUeMetrics::getImuHz();
                    int pwm_hz = dt_sec > 0.0 ? static_cast<int>(std::lround(pwm_read_count_sec_ / dt_sec)) : 0;
                    // populate from last telemetry if available (gyro deg/s, acc m/s^2)
                    double gx = have_telem_ ? (last_telem_.gyro[0] * 57.2957795131) : 0.0;
                    double gy = have_telem_ ? (last_telem_.gyro[1] * 57.2957795131) : 0.0;
                    double gz = have_telem_ ? (last_telem_.gyro[2] * 57.2957795131) : 0.0;
                    double ax = have_telem_ ? last_telem_.acc[0] : 0.0;
                    double ay = have_telem_ ? last_telem_.acc[1] : 0.0;
                    double az = have_telem_ ? last_telem_.acc[2] : 0.0;
                    double mx = have_telem_ ? last_telem_.mag[0] : 0.0;
                    double my = have_telem_ ? last_telem_.mag[1] : 0.0;
                    double mz = have_telem_ ? last_telem_.mag[2] : 0.0; int mag_hz=0;
                    double alt = have_telem_ ? last_telem_.alt : 0.0; int baro_hz = 0;
                    double nx = have_telem_ ? last_telem_.loc_ned[0] : 0.0;
                    double ny = have_telem_ ? last_telem_.loc_ned[1] : 0.0;
                    double nz = have_telem_ ? last_telem_.loc_ned[2] : 0.0; int loc_hz=0;
                    Utils::log(Utils::stringf(
                        "imu: %.2f %.2f %.2f %.2f %.2f %.2f(%dhz), mag: %.2f %.2f %.2f (%dhz) baro: %.2f (%dhz) loc: %.2f %.2f %.2f (%dhz) pwm: %d %d %d %d  (%dhz)",
                        gx,gy,gz,ax,ay,az, imu_hz, mx,my,mz, mag_hz, alt, baro_hz, nx,ny,nz, loc_hz,
                        last_pwm_r_[0], last_pwm_r_[1], last_pwm_r_[2], last_pwm_r_[3], pwm_hz));
                    last_log_tp_ = now;
                    pwm_read_count_sec_ = 0;
                }
            }
        }
        virtual bool isApiControlEnabled() const override
        {
            return firmware_->offboardApi().hasApiControl();
        }
        virtual void enableApiControl(bool is_enabled) override
        {
            if (is_enabled) {
                //comm_link should print message so no extra handling for errors
                std::string message;
                firmware_->offboardApi().requestApiControl(message);
            }
            else
                firmware_->offboardApi().releaseApiControl();
        }
        virtual bool armDisarm(bool arm) override
        {
            std::string message;
            if (arm)
                return firmware_->offboardApi().arm(message);
            else
                return firmware_->offboardApi().disarm(message);
        }
        virtual GeoPoint getHomeGeoPoint() const override
        {
            return AirSimSimpleFlightCommon::toGeoPoint(firmware_->offboardApi().getHomeGeoPoint());
        }
        virtual void getStatusMessages(std::vector<std::string>& messages) override
        {
            comm_link_->getStatusMessages(messages);
        }

        virtual const SensorCollection& getSensors() const override
        {
            return vehicle_params_->getSensors();
        }

    public: //MultirotorApiBase implementation
        virtual real_T getActuation(unsigned int rotor_index) const override
        {
            auto control_signal = board_->getMotorControlSignal(rotor_index);
            return control_signal;
        }
        virtual size_t getActuatorCount() const override
        {
            return vehicle_params_->getParams().rotor_count;
        }
        virtual void moveByRC(const RCData& rc_data) override
        {
            setRCData(rc_data);
        }
        virtual void setSimulatedGroundTruth(const Kinematics::State* kinematics, const Environment* environment) override
        {
            board_->setGroundTruthKinematics(kinematics);
            estimator_->setGroundTruthKinematics(kinematics, environment);
        }
        virtual bool setRCData(const RCData& rc_data) override
        {
            last_rcData_ = rc_data;
            if (rc_data.is_valid) {
                board_->setIsRcConnected(true);
                board_->setInputChannel(0, rc_data.roll); //X
                board_->setInputChannel(1, rc_data.yaw); //Y
                board_->setInputChannel(2, rc_data.throttle); //F
                board_->setInputChannel(3, -rc_data.pitch); //Z
                board_->setInputChannel(4, static_cast<float>(rc_data.getSwitch(0))); //angle rate or level
                board_->setInputChannel(5, static_cast<float>(rc_data.getSwitch(1))); //Allow API control
                board_->setInputChannel(6, static_cast<float>(rc_data.getSwitch(2)));
                board_->setInputChannel(7, static_cast<float>(rc_data.getSwitch(3)));
                board_->setInputChannel(8, static_cast<float>(rc_data.getSwitch(4)));
                board_->setInputChannel(9, static_cast<float>(rc_data.getSwitch(5)));
                board_->setInputChannel(10, static_cast<float>(rc_data.getSwitch(6)));
                board_->setInputChannel(11, static_cast<float>(rc_data.getSwitch(7)));
            }
            else { //else we don't have RC data
                board_->setIsRcConnected(false);
            }

            return true;
        }

    protected:
        virtual Kinematics::State getKinematicsEstimated() const override
        {
            return AirSimSimpleFlightCommon::toKinematicsState3r(firmware_->offboardApi().getStateEstimator().getKinematicsEstimated());
        }

        virtual Vector3r getPosition() const override
        {
            const auto& val = firmware_->offboardApi().getStateEstimator().getPosition();
            return AirSimSimpleFlightCommon::toVector3r(val);
        }

        virtual Vector3r getVelocity() const override
        {
            const auto& val = firmware_->offboardApi().getStateEstimator().getLinearVelocity();
            return AirSimSimpleFlightCommon::toVector3r(val);
        }

        virtual Quaternionr getOrientation() const override
        {
            const auto& val = firmware_->offboardApi().getStateEstimator().getOrientation();
            return AirSimSimpleFlightCommon::toQuaternion(val);
        }

        virtual LandedState getLandedState() const override
        {
            return firmware_->offboardApi().getLandedState() ? LandedState::Landed : LandedState::Flying;
        }

        virtual RCData getRCData() const override
        {
            //return what we received last time through setRCData
            return last_rcData_;
        }

        virtual GeoPoint getGpsLocation() const override
        {
            return AirSimSimpleFlightCommon::toGeoPoint(firmware_->offboardApi().getGeoPoint());
        }

        virtual float getCommandPeriod() const override
        {
            return 1.0f / 50; //50hz
        }

        virtual float getTakeoffZ() const override
        {
            // pick a number, 3 meters is probably safe
            // enough to get out of the backwash turbulence.  Negative due to NED coordinate system.
            return params_.takeoff.takeoff_z;
        }

        virtual float getDistanceAccuracy() const override
        {
            return 0.5f; //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance traveled
        }

        virtual void commandMotorPWMs(float front_right_pwm, float rear_left_pwm, float front_left_pwm, float rear_right_pwm) override
        {
            //Utils::log(Utils::stringf("commandMotorPWMs %f, %f, %f, %f", front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm));

            typedef simple_flight::GoalModeType GoalModeType;
            simple_flight::GoalMode mode(GoalModeType::Passthrough, GoalModeType::Passthrough, GoalModeType::Passthrough, GoalModeType::Passthrough);

            simple_flight::Axis4r goal(front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm);

            std::string message;
            firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
        }

        virtual void commandRollPitchYawZ(float roll, float pitch, float yaw, float z) override
        {
            //Utils::log(Utils::stringf("commandRollPitchYawZ %f, %f, %f, %f", pitch, roll, z, yaw));

            typedef simple_flight::GoalModeType GoalModeType;
            simple_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::PositionWorld);

            simple_flight::Axis4r goal(roll, pitch, yaw, z);

            std::string message;
            firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
        }

        virtual void commandRollPitchYawThrottle(float roll, float pitch, float yaw, float throttle) override
        {
            //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw, throttle));

            typedef simple_flight::GoalModeType GoalModeType;
            simple_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::Passthrough);

            simple_flight::Axis4r goal(roll, pitch, yaw, throttle);

            std::string message;
            firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
        }

        virtual void commandRollPitchYawrateThrottle(float roll, float pitch, float yaw_rate, float throttle) override
        {
            //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw, throttle));

            typedef simple_flight::GoalModeType GoalModeType;
            simple_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleRate, GoalModeType::Passthrough);

            simple_flight::Axis4r goal(roll, pitch, yaw_rate, throttle);

            std::string message;
            firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
        }

        virtual void commandRollPitchYawrateZ(float roll, float pitch, float yaw_rate, float z) override
        {
            //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw_rate, throttle));

            typedef simple_flight::GoalModeType GoalModeType;
            simple_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleRate, GoalModeType::PositionWorld);

            simple_flight::Axis4r goal(roll, pitch, yaw_rate, z);

            std::string message;
            firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
        }

        virtual void commandAngleRatesZ(float roll_rate, float pitch_rate, float yaw_rate, float z) override
        {
            //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw_rate, throttle));

            typedef simple_flight::GoalModeType GoalModeType;
            simple_flight::GoalMode mode(GoalModeType::AngleRate, GoalModeType::AngleRate, GoalModeType::AngleRate, GoalModeType::PositionWorld);

            simple_flight::Axis4r goal(roll_rate, pitch_rate, yaw_rate, z);

            std::string message;
            firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
        }

        virtual void commandAngleRatesThrottle(float roll_rate, float pitch_rate, float yaw_rate, float throttle) override
        {
            //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw_rate, throttle));

            typedef simple_flight::GoalModeType GoalModeType;
            simple_flight::GoalMode mode(GoalModeType::AngleRate, GoalModeType::AngleRate, GoalModeType::AngleRate, GoalModeType::Passthrough);

            simple_flight::Axis4r goal(roll_rate, pitch_rate, yaw_rate, throttle);

            std::string message;
            firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
        }

        virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
        {
            //Utils::log(Utils::stringf("commandVelocity %f, %f, %f, %f", vx, vy, vz, yaw_mode.yaw_or_rate));

            typedef simple_flight::GoalModeType GoalModeType;
            simple_flight::GoalMode mode(GoalModeType::VelocityWorld, GoalModeType::VelocityWorld, yaw_mode.is_rate ? GoalModeType::AngleRate : GoalModeType::AngleLevel, GoalModeType::VelocityWorld);

            simple_flight::Axis4r goal(vy, vx, Utils::degreesToRadians(yaw_mode.yaw_or_rate), vz);

            std::string message;
            firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
        }

        virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
        {
            //Utils::log(Utils::stringf("commandVelocityZ %f, %f, %f, %f", vx, vy, z, yaw_mode.yaw_or_rate));

            typedef simple_flight::GoalModeType GoalModeType;
            simple_flight::GoalMode mode(GoalModeType::VelocityWorld, GoalModeType::VelocityWorld, yaw_mode.is_rate ? GoalModeType::AngleRate : GoalModeType::AngleLevel, GoalModeType::PositionWorld);

            simple_flight::Axis4r goal(vy, vx, Utils::degreesToRadians(yaw_mode.yaw_or_rate), z);

            std::string message;
            firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
        }

        virtual void setControllerGains(uint8_t controller_type, const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) override
        {
            simple_flight::GoalModeType controller_type_enum = static_cast<simple_flight::GoalModeType>(controller_type);

            vector<float> kp_axis4(4);
            vector<float> ki_axis4(4);
            vector<float> kd_axis4(4);

            switch (controller_type_enum) {
            // roll gain, pitch gain, yaw gain, and no gains in throttle / z axis
            case simple_flight::GoalModeType::AngleRate:
                kp_axis4 = { kp[0], kp[1], kp[2], 1.0 };
                ki_axis4 = { ki[0], ki[1], ki[2], 0.0 };
                kd_axis4 = { kd[0], kd[1], kd[2], 0.0 };
                params_.angle_rate_pid.p.setValues(kp_axis4);
                params_.angle_rate_pid.i.setValues(ki_axis4);
                params_.angle_rate_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            case simple_flight::GoalModeType::AngleLevel:
                kp_axis4 = { kp[0], kp[1], kp[2], 1.0 };
                ki_axis4 = { ki[0], ki[1], ki[2], 0.0 };
                kd_axis4 = { kd[0], kd[1], kd[2], 0.0 };
                params_.angle_level_pid.p.setValues(kp_axis4);
                params_.angle_level_pid.i.setValues(ki_axis4);
                params_.angle_level_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            case simple_flight::GoalModeType::VelocityWorld:
                kp_axis4 = { kp[1], kp[0], 0.0, kp[2] };
                ki_axis4 = { ki[1], ki[0], 0.0, ki[2] };
                kd_axis4 = { kd[1], kd[0], 0.0, kd[2] };
                params_.velocity_pid.p.setValues(kp_axis4);
                params_.velocity_pid.i.setValues(ki_axis4);
                params_.velocity_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            case simple_flight::GoalModeType::PositionWorld:
                kp_axis4 = { kp[1], kp[0], 0.0, kp[2] };
                ki_axis4 = { ki[1], ki[0], 0.0, ki[2] };
                kd_axis4 = { kd[1], kd[0], 0.0, kd[2] };
                params_.position_pid.p.setValues(kp_axis4);
                params_.position_pid.i.setValues(ki_axis4);
                params_.position_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            default:
                Utils::log("Unimplemented controller type");
                break;
            }
        }

        virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
        {
            //Utils::log(Utils::stringf("commandPosition %f, %f, %f, %f", x, y, z, yaw_mode.yaw_or_rate));

            typedef simple_flight::GoalModeType GoalModeType;
            simple_flight::GoalMode mode(GoalModeType::PositionWorld, GoalModeType::PositionWorld, yaw_mode.is_rate ? GoalModeType::AngleRate : GoalModeType::AngleLevel, GoalModeType::PositionWorld);

            simple_flight::Axis4r goal(y, x, Utils::degreesToRadians(yaw_mode.yaw_or_rate), z);

            std::string message;
            firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
        }

        virtual const MultirotorApiParams& getMultirotorApiParams() const override
        {
            return safety_params_;
        }

        //*** End: MultirotorApiBase implementation ***//

    private:
        //convert pitch, roll, yaw from -1 to 1 to PWM
        static uint16_t angleToPwm(float angle)
        {
            return static_cast<uint16_t>(angle * 500.0f + 1500.0f);
        }
        static uint16_t thrustToPwm(float thrust)
        {
            return static_cast<uint16_t>((thrust < 0 ? 0 : thrust) * 1000.0f + 1000.0f);
        }
        static uint16_t switchTopwm(float switchVal, uint maxSwitchVal = 1)
        {
            return static_cast<uint16_t>(1000.0f * switchVal / maxSwitchVal + 1000.0f);
        }

        void readSettings(const AirSimSettings::VehicleSetting& vehicle_setting)
        {
            params_.default_vehicle_state = simple_flight::VehicleState::fromString(
                vehicle_setting.default_vehicle_state == "" ? "Armed" : vehicle_setting.default_vehicle_state);

            remote_control_id_ = vehicle_setting.rc.remote_control_id;
            params_.rc.allow_api_when_disconnected = vehicle_setting.rc.allow_api_when_disconnected;
            params_.rc.allow_api_always = vehicle_setting.allow_api_always;
        }

    private:
        const MultiRotorParams* vehicle_params_;

        int remote_control_id_ = 0;
        simple_flight::Params params_;

        unique_ptr<AirSimSimpleFlightBoard> board_;
        unique_ptr<AirSimSimpleFlightCommLink> comm_link_;
        unique_ptr<AirSimSimpleFlightEstimator> estimator_;
        unique_ptr<simple_flight::IFirmware> firmware_;

        MultirotorApiParams safety_params_;

        RCData last_rcData_;

        // External PWM reader (shared memory ring)
        bool pwm_reader_inited_ = false;
        std::unique_ptr<x_xsim> xsim_;
        std::chrono::steady_clock::time_point last_log_tp_{};
        int pwm_read_count_sec_ = 0;
        int last_pwm_r_[4] = {0, 0, 0, 0};
        // latest telemetry snapshot for combined logging
        XSimTelemetry last_telem_{};
        bool have_telem_ = false;
    };
}
} //namespace
#endif
