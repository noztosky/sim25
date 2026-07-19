// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_MultiRotorParameters_hpp
#define msr_airlib_MultiRotorParameters_hpp

#include "common/Common.hpp"
#include "RotorParams.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"

namespace msr
{
namespace airlib
{

    class MultiRotorParams
    {
        //All units are SI
    public: //types
        struct RotorPose
        {
            Vector3r position; //relative to center of gravity of vehicle body
            Vector3r normal;
            RotorTurningDirection direction;

            RotorPose()
            {
            }
            RotorPose(const Vector3r& position_val, const Vector3r& normal_val, RotorTurningDirection direction_val)
                : position(position_val), normal(normal_val), direction(direction_val)
            {
            }
        };

        struct Params
        {
            /*********** required parameters ***********/
            uint rotor_count;
            vector<RotorPose> rotor_poses;
            real_T mass;
            Matrix3x3r inertia;
            Vector3r body_box;

            /*********** optional parameters with defaults ***********/
            real_T linear_drag_coefficient = 1.3f / 4.0f;
            //sample value 1.3 from http://klsin.bpmsg.com/how-fast-can-a-quadcopter-fly/, but divided by 4 to account
            // for nice streamlined frame design and allow higher top speed which is more fun.
            //angular coefficient is usually 10X smaller than linear, however we should replace this with exact number
            //http://physics.stackexchange.com/q/304742/14061
            real_T angular_drag_coefficient = linear_drag_coefficient;
            real_T restitution = 0.55f; // value of 1 would result in perfectly elastic collisions, 0 would be completely inelastic.
            real_T friction = 0.5f;
            RotorParams rotor_params;
        };

    protected: //must override by derived class
        virtual void setupParams() = 0;
        virtual const SensorFactory* getSensorFactory() const = 0;

    public: //interface
        virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() = 0;

        virtual ~MultiRotorParams() = default;
        virtual void initialize(const AirSimSettings::VehicleSetting* vehicle_setting)
        {
            sensor_storage_.clear();
            sensors_.clear();

            setupParams();

            addSensorsFromSettings(vehicle_setting);
        }

        const Params& getParams() const
        {
            return params_;
        }
        Params& getParams()
        {
            return params_;
        }
        SensorCollection& getSensors()
        {
            return sensors_;
        }
        const SensorCollection& getSensors() const
        {
            return sensors_;
        }

        void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
        {
            const auto& sensor_settings = vehicle_setting->sensors;

            getSensorFactory()->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
        }

    protected: //static utility functions for derived classes to use
        /// Initializes 4 rotors in the usual QuadX pattern:  http://ardupilot.org/copter/_images/MOTORS_QuadX_QuadPlus.jpg
        /// which shows that given an array of 4 motors, the first is placed top right and flies counter clockwise (CCW) and
        /// the second is placed bottom left, and also flies CCW.  The third in the array is placed top left and flies clockwise (CW)
        /// while the last is placed bottom right and also flies clockwise.  This is how the 4 items in the arm_lengths and
        /// arm_angles arrays will be used.  So arm_lengths is 4 numbers (in meters) where four arm lengths, 0 is top right,
        /// 1 is bottom left, 2 is top left and 3 is bottom right.  arm_angles is 4 numbers (in degrees)  relative to forward vector (0,1),
        /// provided in the same order where 0 is top right, 1 is bottom left, 2 is top left and 3 is bottom right, so for example,
        /// the angles for a regular symmetric X pattern would be 45, 225, 315, 135.  The rotor_z is the offset of each motor upwards
        /// relative to the center of mass (in meters).
        static void initializeRotorQuadX(vector<RotorPose>& rotor_poses /* the result we are building */,
                                         uint rotor_count /* must be 4 */,
                                         real_T arm_lengths[],
                                         real_T rotor_z /* z relative to center of gravity */)
        {
            Vector3r unit_z(0, 0, -1); //NED frame
            if (rotor_count == 4) {
                rotor_poses.clear();

                /* Note: rotor_poses are built in this order:
                 x-axis
            (2)  |   (0)
                 |
            -------------- y-axis
                 |
            (1)  |   (3)
            */
                // vectors below are rotated according to NED left hand rule (so the vectors are rotated counter clockwise).
                Quaternionr quadx_rot(AngleAxisr(M_PIf / 4, unit_z));
                rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, arm_lengths[0], rotor_z), quadx_rot, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCCW);
                rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, -arm_lengths[1], rotor_z), quadx_rot, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCCW);
                rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(arm_lengths[2], 0, rotor_z), quadx_rot, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCW);
                rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(-arm_lengths[3], 0, rotor_z), quadx_rot, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCW);
            }
            else
                throw std::invalid_argument("Rotor count other than 4 is not supported by this method!");
        }

        static void initializeRotorHexX(vector<RotorPose>& rotor_poses /* the result we are building */,
                                        uint rotor_count /* must be 6 */,
                                        real_T arm_lengths[],
                                        real_T rotor_z /* z relative to center of gravity */)
        {
            Vector3r unit_z(0, 0, -1); //NED frame
            if (rotor_count == 6) {
                rotor_poses.clear();
                /* Note: rotor_poses are built in this order: rotor 0 is CW
              See HEXA X configuration on http://ardupilot.org/copter/docs/connect-escs-and-motors.html

                     x-axis
                (2)    (4)
                   \  /
                    \/
               (1)-------(0) y-axis
                    /\
                   /  \
                 (5)  (3)

            */

                // vectors below are rotated according to NED left hand rule (so the vectors are rotated counter clockwise).
                Quaternionr hexa_rot30(AngleAxisr(M_PIf / 6, unit_z)); // 30 degrees
                Quaternionr hexa_rot60(AngleAxisr(M_PIf / 3, unit_z)); // 60 degrees
                Quaternionr no_rot(AngleAxisr(0, unit_z));
                rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, arm_lengths[0], rotor_z), no_rot, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCW);
                rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, -arm_lengths[1], rotor_z), no_rot, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCCW);
                rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(arm_lengths[2], 0, rotor_z), hexa_rot30, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCW);
                rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(-arm_lengths[3], 0, rotor_z), hexa_rot30, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCCW);
                rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, arm_lengths[4], rotor_z), hexa_rot60, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCCW);
                rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, -arm_lengths[5], rotor_z), hexa_rot60, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCW);
            }
            else
                throw std::invalid_argument("Rotor count other than 6 is not supported by this method!");
        }

        static void initializeRotorOctoX(vector<RotorPose>& rotor_poses /* the result we are building */,
                                         uint rotor_count /* must be 8 */,
                                         real_T arm_lengths[],
                                         real_T rotor_z /* z relative to center of gravity */)
        {
            Vector3r unit_z(0, 0, -1); //NED frame
            if (rotor_count == 8) {
                rotor_poses.clear();
                /* Note: rotor_poses are built in this order: rotor 0 is CW
              See OCTO X configuration on http://ardupilot.org/copter/docs/connect-escs-and-motors.html

                     x-axis
                  
                 (4)  |  (0) 
                      |
            (6)       |       (2)
            __________|__________  y-axis
                      |
            (5)       |       (7)
                      |
                 (1)  |  (3)

            0 CW: 67.5 from +Y
            1 CW: 67.5 from -Y 
            2 CCW: 22.5 from +Y
            3 CCW: 22.5 from -X
            4 CCW: 22.5 from +X
            5 CCW: 22.5 from -Y
            6 CW: 67.5 from +X
            7 CW: 67.5 from -X
            
            */

                // vectors below are rotated according to NED left hand rule (so the vectors are rotated counter clockwise).
                Quaternionr octo_rot22(AngleAxisr(M_PIf / 8, unit_z)); // 22.5 degrees
                Quaternionr octo_rot67(AngleAxisr(3 * M_PIf / 8, unit_z)); // 67.5 degrees

                rotor_poses.emplace_back(VectorMath::rotateVector(
                                             Vector3r(0, arm_lengths[0], rotor_z), octo_rot67, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCW);

                rotor_poses.emplace_back(VectorMath::rotateVector(
                                             Vector3r(0, -arm_lengths[1], rotor_z), octo_rot67, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCW);

                rotor_poses.emplace_back(VectorMath::rotateVector(
                                             Vector3r(0, arm_lengths[2], rotor_z), octo_rot22, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCCW);

                rotor_poses.emplace_back(VectorMath::rotateVector(
                                             Vector3r(-arm_lengths[3], 0, rotor_z), octo_rot22, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCCW);

                rotor_poses.emplace_back(VectorMath::rotateVector(
                                             Vector3r(arm_lengths[4], 0, rotor_z), octo_rot22, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCCW);

                rotor_poses.emplace_back(VectorMath::rotateVector(
                                             Vector3r(0, -arm_lengths[5], rotor_z), octo_rot22, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCCW);

                rotor_poses.emplace_back(VectorMath::rotateVector(
                                             Vector3r(arm_lengths[6], 0, rotor_z), octo_rot67, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCW);

                rotor_poses.emplace_back(VectorMath::rotateVector(
                                             Vector3r(-arm_lengths[7], 0, rotor_z), octo_rot67, true),
                                         unit_z,
                                         RotorTurningDirection::RotorTurningDirectionCW);
            }
            else
                throw std::invalid_argument("Rotor count other than 8 is not supported by this method!");
        }

        /// Initialize the rotor_poses given the rotor_count, the arm lengths and the arm angles (relative to forwards vector).
        /// Also provide the direction you want to spin each rotor and the z-offset of the rotors relative to the center of gravity.
        static void initializeRotors(vector<RotorPose>& rotor_poses, uint rotor_count, real_T arm_lengths[], real_T arm_angles[], RotorTurningDirection rotor_directions[], real_T rotor_z /* z relative to center of gravity */)
        {
            Vector3r unit_z(0, 0, -1); //NED frame
            rotor_poses.clear();
            for (uint i = 0; i < rotor_count; i++) {
                Quaternionr angle(AngleAxisr(arm_angles[i] * M_PIf / 180, unit_z));
                rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, arm_lengths[i], rotor_z), angle, true),
                                         unit_z,
                                         rotor_directions[i]);
            }
        }

        static void computeInertiaMatrix(Matrix3x3r& inertia, const Vector3r& body_box, const vector<RotorPose>& rotor_poses,
                                         real_T box_mass, real_T motor_assembly_weight)
        {
            inertia = Matrix3x3r::Zero();

            //http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node64.html
            inertia(0, 0) = box_mass / 12.0f * (body_box.y() * body_box.y() + body_box.z() * body_box.z());
            inertia(1, 1) = box_mass / 12.0f * (body_box.x() * body_box.x() + body_box.z() * body_box.z());
            inertia(2, 2) = box_mass / 12.0f * (body_box.x() * body_box.x() + body_box.y() * body_box.y());

            for (size_t i = 0; i < rotor_poses.size(); ++i) {
                const auto& pos = rotor_poses.at(i).position;
                inertia(0, 0) += (pos.y() * pos.y() + pos.z() * pos.z()) * motor_assembly_weight;
                inertia(1, 1) += (pos.x() * pos.x() + pos.z() * pos.z()) * motor_assembly_weight;
                inertia(2, 2) += (pos.x() * pos.x() + pos.y() * pos.y()) * motor_assembly_weight;
            }
        }

        // Some Frame types which can be used by different firmwares
        // Specific frame configurations, modifications can be done in the Firmware Params

        void setupFrameGenericQuad(Params& params)
        {
            //set up arm lengths
            //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
            params.rotor_count = 4;
            std::vector<real_T> arm_lengths(params.rotor_count, 0.2275f);

            //set up mass
            //this has to be between max_thrust*rotor_count/10 (1.6kg using default parameters in RotorParams.hpp) and (idle throttle percentage)*max_thrust*rotor_count/10 (0.8kg using default parameters and SimpleFlight)
            //any value above the maximum would result in the motors not being able to lift the body even at max thrust,
            //and any value below the minimum would cause the drone to fly upwards on idling throttle (50% of the max throttle if using SimpleFlight)
            //Note that the default idle throttle percentage is 50% if you are using SimpleFlight
            params.mass = 1.0f;

            real_T motor_assembly_weight = 0.055f; //weight for MT2212 motor for F450 frame
            real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

            // using rotor_param default, but if you want to change any of the rotor_params, call calculateMaxThrust() to recompute the max_thrust
            // given new thrust coefficients, motor max_rpm and propeller diameter.
            params.rotor_params.calculateMaxThrust();

            //set up dimensions of core body box or abdomen (not including arms).
            params.body_box.x() = 0.180f;
            params.body_box.y() = 0.11f;
            params.body_box.z() = 0.040f;
            real_T rotor_z = 2.5f / 100;

            //computer rotor poses
            initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

            //compute inertia matrix
            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        }

        void setupFrameGenericHex(Params& params)
        {
            //set up arm lengths
            //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
            params.rotor_count = 6;
            std::vector<real_T> arm_lengths(params.rotor_count, 0.2275f);

            //set up mass
            //this has to be between max_thrust*rotor_count/10 (2.5kg using default parameters in RotorParams.hpp) and (idle throttle percentage)*max_thrust*rotor_count/10 (1.25kg using default parameters and SimpleFlight)
            //any value above the maximum would result in the motors not being able to lift the body even at max thrust,
            //and any value below the minimum would cause the drone to fly upwards on idling throttle (50% of the max throttle if using SimpleFlight)
            params.mass = 1.0f;

            real_T motor_assembly_weight = 0.055f; //weight for MT2212 motor for F450 frame
            real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

            // using rotor_param default, but if you want to change any of the rotor_params, call calculateMaxThrust() to recompute the max_thrust
            // given new thrust coefficients, motor max_rpm and propeller diameter.
            params.rotor_params.calculateMaxThrust();

            //set up dimensions of core body box or abdomen (not including arms).
            params.body_box.x() = 0.180f;
            params.body_box.y() = 0.11f;
            params.body_box.z() = 0.040f;
            real_T rotor_z = 2.5f / 100;

            //computer rotor poses
            initializeRotorHexX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

            //compute inertia matrix
            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        }

        void setupFrameGenericOcto(Params& params)
        {
            //set up arm lengths
            //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
            params.rotor_count = 8;
            std::vector<real_T> arm_lengths(params.rotor_count, 0.2275f);

            //set up mass
            //this has to be between max_thrust*rotor_count/10 (2.5kg using default parameters in RotorParams.hpp) and (idle throttle percentage)*max_thrust*rotor_count/10 (1.25kg using default parameters and SimpleFlight)
            //any value above the maximum would result in the motors not being able to lift the body even at max thrust,
            //and any value below the minimum would cause the drone to fly upwards on idling throttle (50% of the max throttle if using SimpleFlight)

            params.mass = 1.0f; //can be varied from 0.800 to 1.600
            real_T motor_assembly_weight = 0.055f; //weight for MT2212 motor for F450 frame  0.148
            real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

            // using rotor_param default, but if you want to change any of the rotor_params, call calculateMaxThrust() to recompute the max_thrust
            // given new thrust coefficients, motor max_rpm and propeller diameter.
            params.rotor_params.calculateMaxThrust();

            //set up dimensions of core body box or abdomen (not including arms).
            params.body_box.x() = 0.180f;
            params.body_box.y() = 0.11f;
            params.body_box.z() = 0.040f;
            real_T rotor_z = 2.5f / 100;

            //computer rotor poses
            initializeRotorOctoX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

            //compute inertia matrix
            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        }

        void setupFrameFlamewheel(Params& params)
        {
            //set up arm lengths
            //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
            params.rotor_count = 4;
            std::vector<real_T> arm_lengths(params.rotor_count, 0.225f);

            //set up mass
            params.mass = 1.635f;
            real_T motor_assembly_weight = 0.052f;
            real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

            params.rotor_params.C_T = 0.11f;
            params.rotor_params.C_P = 0.047f;
            params.rotor_params.max_rpm = 9500;
            params.rotor_params.calculateMaxThrust();
            params.linear_drag_coefficient *= 4; // make top speed more real.

            //set up dimensions of core body box or abdomen (not including arms).
            params.body_box.x() = 0.16f;
            params.body_box.y() = 0.10f;
            params.body_box.z() = 0.14f;
            real_T rotor_z = 0.15f;

            //computer rotor poses
            initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

            //compute inertia matrix
            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        }

        void setupFrameFlamewheelFLA(Params& params)
        {
            //set up arm lengths
            //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
            params.rotor_count = 4;
            std::vector<real_T> arm_lengths(params.rotor_count, 0.225f);

            //set up mass
            params.mass = 2.25f;
            real_T motor_assembly_weight = 0.1f;
            real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

            params.rotor_params.C_T = 0.2f;
            params.rotor_params.C_P = 0.1f;
            params.rotor_params.max_rpm = 9324;
            params.rotor_params.calculateMaxThrust();
            params.linear_drag_coefficient *= 4; // make top speed more real.

            //set up dimensions of core body box or abdomen (not including arms).
            params.body_box.x() = 0.16f;
            params.body_box.y() = 0.10f;
            params.body_box.z() = 0.14f;
            real_T rotor_z = 0.15f;

            //computer rotor poses
            initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

            //compute inertia matrix
            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        }

        void setupFrameBlacksheep(Params& params)
        {
            /*
        Motor placement:
        x
        (2)  |   (0)
        |
        ------------ y
        |
        (1)  |   (3)
        |

        */
            //set up arm lengths
            //dimensions are for Team Blacksheep Discovery (http://team-blacksheep.com/products/product:98)
            params.rotor_count = 4;
            std::vector<real_T> arm_lengths;

            Vector3r unit_z(0, 0, -1); //NED frame

            // relative to Forward vector in the order (0,3,1,2) required by quad X pattern
            // http://ardupilot.org/copter/_images/MOTORS_QuadX_QuadPlus.jpg
            arm_lengths.push_back(0.22f);
            arm_lengths.push_back(0.255f);
            arm_lengths.push_back(0.22f);
            arm_lengths.push_back(0.255f);

            // note: the Forward vector is actually the "x" axis, and the AngleAxisr rotation is pointing down and is left handed, so this means the rotation
            // is counter clockwise, so the vector (arm_lengths[i], 0) is the X-axis, so the CCW rotations to position each arm correctly are listed below:
            // See measurements here: http://diydrones.com/profiles/blogs/arducopter-tbs-discovery-style (angles reversed because we are doing CCW rotation)
            std::vector<real_T> arm_angles;
            arm_angles.push_back(-55.0f);
            arm_angles.push_back(125.0f);
            arm_angles.push_back(55.0f);
            arm_angles.push_back(-125.0f);

            // quad X pattern
            std::vector<RotorTurningDirection> rotor_directions;
            rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCCW);
            rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCCW);
            rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCW);
            rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCW);

            // data from
            // http://dronesvision.net/team-blacksheep-750kv-motor-esc-set-for-tbs-discovery-fpv-quadcopter/
            //set up mass
            params.mass = 2.0f; //can be varied from 0.800 to 1.600
            real_T motor_assembly_weight = 0.052f; // weight for TBS motors
            real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

            // the props we are using a E-Prop, which I didn't find in UIUC database, but this one is close:
            // http://m-selig.ae.illinois.edu/props/volume-2/plots/ef_130x70_static_ctcp.png
            params.rotor_params.C_T = 0.11f;
            params.rotor_params.C_P = 0.047f;
            params.rotor_params.max_rpm = 9500;
            params.rotor_params.calculateMaxThrust();

            //set up dimensions of core body box or abdomen (not including arms).
            params.body_box.x() = 0.20f;
            params.body_box.y() = 0.12f;
            params.body_box.z() = 0.04f;
            real_T rotor_z = 2.5f / 100;

            //computer rotor poses
            params.rotor_poses.clear();
            for (uint i = 0; i < 4; i++) {
                Quaternionr angle(AngleAxisr(arm_angles[i] * M_PIf / 180, unit_z));
                params.rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(arm_lengths[i], 0, rotor_z), angle, true), unit_z, rotor_directions[i]);
            };

            //compute inertia matrix
            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        }

        void setupFrameEFTZ30(Params& params)
        {
            /*
            EFT Z30 agricultural quad frame, EMPTY-TANK configuration (tank 0 L, battery installed).

            Manufacturer specs (effort-tech.com Z series):
              wheelbase 2025 mm (quad-X), 41135 propeller (D = 1.041 m),
              11115 95KV motor rated 18 kgf/axis on 14S,
              empty weight 29.8 kg (no battery) + 14S 30000 mAh battery ~10 kg -> 40 kg,
              max take-off weight 70 kg (30 L tank full).
            */
            params.rotor_count = 4;
            std::vector<real_T> arm_lengths(params.rotor_count, 1.0125f); //wheelbase 2025mm / 2

            params.mass = 40.0f; //empty tank + battery
            real_T motor_assembly_weight = 1.3f; //11115 motor + 41135 prop + ESC per arm end
            real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

            //Thrust model: keep UIUC C_T/C_P shape coefficients, solve max_rpm so that
            //C_T * rho * n^2 * D^4 == 18 kgf (176.5 N):  n = 33.4 rev/s -> ~2005 RPM.
            //Tip speed at max: pi * 1.041 * 33.4 = 109 m/s (Mach 0.32, plausible for 41" prop).
            params.rotor_params.C_T = 0.109919f;
            params.rotor_params.C_P = 0.040164f;
            params.rotor_params.max_rpm = 2005.0f;
            params.rotor_params.propeller_diameter = 1.041f;
            params.rotor_params.control_signal_filter_tc = 0.06f; //~60ms: computed spool-up tau from real specs (Hobbywing X11 G2: MFP 43x14 prop 146g -> J~0.011 kg.m^2, 95KV -> Kt=0.1005, R~0.08ohm). tau = J/(Kt^2/R + 2Q0/w0) ~= 58ms. Range 45-85ms (R uncertainty). Was 0.03 (artificially fast).
            params.rotor_params.calculateMaxThrust();
            //hover check: 40 kg -> 392 N -> 98 N/rotor = 55.6% of 176.5 N max

            //central truss + empty tank + battery envelope (folded body is 979x684x752 mm)
            params.body_box.x() = 0.75f;
            params.body_box.y() = 0.60f;
            params.body_box.z() = 0.50f;
            real_T rotor_z = 0.05f;

            initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

            //Empty-tank front/rear imbalance: battery sits aft-top while the (empty) tank is
            //forward-bottom, so the CG sits aft of the geometric frame center. Rotor positions
            //are defined relative to the CG, so shift every rotor forward by |cg_offset_x|.
            //-0.20 m (CG 20 cm aft, NED +x = forward) is an estimate - measure on the physical
            //frame (balance point) and update. Set to 0 for a symmetric baseline run.
            const real_T cg_offset_x = -0.20f;
            for (auto& pose : params.rotor_poses)
                pose.position.x() -= cg_offset_x;

            //compute inertia matrix (uses the CG-shifted rotor arms, so the asymmetry is captured)
            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        }

        void setupFrameEFTK20(Params& params)
        {
            /*
            EFT K20 modular quad platform (2026), EMPTY-TANK configuration.

            Confirmed specs (effort-tech.com K20 PNP + Hobbywing X9/9620):
              quad, 20 kg payload / 20 L tank (40 L spreader option),
              Hobbywing 9620 motor 100KV on 14S, 36" composite folding props,
              MAX thrust 26.5 kgf/axis (peak spec - lesson from Z30 where the 18 kgf
              "rated" figure halved the modeled control authority), powertrain 1774 g/axis.

            ESTIMATES (measure on the physical vehicle and update):
              wheelbase ~1550 mm (45x575 arm tube + motor mount + center frame),
              empty mass ~24 kg (frame ~7.5 + 4x1.8 powertrain + 14S battery ~9 + avionics),
              CG 5 cm aft (battery aft of center; measure the balance point!).
            */
            params.rotor_count = 4;
            std::vector<real_T> arm_lengths(params.rotor_count, 0.775f); //wheelbase 1550mm / 2 (estimate)

            params.mass = 18.0f; //empty tank + battery. Derived from REAL log: MOT_THST_HOVER=0.165 x (4x26.5 kgf) ~= 17.5 kg
            real_T motor_assembly_weight = 1.8f; //9620 motor + 36" prop + ESC combo (1774 g spec)
            real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

            //Thrust: keep UIUC C_T/C_P shape, solve max_rpm so C_T*rho*n^2*D^4 == 26.5 kgf (259.9 N):
            //n = 52.5 rev/s -> ~3150 RPM. Tip speed pi*0.914*52.5 = 151 m/s (peak only).
            params.rotor_params.C_T = 0.109919f;
            params.rotor_params.C_P = 0.040164f;
            params.rotor_params.max_rpm = 3150.0f;
            params.rotor_params.propeller_diameter = 0.914f; //36 inch
            params.rotor_params.control_signal_filter_tc = 0.04f; //~40ms spool-up: 36" prop J scales to ~0.54x of the 43" (mass~300g, D^2), similar electrical damping (100KV, R~0.07) -> tau ~= 0.54*58ms
            params.rotor_params.calculateMaxThrust();
            //hover check (empty): 18 kg -> 176.6 N -> 44.1 N/rotor = 17.0% of 259.9 N max (real log hovers at 16.5% - matches!)
            //hover check (20 L full): 38 kg -> 372.8 N -> 93.2 N/rotor = 35.9% of max

            //central modular body (tank bay) - estimate
            params.body_box.x() = 0.60f;
            params.body_box.y() = 0.50f;
            params.body_box.z() = 0.45f;
            real_T rotor_z = 0.05f;

            initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

            //Small aft CG (battery behind center). ESTIMATE - measure the balance point and update.
            const real_T cg_offset_x = -0.05f;
            //Lateral CG offset TO THE RIGHT, derived from the REAL autotune log: right-side motors
            //(DJI-X C1,C2) averaged ~73 PWM higher than left (C3,C4) -> CG ~2.5 cm right (+y, NED).
            //This lateral imbalance is a prime suspect for the real "tips sideways" symptom.
            const real_T cg_offset_y = 0.025f;
            for (auto& pose : params.rotor_poses) {
                pose.position.x() -= cg_offset_x;
                pose.position.y() -= cg_offset_y;
            }

            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        }

    private:
        Params params_;
        SensorCollection sensors_; //maintains sensor type indexed collection of sensors
        vector<shared_ptr<SensorBase>> sensor_storage_; //RAII for created sensors
    };
}
} //namespace
#endif
