// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef rotor_actuator_hpp
#define rotor_actuator_hpp

#include <limits>
#include "common/Common.hpp"
#include "physics/Environment.hpp"
#include "common/FirstOrderFilter.hpp"
#include "physics/PhysicsBodyVertex.hpp"
#include "RotorParams.hpp"
#include "common/MultirotorPhysicsConfig.hpp"

namespace msr
{
namespace airlib
{

    //Rotor gets control signal as input (PWM or voltage represented from 0 to 1) which causes
    //change in rotation speed and turning direction and ultimately produces force and thrust as
    //output. Also scales by air density, ground effect, axial flow, and battery when enabled.
    class RotorActuator : public PhysicsBodyVertex
    {
    public: //types
        struct Output
        {
            real_T thrust;
            real_T torque_scaler;
            real_T speed;
            RotorTurningDirection turning_direction;
            real_T control_signal_filtered;
            real_T control_signal_input;
            real_T ground_effect_scale = 1.0f;
            real_T air_speed_scale = 1.0f;
            real_T battery_scale = 1.0f;
        };

    public: //methods
        RotorActuator()
        {
            //allow default constructor with later call for initialize
        }
        RotorActuator(const Vector3r& position, const Vector3r& normal, RotorTurningDirection turning_direction,
                      const RotorParams& params, const Environment* environment, uint id = -1)
        {
            initialize(position, normal, turning_direction, params, environment, id);
        }
        void initialize(const Vector3r& position, const Vector3r& normal, RotorTurningDirection turning_direction,
                        const RotorParams& params, const Environment* environment, uint id = -1)
        {
            id_ = id;
            params_ = params;
            turning_direction_ = turning_direction;
            environment_ = environment;
            air_density_sea_level_ = EarthUtils::getAirDensity(0.0f);

            control_signal_filter_.initialize(params_.control_signal_filter_tc, 0, 0);

            PhysicsBodyVertex::initialize(position, normal); //call base initializer
        }

        //0 to 1 - will be scaled to 0 to max_speed
        void setControlSignal(real_T control_signal)
        {
            control_signal_filter_.setInput(Utils::clip(control_signal, 0.0f, 1.0f));
        }

        /** Body-frame linear velocity (m/s, NED body). Used for axial-flow thrust reduction. */
        void setBodyLinearVelocity(const Vector3r& body_linear_vel)
        {
            body_linear_vel_ = body_linear_vel;
        }

        /** Flat-world AGL proxy (m). Typically max(0, -NED.z) when origin is on the ground plane. */
        void setAltitudeAgl(real_T agl_m)
        {
            agl_m_ = agl_m;
        }

        /** Multiplier from battery voltage (1 = full). */
        void setBatteryThrustScale(real_T scale)
        {
            battery_scale_ = Utils::clip(scale, 0.0f, 1.0f);
        }

        Output getOutput() const
        {
            return output_;
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            PhysicsBodyVertex::resetImplementation();

            //update environmental factors before we call base
            updateEnvironmentalFactors();

            control_signal_filter_.reset();
            body_linear_vel_ = Vector3r::Zero();
            agl_m_ = 1000.0f;
            battery_scale_ = 1.0f;

            setOutput(output_, params_, control_signal_filter_, turning_direction_, getNormal(),
                      body_linear_vel_, agl_m_, battery_scale_);
        }

        virtual void update(float delta = 0) override
        {
            //update environmental factors before we call base
            updateEnvironmentalFactors();

            //this will in turn call setWrench
            PhysicsBodyVertex::update(delta);

            //update our state
            setOutput(output_, params_, control_signal_filter_, turning_direction_, getNormal(),
                      body_linear_vel_, agl_m_, battery_scale_);

            //update filter - this should be after so that first output is same as initial
            control_signal_filter_.update(delta);
        }

        virtual void reportState(StateReporter& reporter) override
        {
            reporter.writeValue("Dir", static_cast<int>(turning_direction_));
            reporter.writeValue("Ctrl-in", output_.control_signal_input);
            reporter.writeValue("Ctrl-fl", output_.control_signal_filtered);
            reporter.writeValue("speed", output_.speed);
            reporter.writeValue("thrust", output_.thrust);
            reporter.writeValue("torque", output_.torque_scaler);
            reporter.writeValue("GE", output_.ground_effect_scale);
            reporter.writeValue("AirSpdSc", output_.air_speed_scale);
            reporter.writeValue("BattSc", output_.battery_scale);
        }
        //*** End: UpdatableState implementation ***//

    protected:
        virtual void setWrench(Wrench& wrench) override
        {
            Vector3r normal = getNormal();
            //forces and torques are proportional to air density: http://physics.stackexchange.com/a/32013/14061
            wrench.force = normal * output_.thrust * air_density_ratio_;
            wrench.torque = normal * output_.torque_scaler * air_density_ratio_;
        }

    private: //methods
        static void setOutput(Output& output, const RotorParams& params,
                              const FirstOrderFilter<real_T>& control_signal_filter,
                              RotorTurningDirection turning_direction,
                              const Vector3r& rotor_normal_body,
                              const Vector3r& body_linear_vel,
                              real_T agl_m,
                              real_T battery_scale)
        {
            output.control_signal_input = control_signal_filter.getInput();
            output.control_signal_filtered = control_signal_filter.getOutput();
            //see relationship of rotation speed with thrust: http://physics.stackexchange.com/a/32013/14061
            output.speed = sqrt(output.control_signal_filtered * params.max_speed_square);

            real_T ge = 1.0f;
            if (params.enable_ground_effect) {
                ge = MultirotorPhysicsConfig::groundEffectScale(
                    agl_m,
                    params.ground_effect_max_height,
                    params.ground_effect_max_boost,
                    params.ground_effect_min_height);
            }
            output.ground_effect_scale = ge;

            real_T air = 1.0f;
            if (params.enable_thrust_air_speed) {
                // Axial inflow along rotor thrust axis (body frame). Positive when air approaches
                // from the inflow side (reduces effective thrust in climb/forward flight).
                const real_T v_axial = body_linear_vel.dot(rotor_normal_body);
                const real_T tip = params.tip_speed > 0
                    ? params.tip_speed
                    : MultirotorPhysicsConfig::tipSpeedMps(params.max_speed, params.propeller_diameter);
                air = MultirotorPhysicsConfig::thrustAirSpeedScale(
                    v_axial, tip, params.thrust_air_speed_coeff, params.thrust_air_speed_min_scale);
            }
            output.air_speed_scale = air;
            output.battery_scale = battery_scale;

            const real_T scale = ge * air * battery_scale;
            output.thrust = output.control_signal_filtered * params.max_thrust * scale;
            output.torque_scaler = output.control_signal_filtered * params.max_torque * scale *
                                   static_cast<int>(turning_direction);
            output.turning_direction = turning_direction;
        }

        void updateEnvironmentalFactors()
        {
            //update air density ration - this will affect generated force and torques by rotors
            air_density_ratio_ = environment_->getState().air_density / air_density_sea_level_;
        }

    private: //fields
        uint id_; //only used for debug messages
        RotorTurningDirection turning_direction_;
        RotorParams params_;
        FirstOrderFilter<real_T> control_signal_filter_;
        const Environment* environment_ = nullptr;
        real_T air_density_sea_level_, air_density_ratio_;
        Output output_;
        Vector3r body_linear_vel_ = Vector3r::Zero();
        real_T agl_m_ = 1000.0f;
        real_T battery_scale_ = 1.0f;
    };
}
} //namespace
#endif
