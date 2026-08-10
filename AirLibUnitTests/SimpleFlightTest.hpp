
#ifndef msr_AirLibUnitTests_SimpleFlightTest_hpp
#define msr_AirLibUnitTests_SimpleFlightTest_hpp

#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "TestBase.hpp"
#include "common/AirSimSettings.hpp"
#include "common/EarthUtils.hpp"
#include <cmath>
#include <iostream>

namespace msr
{
namespace airlib
{

    /**
     * Validates settings-driven multirotor plant configuration used by SimpleFlight / Blocks.
     * Full async PhysicsWorld takeoff is covered manually in UE; MultirotorPhysicsTest covers
     * rotor/battery/GPS/IMU/wind unit behavior with a deterministic clock.
     */
    class SimpleFlightTest : public TestBase
    {
    public:
        virtual void run() override
        {
            AirSimSettings::initializeSettings(R"({
                "SettingsVersion": 2.0,
                "SimMode": "Multirotor",
                "WindTurbulence": { "Enabled": true, "Sigma": 1.2, "Tau": 2.0 },
                "Vehicles": {
                    "SimpleFlight": {
                        "VehicleType": "SimpleFlight",
                        "Physics": {
                            "Mass": 1.15,
                            "ArmLength": 0.23,
                            "AngularDragCoefficient": 0.03,
                            "LinearDragCoefficient": 0.4,
                            "EnableGroundEffect": true,
                            "GroundEffectMaxBoost": 0.3,
                            "EnableThrustAirSpeed": true,
                            "ThrustAirSpeedCoeff": 0.12,
                            "EnableBattery": true,
                            "BatteryCapacityMah": 3000,
                            "RotorMaxRpm": 7000,
                            "ControlSignalFilterTC": 0.01
                        }
                    }
                }
            })");
            AirSimSettings::singleton().load([]() { return std::string("Multirotor"); });

            testAssert(AirSimSettings::singleton().wind_turbulence_enabled, "wind turb setting");
            testAssert(std::abs(AirSimSettings::singleton().wind_turbulence_sigma - 1.2f) < 1e-4f, "sigma");

            auto params = MultiRotorParamsFactory::createConfig(
                AirSimSettings::singleton().getVehicleSetting("SimpleFlight"),
                std::make_shared<SensorFactory>());

            const auto& p = params->getParams();
            const auto& cfg = params->getPhysicsConfig();

            testAssert(std::abs(p.mass - 1.15f) < 1e-3f, "mass");
            testAssert(std::abs(p.angular_drag_coefficient - 0.03f) < 1e-4f, "angular drag");
            testAssert(std::abs(p.linear_drag_coefficient - 0.4f) < 1e-4f, "linear drag");
            testAssert(p.rotor_params.enable_ground_effect, "ground effect");
            testAssert(std::abs(p.rotor_params.ground_effect_max_boost - 0.3f) < 1e-4f, "ge boost");
            testAssert(p.rotor_params.enable_thrust_air_speed, "air speed thrust");
            testAssert(std::abs(p.rotor_params.thrust_air_speed_coeff - 0.12f) < 1e-4f, "air coeff");
            testAssert(cfg.enable_battery, "battery");
            testAssert(std::abs(cfg.battery_capacity_mah - 3000.f) < 1e-2f, "capacity");
            testAssert(p.rotor_params.max_rpm > 6900.f, "rpm");
            testAssert(std::abs(p.rotor_params.control_signal_filter_tc - 0.01f) < 1e-4f, "filter tc");
            testAssert(p.rotor_count == 4, "quad");
            testAssert(p.rotor_params.max_thrust > 0 && p.rotor_params.tip_speed > 0, "thrust table");

            const real_T max_lift = p.rotor_params.max_thrust * static_cast<real_T>(p.rotor_count);
            const real_T weight = p.mass * EarthUtils::Gravity;
            testAssert(max_lift > weight * 1.05f, "hover margin after settings");

            // GE / airspeed helpers still consistent with applied rotor params
            const real_T ge = MultirotorPhysicsConfig::groundEffectScale(
                0.05f, p.rotor_params.ground_effect_max_height, p.rotor_params.ground_effect_max_boost,
                p.rotor_params.ground_effect_min_height);
            testAssert(ge > 1.0f, "ge boost near ground");

            auto api = params->createMultirotorApi();
            testAssert(api != nullptr, "api created");
            std::string msg;
            testAssert(api->isReady(msg), msg.empty() ? "api ready" : msg);

            std::cout << "SimpleFlightTest OK mass=" << p.mass
                      << " maxLift=" << max_lift
                      << " weight=" << weight
                      << " margin=" << (max_lift / weight)
                      << " tipSpeed=" << p.rotor_params.tip_speed
                      << std::endl;
        }
    };
}
}
#endif
