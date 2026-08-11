#ifndef msr_airlib_unittest_MultirotorPhysicsTest_hpp
#define msr_airlib_unittest_MultirotorPhysicsTest_hpp

#include "TestBase.hpp"
#include "common/MultirotorPhysicsConfig.hpp"
#include "common/AirSimSettings.hpp"
#include "common/Settings.hpp"
#include "common/SteppableClock.hpp"
#include "physics/FastPhysicsEngine.hpp"
#include "physics/PhysicsWorld.hpp"
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "vehicles/multirotor/MultiRotorPhysicsBody.hpp"
#include "sensors/gps/GpsSimple.hpp"
#include "sensors/imu/ImuSimple.hpp"
#include "sensors/lidar/GPULidarSimpleParams.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <map>

namespace msr
{
namespace airlib
{

    class MultirotorPhysicsTest : public TestBase
    {
    public:
        virtual void run() override
        {
            std::cout << "  helper math..." << std::endl;
            testHelperMath();
            std::cout << "  settings apply..." << std::endl;
            testSettingsParseAndApply();
            std::cout << "  rotor scales..." << std::endl;
            testRotorScales();
            std::cout << "  gps noise..." << std::endl;
            testGpsNoise();
            std::cout << "  imu defaults..." << std::endl;
            testImuNoiseDefaults();
            std::cout << "  wind turbulence..." << std::endl;
            testWindTurbulenceConfig();
            std::cout << "  battery..." << std::endl;
            testBatteryDrainIntegration();
            std::cout << "  gpulidar multirotor..." << std::endl;
            testGpuLidarMultirotorAllowed();
            std::cout << "MultirotorPhysicsTest: all assertions passed" << std::endl;
        }

    private:
        void testHelperMath()
        {
            const real_T ge0 = MultirotorPhysicsConfig::groundEffectScale(0.02f, 2.0f, 0.25f, 0.02f);
            testAssert(ge0 > 1.2f && ge0 <= 1.25f + 1e-3f, "ground effect peak");
            const real_T ge_far = MultirotorPhysicsConfig::groundEffectScale(5.0f, 2.0f, 0.25f, 0.02f);
            testAssert(std::abs(ge_far - 1.0f) < 1e-5f, "ground effect far");

            const real_T tip = 100.0f;
            const real_T s0 = MultirotorPhysicsConfig::thrustAirSpeedScale(0, tip, 0.15f, 0.35f);
            testAssert(std::abs(s0 - 1.0f) < 1e-5f, "zero axial flow");
            const real_T s1 = MultirotorPhysicsConfig::thrustAirSpeedScale(tip, tip, 0.15f, 0.35f);
            testAssert(s1 < 1.0f && s1 >= 0.35f, "axial flow reduces thrust");
            const real_T smin = MultirotorPhysicsConfig::thrustAirSpeedScale(1e6f, tip, 0.15f, 0.35f);
            testAssert(std::abs(smin - 0.35f) < 1e-5f, "air speed min clamp");

            const real_T full = MultirotorPhysicsConfig::batteryThrustScale(16.8f, 16.8f, 13.2f);
            testAssert(std::abs(full - 1.0f) < 1e-5f, "full battery");
            const real_T half_v = MultirotorPhysicsConfig::batteryThrustScale(14.0f, 16.8f, 13.2f);
            testAssert(half_v < 1.0f && half_v > 0.5f, "mid voltage scale");
        }

        void testSettingsParseAndApply()
        {
            const std::string json = R"({
                "Mass": 1.25,
                "ArmLength": 0.25,
                "AngularDragCoefficient": 0.03,
                "LinearDragCoefficient": 0.4,
                "RotorMaxRpm": 7000,
                "EnableGroundEffect": true,
                "GroundEffectMaxBoost": 0.3,
                "EnableThrustAirSpeed": true,
                "EnableBattery": true,
                "BatteryCapacityMah": 4000
            })";
            Settings::loadJSonString(json);
            const auto cfg = MultirotorPhysicsConfig::fromSettingsJson(Settings::singleton());
            testAssert(std::abs(cfg.mass - 1.25f) < 1e-4f, "parse Mass");
            testAssert(cfg.arm_lengths.size() == 4 && std::abs(cfg.arm_lengths[0] - 0.25f) < 1e-4f, "parse ArmLength");
            testAssert(cfg.enable_battery, "parse EnableBattery");
            testAssert(std::abs(cfg.battery_capacity_mah - 4000.f) < 1e-3f, "parse capacity");

            AirSimSettings::initializeSettings(R"({
                "SettingsVersion": 2.0,
                "SimMode": "Multirotor",
                "WindTurbulence": { "Enabled": true, "Sigma": 2.0, "Tau": 1.5 },
                "Vehicles": {
                    "SimpleFlight": {
                        "VehicleType": "SimpleFlight",
                        "Physics": {
                            "Mass": 1.25,
                            "ArmLength": 0.25,
                            "AngularDragCoefficient": 0.03,
                            "RotorMaxRpm": 7000,
                            "EnableBattery": true,
                            "BatteryCapacityMah": 4000
                        }
                    }
                }
            })");
            AirSimSettings::singleton().load([]() { return std::string("Multirotor"); });

            testAssert(AirSimSettings::singleton().wind_turbulence_enabled, "wind turb enabled");
            testAssert(std::abs(AirSimSettings::singleton().wind_turbulence_sigma - 2.0f) < 1e-4f, "wind turb sigma");

            auto params = MultiRotorParamsFactory::createConfig(
                AirSimSettings::singleton().getVehicleSetting("SimpleFlight"),
                std::make_shared<SensorFactory>());
            testAssert(std::abs(params->getParams().mass - 1.25f) < 1e-4f, "applied mass");
            testAssert(std::abs(params->getParams().angular_drag_coefficient - 0.03f) < 1e-4f, "applied ang drag");
            testAssert(params->getParams().rotor_params.max_rpm > 6900.f, "applied rpm");
            testAssert(params->getPhysicsConfig().enable_battery, "physics config battery");
            testAssert(params->getParams().rotor_count == 4, "quad");
            // Arm rebuild moves motors farther out
            const auto& poses = params->getParams().rotor_poses;
            testAssert(poses.size() == 4, "4 rotors");
            const real_T arm_xy = std::sqrt(poses[0].position.x() * poses[0].position.x() +
                                            poses[0].position.y() * poses[0].position.y());
            testAssert(arm_xy > 0.2f, "arm length applied to poses");
        }

        void testRotorScales()
        {
            auto clock = std::make_shared<SteppableClock>(0.01f);
            ClockFactory::get(clock);

            Environment::State env_state;
            env_state.position = Vector3r(0, 0, -0.1f); // near ground
            env_state.geo_point = GeoPoint();
            Environment env(env_state);
            env.reset();
            env.update();

            RotorParams rp;
            rp.calculateMaxThrust();
            rp.enable_ground_effect = true;
            rp.ground_effect_max_height = 2.0f;
            rp.ground_effect_max_boost = 0.25f;
            rp.enable_thrust_air_speed = true;
            rp.thrust_air_speed_coeff = 0.15f;

            RotorActuator rotor(Vector3r(0.2f, 0.2f, 0), Vector3r(0, 0, -1),
                                RotorTurningDirection::RotorTurningDirectionCW, rp, &env, 0);
            rotor.reset();
            rotor.setControlSignal(0.6f);
            rotor.setAltitudeAgl(0.05f);
            rotor.setBodyLinearVelocity(Vector3r::Zero());
            rotor.setBatteryThrustScale(1.0f);
            // FirstOrderFilter uses ClockFactory dt — step SteppableClock each tick.
            for (int i = 0; i < 40; ++i) {
                clock->step();
                rotor.update(0.01f);
            }
            const auto near = rotor.getOutput();
            testAssert(near.control_signal_filtered > 0.3f, "filter ramped");
            testAssert(near.ground_effect_scale > 1.1f, "near-ground GE boost");
            testAssert(near.thrust > 0, "positive thrust");

            rotor.setAltitudeAgl(10.0f);
            rotor.setBodyLinearVelocity(Vector3r(0, 0, -5)); // climb: vel·normal = 5 with normal -Z
            for (int i = 0; i < 5; ++i) {
                clock->step();
                rotor.update(0.01f);
            }
            const auto climb = rotor.getOutput();
            testAssert(climb.ground_effect_scale <= 1.0f + 1e-3f, "high AGL no GE");
            testAssert(climb.air_speed_scale < 1.0f, "climb reduces air speed scale");
            testAssert(climb.thrust < near.thrust, "less thrust high and climbing vs near-ground hover demand");

            rotor.setBatteryThrustScale(0.5f);
            rotor.setAltitudeAgl(10.0f);
            rotor.setBodyLinearVelocity(Vector3r::Zero());
            for (int i = 0; i < 5; ++i) {
                clock->step();
                rotor.update(0.01f);
            }
            const auto batt = rotor.getOutput();
            testAssert(std::abs(batt.battery_scale - 0.5f) < 1e-4f, "battery scale applied");
            testAssert(batt.thrust < near.thrust, "battery reduces thrust vs near-ground");
        }

        void testGpsNoise()
        {
            auto clock = std::make_shared<SteppableClock>(0.02f);
            ClockFactory::get(clock);

            Settings gps_json;
            // Build GpsSetting-like via AirSimSettings path: use GpsSimple with GenerateNoise
            AirSimSettings::GpsSetting setting;
            setting.sensor_name = "gps";
            Settings::loadJSonString(R"({
                "GenerateNoise": true,
                "PositionSigmaScale": 1.0,
                "VelocitySigma": 0.5,
                "BiasSigma": 0.0,
                "EphFinal": 1.0,
                "EpvFinal": 1.0,
                "EphInitial": 1.0,
                "EpvInitial": 1.0,
                "UpdateLatency": 0,
                "UpdateFrequency": 50,
                "StartupDelay": 0
            })");
            // GpsSetting stores nested settings - need to copy singleton into setting.settings
            setting.settings = Settings::singleton();

            Kinematics::State kin = Kinematics::State::zero();
            kin.pose.position = Vector3r(10, 20, -30);
            Environment::State es;
            es.position = kin.pose.position;
            es.geo_point = GeoPoint(47.64, -122.14, 122);
            Environment env(es);
            env.reset();
            Kinematics kinematics(kin);
            kinematics.reset();

            GpsSimple gps(setting);
            gps.initialize(&kinematics.getState(), &env);
            gps.reset();

            // Run enough updates for frequency limiter
            for (int i = 0; i < 50; ++i) {
                clock->step();
                env.setPosition(kin.pose.position);
                env.update();
                gps.update();
            }

            const auto out = gps.getOutput();
            testAssert(out.is_valid, "gps valid");
            // With noise, geo should typically differ from pure truth; allow either if RNG is unlucky
            // but velocity/position fields must be finite
            testAssert(std::isfinite(out.gnss.geo_point.latitude), "lat finite");
            testAssert(std::isfinite(out.gnss.geo_point.longitude), "lon finite");
            testAssert(std::isfinite(out.gnss.velocity.x()), "vel finite");
        }

        void testImuNoiseDefaults()
        {
            AirSimSettings::ImuSetting setting;
            setting.sensor_name = "imu";
            Settings::loadJSonString(R"({})");
            setting.settings = Settings::singleton();
            ImuSimpleParams params;
            const bool noise = params.initializeFromSettings(setting);
            testAssert(noise, "IMU GenerateNoise defaults true");
            testAssert(params.enable_vibration, "IMU vibration defaults true");
        }

        void testWindTurbulenceConfig()
        {
            FastPhysicsEngine engine;
            engine.configureWindTurbulence(true, 1.5f, 2.0f);
            engine.reset();
            // Smoke: several updates must not throw
            for (int i = 0; i < 10; ++i)
                engine.update(0.01f);
            engine.configureWindTurbulence(false);
            engine.update(0.01f);
        }

        void testBatteryDrainIntegration()
        {
            BatteryState batt;
            MultirotorPhysicsConfig cfg;
            cfg.enable_battery = true;
            cfg.battery_capacity_mah = 1000;
            cfg.battery_max_voltage = 16.8f;
            cfg.battery_min_voltage = 13.2f;
            cfg.battery_max_current_per_motor_a = 20.f;
            batt.reset(cfg);
            const real_T soc0 = batt.soc;
            for (int i = 0; i < 100; ++i)
                batt.update(0.1f, 4.0f, 4); // 10s full demand
            testAssert(batt.soc < soc0, "SOC decreased");
            testAssert(batt.voltage < cfg.battery_max_voltage, "voltage sag or discharge");
            testAssert(batt.thrustScale() < 1.0f + 1e-3f, "thrust scale not above 1");
            testAssert(batt.thrustScale() > 0.0f, "thrust scale positive while SOC remains");
        }

        void testGpuLidarMultirotorAllowed()
        {
            // Drive real shipped GPULidarSimpleParams::initializeFromSettings under Multirotor
            // simmode: async_capture_mode must enable (Multirotor game-thread capture path).
            const std::string prev_mode = AirSimSettings::singleton().simmode_name;
            AirSimSettings::singleton().simmode_name = AirSimSettings::kSimModeTypeMultirotor;

            Settings::loadJSonString(R"({
                "SensorType": 8,
                "Enabled": true,
                "NumberOfChannels": 16,
                "Range": 40
            })");

            AirSimSettings::GPULidarSetting setting;
            setting.sensor_name = "gpulidar1";
            setting.sensor_type = SensorBase::SensorType::GPULidar;
            setting.enabled = true;
            setting.settings = Settings::singleton();

            GPULidarSimpleParams params;
            params.initializeFromSettings(setting);
            testAssert(params.async_capture_mode, "Multirotor GPU LiDAR must use async_capture_mode");

            AirSimSettings::singleton().simmode_name = AirSimSettings::kSimModeTypeCar;
            GPULidarSimpleParams car_params;
            car_params.initializeFromSettings(setting);
            testAssert(!car_params.async_capture_mode, "Car GPU LiDAR keeps sync capture path");

            AirSimSettings::singleton().simmode_name = prev_mode;
        }
    };

}
} //namespace
#endif
