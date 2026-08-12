// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_MultirotorPhysicsConfig_hpp
#define msr_airlib_MultirotorPhysicsConfig_hpp

#include "Common.hpp"
#include "Settings.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace msr
{
namespace airlib
{

    /** Multirotor plant overrides loaded from vehicle settings "Physics". */
    struct MultirotorPhysicsConfig
    {
        // --- Frame / mass (NaN = keep frame defaults) ---
        real_T mass = Utils::nan<real_T>();
        real_T motor_assembly_weight = Utils::nan<real_T>();
        Vector3r body_box = Vector3r(Utils::nan<real_T>(), Utils::nan<real_T>(), Utils::nan<real_T>());
        Vector3r inertia_diagonal = Vector3r(Utils::nan<real_T>(), Utils::nan<real_T>(), Utils::nan<real_T>());
        std::vector<real_T> arm_lengths; // if size matches rotor_count, rebuild poses
        /** Symmetric ArmLength from settings; expanded to rotor_count in applyPhysicsConfig. */
        real_T arm_length = Utils::nan<real_T>();
        real_T rotor_z = Utils::nan<real_T>();

        real_T linear_drag_coefficient = Utils::nan<real_T>();
        real_T angular_drag_coefficient = Utils::nan<real_T>();
        real_T restitution = Utils::nan<real_T>();
        real_T friction = Utils::nan<real_T>();

        // --- Rotor coefficients ---
        real_T rotor_C_T = Utils::nan<real_T>();
        real_T rotor_C_P = Utils::nan<real_T>();
        real_T rotor_max_rpm = Utils::nan<real_T>();
        real_T propeller_diameter = Utils::nan<real_T>();
        real_T propeller_height = Utils::nan<real_T>();
        real_T control_signal_filter_tc = Utils::nan<real_T>();

        // --- Aero enhancements (enabled by default for realism; disable via settings) ---
        bool enable_ground_effect = true;
        real_T ground_effect_max_height = 2.0f; // m AGL (flat-world: -NED.z relative to origin)
        real_T ground_effect_max_boost = 0.25f; // fraction extra thrust at z→0
        real_T ground_effect_min_height = 0.02f;

        bool enable_thrust_air_speed = true;
        // thrust *= clamp(1 - k * |v_axial| / tip_speed, min_scale, 1)
        real_T thrust_air_speed_coeff = 0.15f;
        real_T thrust_air_speed_min_scale = 0.35f;

        // --- Battery (optional) ---
        bool enable_battery = false;
        real_T battery_capacity_mah = 5000.0f;
        real_T battery_max_voltage = 16.8f; // 4S full
        real_T battery_min_voltage = 13.2f; // empty cutoff
        real_T battery_internal_resistance = 0.015f; // ohm
        real_T battery_idle_current_a = 0.3f;
        real_T battery_max_current_per_motor_a = 25.0f;

        static MultirotorPhysicsConfig fromSettingsJson(const Settings& json)
        {
            MultirotorPhysicsConfig cfg;
            cfg.mass = json.getFloat("Mass", cfg.mass);
            cfg.motor_assembly_weight = json.getFloat("MotorAssemblyWeight", cfg.motor_assembly_weight);
            cfg.rotor_z = json.getFloat("RotorZ", cfg.rotor_z);
            cfg.linear_drag_coefficient = json.getFloat("LinearDragCoefficient", cfg.linear_drag_coefficient);
            cfg.angular_drag_coefficient = json.getFloat("AngularDragCoefficient", cfg.angular_drag_coefficient);
            cfg.restitution = json.getFloat("Restitution", cfg.restitution);
            cfg.friction = json.getFloat("Friction", cfg.friction);

            cfg.rotor_C_T = json.getFloat("RotorCT", cfg.rotor_C_T);
            cfg.rotor_C_P = json.getFloat("RotorCP", cfg.rotor_C_P);
            cfg.rotor_max_rpm = json.getFloat("RotorMaxRpm", cfg.rotor_max_rpm);
            cfg.propeller_diameter = json.getFloat("PropellerDiameter", cfg.propeller_diameter);
            cfg.propeller_height = json.getFloat("PropellerHeight", cfg.propeller_height);
            cfg.control_signal_filter_tc = json.getFloat("ControlSignalFilterTC", cfg.control_signal_filter_tc);

            cfg.enable_ground_effect = json.getBool("EnableGroundEffect", cfg.enable_ground_effect);
            cfg.ground_effect_max_height = json.getFloat("GroundEffectMaxHeight", cfg.ground_effect_max_height);
            cfg.ground_effect_max_boost = json.getFloat("GroundEffectMaxBoost", cfg.ground_effect_max_boost);
            cfg.ground_effect_min_height = json.getFloat("GroundEffectMinHeight", cfg.ground_effect_min_height);

            cfg.enable_thrust_air_speed = json.getBool("EnableThrustAirSpeed", cfg.enable_thrust_air_speed);
            cfg.thrust_air_speed_coeff = json.getFloat("ThrustAirSpeedCoeff", cfg.thrust_air_speed_coeff);
            cfg.thrust_air_speed_min_scale = json.getFloat("ThrustAirSpeedMinScale", cfg.thrust_air_speed_min_scale);

            cfg.enable_battery = json.getBool("EnableBattery", cfg.enable_battery);
            cfg.battery_capacity_mah = json.getFloat("BatteryCapacityMah", cfg.battery_capacity_mah);
            cfg.battery_max_voltage = json.getFloat("BatteryMaxVoltage", cfg.battery_max_voltage);
            cfg.battery_min_voltage = json.getFloat("BatteryMinVoltage", cfg.battery_min_voltage);
            cfg.battery_internal_resistance = json.getFloat("BatteryInternalResistance", cfg.battery_internal_resistance);
            cfg.battery_idle_current_a = json.getFloat("BatteryIdleCurrent", cfg.battery_idle_current_a);
            cfg.battery_max_current_per_motor_a = json.getFloat("BatteryMaxCurrentPerMotor", cfg.battery_max_current_per_motor_a);

            Settings box;
            if (json.getChild("BodyBox", box)) {
                cfg.body_box.x() = box.getFloat("X", cfg.body_box.x());
                cfg.body_box.y() = box.getFloat("Y", cfg.body_box.y());
                cfg.body_box.z() = box.getFloat("Z", cfg.body_box.z());
            }

            Settings inertia;
            if (json.getChild("InertiaDiagonal", inertia)) {
                cfg.inertia_diagonal.x() = inertia.getFloat("X", cfg.inertia_diagonal.x());
                cfg.inertia_diagonal.y() = inertia.getFloat("Y", cfg.inertia_diagonal.y());
                cfg.inertia_diagonal.z() = inertia.getFloat("Z", cfg.inertia_diagonal.z());
            }

            // Symmetric ArmLength (expanded to rotor_count in applyPhysicsConfig) and optional
            // per-rotor ArmLengths array (must match frame rotor_count for rebuild).
            cfg.arm_length = json.getFloat("ArmLength", cfg.arm_length);
            {
                const std::vector<float> arms = json.getFloatArray("ArmLengths");
                if (!arms.empty()) {
                    cfg.arm_lengths.assign(arms.begin(), arms.end());
                }
            }

            return cfg;
        }

        /** Ground-effect thrust multiplier (>= 1). */
        static real_T groundEffectScale(real_T agl_m, real_T max_height, real_T max_boost, real_T min_height)
        {
            if (max_height <= 0 || max_boost <= 0)
                return 1.0f;
            const real_T z = std::max(agl_m, min_height);
            if (z >= max_height)
                return 1.0f;
            const real_T t = 1.0f - (z / max_height);
            // Smooth quadratic fade: peak boost at ground
            return 1.0f + max_boost * t * t;
        }

        /** Axial-flow thrust scale in [min_scale, 1]. v_axial along rotor normal (m/s). */
        static real_T thrustAirSpeedScale(real_T v_axial_mps, real_T tip_speed_mps, real_T coeff, real_T min_scale)
        {
            if (tip_speed_mps <= 1e-3f || coeff <= 0)
                return 1.0f;
            const real_T ratio = std::abs(v_axial_mps) / tip_speed_mps;
            const real_T scale = 1.0f - coeff * ratio;
            return Utils::clip(scale, min_scale, 1.0f);
        }

        static real_T tipSpeedMps(real_T max_speed_rad_s, real_T propeller_diameter_m)
        {
            return max_speed_rad_s * (propeller_diameter_m * 0.5f);
        }

        /** Voltage / SOC thrust scale (approx motor V^2). */
        static real_T batteryThrustScale(real_T voltage, real_T max_voltage, real_T min_voltage)
        {
            if (max_voltage <= min_voltage)
                return 1.0f;
            const real_T v = Utils::clip(voltage, min_voltage, max_voltage);
            const real_T n = v / max_voltage;
            return n * n;
        }
    };

    struct BatteryState
    {
        bool enabled = false;
        real_T capacity_mah = 5000.0f;
        real_T soc = 1.0f; // 0..1
        real_T voltage = 16.8f;
        real_T current_a = 0.0f;
        real_T max_voltage = 16.8f;
        real_T min_voltage = 13.2f;
        real_T internal_resistance = 0.015f;
        real_T idle_current_a = 0.3f;
        real_T max_current_per_motor_a = 25.0f;

        void reset(const MultirotorPhysicsConfig& cfg)
        {
            enabled = cfg.enable_battery;
            capacity_mah = std::max(1.0f, cfg.battery_capacity_mah);
            max_voltage = cfg.battery_max_voltage;
            min_voltage = cfg.battery_min_voltage;
            internal_resistance = std::max(0.0f, cfg.battery_internal_resistance);
            idle_current_a = std::max(0.0f, cfg.battery_idle_current_a);
            max_current_per_motor_a = std::max(0.0f, cfg.battery_max_current_per_motor_a);
            soc = 1.0f;
            current_a = 0.0f;
            voltage = max_voltage;
        }

        real_T thrustScale() const
        {
            if (!enabled)
                return 1.0f;
            return MultirotorPhysicsConfig::batteryThrustScale(voltage, max_voltage, min_voltage);
        }

        void update(real_T dt, real_T sum_control_signals, uint /*motor_count*/)
        {
            if (!enabled || dt <= 0)
                return;

            // sum_control_signals is sum of 0..1 motor demands (≈ count * throttle).
            const real_T motor_current =
                sum_control_signals * max_current_per_motor_a;
            current_a = idle_current_a + motor_current;

            // mAh drain
            const real_T mah = current_a * (dt / 3600.0f) * 1000.0f;
            soc = Utils::clip(soc - mah / capacity_mah, 0.0f, 1.0f);

            const real_T ocv = min_voltage + (max_voltage - min_voltage) * soc;
            voltage = std::max(min_voltage * 0.5f, ocv - current_a * internal_resistance);
        }
    };

}
} //namespace
#endif
