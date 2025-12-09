// Copyright (c) Cosys-Lab. All rights reserved.
// Simple Battery Sensor Implementation

#ifndef airsim_core_BatterySensorSimple_hpp
#define airsim_core_BatterySensorSimple_hpp

#include "sensors/battery/BatterySensorBase.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {

class BatterySensorSimple : public BatterySensorBase {
public:
    BatterySensorSimple(const std::string& sensor_name = "")
        : BatterySensorBase(sensor_name)
    {
        // Initialize with default config
        config_ = BatteryConfig();
        resetBattery();
    }

    virtual ~BatterySensorSimple() = default;

    // Initialize with custom config
    void initialize(const BatteryConfig& config) {
        config_ = config;
        resetBattery();
    }

    // SensorBase overrides
    virtual void reportState(StateReporter& reporter) override {
        reporter.writeValue("Battery-Voltage", state_.voltage);
        reporter.writeValue("Battery-Current", state_.current);
        reporter.writeValue("Battery-Percentage", state_.percentage);
        reporter.writeValue("Battery-TimeRemaining", state_.time_remaining);
    }

    // BatterySensorBase implementations
    virtual const BatteryState& getBatteryState() const override {
        return state_;
    }

    virtual void updateBattery(float power_watts, float dt) override {
        if (dt <= 0 || state_.is_charging) {
            return;
        }

        switch (config_.simulation_mode) {
            case SimulationMode::Linear:
                updateLinear(power_watts, dt);
                break;
            case SimulationMode::VoltageBased:
                updateVoltageBased(power_watts, dt);
                break;
            case SimulationMode::PhysicsBased:
                updatePhysicsBased(power_watts, dt);
                break;
        }

        // Update derived values
        state_.percentage = (state_.charge_remaining / config_.capacity_mah) * 100.0f;
        state_.is_critical = state_.percentage < config_.critical_percentage;
        
        // Estimate time remaining
        if (state_.current > 0) {
            state_.time_remaining = (state_.charge_remaining / state_.current) * 3600.0f; // Convert to seconds
        } else {
            state_.time_remaining = std::numeric_limits<float>::infinity();
        }

        state_.time_stamp = clock()->nowNanos();
    }

    virtual void resetBattery() override {
        state_ = BatteryState();
        state_.voltage = config_.max_voltage;
        state_.charge_remaining = config_.capacity_mah;
        state_.percentage = 100.0f;
        state_.time_stamp = clock()->nowNanos();
    }

    virtual void setCharging(bool charging) override {
        state_.is_charging = charging;
    }

    virtual const BatteryConfig& getConfig() const override {
        return config_;
    }

protected:
    virtual void update() override {
        // Update is handled externally via updateBattery()
        BatterySensorBase::update();
    }

private:
    void updateLinear(float power_watts, float dt) {
        // Simple linear discharge
        state_.current = power_watts / config_.nominal_voltage;
        float charge_used_ah = (state_.current * dt) / 3600.0f; // Convert seconds to hours
        float charge_used_mah = charge_used_ah * 1000.0f;
        
        state_.charge_consumed += charge_used_mah;
        state_.charge_remaining = std::max(0.0f, config_.capacity_mah - state_.charge_consumed);
        state_.voltage = config_.nominal_voltage;
        state_.power = power_watts;
    }

    void updateVoltageBased(float power_watts, float dt) {
        // Voltage drops with discharge percentage
        state_.current = power_watts / config_.nominal_voltage;
        float charge_used_ah = (state_.current * dt) / 3600.0f;
        float charge_used_mah = charge_used_ah * 1000.0f;
        
        state_.charge_consumed += charge_used_mah;
        state_.charge_remaining = std::max(0.0f, config_.capacity_mah - state_.charge_consumed);
        
        // Voltage curve: max at 100%, nominal at 50%, min at 0%
        float soc = state_.charge_remaining / config_.capacity_mah;
        if (soc > 0.5f) {
            // Upper curve: linear from nominal to max
            state_.voltage = config_.nominal_voltage + 
                            (config_.max_voltage - config_.nominal_voltage) * (soc - 0.5f) * 2.0f;
        } else {
            // Lower curve: exponential drop from nominal to min
            float normalized = soc * 2.0f; // 0-1 range
            state_.voltage = config_.min_voltage + 
                            (config_.nominal_voltage - config_.min_voltage) * std::sqrt(normalized);
        }
        
        state_.power = power_watts;
    }

    void updatePhysicsBased(float power_watts, float dt) {
        // Realistic model with internal resistance
        float soc = state_.charge_remaining / config_.capacity_mah;
        
        // Base voltage from SOC
        float open_circuit_voltage;
        if (soc > 0.5f) {
            open_circuit_voltage = config_.nominal_voltage + 
                                  (config_.max_voltage - config_.nominal_voltage) * (soc - 0.5f) * 2.0f;
        } else {
            float normalized = soc * 2.0f;
            open_circuit_voltage = config_.min_voltage + 
                                  (config_.nominal_voltage - config_.min_voltage) * std::sqrt(normalized);
        }
        
        // Calculate current considering internal resistance
        // V_terminal = V_oc - I * R_internal
        // P = V_terminal * I
        // Solving quadratic: I^2 * R - I * V_oc + P = 0
        float a = config_.internal_resistance;
        float b = -open_circuit_voltage;
        float c = power_watts;
        
        float discriminant = b * b - 4 * a * c;
        if (discriminant >= 0) {
            state_.current = (-b - std::sqrt(discriminant)) / (2 * a);
        } else {
            // Power demand too high, use maximum current
            state_.current = open_circuit_voltage / config_.internal_resistance;
        }
        
        // Terminal voltage with voltage drop
        state_.voltage = open_circuit_voltage - state_.current * config_.internal_resistance;
        state_.voltage = std::max(config_.min_voltage, state_.voltage);
        
        // Update charge
        float charge_used_ah = (state_.current * dt) / 3600.0f;
        float charge_used_mah = charge_used_ah * 1000.0f;
        state_.charge_consumed += charge_used_mah;
        state_.charge_remaining = std::max(0.0f, config_.capacity_mah - state_.charge_consumed);
        
        state_.power = state_.voltage * state_.current;
    }

private:
    BatteryConfig config_;
    BatteryState state_;
};

}} // namespace
#endif
