// Copyright (c) Cosys-Lab. All rights reserved.
// Battery Sensor - Integrated from ProjectAirSim architecture

#ifndef airsim_core_BatterySensorBase_hpp
#define airsim_core_BatterySensorBase_hpp

#include "sensors/SensorBase.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {

class BatterySensorBase : public SensorBase {
public:
    enum class SimulationMode {
        Linear,          // Simple linear discharge
        VoltageBased,    // Voltage curve based discharge
        PhysicsBased     // Realistic physics with load calculation
    };

    struct BatteryState {
        TTimePoint time_stamp;
        float voltage;              // Current voltage (V)
        float current;              // Current draw (A)
        float charge_remaining;     // Remaining charge (mAh)
        float charge_consumed;      // Consumed charge (mAh)
        float percentage;           // State of charge (0-100%)
        float power;                // Current power draw (W)
        float time_remaining;       // Estimated time remaining (seconds)
        bool is_charging;           // Charging state
        bool is_critical;           // Critical battery level
        
        BatteryState()
            : time_stamp(0), voltage(0), current(0), 
              charge_remaining(0), charge_consumed(0), percentage(100),
              power(0), time_remaining(0), is_charging(false), is_critical(false)
        {}
    };

    struct BatteryConfig {
        float capacity_mah;         // Battery capacity (mAh)
        float nominal_voltage;      // Nominal voltage (V)
        float max_voltage;          // Maximum voltage (V)
        float min_voltage;          // Minimum voltage (V)
        float internal_resistance;  // Internal resistance (Ohm)
        float critical_percentage;  // Critical level percentage
        SimulationMode simulation_mode;
        
        BatteryConfig()
            : capacity_mah(5000.0f), nominal_voltage(14.8f),
              max_voltage(16.8f), min_voltage(12.0f),
              internal_resistance(0.05f), critical_percentage(20.0f),
              simulation_mode(SimulationMode::Linear)
        {}
    };

public:
    BatterySensorBase(const std::string& sensor_name = "")
        : SensorBase(sensor_name)
    {}

    virtual ~BatterySensorBase() = default;

    // Get current battery state
    virtual const BatteryState& getBatteryState() const = 0;
    
    // Update battery based on power consumption
    virtual void updateBattery(float power_watts, float dt) = 0;
    
    // Reset battery to full charge
    virtual void resetBattery() = 0;
    
    // Set charging state
    virtual void setCharging(bool charging) = 0;
    
    // Get configuration
    virtual const BatteryConfig& getConfig() const = 0;

public:
    // Sensor type for registration
    static constexpr int SensorType = 12;
};

}} // namespace
#endif
