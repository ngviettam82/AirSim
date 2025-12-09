// Copyright (c) Cosys-Lab. All rights reserved.
// Battery Sensor Parameters for settings.json

#ifndef airsim_core_BatterySensorParams_hpp
#define airsim_core_BatterySensorParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorParams.hpp"
#include "sensors/battery/BatterySensorBase.hpp"

namespace msr { namespace airlib {

class BatterySensorParams : public SensorParams {
public:
    BatterySensorParams() = default;
    virtual ~BatterySensorParams() = default;

    BatterySensorBase::BatteryConfig battery_config;

    virtual void initializeFromSettings(const AirSimSettings::SensorSetting* sensor_setting) override {
        SensorParams::initializeFromSettings(sensor_setting);

        if (sensor_setting != nullptr) {
            battery_config.capacity_mah = sensor_setting->settings.getFloat("Capacity", 5000.0f);
            battery_config.nominal_voltage = sensor_setting->settings.getFloat("Voltage", 14.8f);
            battery_config.max_voltage = sensor_setting->settings.getFloat("MaxVoltage", 16.8f);
            battery_config.min_voltage = sensor_setting->settings.getFloat("MinVoltage", 12.0f);
            battery_config.internal_resistance = sensor_setting->settings.getFloat("InternalResistance", 0.05f);
            battery_config.critical_percentage = sensor_setting->settings.getFloat("CriticalPercentage", 20.0f);
            
            std::string mode_str = sensor_setting->settings.getString("SimulationMode", "Linear");
            if (mode_str == "Linear") {
                battery_config.simulation_mode = BatterySensorBase::SimulationMode::Linear;
            } else if (mode_str == "VoltageBased") {
                battery_config.simulation_mode = BatterySensorBase::SimulationMode::VoltageBased;
            } else if (mode_str == "PhysicsBased") {
                battery_config.simulation_mode = BatterySensorBase::SimulationMode::PhysicsBased;
            }
        }
    }

    virtual std::unique_ptr<SensorBase> createSensor(const std::string& sensor_name) override {
        auto sensor = std::make_unique<BatterySensorSimple>(sensor_name);
        sensor->initialize(battery_config);
        return sensor;
    }

    static std::unique_ptr<SensorParams> createFromSettings(const AirSimSettings::SensorSetting* sensor_setting) {
        auto params = std::make_unique<BatterySensorParams>();
        params->initializeFromSettings(sensor_setting);
        return params;
    }
};

}} // namespace
#endif
