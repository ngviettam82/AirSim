# C++ Implementation Guide - ProjectAirSim Integration

**Version:** 1.0  
**Date:** January 2025  
**Target:** Cosys-AirSim Codebase

---

## Table of Contents

1. [Coding Standards](#coding-standards)
2. [Project Structure](#project-structure)
3. [Class Design Patterns](#class-design-patterns)
4. [Sensor Implementation](#sensor-implementation)
5. [RPC Integration](#rpc-integration)
6. [Configuration Management](#configuration-management)
7. [Error Handling](#error-handling)
8. [Build System](#build-system)
9. [Debugging Strategies](#debugging-strategies)
10. [Best Practices](#best-practices)

---

## Coding Standards

### Naming Conventions

```cpp
// Classes: PascalCase
class BatterySensor { };
class BatterySettings { };

// Member variables: snake_case with trailing underscore
class Example {
private:
    float battery_percentage_;
    uint64_t timestamp_nanos_;
    std::string status_string_;
};

// Methods: camelCase
void updateBattery(float delta_time);
BatteryData getBatteryData() const;

// Constants: UPPER_SNAKE_CASE
constexpr float DEFAULT_DRAIN_RATE = 1.0f;
constexpr uint64_t NANOS_PER_SECOND = 1000000000ULL;

// Enums: PascalCase, values UPPER_SNAKE_CASE
enum class BatteryMode {
    SIMPLE_DISCHARGE,
    ENERGY_CONSUMPTION
};

// Files: PascalCase matching primary class
// BatterySensor.hpp
// BatterySensor.cpp
```

---

### Header File Structure

```cpp
// BatterySensor.hpp
#ifndef AIRSIM_SENSORS_BATTERYSENSOR_HPP
#define AIRSIM_SENSORS_BATTERYSENSOR_HPP

// 1. System includes (alphabetical)
#include <cstdint>
#include <memory>
#include <string>

// 2. External library includes (alphabetical)
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"

// 3. Project includes (alphabetical)
#include "sensors/SensorBase.hpp"
#include "sensors/SensorFactory.hpp"

// 4. Namespace
namespace msr { namespace airlib {

// 5. Forward declarations (if needed)
class VehicleParams;

// 6. Type definitions
struct BatteryData {
    // Member declarations
};

// 7. Main class declaration
class BatterySensor : public SensorBase {
public:
    // Public types
    using Settings = BatterySettings;
    
    // Constructor/Destructor
    BatterySensor(const std::string& sensor_name = "");
    virtual ~BatterySensor() = default;
    
    // Public interface (grouped logically)
    // - Configuration
    void initialize(const Settings& settings);
    const Settings& getSettings() const { return settings_; }
    
    // - Data access
    const BatteryData& getBatteryData() const;
    
    // - Simulation control
    void update(float delta_time) override;
    void reset() override;
    
protected:
    // Protected methods (if any)
    
private:
    // Private methods
    void updateSimpleDischarge(float delta_time);
    void updateEnergyConsumption(float delta_time);
    void updateHealthStatus();
    
    // Private data members (grouped logically)
    Settings settings_;
    BatteryData battery_data_;
    
    // Simulation state
    float current_percentage_;
    uint64_t last_update_time_;
};

}} // namespace msr::airlib

#endif // AIRSIM_SENSORS_BATTERYSENSOR_HPP
```

---

### Source File Structure

```cpp
// BatterySensor.cpp
#include "sensors/BatterySensor.hpp"

// System includes
#include <algorithm>
#include <cmath>

// Project includes
#include "common/ClockBase.hpp"
#include "common/VectorMath.hpp"

namespace msr { namespace airlib {

// Anonymous namespace for file-local helpers
namespace {
    constexpr float LOW_BATTERY_THRESHOLD = 30.0f;
    constexpr float CRITICAL_BATTERY_THRESHOLD = 15.0f;
    
    std::string calculateHealthStatus(float percentage, bool is_healthy) {
        if (!is_healthy)
            return "UNHEALTHY";
        if (percentage <= CRITICAL_BATTERY_THRESHOLD)
            return "CRITICAL";
        if (percentage <= LOW_BATTERY_THRESHOLD)
            return "LOW";
        return "OK";
    }
}

// Constructor implementation
BatterySensor::BatterySensor(const std::string& sensor_name)
    : SensorBase(sensor_name)
    , current_percentage_(100.0f)
    , last_update_time_(0)
{
    // Initialize battery data
    battery_data_.battery_pct_remaining = 100.0f;
    battery_data_.battery_charge_state = "OK";
}

// Public method implementations
void BatterySensor::initialize(const Settings& settings) {
    settings_ = settings;
    reset();
}

void BatterySensor::update(float delta_time) {
    if (settings_.mode == BatteryMode::SIMPLE_DISCHARGE) {
        updateSimpleDischarge(delta_time);
    } else {
        updateEnergyConsumption(delta_time);
    }
    
    updateHealthStatus();
    
    last_update_time_ = ClockBase::getSimTime();
}

void BatterySensor::reset() {
    current_percentage_ = 100.0f;
    battery_data_.battery_pct_remaining = 100.0f;
    battery_data_.battery_charge_state = "OK";
    last_update_time_ = ClockBase::getSimTime();
}

// Private method implementations
void BatterySensor::updateSimpleDischarge(float delta_time) {
    float discharge_pct = settings_.battery_drain_rate * delta_time;
    current_percentage_ = std::max(0.0f, current_percentage_ - discharge_pct);
    battery_data_.battery_pct_remaining = current_percentage_;
}

void BatterySensor::updateEnergyConsumption(float delta_time) {
    // Energy consumed = Power * Time * Coefficient
    float energy_joules = settings_.power_consumption_watts 
                        * delta_time 
                        * settings_.power_coefficient;
    
    // Convert to percentage
    float pct_consumed = (energy_joules / settings_.total_capacity_joules) * 100.0f;
    
    current_percentage_ = std::max(0.0f, current_percentage_ - pct_consumed);
    battery_data_.battery_pct_remaining = current_percentage_;
}

void BatterySensor::updateHealthStatus() {
    battery_data_.battery_charge_state = calculateHealthStatus(
        current_percentage_, 
        settings_.is_healthy
    );
}

}} // namespace msr::airlib
```

---

## Project Structure

### Directory Layout for Battery Sensor

```
AirLib/
├── include/
│   ├── sensors/
│   │   ├── SensorBase.hpp          [Existing]
│   │   ├── SensorFactory.hpp        [Existing]
│   │   ├── battery/                 [NEW]
│   │   │   ├── BatterySensor.hpp    [NEW]
│   │   │   ├── BatteryData.hpp      [NEW]
│   │   │   └── BatterySettings.hpp  [NEW]
│   │   └── ...
│   └── common/
│       ├── ClockBase.hpp            [MODIFY - Clock control]
│       └── CommonStructs.hpp        [MODIFY - Add BatteryData]
│
├── src/
│   ├── sensors/
│   │   ├── battery/                 [NEW]
│   │   │   └── BatterySensor.cpp    [NEW]
│   │   └── ...
│   ├── api/
│   │   └── RpcLibServerBase.cpp     [MODIFY - Add RPC methods]
│   └── common/
│       └── ClockBase.cpp            [MODIFY - Clock control]
│
└── ...
```

---

## Class Design Patterns

### 1. Sensor Base Class Pattern

All sensors inherit from `SensorBase`:

```cpp
// SensorBase.hpp (existing)
class SensorBase {
public:
    SensorBase(const std::string& sensor_name);
    virtual ~SensorBase() = default;
    
    // Core interface (pure virtual)
    virtual void update(float delta_time) = 0;
    virtual void reset() = 0;
    
    // Common functionality
    const std::string& getSensorName() const { return sensor_name_; }
    void setSensorName(const std::string& name) { sensor_name_ = name; }
    
protected:
    std::string sensor_name_;
};

// BatterySensor.hpp (new)
class BatterySensor : public SensorBase {
public:
    // Implement required interface
    void update(float delta_time) override;
    void reset() override;
    
    // Battery-specific interface
    const BatteryData& getBatteryData() const;
    void setBatteryRemaining(float percentage);
};
```

---

### 2. Settings Pattern

Use composition for sensor configuration:

```cpp
// BatterySettings.hpp
struct BatterySettings {
    // Mode selection
    BatteryMode mode = BatteryMode::SIMPLE_DISCHARGE;
    
    // Simple discharge parameters
    float battery_drain_rate = 1.0f;  // % per second
    
    // Energy consumption parameters
    float total_capacity_joules = 36000.0f;  // 10 Wh default
    float power_consumption_watts = 0.0f;
    float power_coefficient = 1.0f;
    
    // Health parameters
    bool is_healthy = true;
    
    // Constructor with defaults
    BatterySettings() = default;
    
    // Factory methods for common configs
    static BatterySettings createSimpleDischarge(float drain_rate);
    static BatterySettings createEnergyBased(float capacity_wh);
};

// Usage
BatterySensor sensor;
auto settings = BatterySettings::createSimpleDischarge(2.0f);
sensor.initialize(settings);
```

---

### 3. Data Structure Pattern

Use plain structs for data transfer:

```cpp
// BatteryData.hpp
struct BatteryData {
    // Data fields (public)
    float battery_pct_remaining = 100.0f;
    std::string battery_charge_state = "OK";
    uint64_t timestamp_nanos = 0;
    uint32_t time_remaining_secs = UINT32_MAX;
    
    // Msgpack serialization support
    MSGPACK_DEFINE_MAP(
        battery_pct_remaining,
        battery_charge_state,
        timestamp_nanos,
        time_remaining_secs
    );
    
    // Optional: Convenience methods
    bool isLow() const { return battery_charge_state == "LOW"; }
    bool isCritical() const { return battery_charge_state == "CRITICAL"; }
};
```

---

### 4. Factory Pattern for Sensor Creation

```cpp
// SensorFactory.hpp (modify existing)
class SensorFactory {
public:
    static std::unique_ptr<SensorBase> createSensor(
        SensorType type,
        const std::string& sensor_name,
        const rapidjson::Value& settings_json
    );
    
private:
    // Factory methods
    static std::unique_ptr<BatterySensor> createBatterySensor(
        const std::string& sensor_name,
        const rapidjson::Value& settings_json
    );
};

// SensorFactory.cpp
std::unique_ptr<SensorBase> SensorFactory::createSensor(
    SensorType type,
    const std::string& sensor_name,
    const rapidjson::Value& settings_json
) {
    switch (type) {
        case SensorType::Battery:
            return createBatterySensor(sensor_name, settings_json);
        // ... other sensor types
    }
}

std::unique_ptr<BatterySensor> SensorFactory::createBatterySensor(
    const std::string& sensor_name,
    const rapidjson::Value& settings_json
) {
    auto sensor = std::make_unique<BatterySensor>(sensor_name);
    
    BatterySettings settings;
    // Parse settings from JSON
    if (settings_json.HasMember("Mode")) {
        std::string mode_str = settings_json["Mode"].GetString();
        settings.mode = parseBatteryMode(mode_str);
    }
    // ... parse other fields
    
    sensor->initialize(settings);
    return sensor;
}
```

---

## Sensor Implementation

### Complete Battery Sensor Implementation

#### BatterySensor.hpp (Full)

```cpp
#ifndef AIRSIM_SENSORS_BATTERYSENSOR_HPP
#define AIRSIM_SENSORS_BATTERYSENSOR_HPP

#include <cstdint>
#include <string>
#include "sensors/SensorBase.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {

// Forward declarations
struct BatteryData;
struct BatterySettings;

// Enums
enum class BatteryMode : uint8_t {
    SIMPLE_DISCHARGE = 0,
    ENERGY_CONSUMPTION = 1
};

// Data structures
struct BatteryData {
    float battery_pct_remaining = 100.0f;
    std::string battery_charge_state = "OK";
    uint64_t timestamp_nanos = 0;
    uint32_t time_remaining_secs = UINT32_MAX;
    
    MSGPACK_DEFINE_MAP(
        battery_pct_remaining,
        battery_charge_state,
        timestamp_nanos,
        time_remaining_secs
    );
};

struct BatterySettings {
    BatteryMode mode = BatteryMode::SIMPLE_DISCHARGE;
    
    // Simple discharge
    float battery_drain_rate = 1.0f;
    
    // Energy consumption
    float total_capacity_joules = 36000.0f;
    float power_consumption_watts = 0.0f;
    float power_coefficient = 1.0f;
    
    // Health
    bool is_healthy = true;
    
    // Thresholds
    float low_battery_threshold = 30.0f;
    float critical_battery_threshold = 15.0f;
};

// Main class
class BatterySensor : public SensorBase {
public:
    BatterySensor(const std::string& sensor_name = "");
    virtual ~BatterySensor() = default;
    
    // SensorBase interface
    void update(float delta_time) override;
    void reset() override;
    
    // Configuration
    void initialize(const BatterySettings& settings);
    const BatterySettings& getSettings() const { return settings_; }
    
    // Data access
    const BatteryData& getBatteryData() const { return battery_data_; }
    
    // Control methods
    void setBatteryRemaining(float percentage);
    void setBatteryDrainRate(float rate);
    float getBatteryDrainRate() const { return settings_.battery_drain_rate; }
    void setPowerConsumption(float watts);
    void setBatteryHealthStatus(bool is_healthy);
    
private:
    // Update logic
    void updateSimpleDischarge(float delta_time);
    void updateEnergyConsumption(float delta_time);
    void updateHealthStatus();
    void updateTimeRemaining();
    
    // Validation
    void clampPercentage();
    
    // Data members
    BatterySettings settings_;
    BatteryData battery_data_;
    float current_percentage_;
    uint64_t last_update_time_;
};

}} // namespace msr::airlib

#endif // AIRSIM_SENSORS_BATTERYSENSOR_HPP
```

---

#### BatterySensor.cpp (Full)

```cpp
#include "sensors/BatterySensor.hpp"
#include "common/ClockBase.hpp"
#include <algorithm>
#include <cmath>

namespace msr { namespace airlib {

namespace {
    constexpr float MIN_PERCENTAGE = 0.0f;
    constexpr float MAX_PERCENTAGE = 100.0f;
    constexpr float EPSILON = 0.001f;
}

// Constructor
BatterySensor::BatterySensor(const std::string& sensor_name)
    : SensorBase(sensor_name)
    , current_percentage_(100.0f)
    , last_update_time_(0)
{
    reset();
}

// Public methods
void BatterySensor::initialize(const BatterySettings& settings) {
    settings_ = settings;
    reset();
}

void BatterySensor::reset() {
    current_percentage_ = 100.0f;
    battery_data_.battery_pct_remaining = 100.0f;
    battery_data_.battery_charge_state = "OK";
    battery_data_.timestamp_nanos = ClockBase::getSimTime();
    battery_data_.time_remaining_secs = UINT32_MAX;
    last_update_time_ = battery_data_.timestamp_nanos;
}

void BatterySensor::update(float delta_time) {
    // Skip if time delta is zero or negative
    if (delta_time <= 0.0f)
        return;
    
    // Update based on mode
    if (settings_.mode == BatteryMode::SIMPLE_DISCHARGE) {
        updateSimpleDischarge(delta_time);
    } else {
        updateEnergyConsumption(delta_time);
    }
    
    // Update derived fields
    updateHealthStatus();
    updateTimeRemaining();
    
    // Update timestamp
    battery_data_.timestamp_nanos = ClockBase::getSimTime();
    last_update_time_ = battery_data_.timestamp_nanos;
}

void BatterySensor::setBatteryRemaining(float percentage) {
    current_percentage_ = std::clamp(percentage, MIN_PERCENTAGE, MAX_PERCENTAGE);
    battery_data_.battery_pct_remaining = current_percentage_;
    updateHealthStatus();
    updateTimeRemaining();
}

void BatterySensor::setBatteryDrainRate(float rate) {
    if (rate < 0.0f) {
        throw std::invalid_argument("Drain rate cannot be negative");
    }
    settings_.battery_drain_rate = rate;
    updateTimeRemaining();
}

void BatterySensor::setPowerConsumption(float watts) {
    if (watts < 0.0f) {
        throw std::invalid_argument("Power consumption cannot be negative");
    }
    settings_.power_consumption_watts = watts;
    updateTimeRemaining();
}

void BatterySensor::setBatteryHealthStatus(bool is_healthy) {
    settings_.is_healthy = is_healthy;
    updateHealthStatus();
}

// Private methods
void BatterySensor::updateSimpleDischarge(float delta_time) {
    float discharge_pct = settings_.battery_drain_rate * delta_time;
    current_percentage_ -= discharge_pct;
    clampPercentage();
    battery_data_.battery_pct_remaining = current_percentage_;
}

void BatterySensor::updateEnergyConsumption(float delta_time) {
    // Calculate energy consumed in Joules
    float energy_joules = settings_.power_consumption_watts 
                        * delta_time 
                        * settings_.power_coefficient;
    
    // Convert to percentage
    float pct_consumed = (energy_joules / settings_.total_capacity_joules) * 100.0f;
    
    current_percentage_ -= pct_consumed;
    clampPercentage();
    battery_data_.battery_pct_remaining = current_percentage_;
}

void BatterySensor::updateHealthStatus() {
    if (!settings_.is_healthy) {
        battery_data_.battery_charge_state = "UNHEALTHY";
        return;
    }
    
    if (current_percentage_ <= settings_.critical_battery_threshold) {
        battery_data_.battery_charge_state = "CRITICAL";
    } else if (current_percentage_ <= settings_.low_battery_threshold) {
        battery_data_.battery_charge_state = "LOW";
    } else {
        battery_data_.battery_charge_state = "OK";
    }
}

void BatterySensor::updateTimeRemaining() {
    float rate = 0.0f;
    
    if (settings_.mode == BatteryMode::SIMPLE_DISCHARGE) {
        rate = settings_.battery_drain_rate;
        if (rate < EPSILON) {
            battery_data_.time_remaining_secs = UINT32_MAX;
            return;
        }
        
        float time_remaining = current_percentage_ / rate;
        battery_data_.time_remaining_secs = static_cast<uint32_t>(std::round(time_remaining));
        
    } else { // ENERGY_CONSUMPTION
        float power = settings_.power_consumption_watts * settings_.power_coefficient;
        if (power < EPSILON) {
            battery_data_.time_remaining_secs = UINT32_MAX;
            return;
        }
        
        float energy_remaining = (current_percentage_ / 100.0f) * settings_.total_capacity_joules;
        float time_remaining = energy_remaining / power;
        
        battery_data_.time_remaining_secs = static_cast<uint32_t>(std::round(time_remaining));
    }
}

void BatterySensor::clampPercentage() {
    current_percentage_ = std::clamp(current_percentage_, MIN_PERCENTAGE, MAX_PERCENTAGE);
}

}} // namespace msr::airlib
```

---

## RPC Integration

### Adding RPC Methods

#### Step 1: Define RPC API in RpcLibServerBase.hpp

```cpp
// RpcLibServerBase.hpp (modify existing)
class RpcLibServerBase {
public:
    // ... existing methods
    
    // Battery methods (add)
    msr::airlib::BatteryData getBatteryData(
        const std::string& sensor_name,
        const std::string& vehicle_name
    );
    
    void setBatteryRemaining(
        float percentage,
        const std::string& vehicle_name
    );
    
    void setBatteryDrainRate(
        float rate,
        const std::string& vehicle_name
    );
    
    float getBatteryDrainRate(
        const std::string& vehicle_name
    );
    
    void setBatteryHealthStatus(
        bool is_healthy,
        const std::string& vehicle_name
    );
    
    // Clock methods (add)
    std::string getSimClockType();
    uint64_t getSimTime();
    void setClockSpeed(float speed);
    float getClockSpeed();
    void pause();
    std::string resume();
    uint64_t continueForTime(uint64_t time_nanos, bool wait_until_complete);
    uint64_t continueForSteps(uint32_t num_steps, bool wait_until_complete);
    uint64_t continueForSingleStep(bool wait_until_complete);
    bool isPaused();
    
private:
    // Helper methods
    BatterySensor* getBatterySensor(const std::string& vehicle_name);
};
```

---

#### Step 2: Implement RPC Methods in RpcLibServerBase.cpp

```cpp
// RpcLibServerBase.cpp (modify existing)

// In constructor, register RPC methods
RpcLibServerBase::RpcLibServerBase(/* ... */) {
    // ... existing registrations
    
    // Register battery methods
    pimpl_->server.bind("getBatteryData", [this](
        const std::string& sensor_name,
        const std::string& vehicle_name
    ) {
        return getBatteryData(sensor_name, vehicle_name);
    });
    
    pimpl_->server.bind("setBatteryRemaining", [this](
        float percentage,
        const std::string& vehicle_name
    ) {
        setBatteryRemaining(percentage, vehicle_name);
    });
    
    pimpl_->server.bind("setBatteryDrainRate", [this](
        float rate,
        const std::string& vehicle_name
    ) {
        setBatteryDrainRate(rate, vehicle_name);
    });
    
    pimpl_->server.bind("getBatteryDrainRate", [this](
        const std::string& vehicle_name
    ) {
        return getBatteryDrainRate(vehicle_name);
    });
    
    pimpl_->server.bind("setBatteryHealthStatus", [this](
        bool is_healthy,
        const std::string& vehicle_name
    ) {
        setBatteryHealthStatus(is_healthy, vehicle_name);
    });
    
    // Register clock methods
    pimpl_->server.bind("getSimClockType", [this]() {
        return getSimClockType();
    });
    
    pimpl_->server.bind("getSimTime", [this]() {
        return getSimTime();
    });
    
    pimpl_->server.bind("setClockSpeed", [this](float speed) {
        setClockSpeed(speed);
    });
    
    // ... register all 9 clock methods
}

// Implement method bodies
BatteryData RpcLibServerBase::getBatteryData(
    const std::string& sensor_name,
    const std::string& vehicle_name
) {
    auto* sensor = getBatterySensor(vehicle_name);
    if (!sensor) {
        throw std::runtime_error("Battery sensor not found for vehicle: " + vehicle_name);
    }
    return sensor->getBatteryData();
}

void RpcLibServerBase::setBatteryRemaining(
    float percentage,
    const std::string& vehicle_name
) {
    auto* sensor = getBatterySensor(vehicle_name);
    if (!sensor) {
        throw std::runtime_error("Battery sensor not found for vehicle: " + vehicle_name);
    }
    sensor->setBatteryRemaining(percentage);
}

// ... implement other battery methods

// Clock methods
std::string RpcLibServerBase::getSimClockType() {
    return ClockBase::getSimClockType();
}

uint64_t RpcLibServerBase::getSimTime() {
    return ClockBase::getSimTime();
}

void RpcLibServerBase::setClockSpeed(float speed) {
    if (speed <= 0.0f) {
        throw std::invalid_argument("Clock speed must be positive");
    }
    ClockBase::setClockSpeed(speed);
}

// ... implement other clock methods

// Helper method
BatterySensor* RpcLibServerBase::getBatterySensor(const std::string& vehicle_name) {
    // Get vehicle API
    auto* vehicle_api = getVehicleApi(vehicle_name);
    if (!vehicle_api) {
        return nullptr;
    }
    
    // Get sensor collection
    auto* sensor_collection = vehicle_api->getSensors();
    if (!sensor_collection) {
        return nullptr;
    }
    
    // Find battery sensor
    auto* sensor = sensor_collection->getSensor(SensorType::Battery, "");
    return dynamic_cast<BatterySensor*>(sensor);
}
```

---

## Configuration Management

### Parsing settings.json

```cpp
// Example: Parsing battery configuration from settings.json

void parseVehicleSettings(const rapidjson::Value& vehicle_json) {
    // ... existing vehicle parsing
    
    // Parse sensors
    if (vehicle_json.HasMember("Sensors")) {
        const auto& sensors_json = vehicle_json["Sensors"];
        
        if (sensors_json.HasMember("Battery")) {
            const auto& battery_json = sensors_json["Battery"];
            
            BatterySettings settings;
            
            // Parse mode
            if (battery_json.HasMember("Mode")) {
                std::string mode_str = battery_json["Mode"].GetString();
                if (mode_str == "SimpleDischarge") {
                    settings.mode = BatteryMode::SIMPLE_DISCHARGE;
                } else if (mode_str == "EnergyConsumption") {
                    settings.mode = BatteryMode::ENERGY_CONSUMPTION;
                }
            }
            
            // Parse drain rate
            if (battery_json.HasMember("BatteryDrainRate")) {
                settings.battery_drain_rate = battery_json["BatteryDrainRate"].GetFloat();
            }
            
            // Parse capacity
            if (battery_json.HasMember("TotalCapacityJoules")) {
                settings.total_capacity_joules = battery_json["TotalCapacityJoules"].GetFloat();
            }
            
            // ... parse other fields
            
            // Create and register sensor
            auto battery_sensor = std::make_unique<BatterySensor>();
            battery_sensor->initialize(settings);
            vehicle_sensors->addSensor(SensorType::Battery, std::move(battery_sensor));
        }
    }
}
```

---

### Configuration Validation

```cpp
// Validate battery settings
void validateBatterySettings(const BatterySettings& settings) {
    if (settings.battery_drain_rate < 0.0f) {
        throw std::invalid_argument("Battery drain rate cannot be negative");
    }
    
    if (settings.total_capacity_joules <= 0.0f) {
        throw std::invalid_argument("Total capacity must be positive");
    }
    
    if (settings.power_consumption_watts < 0.0f) {
        throw std::invalid_argument("Power consumption cannot be negative");
    }
    
    if (settings.power_coefficient <= 0.0f) {
        throw std::invalid_argument("Power coefficient must be positive");
    }
    
    if (settings.low_battery_threshold <= settings.critical_battery_threshold) {
        throw std::invalid_argument("Low battery threshold must be > critical threshold");
    }
}
```

---

## Error Handling

### Exception Hierarchy

```cpp
// Custom exceptions for ProjectAirSim integration

namespace msr { namespace airlib {

// Base exception
class ProjectAirSimException : public std::runtime_error {
public:
    explicit ProjectAirSimException(const std::string& message)
        : std::runtime_error(message) {}
};

// Configuration errors
class ConfigurationException : public ProjectAirSimException {
public:
    explicit ConfigurationException(const std::string& message)
        : ProjectAirSimException("Configuration error: " + message) {}
};

// Sensor errors
class SensorException : public ProjectAirSimException {
public:
    explicit SensorException(const std::string& message)
        : ProjectAirSimException("Sensor error: " + message) {}
};

// RPC errors
class RpcException : public ProjectAirSimException {
public:
    explicit RpcException(const std::string& message)
        : ProjectAirSimException("RPC error: " + message) {}
};

}} // namespace

// Usage example
void BatterySensor::setBatteryDrainRate(float rate) {
    if (rate < 0.0f) {
        throw SensorException("Drain rate cannot be negative: " + std::to_string(rate));
    }
    settings_.battery_drain_rate = rate;
}
```

---

### Error Handling Best Practices

```cpp
// 1. Validate inputs early
void RpcLibServerBase::setBatteryRemaining(float percentage, const std::string& vehicle_name) {
    // Validate percentage
    if (percentage < 0.0f || percentage > 100.0f) {
        throw std::invalid_argument(
            "Battery percentage must be between 0 and 100, got: " + std::to_string(percentage)
        );
    }
    
    // Get sensor (may throw)
    auto* sensor = getBatterySensor(vehicle_name);
    if (!sensor) {
        throw RpcException("Battery sensor not found for vehicle: " + vehicle_name);
    }
    
    // Set value (may throw)
    try {
        sensor->setBatteryRemaining(percentage);
    } catch (const std::exception& e) {
        throw RpcException("Failed to set battery remaining: " + std::string(e.what()));
    }
}

// 2. Use RAII for resource management
class ScopedPause {
public:
    ScopedPause() {
        was_paused_ = ClockBase::isPaused();
        if (!was_paused_) {
            ClockBase::pause();
        }
    }
    
    ~ScopedPause() {
        if (!was_paused_) {
            ClockBase::resume();
        }
    }
    
private:
    bool was_paused_;
};

// Usage
void performCriticalOperation() {
    ScopedPause pause;  // Automatically pause/resume
    // ... critical operation
}  // Auto-resume when leaving scope

// 3. Log errors appropriately
#include "common/common_utils/Utils.hpp"

void BatterySensor::update(float delta_time) {
    try {
        if (settings_.mode == BatteryMode::SIMPLE_DISCHARGE) {
            updateSimpleDischarge(delta_time);
        } else {
            updateEnergyConsumption(delta_time);
        }
    } catch (const std::exception& e) {
        Utils::log(Utils::stringf("BatterySensor update failed: %s", e.what()), Utils::kLogLevelError);
        // Don't rethrow - sensor should be resilient
    }
}
```

---

## Build System

### CMakeLists.txt Integration

```cmake
# AirLib/CMakeLists.txt (modify)

# Add battery sensor source files
set(BATTERY_SENSOR_SOURCES
    src/sensors/battery/BatterySensor.cpp
)

# Add to AirLib sources
set(AIRLIB_SOURCES
    ${AIRLIB_SOURCES}
    ${BATTERY_SENSOR_SOURCES}
)

# Add include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sensors/battery
)

# Add to library
add_library(AirLib STATIC ${AIRLIB_SOURCES})
```

---

### Visual Studio Project Files

For Windows builds using Visual Studio:

1. **Add files to AirLib.vcxproj:**

```xml
<ItemGroup>
  <ClInclude Include="include\sensors\battery\BatterySensor.hpp" />
  <ClInclude Include="include\sensors\battery\BatteryData.hpp" />
  <ClInclude Include="include\sensors\battery\BatterySettings.hpp" />
</ItemGroup>

<ItemGroup>
  <ClCompile Include="src\sensors\battery\BatterySensor.cpp" />
</ItemGroup>
```

2. **Add filters in AirLib.vcxproj.filters:**

```xml
<Filter Include="sensors\battery">
  <UniqueIdentifier>{...GUID...}</UniqueIdentifier>
</Filter>

<ClInclude Include="include\sensors\battery\BatterySensor.hpp">
  <Filter>sensors\battery</Filter>
</ClInclude>
```

---

## Debugging Strategies

### 1. Logging

```cpp
#include "common/common_utils/Utils.hpp"

void BatterySensor::update(float delta_time) {
    Utils::log(Utils::stringf(
        "BatterySensor::update - delta_time: %.3f, mode: %d, pct: %.2f%%",
        delta_time,
        static_cast<int>(settings_.mode),
        current_percentage_
    ), Utils::kLogLevelInfo);
    
    // ... update logic
}

// Enable detailed logging
Utils::setLogLevel(Utils::kLogLevelDebug);
```

---

### 2. Assertions

```cpp
#include <cassert>

void BatterySensor::updateSimpleDischarge(float delta_time) {
    assert(delta_time >= 0.0f && "Delta time must be non-negative");
    assert(settings_.battery_drain_rate >= 0.0f && "Drain rate must be non-negative");
    
    // ... update logic
    
    assert(current_percentage_ >= 0.0f && current_percentage_ <= 100.0f && 
           "Battery percentage out of range");
}
```

---

### 3. Debug Visualization

```cpp
// Add debug rendering for battery status
#include "common/common_utils/Utils.hpp"

void BatterySensor::renderDebugInfo() {
#ifdef _DEBUG
    std::string debug_text = Utils::stringf(
        "Battery: %.1f%% (%s)\n"
        "Mode: %s\n"
        "Drain Rate: %.2f%%/s\n"
        "Time Remaining: %u s",
        current_percentage_,
        battery_data_.battery_charge_state.c_str(),
        settings_.mode == BatteryMode::SIMPLE_DISCHARGE ? "Simple" : "Energy",
        settings_.battery_drain_rate,
        battery_data_.time_remaining_secs
    );
    
    // Render to screen (implementation-dependent)
    Utils::log(debug_text, Utils::kLogLevelDebug);
#endif
}
```

---

### 4. Unit Test Debugging

```cpp
// Use Google Test for debugging
TEST(BatterySimpleDischarge, DebugSpecificCase) {
    BatterySensor sensor;
    BatterySettings settings;
    settings.battery_drain_rate = 1.0f;
    sensor.initialize(settings);
    
    // Set breakpoint here
    sensor.update(10.0f);
    
    const auto& data = sensor.getBatteryData();
    
    // Inspect variables in debugger
    EXPECT_NEAR(data.battery_pct_remaining, 90.0f, 0.1f);
}
```

---

## Best Practices

### 1. Const Correctness

```cpp
// Good: Const methods for read-only access
const BatteryData& getBatteryData() const { return battery_data_; }
const BatterySettings& getSettings() const { return settings_; }

// Good: Const references for parameters
void initialize(const BatterySettings& settings);
void setConfig(const rapidjson::Value& config);

// Good: Const local variables
void update(float delta_time) {
    const float discharge_pct = settings_.battery_drain_rate * delta_time;
    // ...
}
```

---

### 2. Memory Management

```cpp
// Prefer smart pointers
std::unique_ptr<BatterySensor> sensor = std::make_unique<BatterySensor>();

// Use move semantics for ownership transfer
sensor_collection->addSensor(SensorType::Battery, std::move(sensor));

// Avoid raw pointers for ownership
// Bad:
// BatterySensor* sensor = new BatterySensor();  // Manual delete required

// Good for non-owning references:
BatterySensor* getSensorPtr() { return sensor_.get(); }
```

---

### 3. Performance Considerations

```cpp
// Cache frequently accessed values
void BatterySensor::update(float delta_time) {
    // Cache mode check (avoid repeated enum comparisons)
    const bool is_simple_discharge = (settings_.mode == BatteryMode::SIMPLE_DISCHARGE);
    
    if (is_simple_discharge) {
        updateSimpleDischarge(delta_time);
    } else {
        updateEnergyConsumption(delta_time);
    }
}

// Avoid unnecessary copies
const BatteryData& getBatteryData() const { return battery_data_; }  // Good: reference
// BatteryData getBatteryData() const { return battery_data_; }      // Bad: copy

// Use string_view for read-only string parameters (C++17)
#include <string_view>
void setSensorName(std::string_view name) { sensor_name_ = name; }
```

---

### 4. Thread Safety

```cpp
#include <mutex>

class BatterySensor : public SensorBase {
public:
    const BatteryData& getBatteryData() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return battery_data_;
    }
    
    void update(float delta_time) override {
        std::lock_guard<std::mutex> lock(mutex_);
        // ... update logic
    }
    
private:
    mutable std::mutex mutex_;
    BatteryData battery_data_;
};
```

---

### 5. Documentation

```cpp
/**
 * @brief Battery sensor for simulated vehicles.
 * 
 * Supports two discharge modes:
 * - Simple Discharge: Constant percentage drain per second
 * - Energy Consumption: Physics-based power consumption in Watts
 * 
 * @example
 * @code
 * BatterySensor sensor;
 * BatterySettings settings = BatterySettings::createSimpleDischarge(1.0f);
 * sensor.initialize(settings);
 * 
 * sensor.update(1.0f);  // Simulate 1 second
 * const auto& data = sensor.getBatteryData();
 * std::cout << "Battery: " << data.battery_pct_remaining << "%" << std::endl;
 * @endcode
 */
class BatterySensor : public SensorBase {
public:
    /**
     * @brief Construct a new Battery Sensor object.
     * @param sensor_name Unique identifier for this sensor.
     */
    BatterySensor(const std::string& sensor_name = "");
    
    /**
     * @brief Update battery state based on time delta.
     * @param delta_time Time elapsed since last update (seconds).
     * @note Thread-safe if compiled with THREAD_SAFETY enabled.
     */
    void update(float delta_time) override;
    
    // ... more methods
};
```

---

## Summary Checklist

When implementing a new sensor or feature:

- [ ] Follow naming conventions (PascalCase classes, snake_case members, camelCase methods)
- [ ] Create proper header guards (#ifndef/#define/#endif)
- [ ] Use namespaces (msr::airlib)
- [ ] Inherit from SensorBase or appropriate base class
- [ ] Implement required virtual methods (update, reset)
- [ ] Create Settings struct for configuration
- [ ] Create Data struct for output with MSGPACK_DEFINE_MAP
- [ ] Add RPC methods to RpcLibServerBase.hpp/.cpp
- [ ] Register RPC methods in constructor
- [ ] Add configuration parsing in settings loader
- [ ] Validate configuration inputs
- [ ] Add unit tests (AirLibUnitTests/)
- [ ] Add integration tests (PythonClient/tests/)
- [ ] Update CMakeLists.txt
- [ ] Update Visual Studio project files (.vcxproj, .filters)
- [ ] Add logging for debugging
- [ ] Use const correctness
- [ ] Prefer smart pointers
- [ ] Add Doxygen documentation
- [ ] Test on Windows and Linux
- [ ] Verify msgpack serialization
- [ ] Check thread safety if needed
- [ ] Profile performance if critical path
- [ ] Update user documentation

---

**Implementation Guide Version:** 1.0  
**Last Updated:** January 2025  
**Maintainer:** ProjectAirSim Integration Team

---

## Appendix: Common Patterns

### Pattern: Sensor with Multiple Data Outputs

```cpp
class ComplexSensor : public SensorBase {
public:
    struct DataOutput1 { /* ... */ MSGPACK_DEFINE_MAP(...); };
    struct DataOutput2 { /* ... */ MSGPACK_DEFINE_MAP(...); };
    
    const DataOutput1& getData1() const { return data1_; }
    const DataOutput2& getData2() const { return data2_; }
    
private:
    DataOutput1 data1_;
    DataOutput2 data2_;
};
```

### Pattern: Sensor with Callbacks

```cpp
class ObservableSensor : public SensorBase {
public:
    using Callback = std::function<void(const SensorData&)>;
    
    void addCallback(Callback callback) {
        callbacks_.push_back(callback);
    }
    
    void update(float delta_time) override {
        // ... update logic
        
        // Notify callbacks
        for (auto& callback : callbacks_) {
            callback(sensor_data_);
        }
    }
    
private:
    std::vector<Callback> callbacks_;
    SensorData sensor_data_;
};
```

### Pattern: Lazy Initialization

```cpp
class LazySensor : public SensorBase {
public:
    const SensorData& getData() {
        if (!initialized_) {
            initialize();
            initialized_ = true;
        }
        return data_;
    }
    
private:
    void initialize() {
        // Heavy initialization
    }
    
    bool initialized_ = false;
    SensorData data_;
};
```

---

**End of C++ Implementation Guide**
