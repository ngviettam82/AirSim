# Technical Specifications - Battery Sensor

**Feature:** Battery Sensor Integration  
**Priority:** 0 (Highest)  
**Status:** Ready for Implementation  
**Source Verification:** 100% (Source code examined)

---

## Overview

Integrate ProjectAirSim's battery sensor into Cosys-AirSim, providing state-of-charge tracking, discharge simulation, and health status monitoring for vehicles.

---

## Data Structures

### 1. BatteryData (C++)

**Location:** `AirLib/include/sensors/battery/BatteryData.hpp`

```cpp
namespace msr { namespace airlib {

struct BatteryData {
    // === Core ProjectAirSim Fields ===
    float battery_pct_remaining = 100.0f;     // State of charge (0.0 - 100.0)
    uint32_t estimated_time_remaining = 0;    // Estimated time to empty (seconds)
    std::string battery_charge_state = "OK";  // "OK", "LOW", "CRITICAL", "UNHEALTHY"
    TTimePoint time_stamp = 0;                // Timestamp in nanoseconds
    
    // === Enhanced Cosys-AirSim Fields ===
    float voltage = 0.0f;                     // Current battery voltage (V)
    float current_draw = 0.0f;                // Current draw (A), positive = discharging
    float remaining_wh = 0.0f;                // Remaining energy (Wh)
    float temperature = 25.0f;                // Battery temperature (°C)
};

}}
```

**Field Descriptions:**

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| battery_pct_remaining | float | 0.0-100.0 | State of charge percentage |
| estimated_time_remaining | uint32_t | 0-∞ | Seconds until battery empty at current rate |
| battery_charge_state | string | enum | "OK", "LOW" (<30%), "CRITICAL" (<15%), "UNHEALTHY" |
| time_stamp | TTimePoint | 0-∞ | Simulation time when reading was taken (nanoseconds) |
| voltage | float | 0.0-∞ | Battery voltage (V) - optional enhanced field |
| current_draw | float | -∞-∞ | Current flow (A) - positive=discharge, negative=charge |
| remaining_wh | float | 0.0-∞ | Remaining energy in Watt-hours |
| temperature | float | -∞-∞ | Battery temperature in Celsius |

---

### 2. BatteryMode (C++ Enum)

**Location:** `AirLib/include/sensors/battery/BatterySensor.hpp`

```cpp
namespace msr { namespace airlib {

enum class BatteryMode : int {
    SimpleDischarge = 0,      // Constant drain rate mode
    EnergyConsumption = 1     // Power-based discharge mode
};

}}
```

**Mode Descriptions:**

**SimpleDischarge (Mode 0):**
- Fixed percentage drain per second
- Configurable `battery_drain_rate` parameter
- Independent of vehicle power consumption
- Best for: Testing, simplified scenarios

**EnergyConsumption (Mode 1):**
- Dynamic discharge based on actual power consumption
- Proportional to rotor power (torque × angular velocity)
- Configurable `power_coefficient` for calibration
- Best for: Realistic flight simulation

---

### 3. BatterySettings (C++ Configuration)

```cpp
struct BatterySettings {
    BatteryMode mode = BatteryMode::EnergyConsumption;
    float total_capacity_joules = 150000.0f;      // Total battery capacity (Joules)
    float current_capacity_joules = 150000.0f;    // Current remaining capacity (Joules)
    float battery_drain_rate = 0.0f;              // % per second (SimpleDischarge mode)
    float power_coefficient = 1.0f;               // Multiplier (EnergyConsumption mode)
    bool is_healthy = true;                       // Health status flag
    
    // Thresholds
    float low_threshold_pct = 30.0f;              // Below this = "LOW"
    float critical_threshold_pct = 15.0f;         // Below this = "CRITICAL"
};
```

**Conversion Formulas:**
```cpp
// Joules to Watt-hours
float joules_to_wh(float joules) {
    return joules / 3600.0f;
}

// Watt-hours to Joules
float wh_to_joules(float wh) {
    return wh * 3600.0f;
}

// Joules to Amp-hours (requires voltage)
float joules_to_ah(float joules, float voltage) {
    return joules / (voltage * 3600.0f);
}

// Milliamp-hours to Joules (requires voltage)
float mah_to_joules(float mah, float voltage) {
    return (mah / 1000.0f) * voltage * 3600.0f;
}
```

---

## BatterySensor Class

### Class Declaration

**Location:** `AirLib/include/sensors/battery/BatterySensor.hpp`

```cpp
namespace msr { namespace airlib {

class BatterySensor : public SensorBase {
public:
    BatterySensor(const std::string& sensor_name = "");
    virtual ~BatterySensor() = default;

    // SensorBase interface
    virtual void initialize() override;
    virtual void update(real_T dt) override;
    virtual void reportState(StateReporter& reporter) override;
    
    // Battery-specific methods
    const BatteryData& getBatteryData() const;
    void setBatteryRemaining(float percent);
    void setBatteryDrainRate(float rate_pct_per_sec);
    float getBatteryDrainRate() const;
    void setBatteryHealthStatus(bool is_healthy);
    void setPowerConsumption(float power_watts);
    
    // Configuration
    const BatterySettings& getSettings() const;
    void setSettings(const BatterySettings& settings);

private:
    void updateSimpleDischarge(real_T dt);
    void updateEnergyConsumption(real_T dt);
    void updateHealthStatus();
    void calculateEstimatedTimeRemaining();
    
    BatterySettings settings_;
    BatteryData data_;
    float current_power_consumption_ = 0.0f;
};

}}
```

---

### Update Logic

#### Simple Discharge Mode

```cpp
void BatterySensor::updateSimpleDischarge(real_T dt) {
    // Drain battery by fixed percentage per second
    float drain_amount = settings_.battery_drain_rate * dt;
    settings_.current_capacity_joules -= 
        (drain_amount / 100.0f) * settings_.total_capacity_joules;
    
    // Clamp to valid range
    settings_.current_capacity_joules = 
        std::max(0.0f, std::min(settings_.current_capacity_joules, 
                                settings_.total_capacity_joules));
    
    // Update percentage
    data_.battery_pct_remaining = 
        (settings_.current_capacity_joules / settings_.total_capacity_joules) * 100.0f;
}
```

#### Energy Consumption Mode

```cpp
void BatterySensor::updateEnergyConsumption(real_T dt) {
    // Drain based on actual power consumption
    // Power (W) × Time (s) = Energy (J)
    float energy_consumed = current_power_consumption_ * 
                           settings_.power_coefficient * dt;
    
    settings_.current_capacity_joules -= energy_consumed;
    
    // Clamp to valid range
    settings_.current_capacity_joules = 
        std::max(0.0f, std::min(settings_.current_capacity_joules, 
                                settings_.total_capacity_joules));
    
    // Update percentage
    data_.battery_pct_remaining = 
        (settings_.current_capacity_joules / settings_.total_capacity_joules) * 100.0f;
    
    // Update enhanced fields
    data_.current_draw = current_power_consumption_ / data_.voltage;
    data_.remaining_wh = settings_.current_capacity_joules / 3600.0f;
}
```

#### Health Status Update

```cpp
void BatterySensor::updateHealthStatus() {
    if (!settings_.is_healthy) {
        data_.battery_charge_state = "UNHEALTHY";
    } else if (data_.battery_pct_remaining < settings_.critical_threshold_pct) {
        data_.battery_charge_state = "CRITICAL";
    } else if (data_.battery_pct_remaining < settings_.low_threshold_pct) {
        data_.battery_charge_state = "LOW";
    } else {
        data_.battery_charge_state = "OK";
    }
}
```

#### Time Remaining Calculation

```cpp
void BatterySensor::calculateEstimatedTimeRemaining() {
    if (settings_.mode == BatteryMode::SimpleDischarge) {
        // Simple: remaining % / drain rate per second
        if (settings_.battery_drain_rate > 0.0f) {
            data_.estimated_time_remaining = static_cast<uint32_t>(
                data_.battery_pct_remaining / settings_.battery_drain_rate
            );
        } else {
            data_.estimated_time_remaining = UINT32_MAX;  // Infinite
        }
    } else {  // EnergyConsumption
        // remaining energy / current power consumption
        if (current_power_consumption_ > 0.0f) {
            data_.estimated_time_remaining = static_cast<uint32_t>(
                settings_.current_capacity_joules / current_power_consumption_
            );
        } else {
            data_.estimated_time_remaining = UINT32_MAX;  // Infinite
        }
    }
}
```

---

## Configuration Format

### settings.json Schema

```json
{
  "Vehicles": {
    "Drone1": {
      "Sensors": {
        "Battery": {
          "SensorType": 9,
          "Enabled": true,
          
          "BatteryMode": "EnergyConsumption",
          "TotalCapacityJoules": 36000,
          "InitialCapacityJoules": 30000,
          "PowerCoefficient": 1.0,
          
          "LowThreshold": 30.0,
          "CriticalThreshold": 15.0
        }
      }
    }
  }
}
```

**Configuration Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| SensorType | int | 9 | Sensor type ID (TBD - coordinate with existing sensors) |
| Enabled | bool | true | Enable/disable sensor |
| BatteryMode | string | "EnergyConsumption" | "SimpleDischarge" or "EnergyConsumption" |
| TotalCapacityJoules | float | 150000 | Total battery capacity in Joules (~41.67 Wh) |
| InitialCapacityJoules | float | 150000 | Starting capacity in Joules |
| BatteryDrainRate | float | 0.0 | % per second (SimpleDischarge mode only) |
| PowerCoefficient | float | 1.0 | Power multiplier (EnergyConsumption mode only) |
| LowThreshold | float | 30.0 | Percentage for "LOW" status |
| CriticalThreshold | float | 15.0 | Percentage for "CRITICAL" status |

**Configuration Parsing:**

```cpp
void BatterySensor::load(const Settings& settings) {
    settings_.mode = settings.getString("BatteryMode", "") == "SimpleDischarge" 
        ? BatteryMode::SimpleDischarge 
        : BatteryMode::EnergyConsumption;
    
    settings_.total_capacity_joules = 
        settings.getFloat("TotalCapacityJoules", 150000.0f);
    settings_.current_capacity_joules = 
        settings.getFloat("InitialCapacityJoules", settings_.total_capacity_joules);
    
    if (settings_.mode == BatteryMode::SimpleDischarge) {
        settings_.battery_drain_rate = 
            settings.getFloat("BatteryDrainRate", 0.02f);  // 0.02% per second default
    } else {
        settings_.power_coefficient = 
            settings.getFloat("PowerCoefficient", 1.0f);
    }
    
    settings_.low_threshold_pct = settings.getFloat("LowThreshold", 30.0f);
    settings_.critical_threshold_pct = settings.getFloat("CriticalThreshold", 15.0f);
}
```

---

## Python API

### BatteryData Class

**Location:** `PythonClient/cosysairsim/types.py`

```python
class BatteryData(MsgpackMixin):
    # Core ProjectAirSim fields
    battery_pct_remaining: float = 100.0
    estimated_time_remaining: int = 0
    battery_charge_state: str = "OK"
    time_stamp: int = 0
    
    # Enhanced Cosys-AirSim fields
    voltage: float = 0.0
    current_draw: float = 0.0
    remaining_wh: float = 0.0
    temperature: float = 25.0
```

### API Methods

**Location:** `PythonClient/cosysairsim/client.py`

```python
class MultirotorClient(VehicleClient):
    
    def getBatteryData(self, vehicle_name: str = '', sensor_name: str = '') -> BatteryData:
        """Get current battery sensor data.
        
        Args:
            vehicle_name: Name of vehicle. Empty string = default vehicle.
            sensor_name: Name of battery sensor. Empty string = first battery sensor.
            
        Returns:
            BatteryData object with current state.
        """
        return self.client.call('getBatteryData', vehicle_name, sensor_name)
    
    def setBatteryRemaining(self, percent: float, vehicle_name: str = '', 
                           sensor_name: str = '') -> None:
        """Manually set battery remaining percentage.
        
        Args:
            percent: New battery percentage (0.0 - 100.0).
            vehicle_name: Name of vehicle.
            sensor_name: Name of battery sensor.
        """
        self.client.call('setBatteryRemaining', percent, vehicle_name, sensor_name)
    
    def getBatteryDrainRate(self, vehicle_name: str = '', 
                           sensor_name: str = '') -> float:
        """Get current battery drain rate (SimpleDischarge mode only).
        
        Args:
            vehicle_name: Name of vehicle.
            sensor_name: Name of battery sensor.
            
        Returns:
            Drain rate in percent per second.
        """
        return self.client.call('getBatteryDrainRate', vehicle_name, sensor_name)
    
    def setBatteryDrainRate(self, rate: float, vehicle_name: str = '', 
                           sensor_name: str = '') -> None:
        """Set battery drain rate (SimpleDischarge mode only).
        
        Args:
            rate: Drain rate in percent per second.
            vehicle_name: Name of vehicle.
            sensor_name: Name of battery sensor.
        """
        self.client.call('setBatteryDrainRate', rate, vehicle_name, sensor_name)
    
    def setBatteryHealthStatus(self, is_healthy: bool, vehicle_name: str = '', 
                              sensor_name: str = '') -> None:
        """Set battery health status.
        
        Args:
            is_healthy: True for healthy, False for unhealthy.
            vehicle_name: Name of vehicle.
            sensor_name: Name of battery sensor.
        """
        self.client.call('setBatteryHealthStatus', is_healthy, vehicle_name, sensor_name)
```

---

## RPC Integration

### RPC Server Methods

**Location:** `AirLib/src/api/RpcLibServerBase.cpp`

```cpp
// Register RPC methods
server.bind("getBatteryData", [&](const std::string& vehicle_name, 
                                   const std::string& sensor_name) {
    return getVehicleApi(vehicle_name)->getBatteryData(sensor_name);
});

server.bind("setBatteryRemaining", [&](float percent, 
                                        const std::string& vehicle_name,
                                        const std::string& sensor_name) {
    getVehicleApi(vehicle_name)->setBatteryRemaining(percent, sensor_name);
});

server.bind("getBatteryDrainRate", [&](const std::string& vehicle_name,
                                        const std::string& sensor_name) {
    return getVehicleApi(vehicle_name)->getBatteryDrainRate(sensor_name);
});

server.bind("setBatteryDrainRate", [&](float rate,
                                        const std::string& vehicle_name,
                                        const std::string& sensor_name) {
    getVehicleApi(vehicle_name)->setBatteryDrainRate(rate, sensor_name);
});

server.bind("setBatteryHealthStatus", [&](bool is_healthy,
                                           const std::string& vehicle_name,
                                           const std::string& sensor_name) {
    getVehicleApi(vehicle_name)->setBatteryHealthStatus(is_healthy, sensor_name);
});
```

---

## Integration with Vehicle

### Power Consumption Hook

The battery sensor needs to receive power consumption data from the vehicle's rotors.

**Location:** `AirLib/include/vehicles/multirotor/MultiRotorParams.hpp`

```cpp
class MultirotorPhysicsBody {
public:
    void update(real_T dt) override {
        // ... existing physics update ...
        
        // Calculate total rotor power
        float total_power = 0.0f;
        for (const auto& rotor : rotors_) {
            float torque = rotor.getTorque();
            float angular_velocity = rotor.getAngularVelocity();
            total_power += std::abs(torque * angular_velocity);
        }
        
        // Update battery sensor
        if (battery_sensor_) {
            battery_sensor_->setPowerConsumption(total_power);
        }
        
        // ... rest of update ...
    }
    
private:
    BatterySensor* battery_sensor_ = nullptr;
};
```

---

## Testing Requirements

### Unit Tests

**Location:** `AirLibUnitTests/BatterySensorTests.hpp`

```cpp
class BatterySensorTests {
public:
    void testSimpleDischarge() {
        BatterySensor sensor;
        BatterySettings settings;
        settings.mode = BatteryMode::SimpleDischarge;
        settings.battery_drain_rate = 1.0f;  // 1% per second
        settings.total_capacity_joules = 36000.0f;
        settings.current_capacity_joules = 36000.0f;
        sensor.setSettings(settings);
        
        // Simulate 10 seconds
        for (int i = 0; i < 10; ++i) {
            sensor.update(1.0);
        }
        
        // Should be at 90%
        const auto& data = sensor.getBatteryData();
        ASSERT_NEAR(data.battery_pct_remaining, 90.0f, 0.1f);
    }
    
    void testEnergyConsumption() {
        BatterySensor sensor;
        BatterySettings settings;
        settings.mode = BatteryMode::EnergyConsumption;
        settings.total_capacity_joules = 36000.0f;
        settings.current_capacity_joules = 36000.0f;
        sensor.setSettings(settings);
        
        // Apply 100W power for 10 seconds = 1000J consumed
        sensor.setPowerConsumption(100.0f);
        sensor.update(10.0);
        
        // Should have consumed 1000J out of 36000J = ~2.78%
        const auto& data = sensor.getBatteryData();
        ASSERT_NEAR(data.battery_pct_remaining, 97.22f, 0.1f);
    }
    
    void testHealthStatus() {
        // Test OK → LOW → CRITICAL transitions
        // Test UNHEALTHY flag override
    }
};
```

---

## Performance Considerations

### Update Frequency
- Battery updates once per physics tick (~100Hz typical)
- Low computational cost (simple arithmetic)
- No blocking operations

### Memory Usage
- BatterySensor: ~200 bytes per instance
- BatteryData: ~60 bytes per reading
- Minimal heap allocations

### Thread Safety
- BatterySensor methods should be called from physics thread only
- RPC calls queue updates safely

---

## Acceptance Criteria

- [ ] BatterySensor class compiles without warnings
- [ ] Both discharge modes implemented and tested
- [ ] Configuration parsing works from settings.json
- [ ] Python API methods callable and return correct data
- [ ] RPC integration complete
- [ ] Health status updates correctly based on percentage
- [ ] Estimated time remaining calculated accurately
- [ ] Unit tests achieve >90% code coverage
- [ ] Integration with multirotor physics works
- [ ] No memory leaks detected
- [ ] Performance impact <1ms per update

---

**Specification Version:** 1.0  
**Last Updated:** January 2025  
**Status:** Ready for Implementation
