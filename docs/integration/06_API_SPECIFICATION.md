# API Specification

## Complete API Reference for New Features

---

## Overview

This document provides detailed API specifications for all new features being integrated from ProjectAirSim into Cosys-AirSim.

### Architecture Notes

**API Style**: Cosys-AirSim maintains its flat client API (methods on client with `vehicle_name` parameter) rather than adopting ProjectAirSim's object-oriented approach (Client/World/Drone objects). This ensures backward compatibility and consistency with existing APIs.

**Data Access Model**: ProjectAirSim uses Pub/Sub for sensor data streaming. Cosys-AirSim wraps this in a simpler request/response model for ease of use:
```python
# Cosys-AirSim (simple request/response)
battery = client.getBatteryData()  # Synchronous query

# ProjectAirSim (Pub/Sub - for reference)
drone.subscribe("Battery", callback)  # Async streaming
```

**Method Naming**: Cosys-AirSim uses camelCase (`getBatteryData`, `simContinueForTime`) to match existing API conventions, while ProjectAirSim uses snake_case (`get_battery_state`, `continue_for_sim_time`).

---

## 1. Battery Sensor API

### 1.1 BatteryData Structure

**ProjectAirSim Core Fields** (from Pub/Sub API):
```cpp
// Core fields that match ProjectAirSim battery sensor
struct BatteryDataCore {
    int64_t time_stamp;                  // Timestamp in nanoseconds
    float battery_pct_remaining;         // State of charge (0.0 - 100.0)
    float estimated_time_remaining;      // Estimated time to empty (seconds)
    std::string battery_charge_state;    // "OK", "LOW", "CRITICAL", "UNHEALTHY"
};
```

**Cosys-AirSim Enhanced Structure** (with additional physics data):
```cpp
// Enhanced battery data with voltage, current, and temperature
struct BatteryData {
    // Core ProjectAirSim fields
    float battery_pct_remaining;         // State of charge (0.0 - 100.0)
    float estimated_time_remaining;      // Estimated time to empty (seconds)
    std::string battery_charge_state;    // "OK", "LOW", "CRITICAL", "UNHEALTHY"
    int64_t time_stamp;                  // Timestamp in nanoseconds
    
    // Enhanced Cosys-AirSim fields (physics-based)
    float voltage;                       // Current battery voltage (V)
    float current_draw;                  // Current draw (A), positive = discharging
    float remaining_wh;                  // Remaining energy (Wh)
    float temperature;                   // Battery temperature (°C)
};
```

```python
# Python Definition
class BatteryData(MsgpackMixin):
    # Core ProjectAirSim fields
    battery_pct_remaining: float = 100.0         # 0-100
    estimated_time_remaining: float = 0.0        # seconds
    battery_charge_state: str = "OK"             # OK, LOW, CRITICAL, UNHEALTHY
    time_stamp: int = 0                          # nanoseconds
    
    # Enhanced Cosys-AirSim fields
    voltage: float = 0.0                         # Volts
    current_draw: float = 0.0                    # Amperes
    remaining_wh: float = 0.0                    # Watt-hours
    temperature: float = 25.0                    # Celsius
```

**Note**: Cosys-AirSim provides enhanced battery data beyond ProjectAirSim's core fields. The enhanced fields enable more detailed battery modeling and physics-based discharge simulation.

### 1.2 API Methods

#### getBatteryData

Get current battery sensor data.

**C++ Signature:**
```cpp
BatteryData getBatteryData(const std::string& vehicle_name = "", 
                           const std::string& sensor_name = "");
```

**Python Signature:**
```python
def getBatteryData(self, vehicle_name: str = '', sensor_name: str = '') -> BatteryData
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| vehicle_name | string | No | Vehicle to query. Empty = default vehicle |
| sensor_name | string | No | Sensor name. Empty = first battery sensor |

**Returns:** `BatteryData` object with current state

**Example:**
```python
import cosysairsim as airsim

client = airsim.MultirotorClient()
battery = client.getBatteryData()
print(f"Battery: {battery.battery_pct_remaining:.1f}% ({battery.voltage:.2f}V)")
print(f"Time remaining: {battery.estimated_time_remaining:.0f}s")
print(f"Status: {battery.battery_charge_state}")
```

---

#### setBatteryRemaining

Manually set battery remaining percentage (for testing/simulation).

**C++ Signature:**
```cpp
void setBatteryRemaining(float percent, 
                         const std::string& vehicle_name = "",
                         const std::string& sensor_name = "");
```

**Python Signature:**
```python
def setBatteryRemaining(self, percent: float, vehicle_name: str = '', 
                        sensor_name: str = '') -> None
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| percent | float | Yes | New battery percentage (0.0 - 100.0) |
| vehicle_name | string | No | Vehicle to modify |
| sensor_name | string | No | Sensor name |

**Example:**
```python
# Set battery to 50%
client.setBatteryRemaining(50.0)

# Set battery to low for testing emergency landing
client.setBatteryRemaining(10.0)
```

---

#### setBatteryDrainRate

Set battery drain rate multiplier.

**C++ Signature:**
```cpp
void setBatteryDrainRate(float rate_multiplier, 
                         const std::string& vehicle_name = "",
                         const std::string& sensor_name = "");
```

**Python Signature:**
```python
def setBatteryDrainRate(self, rate_multiplier: float, vehicle_name: str = '',
                        sensor_name: str = '') -> None
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| rate_multiplier | float | Yes | Drain rate multiplier (1.0 = normal, 2.0 = 2x faster) |
| vehicle_name | string | No | Vehicle to modify |
| sensor_name | string | No | Sensor name |

**Example:**
```python
# Speed up battery drain for faster testing
client.setBatteryDrainRate(10.0)

# Slow motion battery drain
client.setBatteryDrainRate(0.1)
```

---

#### setBatteryHealthStatus

Manually set battery health status (for fault injection testing).

**C++ Signature:**
```cpp
void setBatteryHealthStatus(const std::string& status, 
                            const std::string& vehicle_name = "",
                            const std::string& sensor_name = "");
```

**Python Signature:**
```python
def setBatteryHealthStatus(self, status: str, vehicle_name: str = '',
                           sensor_name: str = '') -> None
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| status | string | Yes | One of: "OK", "LOW", "CRITICAL", "UNHEALTHY" |
| vehicle_name | string | No | Vehicle to modify |
| sensor_name | string | No | Sensor name |

**Example:**
```python
# Simulate battery critical state
client.setBatteryHealthStatus("CRITICAL")

# Simulate unhealthy battery
client.setBatteryHealthStatus("UNHEALTHY")
```

---

### 1.3 Settings Configuration

**Cosys-AirSim settings.json format**:

```json
{
  "Vehicles": {
    "Drone1": {
      "Sensors": {
        "Battery1": {
          "SensorType": 12,
          "Enabled": true,
          "MaxCapacityJoules": 360000.0,
          "NominalVoltage": 22.2,
          "InternalResistance": 0.1,
          "DischargeMode": "rotor-power-discharge-mode",
          "InitialCharge": 100.0,
          "DrainRate": 1.0,
          "UpdateFrequency": 10.0,
          "Position": { "X": 0, "Y": 0, "Z": 0 },
          "Rotation": { "Pitch": 0, "Roll": 0, "Yaw": 0 }
        }
      }
    }
  }
}
```

> **Note**: ProjectAirSim uses JSONC robot configs with different structure. Cosys-AirSim maintains settings.json for backward compatibility.

**Settings Reference:**
| Setting | Type | Default | Description |
|---------|------|---------|-------------|
| SensorType | int | 12 | Must be 12 for battery |
| Enabled | bool | true | Enable/disable sensor |
| MaxCapacityJoules | float | 360000.0 | Maximum battery capacity in Joules (1 Ah = 3600 J) |
| NominalVoltage | float | 22.2 | Nominal voltage (6S LiPo default) |
| InternalResistance | float | 0.1 | Internal resistance in Ohms |
| DischargeMode | string | "simple-discharge-mode" | "simple-discharge-mode", "rotor-power-discharge-mode" |
| InitialCharge | float | 100.0 | Starting charge percentage |
| DrainRate | float | 1.0 | Drain rate multiplier (for simple mode) |
| UpdateFrequency | float | 10.0 | Sensor update frequency in Hz |

**Capacity Unit Conversions:**
- 1 Amp hour (Ah) = 3600 Joules
- 1 milli Amp hour (mAh) = 3.6 Joules
- For Watt-hours (Wh): Wh = (Joules / 3600) * Voltage

**Discharge Modes:**
- **simple-discharge-mode**: Linear rate-based discharge (abstract units)
- **rotor-power-discharge-mode**: Physics-based energy consumption from rotor power

---

## 2. Airspeed Sensor API

### 2.1 AirspeedData Structure

```cpp
// C++ Definition
struct AirspeedData {
    float indicated_airspeed;     // IAS in m/s
    float true_airspeed;          // TAS in m/s
    float differential_pressure;  // Pitot pressure in Pa
    float air_temperature;        // Outside air temp in Kelvin
    TTimePoint time_stamp;
};
```

```python
# Python Definition
class AirspeedData(MsgpackMixin):
    indicated_airspeed: float = 0.0
    true_airspeed: float = 0.0
    differential_pressure: float = 0.0
    air_temperature: float = 288.15  # 15°C in Kelvin
    time_stamp: int = 0
```

### 2.2 API Methods

#### getAirspeedData

Get current airspeed sensor data.

**C++ Signature:**
```cpp
AirspeedData getAirspeedData(const std::string& vehicle_name = "",
                              const std::string& sensor_name = "");
```

**Python Signature:**
```python
def getAirspeedData(self, vehicle_name: str = '', sensor_name: str = '') -> AirspeedData
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| vehicle_name | string | No | Vehicle to query |
| sensor_name | string | No | Sensor name |

**Returns:** `AirspeedData` object

**Example:**
```python
airspeed = client.getAirspeedData()
print(f"IAS: {airspeed.indicated_airspeed:.1f} m/s")
print(f"TAS: {airspeed.true_airspeed:.1f} m/s")
```

---

### 2.3 Settings Configuration

**Cosys-AirSim settings.json format**:

```json
{
  "Vehicles": {
    "Drone1": {
      "Sensors": {
        "Airspeed1": {
          "SensorType": 13,
          "Enabled": true,
          "PitotTubeDiameter": 0.005,
          "NoiseStdDev": 0.1,
          "UpdateFrequency": 50.0,
          "Position": { "X": 0.1, "Y": 0, "Z": 0 },
          "Rotation": { "Pitch": 0, "Roll": 0, "Yaw": 0 }
        }
      }
    }
  }
}
```

---

## 3. Radar Sensor API

> ⚠️ **Verification Note**: The Radar sensor was not found in ProjectAirSim API documentation during review. This may be a Cosys-AirSim original feature or may require additional verification from ProjectAirSim source code. Implementation priority should be adjusted accordingly.

### 3.1 Data Structures

```cpp
// C++ Definitions
struct RadarDetection {
    float range;              // Distance to target (m)
    float azimuth;            // Horizontal angle (rad)
    float elevation;          // Vertical angle (rad)
    float radial_velocity;    // Doppler velocity (m/s)
    float rcs;                // Radar cross section (m²)
    float signal_strength;    // Signal strength (dB)
};

struct RadarData {
    std::vector<RadarDetection> detections;
    TTimePoint time_stamp;
};
```

```python
# Python Definitions
class RadarDetection(MsgpackMixin):
    range: float = 0.0
    azimuth: float = 0.0
    elevation: float = 0.0
    radial_velocity: float = 0.0
    rcs: float = 0.0
    signal_strength: float = 0.0

class RadarData(MsgpackMixin):
    detections: List[RadarDetection] = []
    time_stamp: int = 0
```

### 3.2 API Methods

#### getRadarData

Get current radar sensor detections.

**C++ Signature:**
```cpp
RadarData getRadarData(const std::string& vehicle_name = "",
                       const std::string& sensor_name = "");
```

**Python Signature:**
```python
def getRadarData(self, vehicle_name: str = '', sensor_name: str = '') -> RadarData
```

**Returns:** `RadarData` object with list of detections

**Example:**
```python
radar = client.getRadarData()
print(f"Detected {len(radar.detections)} objects")
for det in radar.detections:
    print(f"  Range: {det.range:.1f}m, Velocity: {det.radial_velocity:.1f}m/s")
```

---

### 3.3 Settings Configuration

**Cosys-AirSim settings.json format**:

```json
{
  "Vehicles": {
    "Drone1": {
      "Sensors": {
        "Radar1": {
          "SensorType": 14,
          "Enabled": true,
          "RangeMax": 200.0,
          "AzimuthFOV": 60.0,
          "ElevationFOV": 30.0,
          "AzimuthResolution": 64,
          "ElevationResolution": 32,
          "NoiseFloorDb": -80.0,
          "UpdateFrequency": 20.0,
          "Position": { "X": 0.1, "Y": 0, "Z": 0 },
          "Rotation": { "Pitch": 0, "Roll": 0, "Yaw": 0 }
        }
      }
    }
  }
}
```

> **Note**: Radar sensor may be a Cosys-AirSim original feature (not verified in ProjectAirSim).

**Settings Reference:**
| Setting | Type | Default | Description |
|---------|------|---------|-------------|
| RangeMax | float | 200.0 | Maximum detection range (m) |
| AzimuthFOV | float | 60.0 | Horizontal field of view (degrees) |
| ElevationFOV | float | 30.0 | Vertical field of view (degrees) |
| AzimuthResolution | int | 64 | Horizontal angular bins |
| ElevationResolution | int | 32 | Vertical angular bins |
| NoiseFloorDb | float | -80.0 | Noise floor threshold (dB) |
| UpdateFrequency | float | 20.0 | Update frequency (Hz) |

---

## 4. Sim Clock Control API

### 4.1 API Methods

#### simGetSimTime

Get current simulation time in nanoseconds.

**C++ Signature:**
```cpp
uint64_t simGetSimTime();
```

**Python Signature:**
```python
def simGetSimTime(self) -> int
```

**Returns:** Simulation time in nanoseconds since start

**Example:**
```python
sim_time_ns = client.simGetSimTime()
sim_time_sec = sim_time_ns / 1e9
print(f"Simulation time: {sim_time_sec:.3f} seconds")
```

---

#### simGetSimClockType

Get the current clock mode.

**C++ Signature:**
```cpp
std::string simGetSimClockType();
```

**Python Signature:**
```python
def simGetSimClockType(self) -> str
```

**Returns:** One of "RealTime", "Steppable", "Scaled"

---

#### simContinueForTime

Step simulation forward by specified duration.

**C++ Signature:**
```cpp
void simContinueForTime(uint64_t duration_ns, bool wait_until_complete = true);
```

**Python Signature:**
```python
def simContinueForTime(self, duration_ns: int, wait_until_complete: bool = True) -> None
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| duration_ns | uint64 | Yes | Duration to step in nanoseconds |
| wait_until_complete | bool | No | If True, blocks until time reached. Default: True |

**Note**: ProjectAirSim's `continue_for_sim_time()` includes `wait_until_complete` parameter for non-blocking operation.

**Example:**
```python
# Pause first
client.simPause(True)

# Step forward 100ms (blocking)
client.simContinueForTime(100_000_000)  # 100ms in ns

# Get sensor data at this exact time
imu = client.getImuData()

# Non-blocking continue
client.simContinueForTime(50_000_000, wait_until_complete=False)
```

---

#### simContinueUntilTime

Step simulation until specified absolute time.

**C++ Signature:**
```cpp
void simContinueUntilTime(uint64_t target_time_ns, bool wait_until_complete = true);
```

**Python Signature:**
```python
def simContinueUntilTime(self, target_time_ns: int, wait_until_complete: bool = True) -> None
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| target_time_ns | uint64 | Yes | Target absolute time in nanoseconds |
| wait_until_complete | bool | No | If True, blocks until time reached. Default: True |

---

#### simContinueForSteps

Step simulation forward by N physics ticks.

**C++ Signature:**
```cpp
void simContinueForSteps(int num_steps, bool wait_until_complete = true);
```

**Python Signature:**
```python
def simContinueForSteps(self, num_steps: int, wait_until_complete: bool = True) -> None
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| num_steps | int | Yes | Number of physics steps to advance |
| wait_until_complete | bool | No | If True, blocks until steps complete. Default: True |

**Python Signature:**
```python
def simContinueForSteps(self, num_steps: int) -> None
```

---

#### simContinueForSingleStep

Step simulation by exactly one physics tick.

**C++ Signature:**
```cpp
void simContinueForSingleStep();
```

**Python Signature:**
```python
def simContinueForSingleStep(self) -> None
```

---

### 4.2 Clock Control Example

```python
import cosysairsim as airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# Enable stepping mode
client.simPause(True)

# Run control loop at precise 100Hz
dt_ns = 10_000_000  # 10ms = 100Hz

for i in range(1000):  # 10 seconds of sim time
    # Get current state
    state = client.getMultirotorState()
    imu = client.getImuData()
    
    # Compute control (your controller here)
    throttle, roll, pitch, yaw = compute_control(state, imu)
    
    # Apply control
    client.moveByRollPitchYawThrottleAsync(roll, pitch, yaw, throttle, 0.01)
    
    # Step simulation exactly 10ms
    client.simContinueForTime(dt_ns)

# Resume real-time
client.simPause(False)
```

---

## 5. Voxel Grid API

### 5.1 API Methods

#### simCreateVoxelGrid

Create occupancy voxel grid at specified location.

**C++ Signature:**
```cpp
std::vector<int8_t> simCreateVoxelGrid(const Vector3r& position,
                                        float x_size, float y_size, float z_size,
                                        float resolution);
```

**Python Signature:**
```python
def simCreateVoxelGrid(self, position: Vector3r, x_size: float, y_size: float,
                       z_size: float, resolution: float) -> bytes
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| position | Vector3r | Yes | Center position of grid (NED) |
| x_size | float | Yes | Grid extent in X (m) |
| y_size | float | Yes | Grid extent in Y (m) |
| z_size | float | Yes | Grid extent in Z (m) |
| resolution | float | Yes | Voxel size (m) |

**Returns:** 1D byte array of occupancy values (0=free, 1=occupied)

Array ordering: Z-major, Y-minor, X-least significant

**Example:**
```python
import numpy as np

# Create 10m x 10m x 5m grid with 0.5m resolution
position = airsim.Vector3r(0, 0, -2.5)  # Center at 2.5m height
voxels = client.simCreateVoxelGrid(position, 10.0, 10.0, 5.0, 0.5)

# Reshape to 3D
voxel_array = np.frombuffer(voxels, dtype=np.int8)
grid = voxel_array.reshape((10, 20, 20))  # z, y, x

# Count occupied voxels
occupied = np.sum(grid > 0)
print(f"Occupied voxels: {occupied}")
```

---

## 6. Lighting Control API

### 6.1 API Methods

#### simSetSunlightIntensity

Set sun/directional light intensity.

**C++ Signature:**
```cpp
void simSetSunlightIntensity(float intensity);
```

**Python Signature:**
```python
def simSetSunlightIntensity(self, intensity: float) -> None
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| intensity | float | Yes | Light intensity (0.0 - 10.0, default 1.0) |

---

#### simGetSunlightIntensity

Get current sun intensity.

**C++ Signature:**
```cpp
float simGetSunlightIntensity();
```

**Python Signature:**
```python
def simGetSunlightIntensity(self) -> float
```

---

#### simSetCloudShadowStrength

Set cloud shadow strength.

**C++ Signature:**
```cpp
void simSetCloudShadowStrength(float strength);
```

**Python Signature:**
```python
def simSetCloudShadowStrength(self, strength: float) -> None
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| strength | float | Yes | Shadow strength (0.0 - 1.0) |

---

## 7. Manual Actuator Control API

### 7.1 API Methods

#### setControlSignals

Directly set actuator control signals, bypassing autonomous controllers.

**C++ Signature:**
```cpp
void setControlSignals(const std::map<std::string, float>& signals,
                       const std::string& vehicle_name = "");
```

**Python Signature:**
```python
def setControlSignals(self, signals: Dict[str, float], vehicle_name: str = '') -> None
```

**Parameters:**
| Name | Type | Required | Description |
|------|------|----------|-------------|
| signals | Dict[str, float] | Yes | Map of actuator ID to control value |
| vehicle_name | string | No | Vehicle to control |

**Example:**
```python
# Direct motor control for quadcopter
client.setControlSignals({
    "rotor_0": 0.6,  # Front-left
    "rotor_1": 0.6,  # Front-right
    "rotor_2": 0.6,  # Rear-left
    "rotor_3": 0.6,  # Rear-right
})
```

---

## 8. Error Codes

| Code | Name | Description |
|------|------|-------------|
| 0 | SUCCESS | Operation completed successfully |
| 1 | VEHICLE_NOT_FOUND | Specified vehicle does not exist |
| 2 | SENSOR_NOT_FOUND | Specified sensor does not exist |
| 3 | INVALID_PARAMETER | Parameter out of valid range |
| 4 | NOT_IMPLEMENTED | Feature not yet implemented |
| 5 | TIMEOUT | Operation timed out |
| 6 | INTERNAL_ERROR | Unexpected internal error |

---

## 9. API Versioning

All new APIs follow semantic versioning:

- **Major version**: Breaking changes to API signatures
- **Minor version**: New features, backward compatible
- **Patch version**: Bug fixes only

Current API version for new features: **1.0.0**

Check version programmatically:
```python
version = client.getApiVersion()
print(f"API Version: {version}")
```

---

## 10. Thread Safety

All API methods are thread-safe when:
- Using separate client connections per thread
- Not modifying and reading the same sensor simultaneously

For multi-threaded applications:
```python
import threading

def sensor_thread(vehicle_name):
    client = airsim.MultirotorClient()  # New connection per thread
    while True:
        battery = client.getBatteryData(vehicle_name)
        # Process data...

threads = [
    threading.Thread(target=sensor_thread, args=("Drone1",)),
    threading.Thread(target=sensor_thread, args=("Drone2",)),
]
for t in threads:
    t.start()
```
