# Architecture Design

## Technical Architecture for ProjectAirSim Feature Integration

---

## 1. Current Cosys-AirSim Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     COSYS-AIRSIM ARCHITECTURE                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    PYTHON CLIENT                          │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐   │   │
│  │  │MultirotorCli│  │  CarClient  │  │ComputerVisionCli│   │   │
│  │  └─────────────┘  └─────────────┘  └─────────────────┘   │   │
│  │                          │                                │   │
│  │                          ▼                                │   │
│  │              ┌────────────────────┐                       │   │
│  │              │  msgpack/rpclib    │                       │   │
│  │              └────────────────────┘                       │   │
│  └──────────────────────────────────────────────────────────┘   │
│                              │                                   │
│                              │ RPC                               │
│                              ▼                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                      AIRLIB (C++)                         │   │
│  │  ┌─────────────────────────────────────────────────────┐ │   │
│  │  │                   RPC SERVER                         │ │   │
│  │  │   RpcLibServerBase.cpp  ←→  RpcLibAdaptorsBase.hpp   │ │   │
│  │  └─────────────────────────────────────────────────────┘ │   │
│  │                          │                                │   │
│  │  ┌───────────────────────┼───────────────────────────┐   │   │
│  │  │                       ▼                            │   │   │
│  │  │              VehicleApiBase.hpp                    │   │   │
│  │  │    ┌──────────────────────────────────────┐       │   │   │
│  │  │    │  getSensorData() | setControls()     │       │   │   │
│  │  │    └──────────────────────────────────────┘       │   │   │
│  │  │                       │                            │   │   │
│  │  │    ┌──────────────────┴──────────────────┐        │   │   │
│  │  │    ▼                                      ▼        │   │   │
│  │  │ SENSORS                              VEHICLES      │   │   │
│  │  │ ├─ IMU                              ├─ Multirotor │   │   │
│  │  │ ├─ GPS                              ├─ Car        │   │   │
│  │  │ ├─ Barometer                        └─ CV         │   │   │
│  │  │ ├─ Magnetometer                                    │   │   │
│  │  │ ├─ LiDAR                                          │   │   │
│  │  │ ├─ GPULiDAR                                       │   │   │
│  │  │ ├─ Echo                                           │   │   │
│  │  │ ├─ UWB                                            │   │   │
│  │  │ ├─ WiFi                                           │   │   │
│  │  │ └─ Distance                                       │   │   │
│  │  └────────────────────────────────────────────────────┘   │   │
│  └──────────────────────────────────────────────────────────┘   │
│                              │                                   │
│                              │ SimMode                           │
│                              ▼                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                  UNREAL PLUGIN                            │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐   │   │
│  │  │ ASimHUD     │  │ SimModeBase │  │ VehiclePawnBase │   │   │
│  │  └─────────────┘  └─────────────┘  └─────────────────┘   │   │
│  └──────────────────────────────────────────────────────────┘   │
│                              │                                   │
│                              ▼                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                  UNREAL ENGINE 5.x                        │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. ProjectAirSim vs Cosys-AirSim: Key Architectural Differences

### 2.1 API Style: Object-Oriented vs Flat

**ProjectAirSim (Object-Oriented)**:
```python
# Create client and world
client = ProjectAirSimClient()
world = World(client, "scene_config.jsonc")

# Get vehicle object
drone = Drone(client, world, "Drone1")

# Methods are on drone object (no vehicle_name parameter)
battery = drone.get_battery_state("Battery")
await drone.takeoff_async()
drone.set_control_signals([0.5, 0.5, 0.5, 0.5])
```

**Cosys-AirSim (Flat Client API)**:
```python
# Create client (single entry point)
client = airsim.MultirotorClient()
client.confirmConnection()

# All methods on client with vehicle_name parameter
battery = client.getBatteryData(vehicle_name="Drone1", sensor_name="Battery")
client.takeoffAsync(vehicle_name="Drone1")
client.moveByMotorPWMsAsync([0.5, 0.5, 0.5, 0.5], vehicle_name="Drone1")
```

**Why Cosys-AirSim Maintains Flat API:**
1. ✅ **Backward compatibility** - No breaking changes to existing code
2. ✅ **Consistency** - Matches existing 11 sensor APIs
3. ✅ **Simplicity** - Single client object vs managing Client/World/Drone objects
4. ✅ **Default behavior** - Empty vehicle_name = use default vehicle (convenient for single-drone scenarios)

### 2.2 Configuration: JSONC Modular vs JSON Monolithic

**ProjectAirSim (Modular JSONC)**:
```
configs/
  ├── scene_blocks.jsonc       # Environment, weather, time of day
  └── robot_drone.jsonc         # Vehicle structure, sensors, actuators
```
- Separate scene and robot configs
- JSONC format (allows comments)
- Loaded explicitly: `World(client, "scene_blocks.jsonc")`

**Cosys-AirSim (Monolithic JSON)**:
```
settings.json                   # Single file for everything
```
- Single settings.json in Documents/AirSim
- JSON format (standard, no comments)
- Auto-loaded on startup

**Why Cosys-AirSim Maintains settings.json:**
1. ✅ **Backward compatibility** - Existing users have settings.json files
2. ✅ **No breaking changes** - Users don't need to restructure configs
3. ✅ **Simpler for beginners** - One file to manage
4. ✅ **Auto-discovery** - No need to specify config path

### 2.3 Pub/Sub vs Request/Response

**ProjectAirSim (Pub/Sub Model)**:
```python
# Subscribe to battery topic
def battery_callback(data):
    print(f"Battery: {data['battery_pct_remaining']}%")

drone.subscribe("Battery", battery_callback)
```
- Asynchronous data streaming
- Event-driven updates
- Lower latency for high-frequency data

**Cosys-AirSim (Request/Response Model)**:
```python
# Request battery data
battery = client.getBatteryData()
print(f"Battery: {battery.battery_pct_remaining}%")
```
- Synchronous data queries
- On-demand access
- Simpler programming model

**Integration Strategy:**
- Cosys-AirSim can wrap Pub/Sub in request/response internally
- Provides simpler API for users while maintaining ProjectAirSim feature compatibility

### 2.4 Method Naming Conventions

| Feature | ProjectAirSim | Cosys-AirSim |
|---------|---------------|--------------|
| Style | snake_case | camelCase |
| Battery Data | `get_battery_state()` | `getBatteryData()` |
| Set Battery | `set_battery_remaining()` | `setBatteryRemaining()` |
| Drain Rate | `set_battery_drain_rate()` | `setBatteryDrainRate()` |
| Clock Time | `get_sim_time()` | `getSimTime()` |
| Continue Time | `continue_for_sim_time()` | `simContinueForTime()` |
| Pause/Resume | `pause()`, `resume()` | `simPause()`, `simContinue()` |

**Rationale:** Cosys-AirSim maintains camelCase for consistency with existing API surface.

---

## 3. Integration Design Strategy

### Principle: Non-Invasive Integration

All new features will be added as **new modules** alongside existing code:
- New sensor classes in `AirLib/include/sensors/`
- New API methods added to existing classes
- New Python methods added to existing client classes
- Settings additions are backward-compatible

### File Structure for New Features

```
AirLib/
├── include/
│   ├── sensors/
│   │   ├── battery/                    # NEW
│   │   │   ├── BatterySensorBase.hpp
│   │   │   └── BatterySensorSimple.hpp
│   │   ├── airspeed/                   # NEW
│   │   │   ├── AirspeedSensorBase.hpp
│   │   │   └── AirspeedSensorSimple.hpp
│   │   └── radar/                      # NEW
│   │       ├── RadarSensorBase.hpp
│   │       └── RadarSensorSimple.hpp
│   ├── api/
│   │   └── VehicleApiBase.hpp          # ADD methods
│   └── common/
│       └── AirSimSettings.hpp          # ADD sensor types
└── src/
    └── api/
        └── RpcLibServerBase.cpp        # ADD bindings

PythonClient/
└── cosysairsim/
    ├── client.py                       # ADD methods
    └── types.py                        # ADD data types
```

---

## 3. Sensor Integration Design

### 3.1 Battery Sensor

#### Class Hierarchy

```
SensorBase
    └── BatterySensorBase (abstract)
            └── BatterySensorSimple (concrete)
```

#### BatterySensorBase Interface

```cpp
// AirLib/include/sensors/battery/BatterySensorBase.hpp

class BatterySensorBase : public SensorBase {
public:
    struct Output {
        TTimePoint time_stamp;
        float voltage;              // Current voltage (V)
        float current;              // Current draw (A)
        float charge_remaining;     // Remaining charge (mAh)
        float percentage;           // State of charge (0-100%)
        float temperature;          // Battery temperature (°C)
        bool is_healthy;            // Health status
        bool is_charging;           // Charging state
    };
    
    virtual const Output& getOutput() const = 0;
    virtual void setChargeRemaining(float mah) = 0;
    virtual void setDrainRate(float ma) = 0;
    virtual void setHealthStatus(bool healthy) = 0;
};
```

#### Settings Schema

```json
{
    "SensorType": 12,
    "Enabled": true,
    "Capacity": 5000,
    "Voltage": 14.8,
    "MaxVoltage": 16.8,
    "MinVoltage": 12.0,
    "InternalResistance": 0.05,
    "DrainRate": 100.0,
    "SimulationMode": "PhysicsBased"
}
```

### 3.2 Airspeed Sensor

#### Class Hierarchy

```
SensorBase
    └── AirspeedSensorBase (abstract)
            └── AirspeedSensorSimple (concrete)
```

#### AirspeedSensorBase Interface

```cpp
// AirLib/include/sensors/airspeed/AirspeedSensorBase.hpp

class AirspeedSensorBase : public SensorBase {
public:
    struct Output {
        TTimePoint time_stamp;
        float indicated_airspeed;   // IAS (m/s)
        float true_airspeed;        // TAS (m/s)
        float differential_pressure; // Pitot tube pressure (Pa)
        float temperature;          // Air temperature (K)
    };
    
    virtual const Output& getOutput() const = 0;
};
```

#### Settings Schema

```json
{
    "SensorType": 13,
    "Enabled": true,
    "NoiseStdDev": 0.1,
    "UpdateFrequency": 50,
    "PitotPosition": {"X": 0.3, "Y": 0, "Z": 0}
}
```

### 3.3 Radar Sensor

#### Class Hierarchy

```
SensorBase
    └── RadarSensorBase (abstract)
            └── RadarSensorSimple (concrete)
```

#### RadarSensorBase Interface

```cpp
// AirLib/include/sensors/radar/RadarSensorBase.hpp

struct RadarDetection {
    Vector3r position;      // Relative position
    Vector3r velocity;      // Relative velocity
    float rcs;              // Radar cross section
    float snr;              // Signal to noise ratio
};

class RadarSensorBase : public SensorBase {
public:
    struct Output {
        TTimePoint time_stamp;
        std::vector<RadarDetection> detections;
    };
    
    virtual const Output& getOutput() const = 0;
};
```

#### Settings Schema

```json
{
    "SensorType": 14,
    "Enabled": true,
    "Range": 100.0,
    "HorizontalFOV": 30.0,
    "VerticalFOV": 30.0,
    "UpdateFrequency": 20,
    "NoiseStdDev": 0.05
}
```

---

## 4. API Integration Design

### 4.1 New SensorType Enum Values

```cpp
// AirLib/include/sensors/SensorBase.hpp

enum class SensorType : uint {
    Barometer = 1,
    Imu = 2,
    Gps = 3,
    Magnetometer = 4,
    Distance = 5,
    Lidar = 6,
    Echo = 7,
    GPULidar = 8,
    MarLocUwb = 9,
    SensorTemplate = 10,
    Wifi = 11,
    Battery = 12,      // NEW
    Airspeed = 13,     // NEW
    Radar = 14         // NEW
};
```

### 4.2 New VehicleApiBase Methods

```cpp
// AirLib/include/api/VehicleApiBase.hpp

// Battery API
virtual const BatterySensorBase::Output& getBatteryData(
    const std::string& battery_name) const;
virtual void setBatteryRemaining(
    const std::string& battery_name, float mah);
virtual void setBatteryDrainRate(
    const std::string& battery_name, float drain_rate);
virtual void setBatteryHealthStatus(
    const std::string& battery_name, bool is_healthy);

// Airspeed API
virtual const AirspeedSensorBase::Output& getAirspeedData(
    const std::string& airspeed_name) const;

// Radar API  
virtual const RadarSensorBase::Output& getRadarData(
    const std::string& radar_name) const;
```

### 4.3 New Python Client Methods

```python
# PythonClient/cosysairsim/client.py

class AirSimClientBase:
    # Battery API
    def getBatteryData(self, battery_name='', vehicle_name=''):
        """Get battery state data."""
        return BatteryData.from_msgpack(
            self.client.call('getBatteryData', battery_name, vehicle_name))
    
    def setBatteryRemaining(self, charge_mah, battery_name='', vehicle_name=''):
        """Set remaining battery charge in mAh."""
        self.client.call('setBatteryRemaining', charge_mah, battery_name, vehicle_name)
    
    def setBatteryDrainRate(self, drain_rate, battery_name='', vehicle_name=''):
        """Set battery drain rate in mA."""
        self.client.call('setBatteryDrainRate', drain_rate, battery_name, vehicle_name)
    
    def setBatteryHealthStatus(self, is_healthy, battery_name='', vehicle_name=''):
        """Set battery health status."""
        self.client.call('setBatteryHealthStatus', is_healthy, battery_name, vehicle_name)
    
    # Airspeed API
    def getAirspeedData(self, airspeed_name='', vehicle_name=''):
        """Get airspeed sensor data."""
        return AirspeedData.from_msgpack(
            self.client.call('getAirspeedData', airspeed_name, vehicle_name))
    
    # Radar API
    def getRadarData(self, radar_name='', vehicle_name=''):
        """Get radar detections."""
        return RadarData.from_msgpack(
            self.client.call('getRadarData', radar_name, vehicle_name))
```

### 4.4 New Python Data Types

```python
# PythonClient/cosysairsim/types.py

class BatteryData(MsgpackMixin):
    time_stamp = np.uint64(0)
    voltage = 0.0
    current = 0.0
    charge_remaining = 0.0
    percentage = 0.0
    temperature = 0.0
    is_healthy = True
    is_charging = False

class AirspeedData(MsgpackMixin):
    time_stamp = np.uint64(0)
    indicated_airspeed = 0.0
    true_airspeed = 0.0
    differential_pressure = 0.0
    temperature = 0.0

class RadarDetection(MsgpackMixin):
    position = Vector3r()
    velocity = Vector3r()
    rcs = 0.0
    snr = 0.0

class RadarData(MsgpackMixin):
    time_stamp = np.uint64(0)
    detections = []
```

---

## 5. World API Integration Design

### 5.1 New WorldSimApiBase Methods

```cpp
// AirLib/include/api/WorldSimApiBase.hpp

// Lighting Control
virtual void setSunlightIntensity(float intensity) = 0;
virtual float getSunlightIntensity() const = 0;
virtual void setCloudShadowStrength(float strength) = 0;
virtual float getCloudShadowStrength() const = 0;

// Sim Clock Control
virtual uint64_t getSimTime() const = 0;
virtual std::string getSimClockType() const = 0;
virtual void continueForSimTime(uint64_t delta_ns) = 0;
virtual void continueUntilSimTime(uint64_t target_ns) = 0;
virtual void continueForSteps(int n_steps) = 0;
virtual void continueForSingleStep() = 0;

// Voxel Grid
virtual std::vector<bool> createVoxelGrid(
    const Vector3r& position, 
    float x_size, float y_size, float z_size,
    float resolution,
    bool write_file = false,
    const std::string& file_path = "") = 0;
```

### 5.2 New Python Client World Methods

```python
# PythonClient/cosysairsim/client.py

class AirSimClientBase:
    # Lighting
    def simSetSunlightIntensity(self, intensity):
        """Set sun light intensity (0-75000, default 2.75)."""
        self.client.call('simSetSunlightIntensity', intensity)
    
    def simGetSunlightIntensity(self):
        """Get current sun light intensity."""
        return self.client.call('simGetSunlightIntensity')
    
    def simSetCloudShadowStrength(self, strength):
        """Set cloud shadow strength (0.0-1.0)."""
        self.client.call('simSetCloudShadowStrength', strength)
    
    # Sim Clock
    def simGetSimTime(self):
        """Get simulation time in nanoseconds."""
        return self.client.call('simGetSimTime')
    
    def simContinueForTime(self, duration_ns):
        """Continue simulation for duration (nanoseconds)."""
        self.client.call('simContinueForTime', duration_ns)
    
    def simContinueForSteps(self, n_steps):
        """Continue simulation for n physics steps."""
        self.client.call('simContinueForSteps', n_steps)
    
    # Voxel Grid
    def simCreateVoxelGrid(self, position, x_size, y_size, z_size, 
                           resolution, write_file=False, file_path=''):
        """Create voxel occupancy grid."""
        return self.client.call('simCreateVoxelGrid', 
            position, x_size, y_size, z_size, 
            resolution, write_file, file_path)
```

---

## 6. Settings Integration Design

### 6.1 New Settings Classes

```cpp
// AirLib/include/common/AirSimSettings.hpp

struct BatterySetting : SensorSetting {
    float capacity = 5000.0f;         // mAh
    float voltage = 14.8f;            // Nominal voltage
    float max_voltage = 16.8f;        // Fully charged
    float min_voltage = 12.0f;        // Empty
    float internal_resistance = 0.05f; // Ohms
    float drain_rate = 100.0f;        // mA
    std::string simulation_mode = "Linear";
};

struct AirspeedSetting : SensorSetting {
    float noise_std_dev = 0.1f;
    int update_frequency = 50;
    Vector3r pitot_position = Vector3r::Zero();
};

struct RadarSetting : SensorSetting {
    float range = 100.0f;
    float horizontal_fov = 30.0f;
    float vertical_fov = 30.0f;
    int update_frequency = 20;
    float noise_std_dev = 0.05f;
};
```

### 6.2 Settings JSON Example

```json
{
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "Sensors": {
                "Battery1": {
                    "SensorType": 12,
                    "Enabled": true,
                    "Capacity": 5000,
                    "Voltage": 14.8,
                    "DrainRate": 100.0
                },
                "Airspeed1": {
                    "SensorType": 13,
                    "Enabled": true,
                    "NoiseStdDev": 0.1
                },
                "Radar1": {
                    "SensorType": 14,
                    "Enabled": true,
                    "Range": 100.0
                }
            }
        }
    }
}
```

---

## 7. RPC Integration Design

### 7.1 New RPC Bindings

```cpp
// AirLib/src/api/RpcLibServerBase.cpp

// Battery bindings
pimpl_->server.bind("getBatteryData", 
    [&](const std::string& battery_name, const std::string& vehicle_name) -> 
    RpcLibAdaptorsBase::BatteryData {
        return RpcLibAdaptorsBase::BatteryData(
            getVehicleApi(vehicle_name)->getBatteryData(battery_name));
    });

pimpl_->server.bind("setBatteryRemaining",
    [&](float charge, const std::string& battery_name, const std::string& vehicle_name) {
        getVehicleApi(vehicle_name)->setBatteryRemaining(battery_name, charge);
    });

// Similar for airspeed and radar...
```

### 7.2 RPC Adaptor Types

```cpp
// AirLib/include/api/RpcLibAdaptorsBase.hpp

struct BatteryData {
    msr::airlib::TTimePoint time_stamp;
    float voltage;
    float current;
    float charge_remaining;
    float percentage;
    float temperature;
    bool is_healthy;
    bool is_charging;
    
    MSGPACK_DEFINE_MAP(time_stamp, voltage, current, 
        charge_remaining, percentage, temperature, 
        is_healthy, is_charging);
    
    BatteryData() = default;
    BatteryData(const msr::airlib::BatterySensorBase::Output& o);
};
```

---

## 8. Testing Strategy

### Unit Tests

```cpp
// AirLibUnitTests/BatteryTest.hpp

TEST(BatterySensor, DrainRate) {
    BatterySensorSimple sensor(settings);
    sensor.update(1.0f);  // 1 second
    
    EXPECT_LT(sensor.getOutput().percentage, 100.0f);
    EXPECT_GT(sensor.getOutput().charge_remaining, 0.0f);
}
```

### Integration Tests

```python
# test_battery_sensor.py

def test_battery_drain():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    initial = client.getBatteryData()
    time.sleep(10)
    final = client.getBatteryData()
    
    assert final.percentage < initial.percentage
    assert final.charge_remaining < initial.charge_remaining
```

---

## 9. Backward Compatibility

### Guaranteed Compatibility

1. **All existing sensor APIs work unchanged**
2. **All existing settings.json files work unchanged**
3. **All existing Python client code works unchanged**
4. **All existing RPC bindings remain functional**

### Migration Path

For users wanting new features:
1. Add new sensor configurations to settings.json
2. Use new API methods in client code
3. No changes required to existing code
