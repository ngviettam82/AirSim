# ProjectAirSim vs Cosys-AirSim - Detailed Comparison

## Side-by-Side Feature Comparison

### 1. Sensor Inventory

| Sensor Type | Cosys-AirSim | ProjectAirSim | Notes |
|-------------|--------------|---------------|-------|
| **Camera** | ✅ Yes | ✅ Yes | Both support RGB, Depth, Segmentation |
| **IMU** | ✅ Yes (ID=2) | ✅ Yes | Similar implementation |
| **GPS** | ✅ Yes (ID=3) | ✅ Yes | Similar NavSat structures |
| **Barometer** | ✅ Yes (ID=1) | ✅ Yes | Pressure/altitude sensor |
| **Magnetometer** | ✅ Yes (ID=4) | ✅ Yes | Compass sensor |
| **LiDAR** | ✅ Yes (ID=6) | ✅ Yes | Point cloud sensor |
| **GPU-LiDAR** | ✅ Yes (ID=8) | ❌ No | **Cosys-AirSim unique** |
| **Distance Sensor** | ✅ Yes (ID=5) | ❌ Not verified | Single-ray distance |
| **Echo Sensor** | ✅ Yes (ID=7) | ❌ No | **Cosys-AirSim unique** |
| **UWB (MarLocUwb)** | ✅ Yes (ID=9) | ❌ No | **Cosys-AirSim unique** |
| **WiFi Sensor** | ✅ Yes (ID=10) | ❌ No | **Cosys-AirSim unique** |
| **Template Sensor** | ✅ Yes (ID=11) | ❌ No | Custom sensor framework |
| **Battery Sensor** | ❌ No | ✅ Yes | **ProjectAirSim has, Cosys needs** |
| **Airspeed Sensor** | ❌ No | ✅ Yes | **ProjectAirSim has, Cosys needs** |
| **Radar Sensor** | ❌ No | ❓ Unclear | Not in ProjectAirSim API docs |

### 2. API Architecture

| Aspect | Cosys-AirSim | ProjectAirSim |
|--------|--------------|---------------|
| **API Style** | Flat client API | Object-oriented |
| **Client Object** | `MultirotorClient()` | `ProjectAirSimClient()` |
| **World Object** | No separate object | `World(client, scene_config)` |
| **Vehicle Object** | No separate object | `Drone(client, world, name)` |
| **Method Access** | `client.method(vehicle_name, ...)` | `drone.method(...)` |
| **Connection** | `confirmConnection()` | `connect()` / `disconnect()` |

**Example - Takeoff Comparison:**

```python
# Cosys-AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "Drone1")
client.takeoffAsync("Drone1").join()

# ProjectAirSim
client = ProjectAirSimClient()
client.connect()
world = World(client, "scene.jsonc")
drone = Drone(client, world, "Drone1")
await drone.enable_api_control()
await drone.takeoff_async()
```

### 3. Configuration Format

| Aspect | Cosys-AirSim | ProjectAirSim |
|--------|--------------|---------------|
| **Format** | JSON (settings.json) | JSONC (multiple files) |
| **Structure** | Monolithic single file | Scene + Robot configs |
| **Location** | `~/Documents/AirSim/settings.json` | Passed to `World()` constructor |
| **Comments** | Not supported (JSON) | Supported (JSONC) |

#### Cosys-AirSim Configuration
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "Sensors": {
        "Imu1": {
          "SensorType": 2,
          "Enabled": true
        }
      }
    }
  }
}
```

#### ProjectAirSim Configuration
```jsonc
// scene_basic.jsonc
{
  "scene": {
    "time-of-day": "12:00:00",
    "weather": "clear"
  }
}

// robot_quadrotor.jsonc
{
  "type": "multirotor",
  "sensors": [
    {
      "id": "Imu1",
      "type": "imu",
      "enabled": true,
      "parent-link": "Frame"
    }
  ]
}
```

### 4. Battery Sensor Detailed Comparison

#### Configuration

**Cosys-AirSim (Proposed)**:
```json
{
  "Battery1": {
    "SensorType": 12,
    "Enabled": true,
    "MaxCapacityWh": 100.0,
    "NominalVoltage": 22.2,
    "DischargeMode": "PhysicsBased"
  }
}
```

**ProjectAirSim (Actual)**:
```jsonc
{
  "id": "Battery",
  "type": "battery",
  "enabled": true,
  "parent-link": "Frame",
  "battery-mode": "rotor-power-discharge-mode",
  "total-battery-capacity": 36000,  // Joules
  "battery-capacity-on-start": 30000,
  "rotor-power-coefficient": 1
}
```

#### API Methods

| Method | Cosys-AirSim (Proposed) | ProjectAirSim (Actual) |
|--------|-------------------------|------------------------|
| Get battery data | `getBatteryData(vehicle_name, sensor_name)` | `drone.get_battery_state(sensor_name)` |
| Set remaining | `setBatteryRemaining(percent, vehicle_name)` | `drone.set_battery_remaining(value)` |
| Get drain rate | N/A | `drone.get_battery_drain_rate(sensor_name)` |
| Set drain rate | `setBatteryDrainRate(rate, vehicle_name)` | `drone.set_battery_drain_rate(rate)` |
| Set health | `setBatteryHealthStatus(status, vehicle_name)` | `drone.set_battery_health_status(is_healthy)` |

#### Data Structure

**Cosys-AirSim (Proposed)**:
```python
class BatteryData:
    voltage: float              # ✅ Rich data
    current_draw: float         # ✅ Rich data
    remaining_percent: float    # ✅ Common
    remaining_wh: float         # ✅ Rich data
    temperature: float          # ✅ Rich data
    health_status: str          # ✅ Common
    time_stamp: int            # ✅ Common
```

**ProjectAirSim (Actual)**:
```python
{
  'time_stamp': int,                      # ✅ Common
  'battery_pct_remaining': float,         # ✅ Common
  'estimated_time_remaining': float,      # ✅ ProjectAirSim unique
  'battery_charge_state': str            # ✅ Common (enum)
}
```

**Analysis**:
- ✅ Cosys-AirSim can provide MORE detailed data (voltage, current, temperature, Wh)
- ✅ ProjectAirSim provides time-to-empty estimation
- ✅ Both provide percentage and health status
- 🎯 **Recommendation**: Cosys implementation should include ALL fields from both

### 5. Sim Clock Control Comparison

| Method | Cosys-AirSim (Current) | ProjectAirSim | Proposed Cosys Method |
|--------|------------------------|---------------|----------------------|
| Pause | `simPause(is_paused)` | `world.pause()` / `world.resume()` | Keep `simPause(bool)` |
| Get time | ❌ Not available | `world.get_sim_time()` | ✅ Add `simGetSimTime()` |
| Get clock type | ❌ Not available | `world.get_sim_clock_type()` | ✅ Add `simGetSimClockType()` |
| Step by time | ❌ Not available | `world.continue_for_sim_time(ns, wait)` | ✅ Add `simContinueForTime(ns)` |
| Step by steps | ❌ Not available | `world.continue_for_n_steps(n, wait)` | ✅ Add `simContinueForSteps(n)` |
| Single step | ❌ Not available | `world.continue_for_single_step(wait)` | ✅ Add `simContinueForSingleStep()` |

**Key Difference**: ProjectAirSim has `wait_until_complete` parameter for async control

### 6. Voxel Grid Comparison

| Aspect | Cosys-AirSim (Proposed) | ProjectAirSim (Actual) |
|--------|-------------------------|------------------------|
| **Method** | `simCreateVoxelGrid(pos, x, y, z, res)` | `world.create_voxel_grid(pos, x, y, z, res, write_file, file_path)` |
| **Return Type** | `bytes` (1D array) | `1D occupancy array` + optional .binvox file |
| **File Output** | Proposed but not documented | ✅ Built-in .binvox export |
| **Indexing** | Z-major, Y-minor, X-least | ✅ Same |
| **Grid Shape** | Rectangular | ✅ Currently cubic only (x=y=z) |

**Recommendation**: Add file export capability to Cosys implementation

### 7. Lighting Control Comparison

| API | Cosys-AirSim (Proposed) | ProjectAirSim (Actual) | Match? |
|-----|-------------------------|------------------------|---------|
| Set sun intensity | `simSetSunlightIntensity(float)` | `world.set_sunlight_intensity(float)` | ✅ Yes |
| Get sun intensity | `simGetSunlightIntensity()` | `world.get_sunlight_intensity()` | ✅ Yes |
| Set cloud shadow | `simSetCloudShadowStrength(float)` | `world.set_cloud_shadow_strength(float)` | ✅ Yes |
| Get cloud shadow | `simGetCloudShadowStrength()` | `world.get_cloud_shadow_strength()` | ✅ Yes |

**Status**: ✅ API matches well, just different access pattern (world object vs sim methods)

### 8. Manual Actuator Control

**Cosys-AirSim (Proposed)**:
```python
client.setControlSignals({
    "rotor_0": 0.6,
    "rotor_1": 0.6,
    "rotor_2": 0.6,
    "rotor_3": 0.6
}, vehicle_name="Drone1")
```

**ProjectAirSim (Actual)**:
```python
drone.set_control_signals({
    "Prop_FR_actuator": 0.3,
    "Prop_FL_actuator": 0.3,
    "Prop_RR_actuator": 0.3,
    "Prop_RL_actuator": 0.3
})
```

**Key Differences**:
1. ✅ Concept is the same
2. ⚠️ Actuator naming convention differs (depends on robot config)
3. ✅ Both use Dict[str, float] for control signals

### 9. Communication Architecture

#### Cosys-AirSim
```
Python Client ←→ RPC (msgpack) ←→ C++ RpcLibServer ←→ VehicleApi ←→ Sensors
```
- Request/Response model
- Single RPC port
- Synchronous with Async wrappers

#### ProjectAirSim
```
Python Client ←→ Topics (Pub/Sub) ←→ Services (Req/Res) ←→ C++ Backend
              ├─ port_topics (NNG)
              └─ port_services (NNG)
```
- Dual model: Pub/Sub for sensors, Req/Res for commands
- Two separate ports (topics + services)
- Native async with asyncio

**Impact**: 
- Cosys-AirSim simpler for users (single connection, straightforward API)
- ProjectAirSim more flexible (can subscribe to high-rate sensor streams)

### 10. Unreal Engine Integration

| Aspect | Cosys-AirSim | ProjectAirSim |
|--------|--------------|---------------|
| **UE Version** | 5.5 | 5.2 |
| **Plugin Structure** | `Unreal/Plugins/AirSim/` | `rendering/` + `unreal/` |
| **Build System** | VS 2022 + CMake | VS 2022 + CMake |
| **Asset Loading** | Static in project | Dynamic via scene config |
| **Physics** | PhysX | PhysX + Custom |

### 11. Python Client Differences

| Feature | Cosys-AirSim | ProjectAirSim |
|---------|--------------|---------------|
| **Package Name** | `cosysairsim` | `projectairsim` |
| **Async Support** | `Async()` suffix methods | Native `async/await` |
| **Method Naming** | camelCase | snake_case |
| **Error Handling** | Exceptions | Exceptions |
| **Type Hints** | Partial | Extensive |

**Example**:
```python
# Cosys-AirSim
from cosysairsim import MultirotorClient
client = MultirotorClient()
client.moveToPositionAsync(x, y, z, velocity).join()

# ProjectAirSim  
from projectairsim import ProjectAirSimClient, Drone
client = ProjectAirSimClient()
# ... setup world, drone ...
await drone.move_to_position_async(north, east, down, velocity)
```

---

## 🎯 Integration Strategy Matrix

| Feature | Adopt from ProjectAirSim? | Reasoning |
|---------|---------------------------|-----------|
| **Battery Sensor** | ✅ YES | Core feature, well-documented, high value |
| **Airspeed Sensor** | ✅ YES | Essential for fixed-wing, straightforward |
| **Radar Sensor** | ⚠️ VERIFY FIRST | Not in ProjectAirSim docs, may not exist |
| **Clock Control** | ✅ YES | Critical for ML training, testing |
| **Voxel Grid** | ✅ YES | Useful for path planning |
| **Lighting Control** | ✅ YES | Already partially in Cosys, enhance |
| **JSONC Config** | ❌ NO | Cosys has settings.json, keep backward compat |
| **Object API Style** | ❌ NO | Cosys uses flat API, breaking change |
| **Pub/Sub Model** | ⚠️ OPTIONAL | Can wrap internally, keep Req/Res API |

---

## 🔧 Recommended Hybrid Approach

### Keep from Cosys-AirSim:
1. ✅ settings.json configuration format (backward compatibility)
2. ✅ Flat client API structure (simpler for users)
3. ✅ Unique sensors (GPU-LiDAR, Echo, UWB, WiFi)
4. ✅ Request/Response communication model
5. ✅ Unreal Engine 5.5 support

### Adopt from ProjectAirSim:
1. ✅ Battery sensor logic and algorithms
2. ✅ Airspeed sensor implementation
3. ✅ Clock control APIs (for ML/testing)
4. ✅ Voxel grid generation
5. ✅ Enhanced lighting control
6. ✅ Manual actuator control concept

### Enhance Beyond Both:
1. 🚀 Richer battery data (voltage, current, temperature) + ProjectAirSim's time-to-empty
2. 🚀 Hybrid config support (settings.json + optional JSONC for advanced users)
3. 🚀 Optional Pub/Sub for high-frequency sensors (LiDAR, Camera) while keeping Req/Res
4. 🚀 Both sync and async Python API styles

---

## 📊 Implementation Priority Matrix

```
High Value, High Confidence → Implement First
├─ Battery Sensor (Core feature, verified)
├─ Clock Control (ML/testing critical)
└─ Airspeed Sensor (Fixed-wing essential)

Medium Value, High Confidence → Implement Second  
├─ Voxel Grid (Path planning)
├─ Lighting Control (Enhance existing)
└─ Manual Actuator Control (Advanced users)

Unknown Value/Confidence → Research First
└─ Radar Sensor (Not verified in ProjectAirSim)
```

---

## ✅ Summary

| Category | Cosys-AirSim Strengths | ProjectAirSim Strengths |
|----------|------------------------|-------------------------|
| **Sensors** | GPU-LiDAR, Echo, UWB, WiFi, Instance Segmentation | Battery, Airspeed, Cleaner battery model |
| **API** | Simple flat API, easier for beginners | Modern OOP, better for large projects |
| **Config** | Single file, backward compatible | Modular, supports comments |
| **Async** | Works but awkward `.join()` pattern | Native async/await |
| **Documentation** | Academic focus | Professional polish |
| **Community** | Academic/Research | Industry/Enterprise |

**Best Integration Approach**: 
- Keep Cosys-AirSim's architecture and simplicity
- Selectively integrate ProjectAirSim's proven features
- Enhance both where opportunities exist
- Maintain backward compatibility
