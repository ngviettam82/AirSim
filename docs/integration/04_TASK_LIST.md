# ProjectAirSim Integration - Task List

## Master Task Tracking

This document contains all tasks required to integrate ProjectAirSim features into Cosys-AirSim.

---

## Task Status Legend

| Symbol | Meaning |
|--------|---------|
| ⬜ | Not Started |
| 🔄 | In Progress |
| ✅ | Completed |
| ⏸️ | Blocked |
| ❌ | Cancelled |

---

## Phase 1: Sensor Integration

### 1.1 Battery Sensor

#### Core Implementation
- [ ] **TASK-001**: Create `AirLib/include/sensors/battery/` directory
- [ ] **TASK-002**: Create `BatterySensorBase.hpp` with abstract interface
  - [ ] Define `BatteryOutput` struct with fields:
    - `float voltage`
    - `float current_draw`
    - `float remaining_percent`
    - `float remaining_wh`
    - `float temperature`
    - `std::string health_status`
    - `TTimePoint time_stamp`
  - [ ] Define pure virtual methods: `getOutput()`, `update()`, `reset()`
  
- [ ] **TASK-003**: Create `BatterySensorParams.hpp`
  - [ ] Define discharge modes enum: `Linear`, `VoltageBased`, `PhysicsBased`
  - [ ] Define parameters:
    - `float max_capacity_wh = 100.0f`
    - `float nominal_voltage = 22.2f`
    - `float internal_resistance = 0.1f`
    - `float temperature_coefficient = 0.002f`
    - `DischargeMode mode = Linear`
    - `float update_frequency = 10.0f`
    
- [ ] **TASK-004**: Create `BatterySensorSimple.hpp`
  - [ ] Implement `Linear` discharge mode
  - [ ] Implement `VoltageBased` discharge mode
  - [ ] Implement `PhysicsBased` discharge mode with:
    - Power-based capacity consumption
    - Temperature-dependent capacity scaling
    - Internal resistance losses
  - [ ] Implement health status tracking

#### Settings Integration
- [ ] **TASK-005**: Modify `SensorBase.hpp`
  - [ ] Add `Battery = 12` to `SensorBase::SensorType` enum

- [ ] **TASK-006**: Modify `AirSimSettings.hpp`
  - [ ] Create `BatterySetting` struct
  - [ ] Add battery to `Sensors` struct
  - [ ] Add `loadBatterySetting()` method
  - [ ] Integrate into `loadSensorSettings()`

- [ ] **TASK-007**: Modify `SensorFactory.hpp`
  - [ ] Add `createBatterySensor()` method
  - [ ] Add case to `createSensor()` switch

#### API Integration
- [ ] **TASK-008**: Modify `VehicleApiBase.hpp`
  - [ ] Add `getBatteryData()` method
  - [ ] Add `setBatteryRemaining(float percent)` method
  - [ ] Add `setBatteryDrainRate(float rate)` method
  - [ ] Add `setBatteryHealthStatus(std::string status)` method

- [ ] **TASK-009**: Modify `RpcLibAdaptorsBase.hpp`
  - [ ] Create `BatteryData` adaptor struct
  - [ ] Add msgpack DEFINE_MAP for serialization

- [ ] **TASK-010**: Modify `RpcLibServerBase.cpp`
  - [ ] Add `getBatteryData` RPC binding
  - [ ] Add `setBatteryRemaining` RPC binding
  - [ ] Add `setBatteryDrainRate` RPC binding
  - [ ] Add `setBatteryHealthStatus` RPC binding

#### Python Client
- [ ] **TASK-011**: Modify `PythonClient/cosysairsim/types.py`
  - [ ] Create `BatteryData` class with all fields

- [ ] **TASK-012**: Modify `PythonClient/cosysairsim/client.py`
  - [ ] Add `getBatteryData(vehicle_name)` method
  - [ ] Add `setBatteryRemaining(percent, vehicle_name)` method
  - [ ] Add `setBatteryDrainRate(rate, vehicle_name)` method
  - [ ] Add `setBatteryHealthStatus(status, vehicle_name)` method

#### Testing
- [ ] **TASK-013**: Create battery sensor unit test
- [ ] **TASK-014**: Create battery sensor integration test
- [ ] **TASK-015**: Create battery monitoring example script

---

### 1.2 Airspeed Sensor

#### Core Implementation
- [ ] **TASK-016**: Create `AirLib/include/sensors/airspeed/` directory
- [ ] **TASK-017**: Create `AirspeedSensorBase.hpp`
  - [ ] Define `AirspeedOutput` struct:
    - `float indicated_airspeed` (m/s)
    - `float true_airspeed` (m/s)
    - `float differential_pressure` (Pa)
    - `float air_temperature` (K)
    - `TTimePoint time_stamp`
  - [ ] Define abstract interface

- [ ] **TASK-018**: Create `AirspeedSensorParams.hpp`
  - [ ] Define parameters:
    - `float pitot_tube_diameter = 0.005f`
    - `float noise_stddev = 0.1f`
    - `float update_frequency = 50.0f`

- [ ] **TASK-019**: Create `AirspeedSensorSimple.hpp`
  - [ ] Implement pitot tube model:
    - `differential_pressure = 0.5 * air_density * v^2`
    - `IAS = sqrt(2 * differential_pressure / sea_level_density)`
    - `TAS = IAS * sqrt(sea_level_density / current_density)`
  - [ ] Add noise model
  - [ ] Get velocity from kinematics

#### Integration
- [ ] **TASK-020**: Add `SensorType::Airspeed = 13` to enum
- [ ] **TASK-021**: Create `AirspeedSetting` struct in `AirSimSettings.hpp`
- [ ] **TASK-022**: Add `createAirspeedSensor()` to `SensorFactory`
- [ ] **TASK-023**: Add `getAirspeedData()` to `VehicleApiBase`
- [ ] **TASK-024**: Create `AirspeedData` RPC adaptor
- [ ] **TASK-025**: Add RPC bindings for airspeed
- [ ] **TASK-026**: Add `AirspeedData` Python class
- [ ] **TASK-027**: Add `getAirspeedData()` Python method

#### Testing
- [ ] **TASK-028**: Create airspeed sensor unit test
- [ ] **TASK-029**: Create airspeed display example script

---

### 1.3 Radar Sensor

#### Core Implementation
- [ ] **TASK-030**: Create `AirLib/include/sensors/radar/` directory
- [ ] **TASK-031**: Create `RadarSensorBase.hpp`
  - [ ] Define `RadarDetection` struct:
    - `float range` (m)
    - `float azimuth` (rad)
    - `float elevation` (rad)
    - `float radial_velocity` (m/s)
    - `float rcs` (m^2)
    - `float signal_strength` (dB)
  - [ ] Define `RadarOutput` struct:
    - `std::vector<RadarDetection> detections`
    - `TTimePoint time_stamp`

- [ ] **TASK-032**: Create `RadarSensorParams.hpp`
  - [ ] Define parameters:
    - `float range_max = 200.0f`
    - `float azimuth_fov = 60.0f` (degrees)
    - `float elevation_fov = 30.0f` (degrees)
    - `int azimuth_resolution = 64`
    - `int elevation_resolution = 32`
    - `float noise_floor_db = -80.0f`
    - `float update_frequency = 20.0f`

- [ ] **TASK-033**: Create `RadarSensorSimple.hpp`
  - [ ] Implement ray-casting based detection
  - [ ] Implement RCS model for different object types
  - [ ] Implement doppler velocity estimation
  - [ ] Implement noise and clutter model
  - [ ] Implement multi-path/ghost detection model

#### Unreal Integration
- [ ] **TASK-034**: Create `RadarSensor` Unreal component
- [ ] **TASK-035**: Implement efficient ray-casting with spatial queries
- [ ] **TASK-036**: Optimize for performance (batch traces, LOD)

#### API Integration
- [ ] **TASK-037**: Add `SensorType::Radar = 14` to enum
- [ ] **TASK-038**: Create `RadarSetting` struct
- [ ] **TASK-039**: Add radar to `SensorFactory`
- [ ] **TASK-040**: Add `getRadarData()` to `VehicleApiBase`
- [ ] **TASK-041**: Create `RadarData` RPC adaptor
- [ ] **TASK-042**: Add RPC bindings
- [ ] **TASK-043**: Add `RadarData` and `RadarDetection` Python classes
- [ ] **TASK-044**: Add `getRadarData()` Python method

#### Testing
- [ ] **TASK-045**: Create radar sensor unit test
- [ ] **TASK-046**: Create radar visualization example

---

## Phase 2: API Enhancements

### 2.1 World Lighting Control

- [ ] **TASK-047**: Add `simSetSunlightIntensity(intensity)` to WorldSimApiBase
- [ ] **TASK-048**: Implement in Unreal WorldSimApi
- [ ] **TASK-049**: Add `simGetSunlightIntensity()` getter
- [ ] **TASK-050**: Add `simSetCloudShadowStrength(strength)`
- [ ] **TASK-051**: Add `simGetCloudShadowStrength()`
- [ ] **TASK-052**: Add RPC bindings for lighting
- [ ] **TASK-053**: Add Python client methods for lighting

---

### 2.2 Sim Clock Control

- [ ] **TASK-054**: Review existing clock/pause implementation
- [ ] **TASK-055**: Add `simGetSimTime()` - returns nanoseconds
- [ ] **TASK-056**: Add `simGetSimClockType()` - returns clock mode
- [ ] **TASK-057**: Add `simContinueForTime(duration_ns)` - step simulation
- [ ] **TASK-058**: Add `simContinueUntilTime(target_ns)` - step to time
- [ ] **TASK-059**: Add `simContinueForSteps(n)` - step N ticks
- [ ] **TASK-060**: Add `simContinueForSingleStep()` - single tick
- [ ] **TASK-061**: Integrate with existing `simPause()` / `simContinue()`
- [ ] **TASK-062**: Add RPC bindings for clock control
- [ ] **TASK-063**: Add Python client methods for clock

---

### 2.3 Voxel Grid API

- [ ] **TASK-064**: Design voxel grid data structure
- [ ] **TASK-065**: Implement `simCreateVoxelGrid(position, x, y, z, resolution)`
- [ ] **TASK-066**: Implement occupancy detection via line traces
- [ ] **TASK-067**: Return occupancy as 1D array (z-major, y-minor, x-least)
- [ ] **TASK-068**: Support .binvox file format export
- [ ] **TASK-069**: Add RPC binding for voxel grid
- [ ] **TASK-070**: Add Python client `simCreateVoxelGrid()` method
- [ ] **TASK-071**: Create voxel grid visualization example

---

### 2.4 Manual Actuator Control

- [ ] **TASK-072**: Design actuator control interface
- [ ] **TASK-073**: Add `setControlSignals(Dict[actuator_id, value])` to API
- [ ] **TASK-074**: Implement safety validation (bounds, rate limits)
- [ ] **TASK-075**: Add bypass flag for autonomous controllers
- [ ] **TASK-076**: Add RPC binding
- [ ] **TASK-077**: Add Python client method
- [ ] **TASK-078**: Create manual control example

---

## Phase 3: Configuration & Documentation

### 3.1 Sensor Documentation

- [ ] **TASK-079**: Create `docs/sensors/battery.md`
  - [ ] Configuration options
  - [ ] Python API usage
  - [ ] Example settings.json
  
- [ ] **TASK-080**: Create `docs/sensors/airspeed.md`
- [ ] **TASK-081**: Create `docs/sensors/radar.md`

### 3.2 API Documentation

- [ ] **TASK-082**: Update `docs/apis.md` with new methods
- [ ] **TASK-083**: Update `docs/apis_cpp.md` with C++ API
- [ ] **TASK-084**: Create `docs/api/world_api_additions.md`
- [ ] **TASK-085**: Create `docs/api/clock_control.md`
- [ ] **TASK-086**: Create `docs/api/voxel_grid.md`

### 3.3 Example Scripts

- [ ] **TASK-087**: Create `battery_monitoring.py`
- [ ] **TASK-088**: Create `airspeed_display.py`
- [ ] **TASK-089**: Create `radar_visualization.py`
- [ ] **TASK-090**: Create `clock_stepping_example.py`
- [ ] **TASK-091**: Create `voxel_grid_example.py`
- [ ] **TASK-092**: Create `manual_control_example.py`

---

## Phase 4: Testing & Stabilization

### 4.1 Unit Tests

- [ ] **TASK-093**: Create `AirLibUnitTests/BatteryTest.hpp`
- [ ] **TASK-094**: Create `AirLibUnitTests/AirspeedTest.hpp`
- [ ] **TASK-095**: Create `AirLibUnitTests/RadarTest.hpp`
- [ ] **TASK-096**: Create `AirLibUnitTests/ClockTest.hpp`
- [ ] **TASK-097**: Update `AirLibUnitTests/main.cpp` to include new tests

### 4.2 Integration Tests

- [ ] **TASK-098**: Create `PythonClient/tests/test_battery_integration.py`
- [ ] **TASK-099**: Create `PythonClient/tests/test_airspeed_integration.py`
- [ ] **TASK-100**: Create `PythonClient/tests/test_radar_integration.py`
- [ ] **TASK-101**: Create `PythonClient/tests/test_clock_control.py`
- [ ] **TASK-102**: Create `PythonClient/tests/test_voxel_grid.py`

### 4.3 Performance Testing

- [ ] **TASK-103**: Create performance benchmark script
- [ ] **TASK-104**: Measure sensor update overhead
- [ ] **TASK-105**: Measure radar ray-casting performance
- [ ] **TASK-106**: Profile memory usage
- [ ] **TASK-107**: Compare before/after metrics

### 4.4 Final Stabilization

- [ ] **TASK-108**: Run all unit tests
- [ ] **TASK-109**: Run all integration tests
- [ ] **TASK-110**: Fix any failing tests
- [ ] **TASK-111**: Code review
- [ ] **TASK-112**: Final documentation review
- [ ] **TASK-113**: Create release notes
- [ ] **TASK-114**: Merge to dev branch

---

## Task Dependencies

```
TASK-001 → TASK-002 → TASK-003 → TASK-004  (Battery core)
                                    ↓
TASK-005 → TASK-006 → TASK-007            (Battery settings)
                        ↓
TASK-008 → TASK-009 → TASK-010            (Battery API)
                        ↓
TASK-011 → TASK-012                        (Battery Python)
              ↓
TASK-013 → TASK-014 → TASK-015            (Battery testing)

Similar chains for Airspeed and Radar...

Phase 2 tasks can be done in parallel after Phase 1 core is complete.
Phase 3 can begin once individual features are complete.
Phase 4 must wait for all implementation tasks.
```

---

## Priority Matrix

| Priority | Task Range | Description |
|----------|------------|-------------|
| **P0 - Critical** | TASK-001 to TASK-015 | Battery sensor (most requested) |
| **P1 - High** | TASK-054 to TASK-063 | Clock control (enables testing) |
| **P2 - Medium** | TASK-016 to TASK-029 | Airspeed sensor |
| **P3 - Medium** | TASK-030 to TASK-046 | Radar sensor |
| **P4 - Low** | TASK-064 to TASK-071 | Voxel grid |
| **P5 - Low** | TASK-047 to TASK-053 | Lighting control |

---

## Estimated Effort

| Task Range | Description | Story Points | Days |
|------------|-------------|--------------|------|
| TASK-001 to TASK-015 | Battery Sensor | 13 | 4 |
| TASK-016 to TASK-029 | Airspeed Sensor | 8 | 2 |
| TASK-030 to TASK-046 | Radar Sensor | 21 | 4 |
| TASK-047 to TASK-053 | Lighting Control | 5 | 2 |
| TASK-054 to TASK-063 | Clock Control | 8 | 3 |
| TASK-064 to TASK-071 | Voxel Grid | 13 | 3 |
| TASK-072 to TASK-078 | Manual Control | 8 | 2 |
| TASK-079 to TASK-092 | Documentation | 8 | 5 |
| TASK-093 to TASK-114 | Testing | 13 | 5 |
| **Total** | | **97** | **30 days** |
