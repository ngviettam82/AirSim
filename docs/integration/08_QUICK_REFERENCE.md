# ProjectAirSim Integration - Quick Reference

## Document Index

| # | Document | Purpose |
|---|----------|---------|
| 00 | [OVERVIEW.md](00_OVERVIEW.md) | Project summary, objectives, constraints |
| 01 | [FEATURE_COMPARISON.md](01_FEATURE_COMPARISON.md) | Gap analysis, feature matrix |
| 02 | [ARCHITECTURE_DESIGN.md](02_ARCHITECTURE_DESIGN.md) | System design, diagrams |
| 03 | [IMPLEMENTATION_PLAN.md](03_IMPLEMENTATION_PLAN.md) | Phased timeline, milestones |
| 04 | [TASK_LIST.md](04_TASK_LIST.md) | Detailed task tracking |
| 05 | [FLOW_DIAGRAMS.md](05_FLOW_DIAGRAMS.md) | Visual implementation guides |
| 06 | [API_SPECIFICATION.md](06_API_SPECIFICATION.md) | Complete API reference |
| 07 | [TESTING_STRATEGY.md](07_TESTING_STRATEGY.md) | Test plans and examples |

---

## Quick Start

### What We're Building

Adding these ProjectAirSim features to Cosys-AirSim:

| Feature | Priority | Effort | Status |
|---------|----------|--------|--------|
| Battery Sensor | P0 | 4 days | 🔵 Planned |
| Airspeed Sensor | P2 | 2 days | 🔵 Planned |
| Radar Sensor | P3 | 4 days | 🔵 Planned |
| Clock Control | P1 | 3 days | 🔵 Planned |
| Voxel Grid | P4 | 3 days | 🔵 Planned |
| Lighting Control | P5 | 2 days | 🔵 Planned |

---

## Feature Summary

### Battery Sensor

**Purpose:** Track battery state, enable battery-aware mission planning

**Key APIs:**
```python
battery = client.getBatteryData()
client.setBatteryRemaining(50.0)
client.setBatteryDrainRate(2.0)
client.setBatteryHealthStatus("Low")
```

**Settings (settings.json)**:
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
          "DischargeMode": "rotor-power-discharge-mode",
          "DrainRate": 1.0
        }
      }
    }
  }
}
```

**Discharge Modes:**
- `simple-discharge-mode`: Linear rate-based
- `rotor-power-discharge-mode`: Physics-based from rotor power

---

### Airspeed Sensor

**Purpose:** Provide airspeed for fixed-wing/VTOL aircraft

**Key APIs:**
```python
airspeed = client.getAirspeedData()
print(f"IAS: {airspeed.indicated_airspeed} m/s")
print(f"TAS: {airspeed.true_airspeed} m/s")
```

**Settings (settings.json)**:
```json
{
  "Vehicles": {
    "Drone1": {
      "Sensors": {
        "Airspeed1": {
          "SensorType": 13,
          "Enabled": true,
          "NoiseStdDev": 0.1,
          "UpdateFrequency": 50.0
        }
      }
    }
  }
}
```

---

### Radar Sensor

**Purpose:** Object detection with range, velocity, and RCS

**Key APIs:**
```python
radar = client.getRadarData()
for detection in radar.detections:
    print(f"Object at {detection.range}m, velocity {detection.radial_velocity}m/s")
```

**Settings (settings.json)**:
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
          "ElevationFOV": 30.0
        }
      }
    }
  }
}
```

---

### Clock Control

**Purpose:** Deterministic simulation stepping for ML/testing

**Key APIs:**
```python
client.simPause(True)                      # Enable stepping mode
client.simContinueForTime(100_000_000)     # Step 100ms
client.simContinueForSteps(10)             # Step 10 physics ticks
client.simContinueForSingleStep()          # Step 1 tick
sim_time = client.simGetSimTime()          # Get time in nanoseconds
```

---

### Voxel Grid

**Purpose:** 3D occupancy mapping for path planning

**Key APIs:**
```python
voxels = client.simCreateVoxelGrid(
    position=airsim.Vector3r(0, 0, -2.5),
    x_size=10.0, y_size=10.0, z_size=5.0,
    resolution=0.5
)
grid = np.frombuffer(voxels, dtype=np.int8).reshape((10, 20, 20))
```

---

## File Locations

### New Sensor Headers

```
AirLib/include/sensors/
├── battery/
│   ├── BatterySensorBase.hpp
│   ├── BatterySensorSimple.hpp
│   └── BatterySensorParams.hpp
├── airspeed/
│   ├── AirspeedSensorBase.hpp
│   ├── AirspeedSensorSimple.hpp
│   └── AirspeedSensorParams.hpp
└── radar/
    ├── RadarSensorBase.hpp
    ├── RadarSensorSimple.hpp
    └── RadarSensorParams.hpp
```

### Files to Modify

```
AirLib/include/sensors/SensorBase.hpp          # Add sensor type enums
AirLib/include/sensors/SensorFactory.hpp       # Add factory methods
AirLib/include/common/AirSimSettings.hpp       # Add settings parsing
AirLib/include/api/VehicleApiBase.hpp          # Add API methods
AirLib/include/api/RpcLibAdaptorsBase.hpp      # Add RPC adaptors
AirLib/src/api/RpcLibServerBase.cpp            # Add RPC bindings
PythonClient/cosysairsim/types.py              # Add Python types
PythonClient/cosysairsim/client.py             # Add Python methods
```

---

## Sensor Type IDs

| Sensor | ID | Existing? |
|--------|-----|-----------|
| Barometer | 1 | ✅ Cosys |
| IMU | 2 | ✅ Cosys |
| GPS | 3 | ✅ Cosys |
| Magnetometer | 4 | ✅ Cosys |
| Distance | 5 | ✅ Cosys |
| LiDAR | 6 | ✅ Cosys |
| GPULiDAR | 7 | ✅ Cosys |
| Echo | 8 | ✅ Cosys |
| UWB | 9 | ✅ Cosys |
| Wifi | 10 | ✅ Cosys |
| Template | 11 | ✅ Cosys |
| **Battery** | **12** | 🆕 New |
| **Airspeed** | **13** | 🆕 New |
| **Radar** | **14** | 🆕 New |

---

## Development Workflow

```
1. Create feature branch
   git checkout -b feature/battery-sensor

2. Implement C++ headers
   - Create sensor files in AirLib/include/sensors/

3. Update settings parsing
   - Modify AirSimSettings.hpp

4. Add to sensor factory
   - Modify SensorFactory.hpp

5. Add API methods
   - Modify VehicleApiBase.hpp

6. Add RPC bindings
   - Modify RpcLibAdaptorsBase.hpp
   - Modify RpcLibServerBase.cpp

7. Add Python client
   - Modify types.py
   - Modify client.py

8. Build and test
   - Run build_airsim.bat
   - Run unit tests
   - Run integration tests

9. Create pull request
   - Merge to dev branch
```

---

## Build Commands

### Windows (VS 2022)
```batch
build_airsim.bat
```

### Linux/WSL
```bash
./build.sh
```

### CMake Direct
```bash
cd cmake
cmake -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

---

## Test Commands

### Unit Tests
```bash
cd AirLibUnitTests/build
./AirLibUnitTests.exe
```

### Python Integration Tests
```bash
cd PythonClient
pytest tests/ -v
```

### Single Feature Test
```bash
pytest tests/test_battery_integration.py -v
```

---

## Common Issues

### Issue: Sensor not found

**Symptom:** `getSensorData returned None`

**Solution:** Verify sensor is configured in settings.json with correct SensorType ID

---

### Issue: RPC serialization error

**Symptom:** `msgpack unpack error`

**Solution:** Ensure MSGPACK_DEFINE_MAP in adaptor matches Python class fields exactly

---

### Issue: Build fails on sensor include

**Symptom:** `No such file or directory: BatterySensorBase.hpp`

**Solution:** Add new sensor directory to CMakeLists.txt include paths

---

## Contacts

| Role | Responsibility |
|------|---------------|
| Project Lead | Architecture decisions, code review |
| C++ Developer | AirLib implementation |
| Python Developer | Client library, examples |
| Unreal Developer | Plugin integration (for Radar) |
| QA | Testing, validation |

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2024-XX-XX | - | Initial documentation |

---

## Next Steps

1. ✅ Documentation complete
2. ⬜ Begin Phase 1: Battery Sensor
3. ⬜ Code review documentation
4. ⬜ Set up CI/CD pipeline
5. ⬜ Create test environments
