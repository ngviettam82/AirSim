# Implementation Plan

## Phased Approach for ProjectAirSim Integration

---

## Phase Overview

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        IMPLEMENTATION TIMELINE                            │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│  Week 1-2: PHASE 1 - SENSOR INTEGRATION                                  │
│  ═══════════════════════════════════════                                  │
│  ├─ Battery Sensor Implementation                                        │
│  ├─ Airspeed Sensor Implementation                                       │
│  └─ Radar Sensor Implementation                                          │
│                                                                           │
│  Week 3-4: PHASE 2 - API ENHANCEMENTS                                    │
│  ═══════════════════════════════════════                                  │
│  ├─ World/Scene Control APIs                                             │
│  ├─ Sim Clock Control APIs                                               │
│  └─ Voxel Grid API                                                       │
│                                                                           │
│  Week 5: PHASE 3 - CONFIGURATION & DOCUMENTATION                        │
│  ═══════════════════════════════════════════════                          │
│  ├─ Settings Schema Updates                                              │
│  ├─ Documentation                                                        │
│  └─ Examples                                                             │
│                                                                           │
│  Week 6: PHASE 4 - TESTING & STABILIZATION                              │
│  ═════════════════════════════════════════                                │
│  ├─ Unit Tests                                                           │
│  ├─ Integration Tests                                                    │
│  └─ Performance Testing                                                  │
│                                                                           │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Sensor Integration (Week 1-2)

### 1.1 Battery Sensor (Days 1-4)

#### Day 1: Core Implementation

**Files to Create:**
```
AirLib/include/sensors/battery/
├── BatterySensorBase.hpp
├── BatterySensorSimple.hpp
└── BatterySensorParams.hpp
```

**Tasks:**
1. Create `BatterySensorBase.hpp` abstract interface
2. Create `BatterySensorSimple.hpp` with two ProjectAirSim discharge modes:
   - simple-discharge-mode: Linear rate-based discharge (abstract units)
   - rotor-power-discharge-mode: Physics-based energy consumption from rotor power (Joules)
3. Create `BatterySensorParams.hpp` for configuration with capacity in Joules

#### Day 2: Settings Integration

**Files to Modify:**
```
AirLib/include/common/AirSimSettings.hpp
AirLib/include/sensors/SensorBase.hpp
AirLib/include/sensors/SensorFactory.hpp
```

**Tasks:**
1. Add `SensorType::Battery = 12` to enum
2. Create `BatterySetting` struct in AirSimSettings.hpp
3. Add battery sensor creation to `SensorFactory`
4. Parse battery settings from JSON (settings.json format)
5. Support all battery configuration parameters

#### Day 3: API Integration

**Files to Modify:**
```
AirLib/include/api/VehicleApiBase.hpp
AirLib/include/api/RpcLibAdaptorsBase.hpp
AirLib/src/api/RpcLibServerBase.cpp
```

**Tasks:**
1. Add `getBatteryData()` to `VehicleApiBase`
2. Add `setBatteryRemaining()`, `setBatteryDrainRate()`, `setBatteryHealthStatus()`
3. Create `BatteryData` RPC adaptor with msgpack
4. Add RPC bindings for all battery methods

#### Day 4: Python Client

**Files to Modify:**
```
PythonClient/cosysairsim/types.py
PythonClient/cosysairsim/client.py
```

**Tasks:**
1. Create `BatteryData` class in `types.py`
2. Add `getBatteryData()` method to client
3. Add `setBatteryRemaining()`, `setBatteryDrainRate()`, `setBatteryHealthStatus()`
4. Write unit tests

---

### 1.2 Airspeed Sensor (Days 5-6)

#### Day 5: Core Implementation

**Files to Create:**
```
AirLib/include/sensors/airspeed/
├── AirspeedSensorBase.hpp
├── AirspeedSensorSimple.hpp
└── AirspeedSensorParams.hpp
```

**Tasks:**
1. Create abstract interface with IAS, TAS, differential pressure
2. Implement simple pitot tube model
3. Add noise model

#### Day 6: Integration

**Files to Modify:**
```
Same as Battery sensor integration
```

**Tasks:**
1. Add `SensorType::Airspeed = 13`
2. Create `AirspeedSetting` struct
3. Add RPC bindings and Python client methods

---

### 1.3 Radar Sensor (Days 7-10)

#### Day 7-8: Core Implementation

**Files to Create:**
```
AirLib/include/sensors/radar/
├── RadarSensorBase.hpp
├── RadarSensorSimple.hpp
└── RadarSensorParams.hpp
```

**Tasks:**
1. Create radar detection data structures
2. Implement ray-casting based detection
3. Add RCS (radar cross section) model
4. Implement doppler velocity estimation
5. Add noise and false detection model

#### Day 9-10: Integration

**Files to Modify:**
```
Same as Battery sensor integration
```

**Tasks:**
1. Add `SensorType::Radar = 14`
2. Create `RadarSetting` struct
3. Add Unreal component for ray casting
4. Add RPC bindings and Python client methods

---

## Phase 2: API Enhancements (Week 3-4)

### 2.1 Lighting Control APIs (Days 11-12)

**Files to Modify:**
```
AirLib/include/api/WorldSimApiBase.hpp
Unreal/Plugins/AirSim/Source/WorldSimApi.cpp
AirLib/src/api/RpcLibServerBase.cpp
PythonClient/cosysairsim/client.py
```

**Tasks:**
1. Add `simSetSunlightIntensity(float intensity)`
2. Add `simGetSunlightIntensity()`
3. Add `simSetCloudShadowStrength(float strength)`
4. Add `simGetCloudShadowStrength()`
5. Implement Unreal lighting control
6. Add RPC and Python bindings

---

### 2.2 Sim Clock Control APIs (Days 13-15)

**Files to Modify:**
```
AirLib/include/api/WorldSimApiBase.hpp
AirLib/include/common/ClockBase.hpp
Unreal/Plugins/AirSim/Source/WorldSimApi.cpp
AirLib/src/api/RpcLibServerBase.cpp
PythonClient/cosysairsim/client.py
```

**Tasks:**
1. Add `simGetSimTime()` - Get sim time in nanoseconds
2. Add `simGetSimClockType()` - Get clock type
3. Add `simContinueForTime(duration_ns)` - Step for duration
4. Add `simContinueUntilTime(target_ns)` - Step until time
5. Add `simContinueForSteps(n)` - Step N physics ticks
6. Add `simContinueForSingleStep()` - Single step
7. Integrate with existing pause/resume system
8. Add RPC and Python bindings

---

### 2.3 Voxel Grid API (Days 16-18)

**Files to Modify:**
```
AirLib/include/api/WorldSimApiBase.hpp
Unreal/Plugins/AirSim/Source/WorldSimApi.cpp
AirLib/src/api/RpcLibServerBase.cpp
PythonClient/cosysairsim/client.py
```

**Tasks:**
1. Implement `simCreateVoxelGrid(position, size, resolution)`
2. Use Unreal line traces for occupancy detection
3. Support .binvox file output
4. Return 1D occupancy array
5. Add RPC and Python bindings

---

### 2.4 Manual Controller API (Days 19-20)

**Files to Modify:**
```
AirLib/include/vehicles/multirotor/api/MultirotorApiBase.hpp
AirLib/src/api/RpcLibServerBase.cpp
PythonClient/cosysairsim/client.py
```

**Tasks:**
1. Add `setControlSignals(Dict[actuator_id, value])`
2. Implement direct actuator control bypass
3. Add safety limits and validation
4. Add RPC and Python bindings

---

## Phase 3: Configuration & Documentation (Week 5)

### 3.1 Settings Documentation (Days 21-22)

**Files to Create:**
```
docs/sensors/battery.md
docs/sensors/airspeed.md
docs/sensors/radar.md
docs/api/world_api_additions.md
docs/api/clock_control.md
```

**Tasks:**
1. Document all new sensor settings
2. Document all new API methods
3. Provide configuration examples
4. Update settings.json reference

---

### 3.2 Example Scripts (Days 23-24)

**Files to Create:**
```
PythonClient/multirotor/
├── battery_monitoring.py
├── airspeed_display.py
├── radar_visualization.py
├── clock_stepping_example.py
└── voxel_grid_example.py
```

**Tasks:**
1. Create example for each new feature
2. Include visualization where applicable
3. Document usage and expected output

---

### 3.3 API Documentation (Day 25)

**Files to Modify:**
```
docs/apis.md
docs/apis_cpp.md
```

**Tasks:**
1. Add all new methods to API reference
2. Include parameter descriptions
3. Include return type descriptions
4. Include usage examples

---

## Phase 4: Testing & Stabilization (Week 6)

### 4.1 Unit Tests (Days 26-27)

**Files to Create:**
```
AirLibUnitTests/
├── BatteryTest.hpp
├── AirspeedTest.hpp
├── RadarTest.hpp
└── WorldApiTest.hpp
```

**Tasks:**
1. Test sensor output correctness
2. Test settings parsing
3. Test edge cases
4. Test error handling

---

### 4.2 Integration Tests (Days 28-29)

**Files to Create:**
```
PythonClient/tests/
├── test_battery_integration.py
├── test_airspeed_integration.py
├── test_radar_integration.py
├── test_world_api.py
└── test_clock_control.py
```

**Tasks:**
1. End-to-end sensor tests
2. API round-trip tests
3. Multi-vehicle tests
4. Performance benchmarks

---

### 4.3 Performance Testing & Final Stabilization (Day 30)

**Tasks:**
1. Run performance benchmarks
2. Compare before/after metrics
3. Profile CPU/memory usage
4. Fix any regressions
5. Final code review
6. Merge to dev branch

---

## Deliverables Summary

### Code Deliverables

| Component | Files | Lines (Est.) |
|-----------|-------|--------------|
| Battery Sensor | 6 | ~800 |
| Airspeed Sensor | 6 | ~400 |
| Radar Sensor | 6 | ~600 |
| World APIs | 4 | ~300 |
| Clock APIs | 4 | ~200 |
| Voxel Grid | 3 | ~250 |
| Python Client | 2 | ~200 |
| Tests | 10 | ~500 |
| **Total** | **41** | **~3,250** |

### Documentation Deliverables

| Document | Purpose |
|----------|---------|
| battery.md | Battery sensor guide |
| airspeed.md | Airspeed sensor guide |
| radar.md | Radar sensor guide |
| world_api_additions.md | New world APIs |
| clock_control.md | Sim clock control guide |
| 5 example scripts | Usage examples |

---

## Risk Mitigation

### Technical Risks

| Risk | Probability | Mitigation |
|------|-------------|------------|
| Radar ray-casting performance | Medium | Use spatial acceleration, limit detections |
| Clock control stability | Low | Extensive testing, graceful degradation |
| Settings parsing errors | Low | Schema validation, clear error messages |

### Schedule Risks

| Risk | Probability | Mitigation |
|------|-------------|------------|
| Radar more complex than estimated | Medium | Allocate buffer days, simplify if needed |
| Integration conflicts | Low | Feature flags, isolated branches |
| Testing reveals bugs | Medium | Allocate stabilization week |

---

## Success Metrics

### Functional Metrics
- [ ] All 3 new sensors return valid data
- [ ] All new APIs function correctly
- [ ] All existing tests still pass
- [ ] No build errors on supported platforms

### Performance Metrics
- [ ] < 1ms overhead per sensor update
- [ ] < 5% total frame time increase
- [ ] No memory leaks detected

### Quality Metrics
- [ ] 100% new code documented
- [ ] > 80% test coverage for new code
- [ ] All examples run successfully
