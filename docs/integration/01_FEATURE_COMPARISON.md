# Feature Comparison: Cosys-AirSim vs ProjectAirSim

## Detailed Gap Analysis

This document provides a comprehensive comparison of features between Cosys-AirSim and ProjectAirSim, identifying features to integrate.

---

## 1. Sensor Comparison

### Sensors Available in Both

| Sensor | Cosys-AirSim | ProjectAirSim | Notes |
|--------|--------------|---------------|-------|
| IMU | ✅ | ✅ | Similar implementation |
| Barometer | ✅ | ✅ | Similar implementation |
| Magnetometer | ✅ | ✅ | Similar implementation |
| GPS | ✅ | ✅ | Similar implementation |
| Camera | ✅ | ✅ | Cosys-AirSim has more image types |
| LiDAR | ✅ | ✅ | Cosys-AirSim has GPU acceleration |

### Sensors ONLY in Cosys-AirSim (Keep & Preserve)

| Sensor | Description | Priority |
|--------|-------------|----------|
| **GPU LiDAR** | Hardware-accelerated LiDAR with massive point clouds | CRITICAL - Keep |
| **Echo/Sonar** | Ultrasonic echo sensor simulation | HIGH - Keep |
| **UWB** | Ultra-wideband positioning sensors | HIGH - Keep |
| **WiFi** | WiFi signal strength sensors | MEDIUM - Keep |
| **Distance** | Distance/range sensors | MEDIUM - Keep |

### Sensors ONLY in ProjectAirSim (Integrate)

| Sensor | Description | Priority | Integration Effort | Verification Status |
|--------|-------------|----------|-------------------|---------------------|
| **Battery** | Battery state simulation with 2 discharge modes | **HIGH** | Medium | ✅ Verified in API |
| **Airspeed** | Airspeed measurement sensor | **HIGH** | Low | ✅ Verified in API |
| **Radar** | Radar sensor for object detection | **MEDIUM** | Medium | ⚠️ Not found in ProjectAirSim API |

**Note**: Radar sensor could not be verified in ProjectAirSim API documentation. May be Cosys-AirSim original feature or require source code verification.

---

## 2. API Comparison

### Python Client API

#### Methods in Both (Similar)

| Category | Cosys-AirSim | ProjectAirSim |
|----------|--------------|---------------|
| Connection | `confirmConnection()` | `client.connect()` |
| Takeoff | `takeoffAsync()` | `drone.takeoff_async()` |
| Land | `landAsync()` | `drone.land_async()` |
| Move | `moveByVelocityAsync()` | `drone.move_by_velocity_async()` |
| IMU | `getImuData()` | `drone.get_imu_data()` |
| GPS | `getGpsData()` | `drone.get_gps_data()` |
| Camera | `simGetImages()` | `drone.get_images()` |

#### Methods ONLY in Cosys-AirSim (Keep)

| Method | Description |
|--------|-------------|
| `getGPULidarData()` | GPU-accelerated LiDAR data |
| `getEchoData()` | Echo/sonar sensor data |
| `setEchoData()` | Configure echo sensor |
| `getUWBData()` | UWB sensor data |
| `getUWBSensorData()` | UWB sensor configuration |
| `getWifiData()` | WiFi sensor data |
| `getWifiSensorData()` | WiFi sensor configuration |
| `simGetInstanceSegmentation()` | Instance segmentation |
| `simGetAnnotation()` | Multi-layer annotation |

#### Methods ONLY in ProjectAirSim (Integrate)

| Method | Description | Priority |
|--------|-------------|----------|
| `get_battery_state()` | Get battery charge level | **HIGH** |
| `set_battery_remaining()` | Set battery charge | **HIGH** |
| `get_battery_drain_rate()` | Get discharge rate | **HIGH** |
| `set_battery_drain_rate()` | Set discharge rate | **HIGH** |
| `set_battery_health_status()` | Set battery health | **HIGH** |
| `get_airspeed_data()` | Get airspeed | **HIGH** |
| `set_control_signals()` | Manual actuator control | MEDIUM |
| `create_voxel_grid()` | Voxel mapping | MEDIUM |
| `set_authorization_token()` | Client auth | LOW |

---

## 3. World/Scene API Comparison

### Cosys-AirSim World APIs

```python
# Existing APIs
simSetTimeOfDay()
simEnableWeather()
simSetWeatherParameter()
simSetWind()
simSpawnObject()
simDestroyObject()
simListSceneObjects()
simGetObjectPose()
simSetObjectPose()
simGetSegmentationObjectID()
simSetSegmentationObjectID()
simSwapTextures()
```

### ProjectAirSim World APIs (NEW - To Integrate)

```python
# New APIs to add
world.set_sunlight_intensity(intensity)      # HIGH
world.get_sunlight_intensity()               # HIGH
world.set_cloud_shadow_strength(strength)    # MEDIUM
world.get_cloud_shadow_strength()            # MEDIUM
world.create_voxel_grid(...)                 # MEDIUM
world.get_sim_time()                         # HIGH
world.get_sim_clock_type()                   # MEDIUM
world.continue_for_sim_time(delta)           # HIGH
world.continue_until_sim_time(target)        # HIGH
world.continue_for_n_steps(n)                # HIGH
world.continue_for_single_step()             # HIGH
world.plot_debug_transforms_with_names()     # LOW
```

---

## 4. Configuration Comparison

### Settings Format

| Aspect | Cosys-AirSim | ProjectAirSim |
|--------|--------------|---------------|
| Format | JSON | JSONC (with comments) |
| Location | `~/Documents/AirSim/settings.json` | `sim_config/*.jsonc` |
| Sensor Config | Inline with vehicle | Separate robot config |

### Sensor Type Mappings

| Cosys-AirSim SensorType | ProjectAirSim sensor type |
|------------------------|--------------------------|
| 0 | camera |
| 1 | barometer |
| 2 | imu |
| 3 | gps |
| 4 | magnetometer |
| 5 | distance (not in ProjectAirSim) |
| 6 | lidar |
| 7 | echo (not in ProjectAirSim) |
| 8 | gpulidar (not in ProjectAirSim) |
| 9 | marlocuwb (not in ProjectAirSim) |
| 10 | template |
| 11 | wifi (not in ProjectAirSim) |
| N/A | battery (ProjectAirSim only) |
| N/A | airspeed (ProjectAirSim only) |
| N/A | radar (ProjectAirSim only) |

---

## 5. Physics Comparison

| Feature | Cosys-AirSim | ProjectAirSim |
|---------|--------------|---------------|
| FastPhysicsEngine | ✅ | ✅ (fast-physics) |
| ExternalPhysicsEngine | ✅ | ✅ (non-physics) |
| UnrealPhysics | Partial | ✅ (unreal-physics) |
| Aerodynamic drag | ✅ | ✅ Enhanced |
| Collision response | ✅ | ✅ Enhanced |

---

## 6. Controller Comparison

| Controller | Cosys-AirSim | ProjectAirSim |
|------------|--------------|---------------|
| SimpleFlightApi | ✅ | ✅ |
| PX4 SITL | ✅ | ✅ |
| PX4 HITL | ✅ | ✅ |
| ManualController | ❌ | ✅ **NEW** |
| ArduPilot | ✅ | ❌ |

---

## 7. Architecture Comparison

### Cosys-AirSim Architecture
```
AirLib (C++ Core)
    └─> Sensors, Vehicles, Physics
    └─> RPC Server (rpclib)
    └─> Python Client (msgpack)
    
Unreal Plugin
    └─> SimMode, Pawns
    └─> Camera capture
    └─> Collision handling
```

### ProjectAirSim Architecture
```
Layer 1: SimLibs (core_sim)
    └─> Robot structure
    └─> Tick loop

Layer 2: Plugin (Unreal)
    └─> Controller connection
    └─> Physics integration
    └─> Rendering

Layer 3: Client Library
    └─> nng (nanomsg) transport
    └─> Pub/Sub model
    └─> asyncio Python
```

---

## 8. Integration Priority Matrix

### HIGH Priority (Must Have)

| Feature | Effort | Value | Priority Score |
|---------|--------|-------|----------------|
| Battery Sensor | Medium | High | **9/10** |
| Airspeed Sensor | Low | High | **9/10** |
| Sim Clock Control APIs | Low | High | **8/10** |
| Lighting Control APIs | Low | Medium | **7/10** |

### MEDIUM Priority (Should Have)

| Feature | Effort | Value | Priority Score |
|---------|--------|-------|----------------|
| Radar Sensor | Medium | Medium | **6/10** |
| Voxel Grid API | Medium | Medium | **6/10** |
| Manual Controller | Medium | Low | **5/10** |
| Enhanced Weather | Low | Low | **5/10** |

### LOW Priority (Nice to Have)

| Feature | Effort | Value | Priority Score |
|---------|--------|-------|----------------|
| Client Authorization | Medium | Low | **3/10** |
| JSONC Config | High | Low | **3/10** |
| asyncio Client | High | Low | **3/10** |

---

## 9. Architectural Differences

### API Design Philosophy

| Aspect | ProjectAirSim | Cosys-AirSim | Decision |
|--------|---------------|--------------|----------|
| **API Style** | Object-oriented (Client/World/Drone) | Flat client API | Keep Cosys-AirSim style |
| **Method Names** | snake_case (`get_battery_state`) | camelCase (`getBatteryData`) | Keep camelCase |
| **Configuration** | JSONC modular (scene + robot) | JSON monolithic (settings.json) | Keep settings.json |
| **Data Model** | Pub/Sub streaming | Request/Response | Keep Request/Response |
| **Vehicle Access** | Drone object methods | Client methods with vehicle_name | Keep flat API |

### Integration Approach

**Maintain Cosys-AirSim Architecture**:
- ✅ Backward compatible with existing code
- ✅ Consistent with 11 existing sensor APIs
- ✅ Simpler for beginners (single client object)
- ✅ No breaking changes to settings.json format

**Adopt ProjectAirSim Features**:
- ✅ New sensor types (Battery, Airspeed)
- ✅ Clock control capabilities
- ✅ Enhanced simulation control
- ✅ Modern sensor physics models

---

## 10. Feature Decision Summary

### ✅ INTEGRATE from ProjectAirSim

1. **Battery Sensor** - Full implementation (2 discharge modes: simple, rotor-power)
2. **Airspeed Sensor** - Full implementation
3. **Radar Sensor** - Pending verification (not found in ProjectAirSim API docs)
4. **Sim Clock Control** - pause, resume, step APIs with wait_until_complete parameter
5. **Lighting APIs** - sun intensity, cloud shadows
6. **Voxel Grid** - create_voxel_grid()
7. **Manual Controller** - set_control_signals()

### ❌ DO NOT INTEGRATE (Keep Cosys-AirSim)

1. **GPU LiDAR** - Cosys-AirSim version is superior
2. **Echo Sensor** - Unique to Cosys-AirSim
3. **UWB Sensor** - Unique to Cosys-AirSim
4. **WiFi Sensor** - Unique to Cosys-AirSim
5. **Instance Segmentation** - More mature in Cosys-AirSim
6. **Transport Layer** - RPC works well, no need for nng
7. **asyncio Client** - Breaking change for existing code

### ⚠️ CONSIDER LATER

1. **JSONC Config** - Evaluate user demand
2. **Client Auth** - Only if security is needed
3. **Pub/Sub Model** - Major architecture change
