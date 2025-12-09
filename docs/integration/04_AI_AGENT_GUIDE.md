# ProjectAirSim Integration - AI Agent Implementation Guide

**Document Version:** 1.0  
**Date:** January 2025  
**Status:** AI-Assisted Development Guide  
**Target Audience:** AI agents, automated code generators, human developers using AI assistance

---

## Document Overview

This guide provides **step-by-step instructions for AI agents** to implement all 25 ProjectAirSim features. It includes prompts, code templates, workflows, and validation steps optimized for AI-assisted development.

**Related Documents:**
- `00_MASTER_TECHNICAL_SPECIFICATION.md` - Technical specifications
- `01_IMPLEMENTATION_ROADMAP.md` - Implementation timeline
- `02_DETAILED_TASK_LIST.md` - Granular task breakdown
- `03_TESTING_STRATEGY.md` - Testing requirements

---

## AI Agent Workflow

### General Implementation Pattern

```
1. READ specifications from master tech spec
2. RESEARCH similar implementations in existing codebase
3. GENERATE code following existing patterns
4. TEST generated code with unit tests
5. INTEGRATE with existing systems
6. VALIDATE against acceptance criteria
7. DOCUMENT changes
```

### Task Execution Loop

```python
for task in tier_tasks:
    # Phase 1: Understanding
    specifications = read_spec(task.feature)
    existing_code = search_codebase(task.related_classes)
    patterns = identify_patterns(existing_code)
    
    # Phase 2: Implementation
    code = generate_code(specifications, patterns)
    tests = generate_tests(specifications, code)
    
    # Phase 3: Validation
    compile_result = compile(code)
    test_result = run_tests(tests)
    
    if compile_result.success and test_result.all_passed:
        integrate(code, tests)
        mark_complete(task)
    else:
        debug(compile_result, test_result)
```

---

## Tier 1: Critical Sensors - AI Implementation Guide

### Feature: Battery Sensor (Week 1)

#### Step 1: Understand the Specification

**AI Prompt:**
```
Read the Battery Sensor specification from section 2.3.3 of the master technical specification.
Extract:
1. Data structure: BatteryData fields
2. Parameters: BatteryParams fields
3. Discharge modes: Linear, Nonlinear
4. Control APIs: setRemaining, setDrainRate, setHealthStatus

Summarize the key requirements.
```

**Expected AI Response:**
- BatteryData contains voltage, current, SOC, capacity, temperature, health status
- Two discharge modes required: Linear (constant rate) and Nonlinear (exponential)
- Must support runtime control of SOC and drain rate
- Update frequency configurable

#### Step 2: Research Existing Patterns

**AI Prompt:**
```
Search the Cosys-AirSim codebase for existing sensor implementations.
Find files matching: AirLib/include/sensors/*/.*Base.hpp

Analyze the pattern:
1. How do sensors inherit from SensorBase?
2. How are sensor parameters structured?
3. How is getData() implemented?
4. How is update() called?

Show me an example from BarometerBase or GpsBase.
```

**Expected AI Response:**
```cpp
// Pattern identified from existing sensors:
class SensorBase {
protected:
    SensorParams params_;
    SensorData data_;
public:
    virtual void update() = 0;
    virtual SensorData getData() const = 0;
};

// Sensors follow this structure:
// 1. Define [Sensor]Data struct
// 2. Define [Sensor]Params struct
// 3. Create [Sensor]Base.hpp with abstract interface
// 4. Create [Sensor]Simple.cpp with implementation
```

#### Step 3: Generate Code

**AI Prompt:**
```
Generate the complete BatteryBase.hpp file following the pattern from BarometerBase.hpp.

Include:
1. BatteryData struct with all required fields
2. BatteryParams struct with discharge modes enum
3. BatteryBase abstract class inheriting from SensorBase
4. Pure virtual methods: update(), getData()
5. Control methods: setRemaining(), setDrainRate(), setHealthStatus()
6. Proper includes and namespace
7. Doxygen comments for all public methods
```

**AI-Generated Code Template:**
```cpp
// File: AirLib/include/sensors/battery/BatteryBase.hpp
#ifndef msr_airlib_BatteryBase_hpp
#define msr_airlib_BatteryBase_hpp

#include "sensors/SensorBase.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {

struct BatteryData {
    float voltage = 0.0f;              // Volts
    float current = 0.0f;              // Amperes
    float state_of_charge = 1.0f;      // 0.0-1.0 (0% - 100%)
    float remaining_capacity = 0.0f;   // Ampere-hours
    float total_capacity = 0.0f;       // Ampere-hours
    float temperature = 25.0f;         // Celsius
    bool is_healthy = true;
    TTimePoint time_stamp = 0;
};

struct BatteryParams {
    enum class DischargeMode {
        Linear,      // Constant drain rate
        Nonlinear    // Realistic discharge curve
    };
    
    float nominal_voltage = 12.6f;     // Volts (e.g., 3S LiPo)
    float total_capacity = 5.0f;       // Ampere-hours
    float discharge_rate = 1.0f;       // Amperes
    DischargeMode discharge_mode = DischargeMode::Linear;
    float nonlinear_factor = 2.0f;     // For nonlinear mode
    float update_frequency = 10.0f;    // Hz
};

class BatteryBase : public SensorBase {
public:
    BatteryBase(const std::string& sensor_name = "")
        : SensorBase(sensor_name) {}
    virtual ~BatteryBase() = default;

    /**
     * Update battery state based on elapsed time
     */
    virtual void update() = 0;

    /**
     * Get current battery data
     * @return BatteryData struct with current state
     */
    virtual BatteryData getData() const = 0;

    /**
     * Set remaining battery charge
     * @param percentage State of charge (0.0 to 1.0)
     * @return true if successful
     */
    virtual bool setRemaining(float percentage) = 0;

    /**
     * Set battery drain rate
     * @param rate Discharge rate in Amperes
     * @return true if successful
     */
    virtual bool setDrainRate(float rate) = 0;

    /**
     * Set battery health status
     * @param is_healthy Health flag
     * @return true if successful
     */
    virtual bool setHealthStatus(bool is_healthy) = 0;

protected:
    BatteryData data_;
    BatteryParams params_;
};

}} // namespace
#endif
```

#### Step 4: Generate Implementation

**AI Prompt:**
```
Generate BatterySimple.cpp implementing the BatteryBase interface.

Linear discharge:
- SOC decreases by (discharge_rate * dt) / total_capacity
- Voltage = f(SOC) using typical LiPo discharge curve

Nonlinear discharge:
- Use exponential decay: SOC(t) = SOC_0 * exp(-k * t)
- k = nonlinear_factor / total_capacity

Include:
1. Constructor
2. update() method with both discharge modes
3. getData() method
4. Control methods (setRemaining, setDrainRate, setHealthStatus)
5. Voltage calculation from SOC
```

**AI-Generated Code Template:**
```cpp
// File: AirLib/src/sensors/battery/BatterySimple.cpp
#include "sensors/battery/BatterySimple.hpp"
#include <algorithm>
#include <cmath>

namespace msr { namespace airlib {

BatterySimple::BatterySimple(const BatteryParams& params)
    : BatteryBase("BatterySimple")
{
    params_ = params;
    data_.total_capacity = params_.total_capacity;
    data_.remaining_capacity = params_.total_capacity;
    data_.state_of_charge = 1.0f;
    data_.voltage = params_.nominal_voltage;
    data_.current = 0.0f;
    data_.is_healthy = true;
    last_update_time_ = ClockFactory::get()->nowNanos();
}

void BatterySimple::update()
{
    TTimePoint current_time = ClockFactory::get()->nowNanos();
    float dt = static_cast<float>((current_time - last_update_time_) / 1.0e9);
    last_update_time_ = current_time;

    if (dt <= 0 || !data_.is_healthy) {
        return;
    }

    // Update current based on drain rate
    data_.current = params_.discharge_rate;

    // Calculate SOC change based on discharge mode
    float soc_change = 0.0f;
    
    if (params_.discharge_mode == BatteryParams::DischargeMode::Linear) {
        // Linear: constant drain rate
        soc_change = (params_.discharge_rate * dt) / params_.total_capacity;
    } else {
        // Nonlinear: exponential decay
        float decay_rate = params_.nonlinear_factor / params_.total_capacity;
        float new_soc = data_.state_of_charge * std::exp(-decay_rate * dt);
        soc_change = data_.state_of_charge - new_soc;
    }

    // Update state of charge
    data_.state_of_charge = std::max(0.0f, data_.state_of_charge - soc_change);
    data_.remaining_capacity = data_.state_of_charge * data_.total_capacity;

    // Update voltage based on SOC (typical LiPo discharge curve)
    data_.voltage = calculateVoltageFromSOC(data_.state_of_charge);

    data_.time_stamp = current_time;
}

BatteryData BatterySimple::getData() const
{
    return data_;
}

bool BatterySimple::setRemaining(float percentage)
{
    if (percentage < 0.0f || percentage > 1.0f) {
        return false;
    }
    data_.state_of_charge = percentage;
    data_.remaining_capacity = percentage * data_.total_capacity;
    data_.voltage = calculateVoltageFromSOC(percentage);
    return true;
}

bool BatterySimple::setDrainRate(float rate)
{
    if (rate < 0.0f) {
        return false;
    }
    params_.discharge_rate = rate;
    return true;
}

bool BatterySimple::setHealthStatus(bool is_healthy)
{
    data_.is_healthy = is_healthy;
    if (!is_healthy) {
        // Degraded voltage when unhealthy
        data_.voltage *= 0.9f;
    }
    return true;
}

float BatterySimple::calculateVoltageFromSOC(float soc) const
{
    // LiPo discharge curve approximation
    // 100% SOC -> nominal voltage (e.g., 12.6V for 3S)
    // 20% SOC -> cutoff voltage (e.g., 9.0V for 3S)
    // Below 20% -> rapid voltage drop
    
    float nominal_v = params_.nominal_voltage;
    float cutoff_v = nominal_v * 0.714f;  // ~9V for 12.6V nominal
    
    if (soc > 0.2f) {
        // Linear region (20% - 100%)
        float range = nominal_v - cutoff_v;
        return cutoff_v + range * ((soc - 0.2f) / 0.8f);
    } else {
        // Rapid drop region (0% - 20%)
        return cutoff_v * (soc / 0.2f);
    }
}

}} // namespace
```

#### Step 5: Generate Unit Tests

**AI Prompt:**
```
Generate Google Test unit tests for BatterySimple.

Test cases:
1. Linear discharge over time
2. Nonlinear discharge over time
3. setRemaining changes SOC
4. setDrainRate affects discharge
5. setHealthStatus affects voltage
6. SOC reaches zero (full discharge)
7. Voltage calculation accuracy

File: AirLibUnitTests/BatteryTests.cpp
```

**AI-Generated Test Template:**
```cpp
// File: AirLibUnitTests/BatteryTests.cpp
#include "TestBase.hpp"
#include "sensors/battery/BatterySimple.hpp"

TEST(BatteryTest, LinearDischarge) {
    BatteryParams params;
    params.discharge_mode = BatteryParams::DischargeMode::Linear;
    params.total_capacity = 5.0f;  // 5 Ah
    params.discharge_rate = 1.0f;  // 1 A
    
    BatterySimple battery(params);
    
    // Simulate 1 hour (3600 seconds) of discharge
    for (int i = 0; i < 3600; ++i) {
        battery.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    BatteryData data = battery.getData();
    
    // After 1 hour at 1A, should have consumed 1Ah out of 5Ah
    // SOC should be (5-1)/5 = 0.8 (80%)
    EXPECT_NEAR(data.state_of_charge, 0.8f, 0.05f);
    EXPECT_NEAR(data.remaining_capacity, 4.0f, 0.3f);
}

TEST(BatteryTest, NonlinearDischarge) {
    BatteryParams params;
    params.discharge_mode = BatteryParams::DischargeMode::Nonlinear;
    params.total_capacity = 5.0f;
    params.discharge_rate = 1.0f;
    params.nonlinear_factor = 2.0f;
    
    BatterySimple battery(params);
    
    // Initial SOC
    float initial_soc = battery.getData().state_of_charge;
    EXPECT_NEAR(initial_soc, 1.0f, 0.01f);
    
    // After some time, SOC should decrease exponentially
    // (faster initially, slower later)
    // This is more realistic than linear
    battery.update();
    float soc_after_update = battery.getData().state_of_charge;
    
    EXPECT_LT(soc_after_update, initial_soc);
}

TEST(BatteryTest, SetRemaining) {
    BatterySimple battery(BatteryParams());
    
    bool result = battery.setRemaining(0.5f);  // 50%
    
    EXPECT_TRUE(result);
    EXPECT_NEAR(battery.getData().state_of_charge, 0.5f, 0.01f);
}

TEST(BatteryTest, SetDrainRate) {
    BatteryParams params;
    params.discharge_rate = 1.0f;
    BatterySimple battery(params);
    
    bool result = battery.setDrainRate(2.0f);  // Increase to 2A
    
    EXPECT_TRUE(result);
    battery.update();
    EXPECT_NEAR(battery.getData().current, 2.0f, 0.01f);
}

TEST(BatteryTest, SetHealthStatus) {
    BatterySimple battery(BatteryParams());
    float healthy_voltage = battery.getData().voltage;
    
    battery.setHealthStatus(false);
    
    float unhealthy_voltage = battery.getData().voltage;
    EXPECT_LT(unhealthy_voltage, healthy_voltage);
    EXPECT_FALSE(battery.getData().is_healthy);
}
```

#### Step 6: Integration and Validation

**AI Checklist:**
```
✓ Code compiles without errors
✓ All unit tests pass
✓ Memory leaks checked (Valgrind)
✓ Code coverage >80%
✓ Follows Cosys-AirSim coding standards
✓ Doxygen comments complete
✓ Added to CMakeLists.txt
✓ Python bindings created
✓ Example script works
✓ Documentation updated
```

**Validation Prompt:**
```
Verify the battery implementation against the specification:
1. Does BatteryData match the spec exactly?
2. Are both discharge modes implemented correctly?
3. Do control APIs work as specified?
4. Are acceptance criteria from task list met?
```

---

## Code Generation Patterns

### Pattern 1: Sensor Implementation

**Template for Any Sensor:**
```cpp
// Step 1: Define data structure
struct [Sensor]Data {
    [field_type] [field_name];  // Primary measurements
    TTimePoint time_stamp;
};

// Step 2: Define parameters
struct [Sensor]Params {
    [param_type] [param_name];  // Configuration
};

// Step 3: Create base class
class [Sensor]Base : public SensorBase {
public:
    virtual void update() = 0;
    virtual [Sensor]Data getData() const = 0;
};

// Step 4: Create implementation
class [Sensor]Simple : public [Sensor]Base {
private:
    [Sensor]Data data_;
    [Sensor]Params params_;
public:
    [Sensor]Simple(const [Sensor]Params& params);
    void update() override;
    [Sensor]Data getData() const override;
};
```

### Pattern 2: API Method Addition

**Template for Adding API Method:**
```cpp
// Step 1: Add to API base class (WorldSimApiBase.hpp or VehicleSimApiBase.hpp)
class WorldSimApiBase {
public:
    virtual [ReturnType] [methodName]([Params]) = 0;
};

// Step 2: Implement in API implementation (RpcLibClientBase.cpp)
[ReturnType] RpcLibClientBase::[methodName]([Params]) {
    return static_cast<[ReturnType]>(
        pimpl_->client.call("[method_name]", [args]).as<[RpcType]>());
}

// Step 3: Add Python binding (client.py)
def [method_name](self, [params]):
    """
    [Description]
    Args:
        [param]: [description]
    Returns:
        [return_description]
    """
    return self.client.call('[method_name]', [args])

// Step 4: Create example
import airsim
client = airsim.Client()
result = client.[method_name]([args])
print(f"Result: {result}")
```

### Pattern 3: Configuration Parsing

**Template for JSONC Configuration:**
```cpp
// Step 1: Define config structure matching JSONC format
struct [Component]Config {
    std::string id;
    [field_type] [field_name];
    
    static [Component]Config fromJSON(const nlohmann::json& j) {
        [Component]Config config;
        config.id = j.at("id").get<std::string>();
        config.[field_name] = j.at("[field-name]").get<[field_type]>();
        return config;
    }
};

// Step 2: Parse array from JSONC
std::vector<[Component]Config> parse[Component]s(const nlohmann::json& j) {
    std::vector<[Component]Config> configs;
    if (j.contains("[components]")) {
        for (const auto& item : j.at("[components]")) {
            configs.push_back([Component]Config::fromJSON(item));
        }
    }
    return configs;
}

// Step 3: Create objects from configs
for (const auto& config : configs) {
    auto component = std::make_shared<[Component]>(config);
    robot.add[Component](component);
}
```

---

## AI Prompts by Feature

### Tier 1 Features

#### Airspeed Sensor

**Prompt:**
```
Implement AirspeedSimple sensor for Cosys-AirSim.

Requirements:
1. Calculate true airspeed from vehicle velocity and wind
2. Calculate indicated airspeed from dynamic pressure
3. Use ISA atmosphere model for air density
4. Add configurable noise and offset
5. Implement startup delay

Reference existing sensor pattern from BarometerBase.

Generate:
- AirspeedBase.hpp
- AirspeedSimple.cpp
- AirspeedTests.cpp
- Python API in client.py
```

#### Radar Sensor

**Prompt:**
```
Implement RadarSimple sensor with detection and tracking.

Part 1 - Detection:
1. Ray-based detection within FOV (horizontal and vertical)
2. Calculate RCS (Radar Cross Section) based on object size
3. Calculate Doppler velocity (radial velocity)
4. Add range, velocity, and angle noise
5. Detection probability based on RCS and range

Part 2 - Tracking:
1. Implement 6-state Kalman filter (position, velocity)
2. Track association using nearest neighbor
3. Track lifecycle (creation after N detections, deletion after M misses)
4. Maximum tracks limit

Generate complete implementation with tests.
```

#### Clock Control

**Prompt:**
```
Implement ClockControl for precise simulation time management.

Methods:
1. GetSimTime() - returns nanosecond timestamp
2. Pause() / Resume() - control simulation
3. ContinueForSimTime(duration_ns) - advance by duration
4. ContinueUntilSimTime(target_time_ns) - run until target
5. ContinueForNSteps(n_steps) - execute N physics steps
6. ContinueForSingleStep() - execute one step

Requirements:
- Nanosecond precision
- Thread-safe
- Works with existing physics engine

Generate ClockControl.hpp/cpp and Python bindings.
```

### Tier 2 Features

#### Robot Framework

**Prompt:**
```
Implement generic Robot framework for Cosys-AirSim.

Components:
1. Link class - physical components with mass, inertia, collision, visual
2. Joint class - connections with types (fixed, revolute, continuous)
3. TransformTree - hierarchical coordinate transforms
4. Robot class - container for links, joints, sensors, actuators

Requirements:
1. Calculate inertia tensors from geometry (box, sphere, cylinder)
2. Compute forward kinematics
3. Support controller integration (Simple Flight, PX4)
4. Efficient transform caching

Generate complete implementation following existing Cosys patterns.
Reference: MultirotorPawnSimApi for integration.
```

#### JSONC Parser

**Prompt:**
```
Implement JSONC (JSON with Comments) parser for Cosys-AirSim.

Requirements:
1. Strip single-line comments (//)
2. Strip multi-line comments (/* */)
3. Preserve comments within strings
4. Handle edge cases (nested comments, escaped quotes)
5. Integrate with nlohmann/json

Generate:
- ConfigJson.hpp/cpp
- Unit tests for all comment types
- Example robot configurations with comments
```

#### Actuator Framework

**Prompt:**
```
Implement Actuator framework starting with Rotor actuator.

Base Actuator interface:
- UpdateActuatorOutput(float output)
- ApplyForcesAndTorques(Link* link)
- GetActuatedTransforms()

Rotor actuator:
1. Thrust calculation: T = k_t * omega^2
2. Torque calculation: Q = k_q * omega^2
3. Motor time constant (first-order lag)
4. Max RPM limit
5. Apply forces to attached link

Generate complete implementation with tests.
Include configuration parsing from JSONC.
```

### Tier 3 Features

#### Enhanced GPS

**Prompt:**
```
Enhance existing GpsBase with ProjectAirSim features.

Add to GpsData:
1. EPH (horizontal position accuracy)
2. EPV (vertical position accuracy)
3. Fix type (0=No fix, 1=DR, 2=2D, 3=3D, 4=DGPS, 5=RTK)
4. Satellites visible

Add to GpsParams:
1. EPH/EPV configuration
2. Min/max satellites
3. Fix loss simulation (random)
4. Fix loss probability

Generate:
- Updated GpsBase.hpp
- Implementation in GpsSimple.cpp
- Unit tests
- Python API updates
```

#### Enhanced IMU

**Prompt:**
```
Enhance existing ImuBase with bias simulation.

Add to ImuData:
1. Gyro bias (Vector3r)
2. Accel bias (Vector3r)
3. Temperature (affects bias)

Add to ImuParams:
1. Gyro bias sigma, tau (random walk)
2. Accel bias sigma, tau
3. Temperature coefficient

Implementation:
1. Bias random walk: bias(t+dt) = bias(t) + N(0, sigma) * sqrt(dt)
2. Temperature-dependent bias drift
3. Apply bias to measurements

Generate complete implementation with long-term drift tests.
```

### Tier 5 Features

#### Weather APIs

**Prompt:**
```
Implement World Weather API methods for Cosys-AirSim.

Methods:
1. set_sunlight_intensity(intensity: float)
2. set_cloud_shadow_strength(strength: float)
3. enable_weather_visual_effects()
4. set_weather_visual_effects_param(param_id: int, value: float)
5. set_time_of_day(hour, minute, second)
6. set_sun_position_from_date_time(date, time, lat, lon)
7. set_wind_velocity(vx, vy, vz)
8. get_wind_velocity()

Requirements:
1. Integrate with Unreal lighting system
2. Integrate with Unreal weather system
3. Wind affects vehicle dynamics

Generate:
- C++ API in WorldSimApiBase
- RPC implementation
- Python bindings
- Example: dynamic_weather.py
```

#### Trajectory Management

**Prompt:**
```
Implement trajectory import and management system.

Methods:
1. import_ned_trajectory(name, waypoints)
2. import_ned_trajectory_from_csv(name, csv_path)
3. import_geo_trajectory(name, waypoints_lat_lon_alt)
4. import_trajectory_from_kml(name, kml_path)
5. generate_intercept_trajectory(start, target, time)

Requirements:
1. Trajectory storage (waypoints with timestamps)
2. Interpolation (linear, spline)
3. Coordinate conversion (geo ↔ NED)
4. KML parser
5. Intercept algorithm (constant velocity)

Generate complete implementation with CSV/KML examples.
```

### Tier 6 Features

#### Gimbal Actuator

**Prompt:**
```
Implement Gimbal actuator for camera stabilization.

Requirements:
1. 3-axis control (pitch, roll, yaw)
2. Angle limits per axis (min, max)
3. Angular velocity limits (rate limit)
4. Smooth movement (damping)
5. Target orientation control
6. Camera attachment

Generate:
- Gimbal.hpp/cpp
- Configuration parsing
- Control APIs (set_gimbal_angle, get_gimbal_angle)
- Example: gimbal_stabilization.py with camera tracking
```

#### Path Planning

**Prompt:**
```
Implement path planning framework with RRT and A*.

RRT (Rapidly-exploring Random Tree):
1. Sample random points in 3D space
2. Find nearest node in tree
3. Extend toward sample (step size limit)
4. Check collision (use spatial queries)
5. Add to tree if collision-free
6. Goal bias (bias toward goal with probability p)
7. Return path when goal reached

A* Algorithm:
1. 3D occupancy grid
2. Heuristic: Euclidean distance to goal
3. Cost: path length + heuristic
4. Priority queue for open set
5. Reconstruct path

Requirements:
- Collision checking using existing spatial queries
- Path smoothing (remove unnecessary waypoints)
- Configurable parameters (step size, goal bias, grid resolution)

Generate complete implementation with performance tests.
```

### Tier 7 Features

#### Mission Planning

**Prompt:**
```
Implement mission planning framework.

Mission structure:
1. Waypoints with associated actions
2. Actions: takeoff, land, loiter, photo, set_gimbal, etc.
3. Conditions: altitude, distance, time, battery
4. State machine: idle → executing → paused → completed/failed

Mission methods:
1. create_mission(waypoints, actions, conditions)
2. load_mission(json_path)
3. save_mission(json_path)
4. execute_mission()
5. pause_mission()
6. resume_mission()
7. abort_mission()
8. get_mission_status()

Generate:
- Mission class with state machine
- Action types (extensible)
- Condition evaluation
- JSON format specification
- Example: survey_mission.py (grid pattern)
```

---

## Debugging Prompts

### Compilation Errors

**Prompt:**
```
I'm getting compilation errors in [file].

Error messages:
[paste error messages]

The code is:
[paste code excerpt]

Analyze the errors and suggest fixes.
Check for:
1. Missing includes
2. Namespace issues
3. Type mismatches
4. Missing virtual overrides
```

### Test Failures

**Prompt:**
```
Unit test [test_name] is failing.

Test code:
[paste test code]

Expected result: [expected]
Actual result: [actual]

Failure message:
[paste failure message]

Debug the test:
1. Is the test logic correct?
2. Are the expectations reasonable?
3. Is the implementation buggy?
4. Are there precision issues (use EXPECT_NEAR)?

Suggest fix.
```

### Memory Leaks

**Prompt:**
```
Valgrind reports memory leaks in [component].

Valgrind output:
[paste output]

Code:
[paste code]

Identify:
1. Leaked objects (missing delete/free)
2. Smart pointer usage issues
3. Circular references
4. Resource acquisition/release mismatches

Suggest fixes using RAII and smart pointers.
```

### Performance Issues

**Prompt:**
```
Benchmark shows [component] is too slow.

Current performance: [X ms/call]
Target performance: [Y ms/call]

Profiling shows hotspot in:
[paste profiling data]

Optimize:
1. Reduce allocations (use object pools)
2. Cache repeated calculations
3. Use efficient data structures
4. Parallelize if possible

Suggest optimizations.
```

---

## Quality Assurance Checklist

### Pre-Merge Checklist

**AI Validation Prompt:**
```
Validate my implementation of [feature] against this checklist:

Code Quality:
✓ Compiles without warnings (-Wall -Wextra)
✓ No memory leaks (Valgrind clean)
✓ No undefined behavior (sanitizers clean)
✓ Follows coding standards (clang-format applied)
✓ Includes and namespaces correct
✓ Error handling complete

Testing:
✓ Unit tests written (>80% coverage)
✓ All tests pass
✓ Edge cases tested
✓ Performance tests pass (within budget)
✓ Integration tests pass

Documentation:
✓ Doxygen comments for all public methods
✓ Parameter descriptions complete
✓ Return value documented
✓ Usage examples provided
✓ README updated

Integration:
✓ Added to CMakeLists.txt
✓ Python bindings created
✓ Example script works
✓ Backward compatibility maintained
✓ No breaking changes

Review each item and report status.
```

---

## Example AI Session

### Full Feature Implementation Session

**Session: Implementing Battery Sensor**

```
# Step 1: Planning
AI: "I will implement the Battery Sensor following these steps:
1. Analyze specification (section 2.3.3)
2. Study existing sensor patterns (BarometerBase)
3. Generate BatteryBase.hpp
4. Generate BatterySimple.cpp
5. Generate unit tests
6. Create Python bindings
7. Create example script
Estimated time: 2 hours. Proceed?"

User: "Yes, proceed."

# Step 2: Code Generation
AI: "Generating BatteryBase.hpp following BarometerBase pattern..."
[Generates code]

AI: "BatteryBase.hpp created. Key features:
- BatteryData struct with 7 fields
- BatteryParams with discharge modes enum
- Control methods: setRemaining, setDrainRate, setHealthStatus
Next: implementation?"

User: "Yes."

# Step 3: Implementation
AI: "Generating BatterySimple.cpp with linear and nonlinear discharge..."
[Generates code]

AI: "Implementation complete. Features:
- Linear discharge: constant rate
- Nonlinear discharge: exponential decay
- Voltage-SOC curve approximation (LiPo)
- Control methods implemented
Next: tests?"

User: "Yes."

# Step 4: Testing
AI: "Generating 10 unit tests covering:
- Linear discharge over time
- Nonlinear discharge
- setRemaining, setDrainRate, setHealthStatus
- Edge cases (zero drain, full discharge)
- Voltage calculation accuracy"
[Generates tests]

AI: "Tests generated. Running compilation..."
[Compiles]

AI: "✓ Compilation successful. Running tests..."
[Runs tests]

AI: "✓ All 10 tests pass. Code coverage: 87%. Next: Python bindings?"

User: "Yes."

# Step 5: Python API
AI: "Adding Python methods to client.py:
- get_battery_data()
- set_battery_remaining()
- set_battery_drain_rate()
- set_battery_health_status()"
[Generates bindings]

AI: "Creating example: battery_simulation_example.py"
[Generates example]

AI: "✓ Python API complete. Example demonstrates:
- Linear vs nonlinear discharge comparison
- Dynamic drain rate changes
- SOC plotting over time
Next: documentation?"

User: "Yes."

# Step 6: Documentation
AI: "Updating documentation:
- Added Battery Sensor to feature list
- Updated API reference
- Added example to tutorials
Complete! Summary:
- 3 files created (BatteryBase.hpp, BatterySimple.cpp, BatteryTests.cpp)
- 10 unit tests (all passing)
- Python API (4 methods)
- 1 example script
- Documentation updated
Ready for review."

User: "Great! Run final validation."

# Step 7: Validation
AI: "Running final validation checklist..."
[Runs checks]

AI: "✓ All checks passed:
- Compilation: clean
- Tests: 10/10 pass
- Coverage: 87%
- Memory: no leaks
- Performance: update <10μs (within budget)
- Documentation: complete
Ready to merge!"
```

---

## Advanced AI Techniques

### Multi-Step Reasoning

**Prompt:**
```
Implement [complex feature] using chain-of-thought reasoning.

Break down into steps:
1. What are the requirements?
2. What existing patterns can I follow?
3. What are the core algorithms needed?
4. What are the edge cases?
5. How do I test this?
6. What are the performance implications?

Think through each step before generating code.
```

### Self-Correction

**Prompt:**
```
Review my implementation of [feature].

Checklist:
1. Does it match the specification exactly?
2. Are there any bugs or edge cases missed?
3. Is the code efficient?
4. Is it well-documented?
5. Are tests comprehensive?

If you find issues, explain them and generate corrected code.
```

### Iterative Refinement

**Prompt:**
```
Implement [feature] iteratively:

Iteration 1: Basic functionality (minimal features)
Iteration 2: Add error handling and edge cases
Iteration 3: Optimize performance
Iteration 4: Enhance documentation and tests

Show me each iteration before proceeding to the next.
```

---

## Appendices

### Appendix A: Common AI Errors and Fixes

**Error 1: Incorrect Type Conversions**
```cpp
// AI might generate:
float angle = joint.getAngle();  // Returns double

// Should be:
float angle = static_cast<float>(joint.getAngle());
```

**Error 2: Missing Virtual Overrides**
```cpp
// AI might generate:
void update();  // Missing override keyword

// Should be:
void update() override;
```

**Error 3: Memory Management**
```cpp
// AI might generate:
Sensor* sensor = new Sensor();  // Raw pointer

// Should be:
auto sensor = std::make_shared<Sensor>();  // Smart pointer
```

### Appendix B: Useful Code Search Queries

**Find sensor patterns:**
```bash
# Search for sensor base classes
find AirLib/include/sensors -name "*Base.hpp"

# Find sensor implementations
find AirLib/src/sensors -name "*Simple.cpp"

# Find API methods
grep -r "virtual.*=.*0;" AirLib/include/api/
```

**Find configuration patterns:**
```bash
# Search for JSON parsing
grep -r "fromJSON" AirLib/include/

# Search for parameter structures
grep -r "Params {" AirLib/include/sensors/
```

### Appendix C: Testing Commands

**Run unit tests:**
```bash
# Build tests
cmake --build . --target AirLibUnitTests

# Run all tests
./AirLibUnitTests

# Run specific test suite
./AirLibUnitTests --gtest_filter=BatteryTest.*

# Run with verbose output
./AirLibUnitTests --gtest_v=1
```

**Check coverage:**
```bash
# Generate coverage report
lcov --capture --directory . --output-file coverage.info
genhtml coverage.info --output-directory coverage_report

# View report
firefox coverage_report/index.html
```

**Check for memory leaks:**
```bash
# Run with Valgrind
valgrind --leak-check=full ./AirLibUnitTests
```

---

**Document Status:** ACTIVE AI GUIDE  
**Last Updated:** January 2025  
**Maintained By:** Integration Team  
**Version Control:** Git repository `docs/integration/`  
**Feedback:** Report AI-specific issues to improve this guide
