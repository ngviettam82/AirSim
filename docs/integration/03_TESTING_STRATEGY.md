# Cosys-AirSim Dual-Mode Integration - Testing Strategy

**Document Version:** 2.0  
**Date:** January 2025  
**Status:** Active Testing Plan  
**Coverage Goal:** >80% code coverage (both modes)  
**Total Test Count:** ~150 tests (50 Simple + 80 Advanced + 20 Integration)

---

## Document Overview

This document defines the **comprehensive testing strategy** for the dual-mode architecture integration. It covers unit testing, integration testing, system testing, performance testing, and validation approaches for both Simple and Advanced modes.

**Dual-Mode Architecture:**
- **Simple Mode (Default):** Enhanced MultirotorParams, full inertia tensors, single rigid body
- **Advanced Mode (Optional):** Robot/Link/Joint class hierarchy, multi-body articulated systems

**Related Documents:**
- `00_MASTER_TECHNICAL_SPECIFICATION.md` - Technical specifications
- `01_IMPLEMENTATION_ROADMAP.md` - Implementation timeline
- `02_DETAILED_TASK_LIST.md` - Granular task breakdown
- `04_AI_AGENT_GUIDE.md` - AI implementation guide

---

## Testing Philosophy

### Core Principles

1. **Test Early, Test Often:** Write tests alongside implementation
2. **Automated First:** Prefer automated tests over manual
3. **Regression Prevention:** All bugs get test cases
4. **Performance Aware:** Track performance metrics continuously
5. **Backward Compatible:** Ensure existing Cosys-AirSim tests still pass
6. **Mode-Aware Testing:** Test both Simple and Advanced modes independently and together

### Quality Gates

**No code merges without:**
- ✅ All unit tests passing (100%)
- ✅ Code coverage >80% for new code
- ✅ No memory leaks (Valgrind clean)
- ✅ Code review approval
- ✅ Integration tests passing
- ✅ Mode-specific tests passing (if applicable)

---

## Test Levels

### Level 1: Unit Tests

**Scope:** Individual classes, methods, functions in isolation  
**Framework:** Google Test (C++), pytest (Python)  
**Coverage Target:** >85%  
**Frequency:** Run on every commit (CI)

**Characteristics:**
- Fast (<1ms per test)
- No external dependencies (mocked)
- Deterministic results
- High granularity

**Example:**
```cpp
TEST(BatterySimpleTest, LinearDischarge) {
    BatterySimple battery(BatteryParams{
        .nominal_voltage = 12.6f,
        .total_capacity = 5.0f,
        .discharge_rate = 1.0f,
        .discharge_mode = BatteryParams::DischargeMode::Linear
    });
    
    // After 1 hour at 1A drain, SOC should be 80%
    battery.update(3600.0f);
    EXPECT_NEAR(battery.getData().state_of_charge, 0.8f, 0.01f);
}
```

### Level 2: Integration Tests

**Scope:** Multiple components working together  
**Framework:** Google Test (C++), pytest (Python)  
**Coverage Target:** >70%  
**Frequency:** Run on every pull request (CI)

**Characteristics:**
- Moderate speed (<100ms per test)
- Minimal external dependencies
- Tests component interactions
- More complex scenarios

**Example:**
```cpp
TEST(RadarIntegrationTest, DetectsMovingObject) {
    // Setup scene with radar and moving object
    Radar radar = CreateTestRadar();
    Object target = SpawnMovingObject(Vector3r(10, 0, 0), Vector3r(5, 0, 0));
    
    radar.update(0.1f);  // 100ms
    RadarData data = radar.getData();
    
    ASSERT_EQ(data.detections.size(), 1);
    EXPECT_NEAR(data.detections[0].range, 10.0f, 0.5f);
    EXPECT_NEAR(data.detections[0].velocity, -5.0f, 0.5f);  // Approaching
}
```

### Level 3: System Tests

**Scope:** End-to-end scenarios with full simulation  
**Framework:** Python scripts + assertions  
**Coverage Target:** >50% of user scenarios  
**Frequency:** Run nightly (CI)

**Characteristics:**
- Slow (seconds to minutes per test)
- Full Unreal environment
- Tests complete workflows
- User-facing scenarios

**Example:**
```python
def test_quadrotor_mission_with_battery():
    """Test complete mission with battery drain"""
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    
    # Record initial battery
    initial_soc = client.get_battery_data()['state_of_charge']
    
    # Fly mission
    client.takeoffAsync().join()
    client.moveToPositionAsync(0, 0, -10, 5).join()
    time.sleep(30)  # Hover for 30 seconds
    client.landAsync().join()
    
    # Verify battery drained
    final_soc = client.get_battery_data()['state_of_charge']
    assert final_soc < initial_soc, "Battery should have drained during flight"
```

### Level 4: Performance Tests

**Scope:** Performance benchmarks and resource usage  
**Framework:** Google Benchmark (C++), pytest-benchmark (Python)  
**Coverage Target:** All critical paths  
**Frequency:** Run weekly + before releases

**Characteristics:**
- Measures execution time
- Measures memory usage
- Tracks trends over time
- Regression detection

**Example:**
```cpp
static void BM_RadarUpdate(benchmark::State& state) {
    Radar radar = CreateTestRadar();
    SpawnNObjects(50);  // 50 objects in scene
    
    for (auto _ : state) {
        radar.update(0.01f);  // 10ms update
    }
    
    state.SetLabel("50 objects");
}
BENCHMARK(BM_RadarUpdate);
```

---

## Testing Strategy by Tier

### Tier 1: Critical Sensors (Weeks 1-4)

#### Battery Sensor Testing

**Unit Tests (10 tests):**
1. Linear discharge over time
2. Nonlinear discharge curve
3. Set remaining SOC
4. Set drain rate
5. Set health status affects voltage
6. Voltage-SOC relationship
7. Temperature simulation
8. Zero drain rate (no discharge)
9. Full discharge (SOC → 0)
10. Overcharge protection

**Integration Tests (3 tests):**
1. Battery drains during vehicle movement
2. Python API retrieves correct data
3. Control methods affect battery state

**System Tests (1 test):**
1. Full flight mission with battery drain

**Performance Tests (1 test):**
1. Battery update <10μs

#### Airspeed Sensor Testing

**Unit Tests (8 tests):**
1. True airspeed calculation (no wind)
2. True airspeed with wind
3. Indicated airspeed from pressure
4. Altitude affects IAS
5. Noise addition
6. Offset calibration
7. Startup delay
8. Temperature effects

**Integration Tests (2 tests):**
1. Airspeed correlates with vehicle velocity
2. Python API returns correct data

**System Tests (1 test):**
1. Fixed-wing flight with airspeed monitoring

**Performance Tests (1 test):**
1. Airspeed update <5μs

#### Radar Sensor Testing

**Unit Tests (15 tests):**
1. Detection within range
2. No detection beyond range
3. Azimuth angle calculation
4. Elevation angle calculation
5. RCS calculation (size-based)
6. RCS calculation (material-based)
7. Doppler velocity (approaching)
8. Doppler velocity (receding)
9. Range noise
10. Velocity noise
11. Angle noise
12. Kalman filter prediction
13. Kalman filter update
14. Track creation threshold
15. Track deletion (missed detections)

**Integration Tests (5 tests):**
1. Detects multiple objects
2. Tracks moving object
3. Handles object disappearance
4. Maximum tracks limit
5. Python API returns detections and tracks

**System Tests (2 tests):**
1. Track aircraft in formation
2. Detect ground vehicles

**Performance Tests (2 tests):**
1. Detection update <50ms (50 objects)
2. Tracking update <10ms (10 tracks)

#### Clock Control Testing

**Unit Tests (8 tests):**
1. Get sim time
2. Pause simulation
3. Resume simulation
4. Continue for sim time
5. Continue until sim time
6. Continue for N steps
7. Continue for single step
8. Clock type queries

**Integration Tests (2 tests):**
1. Paused simulation doesn't advance
2. Single-step determinism

**System Tests (1 test):**
1. Deterministic replay of flight

**Performance Tests (1 test):**
1. Clock query <1μs

---

### Tier 2: Robot Architecture (Weeks 5-7)

#### Link Class Testing

**Unit Tests (8 tests):**
1. Inertia tensor (box)
2. Inertia tensor (sphere)
3. Inertia tensor (cylinder)
4. Inertia tensor (point mass)
5. Aerodynamic drag force
6. Drag varies with velocity²
7. Collision properties
8. Visual properties

**Performance Tests (1 test):**
1. Inertia calculation <1μs

#### Joint Class Testing

**Unit Tests (8 tests):**
1. Fixed joint (no motion)
2. Revolute joint angle limits
3. Revolute joint within limits
4. Continuous joint unlimited rotation
5. Damping torque
6. Joint axis orientation
7. Parent-child link references
8. Joint origin transform

#### Transform Tree Testing

**Unit Tests (6 tests):**
1. Single-level transform
2. Multi-level transform chain
3. Transform caching
4. Cache invalidation on update
5. Cyclic dependency detection
6. Missing link error handling

**Performance Tests (1 test):**
1. FK computation <1ms (20 links)

#### Robot Class Testing

**Unit Tests (10 tests):**
1. Add link
2. Get link by name
3. Add joint
4. Get joint by ID
5. Add sensor
6. Add actuator
7. Set controller
8. Begin/end update cycle
9. Publish robot pose
10. Callback mechanisms

**Integration Tests (3 tests):**
1. Load robot from JSONC
2. Sensors receive data
3. Actuators apply forces

**System Tests (1 test):**
1. Fly complete robot-based quadrotor

#### JSONC Parser Testing

**Unit Tests (12 tests):**
1. Parse valid JSON
2. Parse JSONC with single-line comments
3. Parse JSONC with multi-line comments
4. Comments in different positions
5. Comments within arrays
6. Comments within objects
7. Preserve strings with //
8. Preserve strings with /* */
9. Nested comments
10. Empty file
11. Malformed JSON error
12. Unclosed comment error

**Integration Tests (2 tests):**
1. Load robot config with comments
2. Validate error messages

#### Actuator (Rotor) Testing

**Unit Tests (10 tests):**
1. Thrust calculation (RPM²)
2. Torque calculation (RPM²)
3. Motor time constant lag
4. Max RPM limit
5. Zero throttle (zero thrust)
6. Full throttle (max thrust)
7. Thrust direction (link frame)
8. Reaction torque direction
9. Force application to link
10. Rotor parameters parsing

**Integration Tests (2 tests):**
1. Four rotors generate stable hover
2. Differential thrust causes rotation

**System Tests (1 test):**
1. Quadrotor flight with rotor actuators

**Performance Tests (1 test):**
1. Rotor update <5μs

---

### Tier 3: Enhanced Sensors (Weeks 8-11)

#### Enhanced Barometer Testing

**Unit Tests (6 tests):**
1. Pressure-altitude conversion
2. Temperature effects on pressure
3. ISA model accuracy
4. Noise addition
5. Temperature coefficient
6. Altitude range (0-10,000m)

#### Enhanced GPS Testing

**Unit Tests (8 tests):**
1. EPH accuracy simulation
2. EPV accuracy simulation
3. Fix type changes
4. Satellite count variation
5. Fix loss probability
6. 2D vs 3D fix
7. DGPS accuracy
8. RTK accuracy

#### Enhanced IMU Testing

**Unit Tests (10 tests):**
1. Gyro bias random walk
2. Accel bias random walk
3. Bias time constant
4. Temperature-dependent bias
5. Bias initialization
6. Bias stability over time
7. Long-term drift (1000s)
8. Noise + bias combination
9. Bias calibration
10. Bias export for analysis

#### Enhanced Magnetometer Testing

**Unit Tests (8 tests):**
1. Declination calculation (latitude/longitude)
2. Inclination calculation
3. Magnetic field vector (NED frame)
4. Bias addition
5. WMM integration
6. Heading from magnetometer
7. Magnetic disturbance
8. Indoor vs outdoor field strength

---

### Tier 4: Utility Sensors (Weeks 12-14)

#### Enhanced Distance Sensor Testing

**Unit Tests (6 tests):**
1. Beam width (cone detection)
2. Multi-echo (first, strongest, last)
3. Echo selection logic
4. Range limits
5. Noise addition
6. Update frequency

#### Enhanced Lidar Testing

**Unit Tests (8 tests):**
1. Point intensity (material-based)
2. Ring information
3. Channel information
4. Timestamp per point
5. Point cloud size
6. Range accuracy
7. Angle accuracy
8. Update frequency

#### Enhanced Camera Testing

**Unit Tests (6 tests):**
1. Exposure control
2. DOF (focus distance, aperture)
3. Motion blur (shutter speed)
4. API parameter validation
5. Unreal integration
6. Frame rate consistency

---

### Tier 5: World APIs (Weeks 15-18)

#### Complete Clock Control Testing

**Unit Tests (5 tests):**
1. Clock type configuration
2. Time scale (scalable clock)
3. Fixed timestep (stepped-realtime)
4. Async continue operations
5. Wait until complete flag

#### Weather APIs Testing

**Unit Tests (12 tests):**
1. Sun intensity (0-1)
2. Cloud shadow strength
3. Weather effects enable/disable
4. Weather parameter ranges
5. Time of day (0-23 hours)
6. Sun position calculation (date/time/lat/lon)
7. Light object intensity
8. Light object color (RGB)
9. Light object radius
10. Wind velocity (NED frame)
11. Wind affects vehicle
12. Weather persistence

**Integration Tests (3 tests):**
1. Sun position changes lighting
2. Wind affects flight
3. Weather effects visible

**System Tests (1 test):**
1. Dynamic weather scenario

#### Spatial Query APIs Testing

**Unit Tests (8 tests):**
1. 3D bounding box (8 corners)
2. Surface elevation raycast
3. Random free position (collision-free)
4. Random position near point
5. Random position near path
6. Min/max distance constraints
7. Collision detection accuracy
8. Query performance

**Integration Tests (2 tests):**
1. Spawn objects at free positions
2. Bounding box accuracy for complex meshes

#### Object Management APIs Testing

**Unit Tests (10 tests):**
1. Spawn object from mesh
2. Destroy object
3. Set object pose
4. Get object pose
5. Set object scale
6. Set object material
7. Load scene
8. List scene objects
9. Object name validation
10. Asset not found handling

**Integration Tests (3 tests):**
1. Spawn, move, destroy workflow
2. Multiple objects interaction
3. Scene loading

**System Tests (1 test):**
1. Dynamic environment creation

---

### Tier 6: Advanced Features (Weeks 19-23)

#### Gimbal Actuator Testing

**Unit Tests (10 tests):**
1. Pitch angle limits
2. Roll angle limits
3. Yaw angle limits
4. Angular velocity limits
5. Gimbal control (target orientation)
6. Gimbal stabilization
7. Gimbal configuration parsing
8. Camera attachment
9. Gimbal kinematics
10. Gimbal damping

**Integration Tests (2 tests):**
1. Camera stabilization during flight
2. Gimbal tracks target

**System Tests (1 test):**
1. Aerial photography with gimbal

#### Tilt Actuator Testing

**Unit Tests (8 tests):**
1. Tilt angle range
2. Tilt rate limit
3. Tilt axis orientation
4. VTOL transition logic
5. Tilt configuration parsing
6. Force reorientation
7. Coupled rotor/wing lift
8. Tilt damping

#### Wheel Actuator Testing

**Unit Tests (8 tests):**
1. Angular velocity control
2. Steering angle (steered wheels)
3. Friction model
4. Wheel radius
5. Contact force
6. Rolling resistance
7. Wheel configuration parsing
8. Drive vs steered wheels

#### Control Surface Actuator Testing

**Unit Tests (12 tests):**
1. Deflection angle range
2. Deflection rate limit
3. Lift coefficient
4. Drag coefficient
5. Force calculation (angle of attack)
6. Moment calculation
7. Aerodynamic center
8. Control surface area
9. Configuration parsing
10. Elevon mixing
11. Rudder control
12. Aileron control

**Integration Tests (2 tests):**
1. Control surfaces affect flight
2. Fixed-wing control

**System Tests (1 test):**
1. Fixed-wing aerobatic maneuvers

#### Trajectory Management Testing

**Unit Tests (20 tests):**
1. NED waypoint import
2. CSV parsing (NED)
3. Geographic waypoint import
4. KML parsing
5. Intercept trajectory generation
6. Linear interpolation
7. Spline interpolation
8. Trajectory timing
9. Waypoint ordering
10. Coordinate conversion (geo ↔ NED)
11. Trajectory validation
12. Missing waypoint handling
13. Duplicate waypoint removal
14. Trajectory smoothing
15. Velocity profile
16. Acceleration limits
17. Trajectory export
18. Trajectory visualization
19. Multi-segment trajectories
20. Looping trajectories

**Integration Tests (3 tests):**
1. Import and fly NED trajectory
2. Import and fly KML trajectory
3. Intercept moving target

**System Tests (2 tests):**
1. Complex multi-waypoint mission
2. Formation flight with trajectories

---

### Tier 7: Extended Capabilities (Weeks 24-33)

#### Mission Planning Testing

**Unit Tests (15 tests):**
1. Mission creation
2. Waypoint actions
3. Conditional actions
4. Action sequencing
5. Mission state machine
6. Mission validation
7. Mission loading (JSON/JSONC)
8. Mission saving
9. Action timeouts
10. Action failures
11. Mission pause/resume
12. Mission abort
13. Mission status query
14. Mission progress tracking
15. Mission branching

**System Tests (2 tests):**
1. Survey mission (grid pattern)
2. Search and rescue mission

#### Path Planning Testing

**Unit Tests (18 tests):**
1. RRT initialization
2. RRT expansion
3. RRT goal bias
4. RRT path found
5. RRT timeout
6. A* initialization
7. A* heuristic function
8. A* path found
9. A* optimal path
10. Collision checking (3D grid)
11. Occupancy grid generation
12. Path smoothing
13. Path validation (collision-free)
14. Path cost calculation
15. Multi-goal planning
16. Dynamic replanning
17. Path export
18. Path visualization

**Integration Tests (2 tests):**
1. Plan and fly RRT path
2. Plan and fly A* path

**System Tests (1 test):**
1. Autonomous navigation in cluttered environment

**Performance Tests (2 tests):**
1. RRT planning <5s (1000 nodes)
2. A* planning <1s (10,000 nodes)

#### Segmentation Testing

**Unit Tests (6 tests):**
1. Instance ID generation
2. Color mapping (ID → RGB)
3. Instance mask rendering
4. Multiple instances
5. Segmentation accuracy
6. Performance (frame rate)

**Integration Tests (1 test):**
1. Segmentation with object detection

**System Tests (1 test):**
1. Real-time segmentation during flight

#### Annotation Testing

**Unit Tests (10 tests):**
1. 2D bounding box calculation
2. 3D bounding box calculation
3. Keypoint annotation
4. Segmentation mask export
5. COCO format export
6. YOLO format export
7. Annotation validation
8. Annotation persistence
9. Multi-object annotation
10. Annotation visualization

**Integration Tests (1 test):**
1. Annotate and export dataset

**System Tests (1 test):**
1. Create training dataset from simulation

#### Mesh Operations Testing

**Unit Tests (8 tests):**
1. Mesh spawning
2. Mesh transformation
3. Mesh material modification
4. Mesh collision update
5. Mesh info query
6. Mesh removal
7. Asset library access
8. Invalid mesh handling

**Integration Tests (1 test):**
1. Dynamic environment modification

---

## Test Infrastructure

### Continuous Integration (CI)

**GitHub Actions Workflow:**
```yaml
name: CI

on: [push, pull_request]

jobs:
  unit-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Build
        run: ./build.sh
      - name: Run Unit Tests
        run: ./run_unit_tests.sh
      - name: Upload Coverage
        run: bash <(curl -s https://codecov.io/bash)
  
  integration-tests:
    runs-on: ubuntu-latest
    needs: unit-tests
    steps:
      - uses: actions/checkout@v2
      - name: Build
        run: ./build.sh
      - name: Run Integration Tests
        run: ./run_integration_tests.sh
  
  system-tests:
    runs-on: ubuntu-latest
    needs: integration-tests
    if: github.ref == 'refs/heads/main'
    steps:
      - uses: actions/checkout@v2
      - name: Run System Tests
        run: ./run_system_tests.sh
```

### Test Execution

**Local Development:**
```bash
# Run all unit tests
./run_tests.sh --unit

# Run specific test suite
./run_tests.sh --suite=BatteryTests

# Run with coverage
./run_tests.sh --coverage

# Run performance benchmarks
./run_benchmarks.sh
```

**CI Pipeline:**
1. **On every commit:** Unit tests
2. **On pull request:** Unit + integration tests
3. **Nightly:** Unit + integration + system tests
4. **Weekly:** Full test suite + performance benchmarks

### Code Coverage

**Tools:**
- C++: gcov + lcov
- Python: coverage.py

**Coverage Reports:**
- HTML reports generated in `test_results/coverage/`
- Uploaded to Codecov.io
- Pull requests show coverage diff

**Coverage Targets:**
- Overall: >80%
- New code: >85%
- Critical paths: >95%

---

## Performance Testing

### Benchmarking Framework

**C++ (Google Benchmark):**
```cpp
#include <benchmark/benchmark.h>

static void BM_SensorUpdate(benchmark::State& state) {
    Sensor sensor = CreateTestSensor();
    for (auto _ : state) {
        sensor.update(0.01f);
    }
    state.SetLabel(sensor.getName());
}
BENCHMARK(BM_SensorUpdate);
```

**Python (pytest-benchmark):**
```python
def test_api_latency(benchmark):
    client = airsim.MultirotorClient()
    result = benchmark(client.get_battery_data)
    assert result is not None
```

### Performance Targets

| Component | Target | Measurement |
|-----------|--------|-------------|
| Battery update | <10μs | Per update call |
| Airspeed update | <5μs | Per update call |
| Radar detection | <50ms | 50 objects |
| Radar tracking | <10ms | 10 tracks |
| Clock query | <1μs | GetSimTime() |
| Link inertia calc | <1μs | Per link |
| Transform tree FK | <1ms | 20 links |
| Rotor update | <5μs | Per rotor |
| World API call | <1ms | Simple query |
| Path planning (RRT) | <5s | 1000 nodes |
| Path planning (A*) | <1s | 10,000 nodes |

### Performance Regression Detection

**Automated Alerts:**
- >10% performance degradation → Warning
- >20% performance degradation → Fail build
- Tracked in time-series database (Prometheus)
- Visualized in Grafana dashboards

---

## Validation and Verification

### Validation Against ProjectAirSim

**Comparison Tests:**
1. Load same robot config in both simulators
2. Execute identical commands
3. Compare sensor outputs (tolerance: 5%)
4. Compare trajectory tracking (tolerance: 10%)

**Validation Scenarios:**
1. Quadrotor hover stability
2. Fixed-wing cruise flight
3. Trajectory following accuracy
4. Sensor noise characteristics
5. Battery discharge curves

### Real-World Validation

**Hardware-in-the-Loop (HIL):**
- Connect real flight controller (PX4)
- Compare simulated sensor data with real sensors
- Validate control algorithms

**Flight Data Comparison:**
- Log real flight data (GPS, IMU, etc.)
- Replay in simulation
- Compare simulated sensors with logged data
- Tune noise models to match reality

---

## Regression Testing

### Regression Test Suite

**Existing Cosys-AirSim Tests:**
- All existing tests must continue to pass
- No breaking changes to existing APIs
- Performance must not degrade >10%

**Regression Scenarios:**
1. Existing multirotor flight
2. Existing car simulation
3. Existing computer vision
4. Existing PX4 integration
5. Existing Python client functionality

### Regression Prevention

**Every Bug Gets a Test:**
1. Bug reported
2. Write failing test that reproduces bug
3. Fix bug
4. Test now passes
5. Test added to regression suite

---

## Test Data Management

### Test Fixtures

**Reusable Test Data:**
- Sample robot configurations (JSONC)
- Test trajectories (CSV, KML)
- Test scenes (Unreal maps)
- Validation datasets

**Location:** `AirLib/tests/fixtures/`

### Test Environments

**Unreal Test Maps:**
1. Empty environment (performance baseline)
2. Simple obstacles (collision testing)
3. Complex urban (path planning)
4. Outdoor terrain (GPS, elevation queries)

---

## Quality Metrics

### Test Quality

**Metrics:**
- Test coverage percentage
- Test execution time
- Flaky test rate (<1%)
- Test maintenance burden

**Tracking:**
- Dashboard showing trends
- Weekly test health report
- Test ownership assignment

### Bug Metrics

**Tracking:**
- Bugs found per 1000 LOC
- Time to fix (TTF)
- Reopen rate
- Severity distribution

**Target:**
- <1 bug per 1000 LOC in production
- Critical bugs fixed within 24 hours
- <5% reopen rate

---

## Test Documentation

### Test Plan Documents

Each feature should have:
1. Test plan (this document section)
2. Test cases (detailed scenarios)
3. Test results (pass/fail logs)
4. Coverage report (code coverage)

### Test Case Template

```markdown
## Test Case: [TC-ID] [Test Name]

**Feature:** [Feature name]  
**Priority:** [High/Medium/Low]  
**Type:** [Unit/Integration/System]

**Preconditions:**
- [Setup requirements]

**Steps:**
1. [Action 1]
2. [Action 2]
3. [Verification]

**Expected Result:**
- [Expected outcome]

**Acceptance Criteria:**
- [ ] Criterion 1
- [ ] Criterion 2
```

---

## Appendices

### Appendix A: Test Coverage Summary

| Tier | Feature | Unit Tests | Integration Tests | System Tests | Total |
|------|---------|------------|-------------------|--------------|-------|
| T1 | Battery | 10 | 3 | 1 | 14 |
| T1 | Airspeed | 8 | 2 | 1 | 11 |
| T1 | Radar | 15 | 5 | 2 | 22 |
| T1 | Clock | 8 | 2 | 1 | 11 |
| T2 | Link | 8 | 0 | 0 | 8 |
| T2 | Joint | 8 | 0 | 0 | 8 |
| T2 | TransformTree | 6 | 0 | 0 | 6 |
| T2 | Robot | 10 | 3 | 1 | 14 |
| T2 | JSONC | 12 | 2 | 0 | 14 |
| T2 | Rotor | 10 | 2 | 1 | 13 |
| **Total T1-T2** | | **103** | **19** | **7** | **129** |

*(Tiers 3-7 follow similar patterns - see roadmap for totals)*

### Appendix B: Test Execution Schedule

**Daily (Automated CI):**
- Unit tests on every commit
- Integration tests on every PR

**Weekly:**
- Full test suite (unit + integration + system)
- Performance benchmarks
- Coverage reports

**Pre-Release:**
- Full regression suite
- Manual exploratory testing
- Performance validation
- Documentation review

---

## Advanced Mode Testing (Phase 3-4)

### Robot/Link/Joint Unit Tests

**Test Count:** ~30 tests

#### Robot Class Tests

```cpp
TEST(RobotTest, ForwardKinematics2LinkArm) {
    // Create 2-link planar arm
    Robot robot = CreateTestArm();
    robot.setJointAngle("joint1", M_PI / 4);  // 45 degrees
    robot.setJointAngle("joint2", M_PI / 4);  // 45 degrees
    
    robot.updateForwardKinematics();
    
    Vector3r end_effector_pose = robot.getLink("end_effector")->getPose().position;
    EXPECT_NEAR(end_effector_pose.x(), 1.414, 0.01);  // sqrt(2)
    EXPECT_NEAR(end_effector_pose.y(), 1.414, 0.01);
}

TEST(RobotTest, SensorAttachmentToLink) {
    Robot robot = CreateTestRobot();
    IMUSensor imu = CreateTestIMU();
    
    robot.attachSensor("link2", imu, Vector3r(0.1, 0, 0));
    
    robot.updateForwardKinematics();
    Vector3r imu_world_pose = robot.getSensorWorldPose("imu");
    
    ASSERT_TRUE(imu_world_pose.norm() > 0);  // Non-zero pose
}

TEST(RobotTest, ActuatorAttachmentToJoint) {
    Robot robot = CreateTestRobot();
    ServoActuator servo = CreateTestServo();
    
    robot.attachActuator("joint1", servo);
    servo.setTargetAngle(M_PI / 2);
    
    robot.update(0.1);  // 100ms
    
    EXPECT_NEAR(robot.getJointAngle("joint1"), M_PI / 2, 0.1);
}
```

#### Link Class Tests

```cpp
TEST(LinkTest, CenterOfMassOffset) {
    Link link;
    link.setMass(10.0);
    link.setCenterOfMass(Vector3r(0.5, 0, 0));  // COM offset
    
    link.applyForce(Vector3r(0, 0, 10), Vector3r::Zero());  // Force at origin
    
    Vector3r torque = link.getNetTorque();
    EXPECT_NEAR(torque.y(), 5.0, 0.01);  // Torque from COM offset
}

TEST(LinkTest, InertiaTensorRotation) {
    Link link;
    link.setInertia(Matrix3x3r::Identity() * 5.0);
    
    Vector3r angular_velocity(1, 0, 0);
    Vector3r angular_momentum = link.computeAngularMomentum(angular_velocity);
    
    EXPECT_NEAR(angular_momentum.x(), 5.0, 0.01);
}

TEST(LinkTest, CollisionDetection) {
    Link link1, link2;
    link1.setCollisionGeometry(CollisionGeometry::Box(Vector3r(1, 1, 1)));
    link2.setCollisionGeometry(CollisionGeometry::Sphere(0.5));
    
    link1.setPose(Pose(Vector3r::Zero(), Quaternionr::Identity()));
    link2.setPose(Pose(Vector3r(0.5, 0, 0), Quaternionr::Identity()));
    
    ASSERT_TRUE(link1.checkCollision(link2));
}
```

#### Joint Class Tests

```cpp
TEST(JointTest, RevoluteAngleLimits) {
    Joint::Revolute joint;
    joint.setLimits(-M_PI / 2, M_PI / 2);  // ±90 degrees
    
    joint.setAngle(M_PI);  // Try to set beyond limit
    
    EXPECT_NEAR(joint.getAngle(), M_PI / 2, 0.01);  // Clamped to upper limit
}

TEST(JointTest, PrismaticPositionLimits) {
    Joint::Prismatic joint;
    joint.setLimits(0.0, 1.0);  // 0-1 meter range
    
    joint.setPosition(1.5);  // Try to set beyond limit
    
    EXPECT_NEAR(joint.getPosition(), 1.0, 0.01);  // Clamped
}

TEST(JointTest, JointDamping) {
    Joint::Revolute joint;
    joint.setDamping(0.5);
    
    joint.setAngularVelocity(10.0);  // High velocity
    float damping_torque = joint.computeDampingTorque();
    
    EXPECT_NEAR(damping_torque, -5.0, 0.01);  // Opposes motion
}

TEST(JointTest, ContinuousJointNoLimits) {
    Joint::Continuous joint;
    
    joint.setAngle(3 * M_PI);  // Beyond 360 degrees
    
    // Should wrap or allow unlimited rotation
    EXPECT_TRUE(std::isfinite(joint.getAngle()));
}
```

### RobotParser Tests

**Test Count:** ~25 tests

```cpp
TEST(RobotParserTest, ParseBasicRobot) {
    std::string config = R"({
        "PhysicsMode": "Advanced",
        "Robot": {
            "Links": [
                {"name": "base", "mass": 1.0, "inertia": [1,1,1]}
            ],
            "Joints": [
                {"name": "joint1", "type": "Fixed", "parent": "base"}
            ]
        }
    })";
    
    RobotParser parser;
    Robot robot = parser.parse(config);
    
    ASSERT_EQ(robot.getLinkCount(), 1);
    ASSERT_EQ(robot.getJointCount(), 1);
}

TEST(RobotParserTest, ModeDetection) {
    std::string simple_config = R"({"PhysicsMode": "Simple"})";
    std::string advanced_config = R"({"PhysicsMode": "Advanced"})";
    std::string no_mode_config = R"({})";
    
    RobotParser parser;
    
    EXPECT_EQ(parser.detectMode(simple_config), PhysicsMode::Simple);
    EXPECT_EQ(parser.detectMode(advanced_config), PhysicsMode::Advanced);
    EXPECT_EQ(parser.detectMode(no_mode_config), PhysicsMode::Simple);  // Default
}

TEST(RobotParserTest, ParseJointTypes) {
    std::string config = R"({
        "Joints": [
            {"name": "j1", "type": "Fixed"},
            {"name": "j2", "type": "Revolute", "axis": [0,0,1], "limits": [-1.57, 1.57]},
            {"name": "j3", "type": "Continuous", "axis": [1,0,0]},
            {"name": "j4", "type": "Prismatic", "axis": [0,1,0], "limits": [0, 1]}
        ]
    })";
    
    RobotParser parser;
    auto joints = parser.parseJoints(config);
    
    ASSERT_EQ(joints.size(), 4);
    EXPECT_EQ(joints[0]->getType(), JointType::Fixed);
    EXPECT_EQ(joints[1]->getType(), JointType::Revolute);
    EXPECT_EQ(joints[2]->getType(), JointType::Continuous);
    EXPECT_EQ(joints[3]->getType(), JointType::Prismatic);
}
```

### Multi-Body Physics Integration Tests

**Test Count:** ~20 tests

```cpp
TEST(MultiBodyPhysicsTest, TwoLinkArmDynamics) {
    Robot robot = CreateTwoLinkArm();
    PhysicsBody physics_body;
    physics_body.setRobot(robot);
    
    // Apply torque to first joint
    robot.getJoint("joint1")->applyTorque(10.0);
    
    physics_body.update(0.01);  // 10ms timestep
    
    float angular_velocity = robot.getJoint("joint1")->getAngularVelocity();
    EXPECT_GT(angular_velocity, 0);  // Should start rotating
}

TEST(MultiBodyPhysicsTest, JointConstraintMaintained) {
    Robot robot = CreateTestRobot();
    PhysicsBody physics_body;
    physics_body.setRobot(robot);
    
    // Apply large force to link
    robot.getLink("link2")->applyForce(Vector3r(1000, 0, 0), Vector3r::Zero());
    
    for (int i = 0; i < 100; i++) {
        physics_body.update(0.01);
    }
    
    // Joint should maintain constraint
    float joint_error = robot.computeJointConstraintError("joint1");
    EXPECT_LT(joint_error, 0.01);  // <1cm error
}

TEST(MultiBodyPhysicsTest, CollisionBetweenLinks) {
    Robot robot = CreateSelfCollidingRobot();
    PhysicsBody physics_body;
    physics_body.setRobot(robot);
    
    robot.setJointAngle("joint1", M_PI);  // Fold robot
    
    physics_body.update(0.01);
    
    ASSERT_TRUE(physics_body.hasCollision());
}
```

### Mode Integration Tests

**Test Count:** ~20 tests

```cpp
TEST(ModeIntegrationTest, SwitchBetweenModes) {
    // Load Simple mode vehicle
    auto simple_vehicle = LoadConfig("simple_quadrotor.json");
    ASSERT_EQ(simple_vehicle->getPhysicsMode(), PhysicsMode::Simple);
    
    // Load Advanced mode vehicle
    auto advanced_vehicle = LoadConfig("advanced_quadrotor.json");
    ASSERT_EQ(advanced_vehicle->getPhysicsMode(), PhysicsMode::Advanced);
    
    // Both should simulate correctly
    simple_vehicle->update(0.01);
    advanced_vehicle->update(0.01);
    
    ASSERT_TRUE(simple_vehicle->isStable());
    ASSERT_TRUE(advanced_vehicle->isStable());
}

TEST(ModeIntegrationTest, MixedModeSimulation) {
    Simulation sim;
    
    // Add Simple mode quadrotor
    auto quad1 = sim.addVehicle("simple_quadrotor.json");
    ASSERT_EQ(quad1->getPhysicsMode(), PhysicsMode::Simple);
    
    // Add Advanced mode robot arm
    auto arm = sim.addVehicle("robot_arm.json");
    ASSERT_EQ(arm->getPhysicsMode(), PhysicsMode::Advanced);
    
    // Run simulation
    for (int i = 0; i < 1000; i++) {
        sim.update(0.01);
    }
    
    // Both should function correctly
    ASSERT_TRUE(quad1->isActive());
    ASSERT_TRUE(arm->isActive());
}

TEST(ModeIntegrationTest, PerformanceComparison) {
    auto simple_config = "simple_quadrotor.json";
    auto advanced_config = "advanced_quadrotor.json";  // Equivalent config
    
    auto simple_time = BenchmarkSimulation(simple_config, 10000);  // 10k steps
    auto advanced_time = BenchmarkSimulation(advanced_config, 10000);
    
    float overhead = (advanced_time - simple_time) / simple_time;
    
    EXPECT_LT(overhead, 0.15);  // <15% overhead for Advanced mode
}
```

---

### Appendix C: Bug Severity Definitions

**Critical:**
- Crash or data loss
- Security vulnerability
- Complete feature failure
- **SLA:** Fix within 24 hours

**High:**
- Major feature impairment
- Performance degradation >20%
- Incorrect results
- **SLA:** Fix within 1 week

**Medium:**
- Minor feature impairment
- Workaround available
- **SLA:** Fix within 2 weeks

**Low:**
- Cosmetic issues
- Enhancement requests
- **SLA:** Backlog

---

## Dual-Mode Test Summary

| Mode | Unit Tests | Integration Tests | System Tests | Performance Tests | Total |
|------|------------|-------------------|--------------|-------------------|-------|
| **Simple** | 35 | 10 | 3 | 2 | **50** |
| **Advanced** | 55 | 15 | 6 | 4 | **80** |
| **Integration** | - | 15 | 3 | 2 | **20** |
| **TOTAL** | **90** | **40** | **12** | **8** | **150** |

**Coverage Goals:**
- Simple Mode: >85%
- Advanced Mode: >80%
- Integration: >75%
- Overall: >80%

---

**Document Status:** ACTIVE TESTING PLAN - Dual-Mode Architecture  
**Last Updated:** January 2025  
**Next Review:** End of Phase 1 (Week 7)

**Document Status:** ACTIVE TESTING PLAN  
**Last Updated:** January 2025  
**Next Review:** End of Tier 1 (Week 4)  
**Maintained By:** QA Lead  
**Version Control:** Git repository `docs/integration/`  
**Test Results:** Jenkins/GitHub Actions dashboards
