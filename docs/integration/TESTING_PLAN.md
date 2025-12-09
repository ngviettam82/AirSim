# Comprehensive Testing Plan - ProjectAirSim Integration

**Version:** 1.0  
**Date:** January 2025  
**Scope:** Battery Sensor, Clock Control, Airspeed Sensor, Radar Sensor

---

## Testing Strategy Overview

### Test Pyramid

```
                    /\
                   /  \
                  / E2E \         5% - End-to-End (10 tests)
                 /______\
                /        \
               /Integration\      15% - Integration (30 tests)
              /____________\
             /              \
            /   Unit Tests   \    80% - Unit Tests (160 tests)
           /__________________\
```

**Total Test Count:** ~200 tests  
**Target Coverage:** >80% code coverage  
**Automation:** 100% automated (CI/CD)

---

## Test Categories

### 1. Unit Tests (160 tests)
- Test individual classes/methods in isolation
- Mock external dependencies
- Fast execution (<1ms per test)
- High code coverage (>90%)

### 2. Integration Tests (30 tests)
- Test component interactions
- Python ↔ C++ communication
- RPC layer functionality
- Multi-vehicle scenarios

### 3. End-to-End Tests (10 tests)
- Complete user scenarios
- Full stack (Python → RPC → C++ → Unreal)
- Real-world use cases
- Performance validation

---

## Battery Sensor Testing

### Unit Tests (60 tests)

#### Test Suite 1: BatteryData Structure
**File:** `AirLibUnitTests/BatteryDataTests.hpp`  
**Test Count:** 8

```cpp
TEST(BatteryDataTests, DefaultConstructor) {
    BatteryData data;
    EXPECT_EQ(data.battery_pct_remaining, 100.0f);
    EXPECT_EQ(data.battery_charge_state, "OK");
}

TEST(BatteryDataTests, MsgpackSerialization) {
    BatteryData data;
    data.battery_pct_remaining = 75.5f;
    // Serialize and deserialize
    // Verify all fields match
}

TEST(BatteryDataTests, FieldRanges) {
    BatteryData data;
    // Test boundary values
    // 0%, 50%, 100%
}
```

**Test Cases:**
1. Default constructor initializes correctly
2. Msgpack serialization/deserialization
3. Field ranges validated
4. Copy constructor works
5. Assignment operator works
6. Equality comparison
7. JSON conversion (if supported)
8. ToString() method (if supported)

---

#### Test Suite 2: Simple Discharge Mode
**File:** `AirLibUnitTests/BatterySimpleDischargeTests.hpp`  
**Test Count:** 15

```cpp
TEST(BatterySimpleDischarge, ConstantDrainRate) {
    BatterySensor sensor;
    BatterySettings settings;
    settings.mode = BatteryMode::SimpleDischarge;
    settings.battery_drain_rate = 1.0f;  // 1% per second
    settings.total_capacity_joules = 36000.0f;
    sensor.setSettings(settings);
    
    // Simulate 10 seconds
    for (int i = 0; i < 10; ++i) {
        sensor.update(1.0);
    }
    
    const auto& data = sensor.getBatteryData();
    EXPECT_NEAR(data.battery_pct_remaining, 90.0f, 0.1f);
}

TEST(BatterySimpleDischarge, ZeroDrainRate) {
    // Test no discharge with 0% rate
    // Battery should remain at 100%
}

TEST(BatterySimpleDischarge, VariableDrainRate) {
    // Change drain rate mid-simulation
    // Verify discharge rate changes immediately
}

TEST(BatterySimpleDischarge, FullDepletion) {
    // Drain battery to 0%
    // Verify it stops at 0%, doesn't go negative
}

TEST(BatterySimpleDischarge, VeryHighDrainRate) {
    // Set 50% per second
    // Verify it empties in 2 seconds
}
```

**Test Cases:**
1. Constant drain rate over time
2. Zero drain rate (no discharge)
3. Variable drain rate changes
4. Full depletion to 0%
5. Very high drain rate (50%/sec)
6. Very low drain rate (0.01%/sec)
7. SetDrainRate() method
8. GetDrainRate() method
9. Negative drain rate rejected
10. Drain rate >100%/sec rejected
11. Pause mid-discharge
12. Resume after pause
13. Time delta variations (0.001s to 1s)
14. Long simulation (1 hour sim time)
15. Fractional percentages (e.g., 75.5%)

---

#### Test Suite 3: Energy Consumption Mode
**File:** `AirLibUnitTests/BatteryEnergyConsumptionTests.hpp`  
**Test Count:** 15

```cpp
TEST(BatteryEnergyConsumption, ConstantPower) {
    BatterySensor sensor;
    BatterySettings settings;
    settings.mode = BatteryMode::EnergyConsumption;
    settings.total_capacity_joules = 36000.0f;
    settings.power_coefficient = 1.0f;
    sensor.setSettings(settings);
    
    // Apply 100W for 10 seconds = 1000J consumed
    sensor.setPowerConsumption(100.0f);
    sensor.update(10.0);
    
    // 1000J / 36000J = 2.78% consumed
    // Remaining: 97.22%
    const auto& data = sensor.getBatteryData();
    EXPECT_NEAR(data.battery_pct_remaining, 97.22f, 0.1f);
}

TEST(BatteryEnergyConsumption, ZeroPower) {
    // No power consumption
    // Battery should remain at 100%
}

TEST(BatteryEnergyConsumption, VariablePower) {
    // Change power consumption mid-simulation
    // 100W for 5s, then 200W for 5s
}

TEST(BatteryEnergyConsumption, PowerCoefficient) {
    // Test power coefficient = 2.0
    // Should discharge 2x faster
}
```

**Test Cases:**
1. Constant power consumption
2. Zero power (no discharge)
3. Variable power changes
4. Power coefficient scaling
5. Very high power (10kW)
6. Very low power (1W)
7. Negative power (charging - if supported)
8. Full depletion with power load
9. Power consumption spikes
10. Realistic flight profile power
11. Idle power consumption
12. Hover power vs forward flight
13. Energy calculation accuracy
14. Joules to Wh conversion
15. Long duration flight simulation

---

#### Test Suite 4: Health Status
**File:** `AirLibUnitTests/BatteryHealthStatusTests.hpp`  
**Test Count:** 12

```cpp
TEST(BatteryHealthStatus, OkToLow) {
    BatterySensor sensor;
    // Set to 30% - should transition to LOW
    sensor.setBatteryRemaining(30.0f);
    sensor.update(0.0);
    EXPECT_EQ(sensor.getBatteryData().battery_charge_state, "LOW");
}

TEST(BatteryHealthStatus, LowToCritical) {
    // Set to 15% - should transition to CRITICAL
}

TEST(BatteryHealthStatus, CriticalTo Zero) {
    // Set to 0% - should remain CRITICAL
}

TEST(BatteryHealthStatus, UnhealthyFlagOverride) {
    // Set unhealthy flag
    // Should override percentage-based status
}
```

**Test Cases:**
1. OK → LOW transition (at 30%)
2. LOW → CRITICAL transition (at 15%)
3. CRITICAL → 0% (remains CRITICAL)
4. Unhealthy flag overrides percentage
5. Custom threshold configuration
6. Health status persistence
7. Health status with discharge
8. SetBatteryHealthStatus() method
9. Healthy flag true/false
10. Boundary testing (29.9%, 30.0%, 30.1%)
11. Health status in msgpack
12. Health status string values

---

#### Test Suite 5: Time Remaining Calculation
**File:** `AirLibUnitTests/BatteryTimeRemainingTests.hpp`  
**Test Count:** 10

```cpp
TEST(BatteryTimeRemaining, SimpleDischargeMode) {
    BatterySensor sensor;
    // 50% remaining, 1% per second drain
    // Should be 50 seconds remaining
}

TEST(BatteryTimeRemaining, EnergyConsumptionMode) {
    // 18000J remaining, 100W power
    // Should be 180 seconds remaining
}

TEST(BatteryTimeRemaining, ZeroDrainRate) {
    // Should return UINT32_MAX (infinite)
}

TEST(BatteryTimeRemaining, ZeroPower) {
    // Should return UINT32_MAX (infinite)
}
```

**Test Cases:**
1. SimpleDischarge mode calculation
2. EnergyConsumption mode calculation
3. Zero drain rate (infinite)
4. Zero power (infinite)
5. Accuracy within 1 second
6. Very long times (>1 year)
7. Very short times (<1 second)
8. Uint32 overflow handling
9. Floating point precision
10. Dynamic recalculation as power changes

---

### Integration Tests (15 tests)

#### Test Suite 6: Python-C++ Communication
**File:** `PythonClient/tests/test_battery_integration.py`  
**Test Count:** 10

```python
def test_getBatteryData():
    client = airsim.MultirotorClient()
    battery = client.getBatteryData()
    assert isinstance(battery, airsim.BatteryData)
    assert 0 <= battery.battery_pct_remaining <= 100

def test_setBatteryRemaining():
    client = airsim.MultirotorClient()
    client.setBatteryRemaining(75.0)
    battery = client.getBatteryData()
    assert battery.battery_pct_remaining == 75.0

def test_vehicleName_parameter():
    # Test with specific vehicle name
    battery = client.getBatteryData(vehicle_name="Drone1")
    assert battery is not None

def test_sensorName_parameter():
    # Test with specific sensor name
    battery = client.getBatteryData(sensor_name="Battery1")
    assert battery is not None

def test_msgpack_serialization():
    # Verify all fields serialize correctly
    pass
```

**Test Cases:**
1. getBatteryData() returns valid data
2. setBatteryRemaining() works
3. getBatteryDrainRate() works
4. setBatteryDrainRate() works
5. setBatteryHealthStatus() works
6. vehicle_name parameter routing
7. sensor_name parameter routing
8. Empty string parameters (default vehicle)
9. Invalid vehicle name error handling
10. Msgpack serialization complete

---

#### Test Suite 7: Multi-Vehicle Battery
**File:** `PythonClient/tests/test_battery_multivehicle.py`  
**Test Count:** 5

```python
def test_independent_battery_states():
    client = airsim.MultirotorClient()
    
    # Set Drone1 to 75%
    client.setBatteryRemaining(75.0, vehicle_name="Drone1")
    
    # Set Drone2 to 50%
    client.setBatteryRemaining(50.0, vehicle_name="Drone2")
    
    # Verify independence
    bat1 = client.getBatteryData(vehicle_name="Drone1")
    bat2 = client.getBatteryData(vehicle_name="Drone2")
    
    assert bat1.battery_pct_remaining == 75.0
    assert bat2.battery_pct_remaining == 50.0
```

**Test Cases:**
1. Independent battery states per vehicle
2. Concurrent access from multiple clients
3. Different discharge modes per vehicle
4. Different drain rates per vehicle
5. Thread safety verification

---

### End-to-End Tests (5 tests)

#### Test Suite 8: Realistic Flight Scenarios
**File:** `PythonClient/tests/test_battery_e2e.py`  
**Test Count:** 5

```python
def test_battery_depletes_during_flight():
    client = airsim.MultirotorClient()
    
    # Get initial battery
    battery_start = client.getBatteryData()
    
    # Fly waypoint mission (5 minutes)
    waypoints = [...]
    client.moveOnPathAsync(waypoints, velocity=5).join()
    
    # Get final battery
    battery_end = client.getBatteryData()
    
    # Verify battery depleted
    assert battery_end.battery_pct_remaining < battery_start.battery_pct_remaining
    
    # Verify reasonable depletion (not too much, not too little)
    depletion = battery_start.battery_pct_remaining - battery_end.battery_pct_remaining
    assert 5 < depletion < 20  # Reasonable range
```

**Test Cases:**
1. Battery depletes during flight
2. Hover uses less power than forward flight
3. Aggressive maneuvers use more power
4. Emergency landing triggered at low battery
5. Battery status visible in telemetry

---

## Clock Control Testing

### Unit Tests (50 tests)

#### Test Suite 9: Clock Type and Time
**File:** `AirLibUnitTests/ClockTypeTests.hpp`  
**Test Count:** 8

```cpp
TEST(ClockType, GetSimClockType) {
    // Should return "steppable" or "real-time"
    std::string type = ClockBase::getSimClockType();
    EXPECT_TRUE(type == "steppable" || type == "real-time");
}

TEST(ClockType, GetSimTime) {
    uint64_t time1 = ClockBase::getSimTime();
    // Wait some simulation time
    uint64_t time2 = ClockBase::getSimTime();
    EXPECT_GT(time2, time1);  // Time should increase
}

TEST(ClockType, TimeMonotonic) {
    // Time should never decrease
    uint64_t prev_time = ClockBase::getSimTime();
    for (int i = 0; i < 1000; ++i) {
        uint64_t curr_time = ClockBase::getSimTime();
        EXPECT_GE(curr_time, prev_time);
        prev_time = curr_time;
    }
}
```

**Test Cases:**
1. getSimClockType() returns valid string
2. getSimTime() returns nanoseconds
3. Time monotonically increases
4. Time advances during simulation
5. Time freezes when paused
6. Nanosecond precision
7. No uint64 overflow for reasonable times
8. Time consistent across queries

---

#### Test Suite 10: Clock Speed Control
**File:** `AirLibUnitTests/ClockSpeedTests.hpp`  
**Test Count:** 12

```cpp
TEST(ClockSpeed, DefaultSpeed) {
    // Default should be 1.0x
    EXPECT_EQ(ClockBase::getClockSpeed(), 1.0);
}

TEST(ClockSpeed, DoubleSpeed) {
    ClockBase::setClockSpeed(2.0);
    
    uint64_t start_time = ClockBase::getSimTime();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Real-world 1s
    uint64_t end_time = ClockBase::getSimTime();
    
    // Sim time should have advanced ~2 seconds (in nanos)
    uint64_t elapsed_nanos = end_time - start_time;
    uint64_t expected_nanos = 2000000000ULL;  // 2 seconds
    
    EXPECT_NEAR(elapsed_nanos, expected_nanos, 100000000ULL);  // ±100ms tolerance
}

TEST(ClockSpeed, HalfSpeed) {
    ClockBase::setClockSpeed(0.5);
    // Verify time advances at half speed
}

TEST(ClockSpeed, InvalidSpeed_Negative) {
    // Should reject negative speeds
    EXPECT_THROW(ClockBase::setClockSpeed(-1.0), std::invalid_argument);
}

TEST(ClockSpeed, InvalidSpeed_Zero) {
    // Should reject zero speed
    EXPECT_THROW(ClockBase::setClockSpeed(0.0), std::invalid_argument);
}
```

**Test Cases:**
1. Default speed is 1.0x
2. Set 2x speed (doubles time advancement)
3. Set 0.5x speed (halves time advancement)
4. Set 10x speed (maximum practical)
5. Set 0.1x speed (minimum practical)
6. Negative speed rejected
7. Zero speed rejected
8. Very high speed (100x) handled
9. Speed changes take effect immediately
10. Speed persists across operations
11. Speed affects all simulation
12. Performance impact acceptable

---

#### Test Suite 11: Pause/Resume
**File:** `AirLibUnitTests/ClockPauseTests.hpp`  
**Test Count:** 10

```cpp
TEST(ClockPause, PauseStopsTime) {
    uint64_t time_before_pause = ClockBase::getSimTime();
    
    ClockBase::pause();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    uint64_t time_after_pause = ClockBase::getSimTime();
    
    EXPECT_EQ(time_before_pause, time_after_pause);  // Time should not advance
}

TEST(ClockPause, ResumeStartsTime) {
    ClockBase::pause();
    uint64_t time_at_pause = ClockBase::getSimTime();
    
    ClockBase::resume();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    uint64_t time_after_resume = ClockBase::getSimTime();
    
    EXPECT_GT(time_after_resume, time_at_pause);  // Time should advance again
}

TEST(ClockPause, IsPausedState) {
    EXPECT_FALSE(ClockBase::isPaused());
    
    ClockBase::pause();
    EXPECT_TRUE(ClockBase::isPaused());
    
    ClockBase::resume();
    EXPECT_FALSE(ClockBase::isPaused());
}
```

**Test Cases:**
1. Pause stops time advancement
2. Resume continues time from pause point
3. isPaused() reflects state accurately
4. Multiple pause/resume cycles
5. Pause while already paused (idempotent)
6. Resume while already running (idempotent)
7. Pause returns state string
8. Resume returns state string
9. Vehicles stop moving when paused
10. Physics frozen when paused

---

#### Test Suite 12: Stepped Execution
**File:** `AirLibUnitTests/ClockSteppedTests.hpp`  
**Test Count:** 20

```cpp
TEST(ClockStepped, ContinueForTime_Wait) {
    ClockBase::pause();
    uint64_t start_time = ClockBase::getSimTime();
    
    // Continue for 1 second (1e9 nanoseconds)
    uint64_t result_time = ClockBase::continueForTime(1000000000ULL, true);
    
    uint64_t end_time = ClockBase::getSimTime();
    
    // Should have advanced 1 second
    EXPECT_NEAR(end_time - start_time, 1000000000ULL, 1000000ULL);  // ±1ms
    
    // Should be paused again
    EXPECT_TRUE(ClockBase::isPaused());
    
    // Return value should match end time
    EXPECT_EQ(result_time, end_time);
}

TEST(ClockStepped, ContinueForTime_NoWait) {
    ClockBase::pause();
    uint64_t start_time = ClockBase::getSimTime();
    
    // Continue for 1 second, don't wait
    uint64_t result_time = ClockBase::continueForTime(1000000000ULL, false);
    
    // Should return immediately (within 10ms)
    uint64_t immediate_time = ClockBase::getSimTime();
    EXPECT_LT(immediate_time - start_time, 10000000ULL);  // <10ms elapsed
    
    // Return value should be the target time
    EXPECT_EQ(result_time, start_time + 1000000000ULL);
}

TEST(ClockStepped, ContinueForSteps_Exact) {
    ClockBase::pause();
    
    // Continue for exactly 100 steps
    ClockBase::continueForSteps(100, true);
    
    // Verify 100 physics steps occurred
    // (implementation-dependent verification)
}

TEST(ClockStepped, ContinueForSingleStep) {
    ClockBase::pause();
    uint64_t start_time = ClockBase::getSimTime();
    
    ClockBase::continueForSingleStep(true);
    
    uint64_t end_time = ClockBase::getSimTime();
    
    // Should have advanced by one time step
    uint64_t step_duration = end_time - start_time;
    EXPECT_GT(step_duration, 0);
    EXPECT_LT(step_duration, 100000000ULL);  // <100ms (reasonable step)
}
```

**Test Cases:**
1. continueForTime with wait=true blocks
2. continueForTime with wait=false immediate
3. continueForTime accuracy within 1ms
4. continueForTime auto-pauses after
5. continueForSteps exact count
6. continueForSteps with wait=true blocks
7. continueForSteps with wait=false immediate
8. continueForSteps auto-pauses after
9. continueForSingleStep executes one step
10. continueForSingleStep with wait
11. continueForSingleStep without wait
12. Long duration (1 hour sim time)
13. Very short duration (1ms sim time)
14. Zero duration (should return immediately)
15. Negative duration rejected
16. Concurrent continueFor calls (should queue)
17. ContinueFor during running sim (should pause first)
18. Return value matches end time
19. Time advancement deterministic
20. Physics deterministic during stepped execution

---

### Integration Tests (10 tests)

#### Test Suite 13: Clock + Battery Integration
**File:** `PythonClient/tests/test_clock_battery_integration.py`  
**Test Count:** 5

```python
def test_battery_with_clock_speed():
    client = airsim.MultirotorClient()
    
    # Set 2x clock speed
    client.setClockSpeed(2.0)
    
    # Monitor battery for 10 real-world seconds
    battery_start = client.getBatteryData()
    time.sleep(10)
    battery_end = client.getBatteryData()
    
    # Battery should have drained for 20 sim seconds
    # (approximately 2x normal depletion)
    depletion = battery_start.battery_pct_remaining - battery_end.battery_pct_remaining
    
    # Verify 2x depletion compared to normal speed
    # (implementation-dependent exact values)

def test_battery_frozen_when_paused():
    client = airsim.MultirotorClient()
    
    battery_before = client.getBatteryData()
    
    client.pause()
    time.sleep(5)
    
    battery_after = client.getBatteryData()
    
    assert battery_before.battery_pct_remaining == battery_after.battery_pct_remaining
```

**Test Cases:**
1. Battery drains faster with 2x clock speed
2. Battery drains slower with 0.5x clock speed
3. Battery frozen when paused
4. Battery resumes depleting after resume
5. Stepped execution affects battery proportionally

---

#### Test Suite 14: Python-C++ Clock Communication
**File:** `PythonClient/tests/test_clock_rpc.py`  
**Test Count:** 5

```python
def test_all_clock_methods_callable():
    client = airsim.MultirotorClient()
    
    # Test all 9 methods
    clock_type = client.getSimClockType()
    assert isinstance(clock_type, str)
    
    sim_time = client.getSimTime()
    assert isinstance(sim_time, int)
    
    client.setClockSpeed(2.0)
    client.pause()
    client.resume()
    
    result = client.continueForTime(1000000000, wait=True)
    assert isinstance(result, int)
    
    result = client.continueForSteps(10, wait=True)
    assert isinstance(result, int)
    
    result = client.continueForSingleStep(wait=True)
    assert isinstance(result, int)
    
    is_paused = client.isPaused()
    assert isinstance(is_paused, bool)
```

**Test Cases:**
1. All 9 methods callable from Python
2. Return types correct
3. Parameter passing works
4. Error handling for invalid params
5. Backward compatibility with old methods

---

### End-to-End Tests (3 tests)

#### Test Suite 15: Complete Clock Control Scenarios
**File:** `PythonClient/tests/test_clock_e2e.py`  
**Test Count:** 3

```python
def test_accelerated_flight_mission():
    """Run a mission at 10x speed, verify it completes 10x faster"""
    client = airsim.MultirotorClient()
    
    # Set 10x speed
    client.setClockSpeed(10.0)
    
    # Fly waypoint mission
    start_real = time.time()
    waypoints = [...]  # 5 minute mission at 1x speed
    client.moveOnPathAsync(waypoints, velocity=5).join()
    end_real = time.time()
    
    # Should complete in ~30 seconds real-time (300s / 10)
    duration_real = end_real - start_real
    assert 25 < duration_real < 35  # ±5s tolerance

def test_pause_resume_mission():
    """Pause mid-mission, verify drone stops, resume and complete"""
    client = airsim.MultirotorClient()
    
    # Start mission
    waypoints = [...]
    future = client.moveOnPathAsync(waypoints, velocity=5)
    
    # Wait 5 seconds, then pause
    time.sleep(5)
    client.pause()
    
    # Get position
    pos_at_pause = client.getMultirotorState().kinematics_estimated.position
    
    # Wait 5 more seconds
    time.sleep(5)
    
    # Position should be unchanged
    pos_after_wait = client.getMultirotorState().kinematics_estimated.position
    assert pos_at_pause == pos_after_wait
    
    # Resume
    client.resume()
    
    # Mission should complete
    future.join()

def test_stepped_debugging_scenario():
    """Use stepped execution for frame-by-frame debugging"""
    client = airsim.MultirotorClient()
    
    client.pause()
    
    # Execute 100 steps one at a time
    for i in range(100):
        client.continueForSingleStep(wait=True)
        
        # Get state after each step
        state = client.getMultirotorState()
        
        # Verify state changes
        # (implementation-dependent checks)
```

---

## Airspeed Sensor Testing

### Unit Tests (30 tests)

*Test suites for airspeed sensor structure, noise model, configuration, API methods*

---

## Radar Sensor Testing

### Unit Tests (20 tests)

*Test suites for radar detection, tracking, masking, RCS calculation*

---

## Performance Testing

### Benchmark Suite
**File:** `AirLibUnitTests/PerformanceBenchmarks.hpp`  
**Test Count:** 10

```cpp
BENCHMARK(BatteryUpdate) {
    BatterySensor sensor;
    benchmark::DoNotOptimize(sensor.update(0.01));
}

BENCHMARK(ClockGetSimTime) {
    uint64_t time = ClockBase::getSimTime();
    benchmark::DoNotOptimize(time);
}
```

**Benchmarks:**
1. BatterySensor update time (<0.1ms)
2. Clock getSimTime overhead (<0.01ms)
3. Clock pause/resume overhead (<1ms)
4. RPC call latency (<10ms)
5. Python → C++ round trip (<20ms)
6. Multi-vehicle simulation (100 vehicles)
7. Memory usage per sensor (<1KB)
8. Memory leaks (zero leaks)
9. CPU usage during flight (<10% increase)
10. Full system stress test

**Acceptance Criteria:**
- All benchmarks meet performance targets
- No memory leaks detected
- CPU overhead acceptable
- Scalable to 10+ vehicles

---

## Continuous Integration

### CI Pipeline

```yaml
name: ProjectAirSim Integration Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: [windows-latest, ubuntu-latest]
    
    steps:
      - name: Checkout
      - name: Build C++
        run: build.cmd  # or build.sh
      
      - name: Run C++ Unit Tests
        run: AirLibUnitTests.exe
      
      - name: Install Python Dependencies
        run: pip install -r requirements.txt
      
      - name: Run Python Tests
        run: pytest PythonClient/tests/
      
      - name: Generate Coverage Report
        run: pytest --cov=cosysairsim --cov-report=html
      
      - name: Upload Coverage
        uses: codecov/codecov-action@v2
```

---

## Test Execution Schedule

### Daily (Automated)
- All unit tests (160)
- Fast integration tests (20)
- **Total:** ~180 tests, ~10 minutes

### Pre-Commit (Local)
- Modified module unit tests
- Related integration tests
- **Total:** ~20-50 tests, ~2 minutes

### Weekly (CI)
- All tests (200)
- Performance benchmarks (10)
- Memory leak checks
- **Total:** ~210 tests, ~30 minutes

### Release (Manual)
- All tests (200)
- All benchmarks (10)
- End-to-end scenarios (10)
- Multi-platform testing
- Stress testing
- **Total:** ~220+ tests, ~2 hours

---

## Test Coverage Goals

### Code Coverage Targets
- **Overall:** >80%
- **Critical Paths:** >95% (battery update, clock control)
- **API Layer:** >90%
- **Configuration Parsing:** >85%

### Coverage Tools
- C++: gcov/lcov
- Python: pytest-cov
- Reporting: Codecov

---

## Test Documentation

### Test Case Template

```cpp
/**
 * @test BatterySimpleDischarge_ConstantRate
 * @brief Verify battery depletes at constant rate in simple discharge mode
 * 
 * @details
 * Sets battery to simple discharge mode with 1% per second drain rate.
 * Simulates 10 seconds and verifies battery is at 90%.
 * 
 * @pre Battery initialized with 100% charge
 * @post Battery at 90% after 10 seconds
 * 
 * @acceptance Battery percentage within 0.1% of expected value
 */
TEST(BatterySimpleDischarge, ConstantRate) {
    // Test implementation
}
```

---

## Acceptance Criteria

- [ ] All 200 tests implemented
- [ ] All tests passing (0 failures)
- [ ] Code coverage >80%
- [ ] Performance benchmarks met
- [ ] No memory leaks
- [ ] CI/CD pipeline configured
- [ ] Test documentation complete
- [ ] Coverage reports generated

---

**Testing Plan Version:** 1.0  
**Last Updated:** January 2025  
**Total Test Count:** ~200 tests  
**Estimated Test Development Time:** 4 weeks
