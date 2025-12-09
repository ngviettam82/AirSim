# Testing Strategy

## Comprehensive Test Plan for ProjectAirSim Integration

---

## 1. Testing Overview

### Test Pyramid

```
                    ┌─────────────────┐
                   /│   E2E Tests    │\
                  / │   (Manual)     │ \
                 /  │    ~10%        │  \
                /   └─────────────────┘   \
               /   ┌───────────────────┐   \
              /   │ Integration Tests │    \
             /    │   (Automated)     │     \
            /     │      ~30%         │      \
           /      └───────────────────┘       \
          /      ┌─────────────────────┐       \
         /      │    Unit Tests       │        \
        /       │   (Automated)       │         \
       /        │       ~60%          │          \
      /         └─────────────────────┘           \
     └─────────────────────────────────────────────┘
```

---

## 2. Unit Tests

### 2.1 Battery Sensor Tests

**File:** `AirLibUnitTests/BatteryTest.hpp`

```cpp
// Test Cases

TEST(BatterySensorTest, InitialState) {
    // Verify battery starts at configured initial charge
    // Verify all fields have sensible defaults
}

TEST(BatterySensorTest, LinearDischarge) {
    // Verify linear discharge over time
    // Verify remaining_percent decreases linearly
    // Verify remaining_wh matches percentage
}

TEST(BatterySensorTest, VoltageBasedDischarge) {
    // Verify voltage curve follows LiPo discharge curve
    // Verify voltage drops as SOC decreases
}

TEST(BatterySensorTest, PhysicsBasedDischarge) {
    // Verify power-based consumption
    // Verify temperature affects capacity
    // Verify internal resistance causes voltage drop
}

TEST(BatterySensorTest, HealthStatusTransitions) {
    // Verify "Normal" -> "Low" at 20%
    // Verify "Low" -> "Critical" at 5%
    // Verify manual status override works
}

TEST(BatterySensorTest, ManualOverride) {
    // Verify setBatteryRemaining works
    // Verify setBatteryDrainRate works
    // Verify setBatteryHealthStatus works
}

TEST(BatterySensorTest, EdgeCases) {
    // Verify 0% doesn't go negative
    // Verify 100% doesn't exceed
    // Verify division by zero protection
}
```

### 2.2 Airspeed Sensor Tests

**File:** `AirLibUnitTests/AirspeedTest.hpp`

```cpp
TEST(AirspeedSensorTest, ZeroVelocity) {
    // Verify IAS = TAS = 0 when stationary
    // Verify differential_pressure = 0
}

TEST(AirspeedSensorTest, ForwardFlight) {
    // Verify IAS calculation from velocity
    // Verify differential_pressure = 0.5 * rho * v^2
}

TEST(AirspeedSensorTest, AltitudeEffect) {
    // Verify TAS > IAS at altitude (lower density)
    // Verify TAS/IAS ratio matches atmosphere model
}

TEST(AirspeedSensorTest, NoiseModel) {
    // Verify noise is added to readings
    // Verify noise follows configured stddev
}
```

### 2.3 Radar Sensor Tests

**File:** `AirLibUnitTests/RadarTest.hpp`

```cpp
TEST(RadarSensorTest, NoTargets) {
    // Verify empty detection list when no objects in FOV
}

TEST(RadarSensorTest, SingleTarget) {
    // Verify single detection for single object
    // Verify range calculation accuracy
    // Verify azimuth/elevation accuracy
}

TEST(RadarSensorTest, MultipleTargets) {
    // Verify all targets detected
    // Verify no duplicate detections
}

TEST(RadarSensorTest, DopplerVelocity) {
    // Verify radial velocity calculation
    // Verify sign (approaching vs receding)
}

TEST(RadarSensorTest, RCSEstimation) {
    // Verify larger objects have larger RCS
    // Verify orientation affects RCS
}

TEST(RadarSensorTest, SignalStrength) {
    // Verify signal decreases with range^4
    // Verify noise floor filtering
}

TEST(RadarSensorTest, FOVLimits) {
    // Verify objects outside FOV not detected
    // Verify edge-of-FOV objects detected
}
```

### 2.4 Clock Control Tests

**File:** `AirLibUnitTests/ClockTest.hpp`

```cpp
TEST(ClockControlTest, SimTimeProgression) {
    // Verify simGetSimTime returns increasing values
}

TEST(ClockControlTest, PauseStopsTime) {
    // Verify time doesn't advance when paused
}

TEST(ClockControlTest, ContinueForTime) {
    // Verify exact time advancement
    // Verify multiple calls accumulate correctly
}

TEST(ClockControlTest, ContinueForSteps) {
    // Verify correct number of physics ticks
    // Verify time matches tick count * dt
}

TEST(ClockControlTest, ContinueUntilTime) {
    // Verify stops at exact target time
    // Verify handles already-past times
}
```

---

## 3. Integration Tests

### 3.1 Battery Integration Tests

**File:** `PythonClient/tests/test_battery_integration.py`

```python
import pytest
import cosysairsim as airsim
import time

class TestBatteryIntegration:
    
    @pytest.fixture
    def client(self):
        client = airsim.MultirotorClient()
        client.confirmConnection()
        yield client
        client.reset()
    
    def test_battery_data_retrieval(self, client):
        """Test that battery data can be retrieved"""
        battery = client.getBatteryData()
        
        assert battery is not None
        assert 0 <= battery.remaining_percent <= 100
        assert battery.voltage > 0
        assert battery.health_status in ["Normal", "Low", "Critical", "Fault"]
    
    def test_battery_drain_during_flight(self, client):
        """Test that battery drains during hover"""
        client.enableApiControl(True)
        client.armDisarm(True)
        
        initial = client.getBatteryData()
        
        # Hover for 10 seconds
        client.takeoffAsync().join()
        client.hoverAsync().join()
        time.sleep(10)
        
        final = client.getBatteryData()
        
        assert final.remaining_percent < initial.remaining_percent
        client.landAsync().join()
    
    def test_set_battery_remaining(self, client):
        """Test manual battery level setting"""
        client.setBatteryRemaining(50.0)
        battery = client.getBatteryData()
        
        assert abs(battery.remaining_percent - 50.0) < 1.0
    
    def test_set_drain_rate(self, client):
        """Test drain rate multiplier"""
        client.setBatteryRemaining(100.0)
        client.setBatteryDrainRate(10.0)  # 10x drain
        
        client.enableApiControl(True)
        client.takeoffAsync().join()
        time.sleep(5)
        
        battery = client.getBatteryData()
        
        # Should have drained significantly with 10x rate
        assert battery.remaining_percent < 90.0
        client.landAsync().join()
    
    def test_health_status_transitions(self, client):
        """Test health status changes at thresholds"""
        client.setBatteryRemaining(100.0)
        assert client.getBatteryData().health_status == "Normal"
        
        client.setBatteryRemaining(15.0)
        assert client.getBatteryData().health_status == "Low"
        
        client.setBatteryRemaining(3.0)
        assert client.getBatteryData().health_status == "Critical"
    
    def test_manual_health_override(self, client):
        """Test manual health status setting"""
        client.setBatteryRemaining(100.0)
        client.setBatteryHealthStatus("Fault")
        
        assert client.getBatteryData().health_status == "Fault"
```

### 3.2 Airspeed Integration Tests

**File:** `PythonClient/tests/test_airspeed_integration.py`

```python
class TestAirspeedIntegration:
    
    def test_airspeed_at_rest(self, client):
        """Test airspeed shows zero when stationary"""
        airspeed = client.getAirspeedData()
        
        assert abs(airspeed.indicated_airspeed) < 1.0
        assert abs(airspeed.true_airspeed) < 1.0
    
    def test_airspeed_during_flight(self, client):
        """Test airspeed increases during forward flight"""
        client.enableApiControl(True)
        client.takeoffAsync().join()
        
        # Fly forward at 10 m/s
        client.moveByVelocityAsync(10, 0, 0, 5).join()
        
        airspeed = client.getAirspeedData()
        
        # Should be close to 10 m/s (with some noise)
        assert 8.0 < airspeed.indicated_airspeed < 12.0
        
        client.landAsync().join()
    
    def test_tas_vs_ias_at_altitude(self, client):
        """Test TAS > IAS at higher altitude"""
        client.enableApiControl(True)
        
        # Fly to high altitude
        client.moveToZAsync(-100, 5).join()
        
        # Fly forward
        client.moveByVelocityAsync(10, 0, 0, 3).join()
        
        airspeed = client.getAirspeedData()
        
        # TAS should be greater than IAS at altitude
        assert airspeed.true_airspeed > airspeed.indicated_airspeed
```

### 3.3 Radar Integration Tests

**File:** `PythonClient/tests/test_radar_integration.py`

```python
class TestRadarIntegration:
    
    def test_radar_data_structure(self, client):
        """Test radar data has correct structure"""
        radar = client.getRadarData()
        
        assert hasattr(radar, 'detections')
        assert hasattr(radar, 'time_stamp')
    
    def test_detection_fields(self, client):
        """Test detection objects have all fields"""
        radar = client.getRadarData()
        
        if len(radar.detections) > 0:
            det = radar.detections[0]
            assert hasattr(det, 'range')
            assert hasattr(det, 'azimuth')
            assert hasattr(det, 'elevation')
            assert hasattr(det, 'radial_velocity')
            assert hasattr(det, 'rcs')
            assert hasattr(det, 'signal_strength')
    
    def test_detection_range_accuracy(self, client):
        """Test that detected range matches actual distance"""
        # This requires a known object at known position
        # Would need custom test environment
        pass
```

### 3.4 Clock Control Integration Tests

**File:** `PythonClient/tests/test_clock_control.py`

```python
class TestClockControl:
    
    def test_sim_time_increases(self, client):
        """Test that sim time increases during simulation"""
        t1 = client.simGetSimTime()
        time.sleep(0.1)
        t2 = client.simGetSimTime()
        
        assert t2 > t1
    
    def test_pause_stops_time(self, client):
        """Test that pause stops sim time"""
        client.simPause(True)
        t1 = client.simGetSimTime()
        time.sleep(0.5)
        t2 = client.simGetSimTime()
        
        assert t2 == t1
        client.simPause(False)
    
    def test_continue_for_time(self, client):
        """Test stepping by specific duration"""
        client.simPause(True)
        t1 = client.simGetSimTime()
        
        # Step 100ms
        duration_ns = 100_000_000
        client.simContinueForTime(duration_ns)
        
        t2 = client.simGetSimTime()
        
        # Should be within 1ms of expected
        assert abs((t2 - t1) - duration_ns) < 1_000_000
        client.simPause(False)
    
    def test_deterministic_stepping(self, client):
        """Test that stepping is deterministic"""
        client.simPause(True)
        
        # Reset to known state
        client.reset()
        client.enableApiControl(True)
        client.armDisarm(True)
        
        # Apply known control
        client.moveByRollPitchYawThrottleAsync(0, 0, 0, 0.5, 0.1)
        client.simContinueForTime(100_000_000)
        
        state1 = client.getMultirotorState()
        
        # Reset and repeat
        client.reset()
        client.enableApiControl(True)
        client.armDisarm(True)
        
        client.moveByRollPitchYawThrottleAsync(0, 0, 0, 0.5, 0.1)
        client.simContinueForTime(100_000_000)
        
        state2 = client.getMultirotorState()
        
        # States should be identical
        assert abs(state1.kinematics_estimated.position.z_val - 
                   state2.kinematics_estimated.position.z_val) < 0.001
```

---

## 4. End-to-End Tests

### 4.1 Battery Management Mission

**Test Scenario:** Complete mission with battery-aware planning

```python
def test_battery_aware_mission():
    """
    Test a mission that uses battery data for planning:
    1. Take off
    2. Fly to waypoints while monitoring battery
    3. Return home when battery < 30%
    4. Land
    """
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    
    waypoints = [
        (10, 0, -5),
        (10, 10, -5),
        (0, 10, -5),
        (0, 0, -5),
    ]
    
    client.takeoffAsync().join()
    
    for wp in waypoints:
        battery = client.getBatteryData()
        
        if battery.remaining_percent < 30:
            print("Low battery - returning home")
            client.moveToPositionAsync(0, 0, -5, 3).join()
            break
        
        client.moveToPositionAsync(wp[0], wp[1], wp[2], 3).join()
    
    client.landAsync().join()
    
    final_battery = client.getBatteryData()
    assert final_battery.remaining_percent > 0
```

### 4.2 Radar-Based Collision Avoidance

**Test Scenario:** Detect and avoid obstacles using radar

```python
def test_radar_collision_avoidance():
    """
    Test radar-based obstacle detection and avoidance:
    1. Take off
    2. Fly forward
    3. Detect obstacle with radar
    4. Stop or avoid
    5. Verify no collision
    """
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    
    client.takeoffAsync().join()
    
    # Fly forward slowly
    velocity = 2.0
    client.moveByVelocityAsync(velocity, 0, 0, 100, drivetrain=airsim.DrivetrainType.ForwardOnly)
    
    collision = False
    while True:
        radar = client.getRadarData()
        
        # Check for obstacles within 10m
        close_detections = [d for d in radar.detections if d.range < 10]
        
        if close_detections:
            print(f"Obstacle detected at {close_detections[0].range}m - stopping")
            client.hoverAsync().join()
            break
        
        info = client.simGetCollisionInfo()
        if info.has_collided:
            collision = True
            break
        
        time.sleep(0.1)
    
    client.landAsync().join()
    assert not collision
```

### 4.3 Stepped Simulation Training

**Test Scenario:** ML-style stepped simulation

```python
def test_stepped_training_loop():
    """
    Test ML training style loop with stepped simulation:
    1. Pause simulation
    2. Get observation
    3. Apply action
    4. Step simulation
    5. Get reward
    6. Repeat
    """
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    
    # Enable stepping mode
    client.simPause(True)
    
    client.armDisarm(True)
    
    episode_length = 100
    step_duration_ns = 10_000_000  # 10ms = 100Hz
    
    total_reward = 0
    
    for step in range(episode_length):
        # Observation
        state = client.getMultirotorState()
        imu = client.getImuData()
        battery = client.getBatteryData()
        
        # Action (simple hover control)
        altitude_error = -5 - state.kinematics_estimated.position.z_val
        throttle = 0.5 + 0.1 * altitude_error
        throttle = max(0, min(1, throttle))
        
        client.moveByRollPitchYawThrottleAsync(0, 0, 0, throttle, 0.01)
        
        # Step
        client.simContinueForTime(step_duration_ns)
        
        # Reward
        new_state = client.getMultirotorState()
        new_altitude = -new_state.kinematics_estimated.position.z_val
        reward = -abs(new_altitude - 5)  # Reward for being at 5m
        total_reward += reward
    
    client.simPause(False)
    client.landAsync().join()
    
    print(f"Total reward: {total_reward}")
    assert total_reward > -500  # Reasonable hover performance
```

---

## 5. Performance Tests

### 5.1 Sensor Update Overhead

```python
def benchmark_sensor_overhead():
    """Measure sensor read time overhead"""
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    iterations = 1000
    
    # Baseline (no sensors)
    start = time.perf_counter()
    for _ in range(iterations):
        pass
    baseline = time.perf_counter() - start
    
    # Battery sensor
    start = time.perf_counter()
    for _ in range(iterations):
        client.getBatteryData()
    battery_time = (time.perf_counter() - start - baseline) / iterations
    
    # Airspeed sensor
    start = time.perf_counter()
    for _ in range(iterations):
        client.getAirspeedData()
    airspeed_time = (time.perf_counter() - start - baseline) / iterations
    
    # Radar sensor
    start = time.perf_counter()
    for _ in range(iterations):
        client.getRadarData()
    radar_time = (time.perf_counter() - start - baseline) / iterations
    
    print(f"Battery read time: {battery_time*1000:.3f}ms")
    print(f"Airspeed read time: {airspeed_time*1000:.3f}ms")
    print(f"Radar read time: {radar_time*1000:.3f}ms")
    
    # Assertions
    assert battery_time < 0.005  # < 5ms
    assert airspeed_time < 0.005
    assert radar_time < 0.010  # Radar may be slower
```

### 5.2 Radar Ray-Casting Performance

```python
def benchmark_radar_performance():
    """Measure radar performance at different resolutions"""
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    iterations = 100
    
    for resolution in [(16, 8), (32, 16), (64, 32), (128, 64)]:
        # Would need to configure radar dynamically
        # This is a placeholder for the concept
        
        start = time.perf_counter()
        for _ in range(iterations):
            client.getRadarData()
        elapsed = time.perf_counter() - start
        
        avg_time = elapsed / iterations
        fps = 1.0 / avg_time
        
        print(f"Resolution {resolution}: {avg_time*1000:.3f}ms ({fps:.1f} FPS)")
```

---

## 6. Test Automation

### 6.1 CI/CD Integration

```yaml
# .github/workflows/test.yml
name: Integration Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: windows-latest
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Setup Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.10'
    
    - name: Install dependencies
      run: |
        pip install pytest numpy
        pip install -e PythonClient/
    
    - name: Build AirLib
      run: build_airsim.bat
    
    - name: Run unit tests
      run: |
        cd AirLibUnitTests/build
        ./AirLibUnitTests.exe
    
    - name: Run Python tests
      run: |
        pytest PythonClient/tests/ -v
```

### 6.2 Test Matrix

| Test Type | Battery | Airspeed | Radar | Clock | Voxel |
|-----------|---------|----------|-------|-------|-------|
| Unit Test | ✓ | ✓ | ✓ | ✓ | ✓ |
| Integration Test | ✓ | ✓ | ✓ | ✓ | ✓ |
| E2E Test | ✓ | ○ | ✓ | ✓ | ○ |
| Performance Test | ✓ | ✓ | ✓ | ✓ | ✓ |

✓ = Required, ○ = Optional

---

## 7. Test Coverage Goals

| Component | Target Coverage |
|-----------|-----------------|
| Battery Sensor | 90% |
| Airspeed Sensor | 85% |
| Radar Sensor | 80% |
| Clock Control | 90% |
| Voxel Grid | 75% |
| RPC Bindings | 80% |
| Python Client | 85% |
| **Overall** | **85%** |

---

## 8. Regression Tests

After each feature is complete, add to regression suite:

1. All unit tests must pass
2. All integration tests must pass
3. Performance benchmarks must not regress by > 10%
4. Memory usage must not increase by > 5%
5. All existing tests must still pass
