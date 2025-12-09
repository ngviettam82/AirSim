# Implementation Task List - Detailed Breakdown

**Project:** ProjectAirSim Integration into Cosys-AirSim  
**Version:** 1.0  
**Date:** January 2025  
**Total Tasks:** 87

---

## Task Organization

- **Priority 0:** Battery Sensor (28 tasks)
- **Priority 1:** Clock Control (24 tasks)
- **Priority 2:** Integration Testing (15 tasks)
- **Priority 3:** Airspeed Sensor (12 tasks)
- **Priority 4:** Radar Sensor (8 tasks - documentation only)

---

## Priority 0: Battery Sensor Implementation

### Phase 1.1: C++ Core Structure (8 tasks)

#### TASK-BAT-001: Create BatteryData.hpp header
**Assignee:** Senior Developer  
**Estimated Time:** 1 hour  
**Files:** `AirLib/include/sensors/battery/BatteryData.hpp`

**Subtasks:**
1. Create directory structure: `AirLib/include/sensors/battery/`
2. Define `BatteryData` struct with 8 fields
3. Add proper namespace (`msr::airlib`)
4. Include guards and copyright header
5. Add inline conversion helper functions

**Acceptance Criteria:**
- [ ] Header compiles without errors
- [ ] All 8 fields defined with correct types
- [ ] Includes msgpack serialization macros
- [ ] Code follows Cosys-AirSim style guide

---

#### TASK-BAT-002: Create BatterySensor.hpp header
**Assignee:** Senior Developer  
**Estimated Time:** 2 hours  
**Files:** `AirLib/include/sensors/battery/BatterySensor.hpp`

**Subtasks:**
1. Define `BatteryMode` enum (2 modes)
2. Define `BatterySettings` struct
3. Define `BatterySensor` class inheriting from `SensorBase`
4. Declare public interface (7 methods)
5. Declare private helpers (4 methods)
6. Add member variables

**Acceptance Criteria:**
- [ ] Header compiles without errors
- [ ] Class inherits from `SensorBase` correctly
- [ ] All method signatures match specification
- [ ] Proper const-correctness
- [ ] Documentation comments added

---

#### TASK-BAT-003: Implement BatterySensor.cpp core logic
**Assignee:** Senior Developer  
**Estimated Time:** 4 hours  
**Files:** `AirLib/src/sensors/battery/BatterySensor.cpp`

**Subtasks:**
1. Implement constructor/destructor
2. Implement `initialize()` override
3. Implement `update(real_T dt)` override
4. Implement `reportState()` override
5. Implement simple discharge logic
6. Implement energy consumption logic
7. Add bounds checking (0-100%)

**Acceptance Criteria:**
- [ ] Compiles and links successfully
- [ ] Both discharge modes functional
- [ ] Battery percentage bounded [0, 100]
- [ ] No divide-by-zero errors
- [ ] Time delta handled correctly

---

#### TASK-BAT-004: Implement health status logic
**Assignee:** Senior Developer  
**Estimated Time:** 1 hour  
**Files:** `AirLib/src/sensors/battery/BatterySensor.cpp`

**Subtasks:**
1. Implement `updateHealthStatus()` method
2. Check unhealthy flag first
3. Check critical threshold (default 15%)
4. Check low threshold (default 30%)
5. Set appropriate state string

**Acceptance Criteria:**
- [ ] Status updates correctly based on percentage
- [ ] Unhealthy flag overrides percentage-based status
- [ ] Thresholds configurable
- [ ] All 4 states reachable

---

#### TASK-BAT-005: Implement time remaining calculation
**Assignee:** Senior Developer  
**Estimated Time:** 2 hours  
**Files:** `AirLib/src/sensors/battery/BatterySensor.cpp`

**Subtasks:**
1. Implement `calculateEstimatedTimeRemaining()` method
2. Handle SimpleDischarge mode (% / rate)
3. Handle EnergyConsumption mode (energy / power)
4. Handle zero power case (infinite time)
5. Handle zero drain rate case
6. Return uint32_t seconds

**Acceptance Criteria:**
- [ ] Calculation accurate within 1% error
- [ ] No integer overflow
- [ ] Zero power returns UINT32_MAX
- [ ] Both modes tested

---

#### TASK-BAT-006: Implement configuration parsing
**Assignee:** Senior Developer  
**Estimated Time:** 2 hours  
**Files:** `AirLib/src/sensors/battery/BatterySensor.cpp`

**Subtasks:**
1. Implement `load(const Settings&)` method
2. Parse BatteryMode string
3. Parse capacity in Joules
4. Parse mode-specific parameters
5. Parse thresholds
6. Set default values for missing params

**Acceptance Criteria:**
- [ ] Parses valid settings.json correctly
- [ ] Defaults applied for missing values
- [ ] Invalid mode string handled gracefully
- [ ] Joules conversion accurate

---

#### TASK-BAT-007: Implement getter/setter methods
**Assignee:** Developer  
**Estimated Time:** 1 hour  
**Files:** `AirLib/src/sensors/battery/BatterySensor.cpp`

**Subtasks:**
1. Implement `getBatteryData() const`
2. Implement `setBatteryRemaining(float)`
3. Implement `setBatteryDrainRate(float)`
4. Implement `getBatteryDrainRate() const`
5. Implement `setBatteryHealthStatus(bool)`
6. Implement `setPowerConsumption(float)`

**Acceptance Criteria:**
- [ ] All getters return correct values
- [ ] All setters validate input ranges
- [ ] Const-correctness maintained
- [ ] No side effects in getters

---

#### TASK-BAT-008: Add sensor to factory
**Assignee:** Developer  
**Estimated Time:** 1 hour  
**Files:** `AirLib/src/sensors/SensorFactory.cpp`

**Subtasks:**
1. Determine sensor type ID (coordinate with existing sensors)
2. Add case to sensor factory switch
3. Instantiate BatterySensor
4. Register sensor type constant

**Acceptance Criteria:**
- [ ] Sensor ID doesn't conflict
- [ ] Factory creates BatterySensor correctly
- [ ] Sensor type registered in enum

---

### Phase 1.2: Python API Bindings (5 tasks)

#### TASK-BAT-009: Add BatteryData Python class
**Assignee:** Developer  
**Estimated Time:** 1 hour  
**Files:** `PythonClient/cosysairsim/types.py`

**Subtasks:**
1. Define `BatteryData` class
2. Add 8 fields with type hints
3. Add msgpack serialization
4. Add `__repr__` method
5. Add docstring

**Acceptance Criteria:**
- [ ] Class definition matches C++ struct
- [ ] Msgpack serialization works
- [ ] Type hints correct
- [ ] Repr output readable

---

#### TASK-BAT-010: Implement getBatteryData() Python method
**Assignee:** Developer  
**Estimated Time:** 30 minutes  
**Files:** `PythonClient/cosysairsim/client.py`

**Subtasks:**
1. Add method to MultirotorClient
2. Add vehicle_name parameter
3. Add sensor_name parameter
4. Call RPC with correct parameters
5. Add docstring

**Acceptance Criteria:**
- [ ] Method callable
- [ ] Returns BatteryData object
- [ ] Parameters work correctly
- [ ] Docstring complete

---

#### TASK-BAT-011: Implement setBatteryRemaining() Python method
**Assignee:** Developer  
**Estimated Time:** 20 minutes  
**Files:** `PythonClient/cosysairsim/client.py`

**Acceptance Criteria:**
- [ ] Method callable with percent parameter
- [ ] Validates percent range [0, 100]
- [ ] RPC call succeeds

---

#### TASK-BAT-012: Implement drain rate methods (get/set)
**Assignee:** Developer  
**Estimated Time:** 30 minutes  
**Files:** `PythonClient/cosysairsim/client.py`

**Acceptance Criteria:**
- [ ] getBatteryDrainRate() returns float
- [ ] setBatteryDrainRate() accepts float
- [ ] Works only in SimpleDischarge mode

---

#### TASK-BAT-013: Implement setBatteryHealthStatus() Python method
**Assignee:** Developer  
**Estimated Time:** 20 minutes  
**Files:** `PythonClient/cosysairsim/client.py`

**Acceptance Criteria:**
- [ ] Method accepts boolean parameter
- [ ] RPC call succeeds
- [ ] Health status updates immediately

---

### Phase 1.3: RPC Integration (3 tasks)

#### TASK-BAT-014: Register RPC server methods
**Assignee:** Developer  
**Estimated Time:** 1 hour  
**Files:** `AirLib/src/api/RpcLibServerBase.cpp`

**Subtasks:**
1. Bind getBatteryData RPC call
2. Bind setBatteryRemaining RPC call
3. Bind getBatteryDrainRate RPC call
4. Bind setBatteryDrainRate RPC call
5. Bind setBatteryHealthStatus RPC call

**Acceptance Criteria:**
- [ ] All 5 methods registered
- [ ] Parameter types correct
- [ ] Vehicle name routing works
- [ ] Sensor name routing works

---

#### TASK-BAT-015: Implement VehicleApiBase battery methods
**Assignee:** Developer  
**Estimated Time:** 1 hour  
**Files:** `AirLib/include/api/VehicleApiBase.hpp`, `AirLib/src/api/VehicleApiBase.cpp`

**Subtasks:**
1. Add virtual methods to VehicleApiBase
2. Implement sensor lookup by name
3. Forward calls to BatterySensor instance
4. Handle missing sensor gracefully

**Acceptance Criteria:**
- [ ] Methods declared in VehicleApiBase
- [ ] Sensor lookup works
- [ ] Empty sensor_name uses first battery
- [ ] Throws exception if sensor not found

---

#### TASK-BAT-016: Implement MultirotorApiBase battery methods
**Assignee:** Developer  
**Estimated Time:** 30 minutes  
**Files:** `AirLib/include/vehicles/multirotor/api/MultirotorApiBase.hpp`

**Subtasks:**
1. Override virtual methods from VehicleApiBase
2. Access battery sensor from multirotor
3. Forward calls

**Acceptance Criteria:**
- [ ] Methods overridden correctly
- [ ] Works with multirotor vehicles
- [ ] No crashes with missing sensor

---

### Phase 1.4: Vehicle Integration (4 tasks)

#### TASK-BAT-017: Add power consumption tracking to rotors
**Assignee:** Senior Developer  
**Estimated Time:** 2 hours  
**Files:** `AirLib/include/vehicles/multirotor/RotorParams.hpp`, `AirLib/src/vehicles/multirotor/MultirotorPhysicsBody.cpp`

**Subtasks:**
1. Add `calculatePower()` method to rotor
2. Calculate power = torque × angular_velocity
3. Sum power from all rotors
4. Update battery sensor each physics tick

**Acceptance Criteria:**
- [ ] Power calculation physically accurate
- [ ] All rotors contribute to total power
- [ ] Battery sensor updated every tick
- [ ] No performance regression

---

#### TASK-BAT-018: Initialize battery sensor in multirotor
**Assignee:** Developer  
**Estimated Time:** 1 hour  
**Files:** `AirLib/src/vehicles/multirotor/MultirotorParamsFactory.cpp`

**Subtasks:**
1. Create BatterySensor in vehicle initialization
2. Add to sensor collection
3. Configure from settings
4. Register with physics body

**Acceptance Criteria:**
- [ ] Battery sensor created
- [ ] Sensor added to vehicle sensor list
- [ ] Configuration loaded
- [ ] Sensor updates with physics

---

#### TASK-BAT-019: Add voltage calculation
**Assignee:** Developer  
**Estimated Time:** 2 hours  
**Files:** `AirLib/src/sensors/battery/BatterySensor.cpp`

**Subtasks:**
1. Model voltage curve (e.g., LiPo 3.7V nominal)
2. Voltage drops with discharge
3. Update voltage in BatteryData
4. Make configurable (voltage_nominal, voltage_cutoff)

**Acceptance Criteria:**
- [ ] Voltage realistic for battery type
- [ ] Voltage decreases with discharge
- [ ] Configurable via settings.json

---

#### TASK-BAT-020: Add temperature simulation (optional)
**Assignee:** Developer  
**Estimated Time:** 3 hours  
**Files:** `AirLib/src/sensors/battery/BatterySensor.cpp`

**Subtasks:**
1. Model temperature increase with current draw
2. Model cooling over time
3. Update temperature in BatteryData
4. Make configurable (thermal_model params)

**Acceptance Criteria:**
- [ ] Temperature increases with high power
- [ ] Temperature decreases when idle
- [ ] Realistic temperature range
- [ ] Optional (can be disabled)

---

### Phase 1.5: Unit Testing (5 tasks)

#### TASK-BAT-021: Create BatterySensorTests.hpp
**Assignee:** QA Engineer  
**Estimated Time:** 1 hour  
**Files:** `AirLibUnitTests/BatterySensorTests.hpp`

**Subtasks:**
1. Create test class structure
2. Add test fixtures
3. Add helper methods
4. Include necessary headers

**Acceptance Criteria:**
- [ ] Test file compiles
- [ ] Can instantiate BatterySensor
- [ ] Fixtures set up correctly

---

#### TASK-BAT-022: Test simple discharge mode
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `AirLibUnitTests/BatterySensorTests.hpp`

**Test Cases:**
1. `testSimpleDischarge_ConstantRate()` - Fixed drain over time
2. `testSimpleDischarge_SetDrainRate()` - Dynamic drain rate changes
3. `testSimpleDischarge_ZeroRate()` - No discharge with 0% rate
4. `testSimpleDischarge_FullDepletion()` - Battery reaches 0%

**Acceptance Criteria:**
- [ ] All 4 tests pass
- [ ] Timing accurate within 0.1%
- [ ] Edge cases handled

---

#### TASK-BAT-023: Test energy consumption mode
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `AirLibUnitTests/BatterySensorTests.hpp`

**Test Cases:**
1. `testEnergyConsumption_ConstantPower()` - Fixed power load
2. `testEnergyConsumption_VariablePower()` - Dynamic power changes
3. `testEnergyConsumption_ZeroPower()` - No discharge with 0W
4. `testEnergyConsumption_HighPower()` - Rapid depletion

**Acceptance Criteria:**
- [ ] All 4 tests pass
- [ ] Energy calculation accurate within 1%
- [ ] Power coefficient works

---

#### TASK-BAT-024: Test health status transitions
**Assignee:** QA Engineer  
**Estimated Time:** 1 hour  
**Files:** `AirLibUnitTests/BatterySensorTests.hpp`

**Test Cases:**
1. `testHealthStatus_OkToLow()` - 100% → 29%
2. `testHealthStatus_LowToCritical()` - 30% → 14%
3. `testHealthStatus_UnhealthyFlag()` - Override with flag
4. `testHealthStatus_CustomThresholds()` - Configurable thresholds

**Acceptance Criteria:**
- [ ] All 4 tests pass
- [ ] Transitions occur at correct percentages
- [ ] Unhealthy flag has priority

---

#### TASK-BAT-025: Test API methods
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `AirLibUnitTests/BatterySensorTests.hpp`

**Test Cases:**
1. `testAPI_SetGetRemaining()` - Set/get percentage
2. `testAPI_SetGetDrainRate()` - Set/get drain rate
3. `testAPI_SetHealthStatus()` - Set health flag
4. `testAPI_EstimatedTimeRemaining()` - Time calculation
5. `testAPI_InvalidInputs()` - Negative values, >100%

**Acceptance Criteria:**
- [ ] All 5 tests pass
- [ ] Invalid inputs rejected gracefully
- [ ] State changes reflected immediately

---

### Phase 1.6: Integration Testing (3 tasks)

#### TASK-BAT-026: Test Python-C++ communication
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `PythonClient/tests/test_battery_sensor.py`

**Test Cases:**
1. Test all 5 Python API methods
2. Test msgpack serialization
3. Test vehicle_name parameter
4. Test sensor_name parameter
5. Test error handling

**Acceptance Criteria:**
- [ ] All Python tests pass
- [ ] Data serializes correctly
- [ ] Exceptions raised for invalid inputs

---

#### TASK-BAT-027: Test multi-vehicle battery independence
**Assignee:** QA Engineer  
**Estimated Time:** 1 hour  
**Files:** `PythonClient/tests/test_battery_multivehicle.py`

**Test Cases:**
1. Two vehicles with different battery states
2. Modify one vehicle's battery
3. Verify other vehicle unaffected
4. Test concurrent access

**Acceptance Criteria:**
- [ ] Battery states independent
- [ ] No cross-vehicle interference
- [ ] Thread-safe operations

---

#### TASK-BAT-028: Test configuration loading
**Assignee:** QA Engineer  
**Estimated Time:** 1 hour  
**Files:** `AirLibUnitTests/BatteryConfigTests.hpp`

**Test Cases:**
1. Load valid settings.json
2. Missing parameters use defaults
3. Invalid mode string handled
4. Joules conversion accurate

**Acceptance Criteria:**
- [ ] Valid configs load correctly
- [ ] Defaults applied appropriately
- [ ] Invalid configs don't crash

---

## Priority 1: Clock Control Implementation

### Phase 2.1: C++ Core Clock System (6 tasks)

#### TASK-CLK-001: Define ClockBase enhancements
**Assignee:** Senior Developer  
**Estimated Time:** 2 hours  
**Files:** `AirLib/include/common/ClockBase.hpp`

**Subtasks:**
1. Add ClockType enum (Steppable, RealTime)
2. Add clock speed multiplier member
3. Add pause state tracking
4. Add step execution methods
5. Add wait_until_complete support

**Acceptance Criteria:**
- [ ] Header compiles
- [ ] All new members added
- [ ] Backward compatible with existing code

---

#### TASK-CLK-002: Implement getSimClockType()
**Assignee:** Developer  
**Estimated Time:** 30 minutes  
**Files:** `AirLib/src/common/ClockBase.cpp`

**Subtasks:**
1. Return "steppable" or "real-time"
2. Determine type based on simulation mode

**Acceptance Criteria:**
- [ ] Returns correct string
- [ ] Matches simulation mode

---

#### TASK-CLK-003: Implement getSimTime()
**Assignee:** Developer  
**Estimated Time:** 30 minutes  
**Files:** `AirLib/src/common/ClockBase.cpp`

**Subtasks:**
1. Return current simulation time
2. Convert to nanoseconds (uint64_t)
3. Handle time overflow gracefully

**Acceptance Criteria:**
- [ ] Returns nanoseconds
- [ ] Monotonically increasing
- [ ] No overflow for reasonable sim times

---

#### TASK-CLK-004: Implement setClockSpeed()
**Assignee:** Developer  
**Estimated Time:** 1 hour  
**Files:** `AirLib/src/common/ClockBase.cpp`

**Subtasks:**
1. Accept speed multiplier (0.1x to 10x)
2. Validate range
3. Apply to time step calculation
4. Thread-safe implementation

**Acceptance Criteria:**
- [ ] Speed changes take effect immediately
- [ ] Valid range enforced
- [ ] Time advancement scales correctly

---

#### TASK-CLK-005: Implement pause/resume
**Assignee:** Developer  
**Estimated Time:** 2 hours  
**Files:** `AirLib/src/common/ClockBase.cpp`

**Subtasks:**
1. Add pause state flag
2. Implement pause() method
3. Implement resume() method
4. Implement isPaused() method
5. Stop time advancement when paused
6. Return state string

**Acceptance Criteria:**
- [ ] Pause stops simulation
- [ ] Resume continues from pause point
- [ ] isPaused() accurate
- [ ] Thread-safe

---

#### TASK-CLK-006: Implement stepped execution
**Assignee:** Senior Developer  
**Estimated Time:** 4 hours  
**Files:** `AirLib/src/common/ClockBase.cpp`

**Subtasks:**
1. Implement continueForTime(nanos, wait)
2. Implement continueForSteps(n, wait)
3. Implement continueForSingleStep(wait)
4. Add synchronization for wait=true
5. Return immediately for wait=false
6. Auto-pause after completion

**Acceptance Criteria:**
- [ ] Time advancement accurate within 1ms
- [ ] Step count accurate
- [ ] wait_until_complete works both ways
- [ ] Auto-pause after completion

---

### Phase 2.2: Python API Bindings (4 tasks)

#### TASK-CLK-007: Implement clock type and time methods
**Assignee:** Developer  
**Estimated Time:** 1 hour  
**Files:** `PythonClient/cosysairsim/client.py`

**Subtasks:**
1. Add getSimClockType() method
2. Add getSimTime() method
3. Add docstrings
4. Add type hints

**Acceptance Criteria:**
- [ ] Methods callable
- [ ] Return correct types
- [ ] Docstrings complete

---

#### TASK-CLK-008: Implement clock speed control
**Assignee:** Developer  
**Estimated Time:** 30 minutes  
**Files:** `PythonClient/cosysairsim/client.py`

**Subtasks:**
1. Add setClockSpeed(speed) method
2. Validate speed range
3. Add docstring

**Acceptance Criteria:**
- [ ] Method callable
- [ ] Range validation works
- [ ] Speed changes applied

---

#### TASK-CLK-009: Implement pause/resume methods
**Assignee:** Developer  
**Estimated Time:** 1 hour  
**Files:** `PythonClient/cosysairsim/client.py`

**Subtasks:**
1. Add pause() method
2. Add resume() method
3. Add isPaused() method
4. Maintain backward compatibility with simPause()

**Acceptance Criteria:**
- [ ] New methods work
- [ ] Old simPause() still works
- [ ] State changes immediate

---

#### TASK-CLK-010: Implement stepped execution methods
**Assignee:** Developer  
**Estimated Time:** 2 hours  
**Files:** `PythonClient/cosysairsim/client.py`

**Subtasks:**
1. Add continueForTime(nanos, wait=True)
2. Add continueForSteps(n, wait=True)
3. Add continueForSingleStep(wait=True)
4. Maintain backward compatibility with simContinueForTime()
5. Add docstrings with wait parameter explanation

**Acceptance Criteria:**
- [ ] All 3 methods callable
- [ ] wait parameter works
- [ ] Backward compatibility maintained

---

### Phase 2.3: RPC Integration (3 tasks)

#### TASK-CLK-011: Register clock control RPC methods
**Assignee:** Developer  
**Estimated Time:** 2 hours  
**Files:** `AirLib/src/api/RpcLibServerBase.cpp`

**Subtasks:**
1. Bind getSimClockType
2. Bind getSimTime
3. Bind setClockSpeed
4. Bind pause
5. Bind resume
6. Bind continueForTime
7. Bind continueForSteps
8. Bind continueForSingleStep
9. Bind isPaused

**Acceptance Criteria:**
- [ ] All 9 methods registered
- [ ] Parameter types correct
- [ ] No naming conflicts

---

#### TASK-CLK-012: Implement VehicleApiBase clock methods
**Assignee:** Developer  
**Estimated Time:** 1 hour  
**Files:** `AirLib/include/api/VehicleApiBase.hpp`

**Subtasks:**
1. Add virtual methods for clock control
2. Access clock through simulation
3. Forward calls to ClockBase

**Acceptance Criteria:**
- [ ] Methods declared
- [ ] Clock accessible
- [ ] Calls forwarded correctly

---

#### TASK-CLK-013: Update existing simPause/simContinue
**Assignee:** Developer  
**Estimated Time:** 1 hour  
**Files:** `AirLib/src/api/VehicleApiBase.cpp`

**Subtasks:**
1. Refactor simPause() to use new pause/resume
2. Refactor simContinueForTime() to use new continueForTime
3. Maintain backward compatibility
4. Add deprecation warnings (optional)

**Acceptance Criteria:**
- [ ] Old methods still work
- [ ] Use new implementation internally
- [ ] No breaking changes

---

### Phase 2.4: Unit Testing (6 tasks)

#### TASK-CLK-014: Test clock type and time
**Assignee:** QA Engineer  
**Estimated Time:** 1 hour  
**Files:** `AirLibUnitTests/ClockControlTests.hpp`

**Test Cases:**
1. `testClockType()` - Returns correct type
2. `testSimTime()` - Time advances correctly
3. `testSimTime_Monotonic()` - Never decreases

**Acceptance Criteria:**
- [ ] All tests pass
- [ ] Time monotonically increases

---

#### TASK-CLK-015: Test clock speed
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `AirLibUnitTests/ClockControlTests.hpp`

**Test Cases:**
1. `testClockSpeed_2x()` - 2x speed doubles advancement
2. `testClockSpeed_Half()` - 0.5x speed halves advancement
3. `testClockSpeed_Normal()` - 1x speed is default
4. `testClockSpeed_InvalidRange()` - Rejects invalid speeds

**Acceptance Criteria:**
- [ ] All tests pass
- [ ] Timing accurate within 5ms
- [ ] Invalid speeds rejected

---

#### TASK-CLK-016: Test pause/resume
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `AirLibUnitTests/ClockControlTests.hpp`

**Test Cases:**
1. `testPause_StopsTime()` - Time frozen when paused
2. `testResume_ContinuesTime()` - Time resumes from pause point
3. `testIsPaused_State()` - isPaused reflects state
4. `testPauseResume_Multiple()` - Multiple pause/resume cycles

**Acceptance Criteria:**
- [ ] All tests pass
- [ ] Time doesn't advance when paused
- [ ] State tracking accurate

---

#### TASK-CLK-017: Test continueForTime
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `AirLibUnitTests/ClockControlTests.hpp`

**Test Cases:**
1. `testContinueForTime_Wait()` - Blocks until complete
2. `testContinueForTime_NoWait()` - Returns immediately
3. `testContinueForTime_AutoPause()` - Pauses after time elapsed
4. `testContinueForTime_Accuracy()` - Time advancement accurate

**Acceptance Criteria:**
- [ ] All tests pass
- [ ] Timing accurate within 1ms
- [ ] wait parameter works

---

#### TASK-CLK-018: Test continueForSteps
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `AirLibUnitTests/ClockControlTests.hpp`

**Test Cases:**
1. `testContinueForSteps_Wait()` - Blocks until complete
2. `testContinueForSteps_NoWait()` - Returns immediately
3. `testContinueForSteps_StepCount()` - Exact N steps executed
4. `testContinueForSingleStep()` - Single step variant

**Acceptance Criteria:**
- [ ] All tests pass
- [ ] Step count exact
- [ ] wait parameter works

---

#### TASK-CLK-019: Test backward compatibility
**Assignee:** QA Engineer  
**Estimated Time:** 1 hour  
**Files:** `AirLibUnitTests/ClockCompatibilityTests.hpp`

**Test Cases:**
1. `testSimPause_StillWorks()` - Old method functional
2. `testSimContinueForTime_StillWorks()` - Old method functional
3. `testMixedAPI()` - Can use old and new methods together

**Acceptance Criteria:**
- [ ] All old methods work
- [ ] No breaking changes
- [ ] Mixed usage works

---

### Phase 2.5: Integration Testing (5 tasks)

#### TASK-CLK-020: Test Python-C++ clock communication
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `PythonClient/tests/test_clock_control.py`

**Test Cases:**
1. Test all 9 Python methods
2. Test parameter passing
3. Test return values
4. Test error handling

**Acceptance Criteria:**
- [ ] All Python tests pass
- [ ] Parameters passed correctly
- [ ] Return values correct

---

#### TASK-CLK-021: Test clock with multi-vehicle
**Assignee:** QA Engineer  
**Estimated Time:** 1 hour  
**Files:** `PythonClient/tests/test_clock_multivehicle.py`

**Test Cases:**
1. Clock affects all vehicles equally
2. Pause stops all vehicles
3. Time sync across vehicles

**Acceptance Criteria:**
- [ ] Clock global to simulation
- [ ] All vehicles synchronized

---

#### TASK-CLK-022: Test clock speed with physics
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `PythonClient/tests/test_clock_physics.py`

**Test Cases:**
1. Vehicle falls 2x faster at 2x speed
2. Vehicle falls 0.5x slower at 0.5x speed
3. Physics deterministic at all speeds

**Acceptance Criteria:**
- [ ] Physics scales with clock speed
- [ ] Results deterministic
- [ ] No instability

---

#### TASK-CLK-023: Performance benchmarking
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `AirLibUnitTests/ClockPerformanceTests.hpp`

**Benchmarks:**
1. Overhead of pause/resume
2. Overhead of continueForTime
3. Overhead of clock speed changes
4. Memory usage

**Acceptance Criteria:**
- [ ] Overhead <1ms per operation
- [ ] No memory leaks
- [ ] Performance acceptable

---

#### TASK-CLK-024: End-to-end integration test
**Assignee:** QA Engineer  
**Estimated Time:** 2 hours  
**Files:** `PythonClient/tests/test_clock_e2e.py`

**Test Scenario:**
1. Start simulation
2. Set 2x clock speed
3. Fly waypoint mission
4. Pause mid-mission
5. Resume and complete
6. Verify timing accurate

**Acceptance Criteria:**
- [ ] Full scenario completes
- [ ] Timing accurate
- [ ] No crashes

---

## Priority 2: Integration Testing (15 tasks)

*Tasks TASK-INT-001 through TASK-INT-015 covering battery+clock integration, multi-vehicle scenarios, configuration testing, etc.*

---

## Priority 3: Airspeed Sensor (12 tasks)

*Tasks TASK-AIR-001 through TASK-AIR-012 for airspeed sensor implementation*

---

## Priority 4: Radar Documentation (8 tasks)

*Tasks TASK-RAD-001 through TASK-RAD-008 for radar sensor specification*

---

## Task Summary

### By Priority
- **Priority 0 (Battery):** 28 tasks, ~6 weeks
- **Priority 1 (Clock):** 24 tasks, ~5 weeks
- **Priority 2 (Integration):** 15 tasks, ~2 weeks
- **Priority 3 (Airspeed):** 12 tasks, ~2 weeks
- **Priority 4 (Radar Doc):** 8 tasks, ~1 week

### By Phase
- **C++ Implementation:** 36 tasks
- **Python Bindings:** 16 tasks
- **RPC Integration:** 8 tasks
- **Unit Testing:** 18 tasks
- **Integration Testing:** 9 tasks

### By Role
- **Senior Developer:** 15 tasks (~3 weeks)
- **Developer:** 42 tasks (~8 weeks)
- **QA Engineer:** 30 tasks (~6 weeks)

---

**Task List Version:** 1.0  
**Last Updated:** January 2025  
**Total Estimated Time:** 16-18 developer-weeks
