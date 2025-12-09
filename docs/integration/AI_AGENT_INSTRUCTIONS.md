# AI Agent Implementation Instructions

**Project:** Cosys-AirSim ProjectAirSim Integration  
**Date:** January 2025  
**Total Tasks:** 87 tasks across 4 priorities  
**Estimated Timeline:** 16-18 developer-weeks

---

## 📋 Overview

This document provides step-by-step instructions for an AI agent to implement ProjectAirSim features into Cosys-AirSim. The implementation follows a systematic approach with clear acceptance criteria for each task.

---

## 📦 Required Documents

You have access to 5 comprehensive implementation documents:

1. **IMPLEMENTATION_ROADMAP.md** - 7-8 week timeline with phase breakdown
2. **TASK_LIST_DETAILED.md** - 87 detailed tasks with acceptance criteria
3. **TECH_SPEC_BATTERY.md** - Complete Battery Sensor technical specification
4. **TESTING_PLAN.md** - Comprehensive testing plan (~200 tests)
5. **CPP_IMPLEMENTATION_GUIDE.md** - C++ coding standards and patterns

**Read all 5 documents before starting implementation.**

---

## 🎯 Implementation Strategy

### Sequential Execution Rules

1. **Work through tasks in numerical order** - Do not skip ahead
2. **Complete one task fully before moving to the next** - Including tests
3. **Mark tasks complete** - Update checkboxes in acceptance criteria
4. **Verify compilation** - After each C++ change
5. **Run tests** - After implementing each component
6. **Fix failures immediately** - Do not proceed with failing tests

---

## 🚀 Phase 1: Battery Sensor Implementation

**Priority:** 0 (Highest)  
**Tasks:** TASK-BAT-001 through TASK-BAT-028 (28 tasks)  
**Estimated Time:** 6 weeks  
**Reference Document:** TECH_SPEC_BATTERY.md

### Starting Prompt

```
IMPLEMENTATION PHASE: Battery Sensor for Cosys-AirSim

CONTEXT:
You are implementing a Battery Sensor feature based on ProjectAirSim specifications.
The battery sensor supports two discharge modes:
1. SimpleDischarge - Constant percentage drain per second
2. EnergyConsumption - Physics-based power consumption in Watts

REQUIRED READING:
1. Read TECH_SPEC_BATTERY.md completely for technical specifications
2. Read CPP_IMPLEMENTATION_GUIDE.md for coding standards
3. Read TASK_LIST_DETAILED.md, focus on TASK-BAT-001 through TASK-BAT-028
4. Read TESTING_PLAN.md for test requirements

CODEBASE CONTEXT:
Before starting, examine these existing files:
- AirLib/include/sensors/SensorBase.hpp (base class to inherit from)
- AirLib/include/common/CommonStructs.hpp (data structure patterns)
- AirLib/src/api/RpcLibServerBase.cpp (RPC registration patterns)
- PythonClient/cosysairsim/types.py (Python type patterns)
- PythonClient/cosysairsim/client.py (Python API patterns)

IMPLEMENTATION PHASES:
Phase 1.1: C++ Core Structure (TASK-BAT-001 to TASK-BAT-008)
Phase 1.2: Python API Bindings (TASK-BAT-009 to TASK-BAT-013)
Phase 1.3: RPC Integration (TASK-BAT-014 to TASK-BAT-016)
Phase 1.4: Vehicle Integration (TASK-BAT-017 to TASK-BAT-020)
Phase 1.5: Unit Testing (TASK-BAT-021 to TASK-BAT-025)
Phase 1.6: Integration Testing (TASK-BAT-026 to TASK-BAT-028)

STARTING TASK: TASK-BAT-001
Task: Create BatteryData.hpp header
File: AirLib/include/sensors/battery/BatteryData.hpp
Time: 1 hour

Requirements:
1. Create directory: AirLib/include/sensors/battery/
2. Define BatteryData struct with 8 fields:
   - float battery_pct_remaining (0-100)
   - uint32_t estimated_time_remaining (seconds)
   - std::string battery_charge_state ("OK", "LOW", "CRITICAL", "UNHEALTHY")
   - int64_t time_stamp (nanoseconds)
   - float voltage (volts)
   - float current_draw (amperes)
   - float remaining_wh (watt-hours)
   - float temperature (celsius)
3. Add msgpack serialization: MSGPACK_DEFINE_MAP(...)
4. Add namespace: msr::airlib
5. Add include guards
6. Follow CPP_IMPLEMENTATION_GUIDE.md style

Acceptance Criteria:
- [ ] Header compiles without errors
- [ ] All 8 fields defined with correct types
- [ ] Msgpack serialization macros included
- [ ] Code follows Cosys-AirSim style guide
- [ ] Proper include guards added
- [ ] Namespace correct

After completing TASK-BAT-001:
1. Verify compilation: Run build.cmd (Windows) or build.sh (Linux)
2. Check for warnings or errors
3. Mark task complete
4. Proceed to TASK-BAT-002

CRITICAL RULES:
- Do NOT skip tasks
- Do NOT implement multiple tasks simultaneously
- Test after each component
- Follow specifications exactly
- Ask for clarification if specifications are ambiguous
```

---

## 🔄 Phase 2: Clock Control Implementation

**Priority:** 1  
**Tasks:** TASK-CLK-001 through TASK-CLK-024 (24 tasks)  
**Estimated Time:** 5 weeks  
**Start After:** Battery Sensor is 100% complete and tested

### Starting Prompt

```
IMPLEMENTATION PHASE: Clock Control for Cosys-AirSim

PREREQUISITES:
- Battery Sensor implementation complete (all 28 tasks)
- All battery tests passing
- Battery feature verified working

CONTEXT:
You are implementing Clock Control with 9 new API methods:
1. getSimClockType() - Returns "steppable" or "real-time"
2. getSimTime() - Returns simulation time in nanoseconds
3. setClockSpeed(float) - Set time multiplier (0.1x to 10x)
4. pause() - Pause simulation
5. resume() - Resume simulation
6. continueForTime(uint64_t, bool) - Advance time by nanoseconds
7. continueForSteps(uint32_t, bool) - Advance by N physics steps
8. continueForSingleStep(bool) - Advance by 1 physics step
9. isPaused() - Check if paused

REQUIRED READING:
1. Read IMPLEMENTATION_ROADMAP.md - Week 2 section
2. Read TASK_LIST_DETAILED.md - TASK-CLK-001 through TASK-CLK-024
3. Read CPP_IMPLEMENTATION_GUIDE.md - Clock patterns
4. Read TESTING_PLAN.md - Clock testing section

IMPLEMENTATION PHASES:
Phase 2.1: C++ Core Clock System (TASK-CLK-001 to TASK-CLK-006)
Phase 2.2: Python API Bindings (TASK-CLK-007 to TASK-CLK-010)
Phase 2.3: RPC Integration (TASK-CLK-011 to TASK-CLK-013)
Phase 2.4: Unit Testing (TASK-CLK-014 to TASK-CLK-019)
Phase 2.5: Integration Testing (TASK-CLK-020 to TASK-CLK-024)

STARTING TASK: TASK-CLK-001
Task: Define ClockBase enhancements
File: AirLib/include/common/ClockBase.hpp
Time: 2 hours

Requirements:
1. Add ClockType enum (Steppable, RealTime)
2. Add clock speed multiplier member (float)
3. Add pause state tracking (bool)
4. Add step execution method declarations
5. Add wait_until_complete support

Acceptance Criteria:
- [ ] Header compiles without errors
- [ ] All new members added
- [ ] Backward compatible with existing code
- [ ] No breaking changes to existing API

CRITICAL: Maintain backward compatibility with existing simPause() and simContinueForTime() methods.

After completing TASK-CLK-001:
1. Verify compilation
2. Run existing tests to ensure no regressions
3. Mark task complete
4. Proceed to TASK-CLK-002
```

---

## 🧪 Testing Requirements

### Unit Testing Rules

1. **Create tests as you implement** - Do not defer testing
2. **Test coverage >80%** - Aim for comprehensive coverage
3. **All tests must pass** - Fix failures before proceeding
4. **Test edge cases** - Negative values, boundary conditions, zero values
5. **Test error handling** - Invalid inputs should be rejected gracefully

### Test Execution

**C++ Unit Tests:**
```bash
# Windows
AirLibUnitTests.exe

# Linux
./AirLibUnitTests
```

**Python Tests:**
```bash
cd PythonClient
python -m pytest tests/test_battery_sensor.py -v
python -m pytest tests/test_clock_control.py -v
```

### Test Coverage Tool

```bash
# Python coverage
pytest --cov=cosysairsim --cov-report=html
```

---

## 📐 Coding Standards

**Follow CPP_IMPLEMENTATION_GUIDE.md strictly:**

### Naming Conventions
- **Classes:** PascalCase (`BatterySensor`, `ClockBase`)
- **Methods:** camelCase (`getBatteryData`, `setClockSpeed`)
- **Member variables:** snake_case with trailing underscore (`battery_data_`, `is_paused_`)
- **Constants:** UPPER_SNAKE_CASE (`MAX_PERCENTAGE`, `NANOS_PER_SECOND`)
- **Files:** PascalCase matching primary class (`BatterySensor.hpp`, `BatterySensor.cpp`)

### Header File Structure
1. Include guards
2. System includes (alphabetical)
3. External library includes
4. Project includes
5. Namespace declaration
6. Forward declarations
7. Type definitions
8. Class declaration
9. Inline implementations (if any)

### Code Quality
- **No compiler warnings** - Treat warnings as errors
- **No memory leaks** - Use RAII and smart pointers
- **Const correctness** - Mark methods and parameters const where appropriate
- **Thread safety** - Use mutexes for shared state if needed
- **Error handling** - Validate inputs, throw exceptions for errors
- **Documentation** - Add Doxygen comments for public API

---

## 🔍 Verification Checklist

After completing each task:

- [ ] Code compiles without errors or warnings
- [ ] Code follows style guide (CPP_IMPLEMENTATION_GUIDE.md)
- [ ] Unit tests written and passing
- [ ] Acceptance criteria met (checkboxes in TASK_LIST_DETAILED.md)
- [ ] No memory leaks detected
- [ ] Documentation comments added
- [ ] Git commit with descriptive message

After completing each phase:

- [ ] All phase tasks complete
- [ ] Integration tests passing
- [ ] Performance benchmarks met
- [ ] Code reviewed (if human available)
- [ ] Feature demonstrated working end-to-end

---

## 🚧 Common Pitfalls to Avoid

### 1. Skipping Tasks
❌ **Don't:** Jump ahead to Python before C++ is complete  
✅ **Do:** Complete all C++ tasks, then move to Python

### 2. Ignoring Test Failures
❌ **Don't:** Proceed with failing tests  
✅ **Do:** Fix all test failures immediately

### 3. Breaking Existing Code
❌ **Don't:** Modify existing APIs without backward compatibility  
✅ **Do:** Add new methods, keep old methods working

### 4. Poor Error Handling
❌ **Don't:** Assume inputs are valid  
✅ **Do:** Validate all inputs, throw exceptions for invalid data

### 5. Inconsistent Style
❌ **Don't:** Use your own naming conventions  
✅ **Do:** Follow CPP_IMPLEMENTATION_GUIDE.md exactly

---

## 📊 Progress Tracking

### Task Completion Format

When completing a task, provide this summary:

```
TASK COMPLETE: TASK-BAT-001
Title: Create BatteryData.hpp header
Status: ✅ COMPLETE
Time Taken: 45 minutes
Files Created:
  - AirLib/include/sensors/battery/BatteryData.hpp

Acceptance Criteria:
  ✅ Header compiles without errors
  ✅ All 8 fields defined with correct types
  ✅ Msgpack serialization macros included
  ✅ Code follows Cosys-AirSim style guide
  ✅ Proper include guards added
  ✅ Namespace correct

Tests:
  ✅ Compilation successful (0 errors, 0 warnings)

Next Task: TASK-BAT-002 - Create BatterySensor.hpp header
```

### Phase Completion Format

```
PHASE COMPLETE: Phase 1.1 - C++ Core Structure
Tasks: TASK-BAT-001 through TASK-BAT-008
Status: ✅ COMPLETE (8/8 tasks)
Time Taken: 12 hours

Summary:
  ✅ BatteryData.hpp created
  ✅ BatterySensor.hpp created
  ✅ BatterySensor.cpp implemented
  ✅ Health status logic working
  ✅ Time remaining calculation working
  ✅ Configuration parsing working
  ✅ Getter/setter methods working
  ✅ Sensor registered in factory

Tests:
  ✅ All C++ unit tests passing (15/15)
  ✅ No memory leaks detected
  ✅ Code coverage: 87%

Next Phase: Phase 1.2 - Python API Bindings
```

---

## 🔄 Iteration and Refinement

### When Tests Fail

1. **Read error message carefully**
2. **Identify root cause** - Don't just fix symptoms
3. **Fix the issue** - Modify code to address root cause
4. **Re-run tests** - Verify fix works
5. **Run full test suite** - Ensure no regressions
6. **Document fix** - Note what was wrong and how you fixed it

### When Specifications are Ambiguous

1. **Check TECH_SPEC_BATTERY.md** - Most details are there
2. **Check CPP_IMPLEMENTATION_GUIDE.md** - Coding patterns explained
3. **Check TESTING_PLAN.md** - Test cases clarify expected behavior
4. **Look at existing code** - Follow established patterns
5. **Make reasonable assumption** - Document it, ask for review

---

## 📁 File Organization

### Directory Structure to Create

```
AirLib/
├── include/
│   ├── sensors/
│   │   ├── battery/              [CREATE]
│   │   │   ├── BatterySensor.hpp
│   │   │   ├── BatteryData.hpp
│   │   │   └── BatterySettings.hpp
│   │   └── ...
│   └── common/
│       └── ClockBase.hpp         [MODIFY]
│
├── src/
│   ├── sensors/
│   │   ├── battery/              [CREATE]
│   │   │   └── BatterySensor.cpp
│   │   └── ...
│   └── api/
│       └── RpcLibServerBase.cpp  [MODIFY]
│
└── ...

AirLibUnitTests/
├── BatterySensorTests.hpp        [CREATE]
├── ClockControlTests.hpp         [CREATE]
└── ...

PythonClient/
├── cosysairsim/
│   ├── types.py                  [MODIFY]
│   └── client.py                 [MODIFY]
└── tests/
    ├── test_battery_sensor.py    [CREATE]
    └── test_clock_control.py     [CREATE]
```

---

## 🎓 Implementation Best Practices

### 1. Read First, Code Second
- Read all specifications before writing any code
- Understand the full picture
- Plan your approach

### 2. Small, Incremental Changes
- Implement one method at a time
- Test after each method
- Commit after each task

### 3. Test-Driven Development
- Write test cases before or alongside implementation
- Run tests frequently
- Maintain high test coverage

### 4. Code Review Yourself
- Re-read your code before marking task complete
- Check for typos, logic errors, edge cases
- Verify against acceptance criteria

### 5. Document as You Go
- Add comments for complex logic
- Update docstrings
- Note any deviations from spec

---

## ⚡ Quick Reference Commands

### Build Commands
```bash
# Windows
build.cmd

# Linux
./build.sh
```

### Test Commands
```bash
# C++ Unit Tests
AirLibUnitTests.exe                                    # Windows
./AirLibUnitTests                                      # Linux

# Python Tests
cd PythonClient
python -m pytest tests/ -v                             # All tests
python -m pytest tests/test_battery_sensor.py -v       # Battery tests
python -m pytest tests/test_clock_control.py -v        # Clock tests

# With coverage
python -m pytest --cov=cosysairsim --cov-report=html tests/
```

### Git Commands
```bash
# After completing each task
git add <files>
git commit -m "TASK-BAT-001: Create BatteryData.hpp header"

# After completing each phase
git add .
git commit -m "Phase 1.1 complete: Battery C++ core structure"
```

---

## 🎯 Success Criteria

### Phase-Level Success
- [ ] All tasks in phase complete
- [ ] All unit tests passing
- [ ] All integration tests passing
- [ ] Code coverage >80%
- [ ] No memory leaks
- [ ] No compiler warnings
- [ ] Performance benchmarks met

### Feature-Level Success
- [ ] Feature functional end-to-end
- [ ] Python API works
- [ ] RPC communication works
- [ ] Multi-vehicle support works
- [ ] Configuration loading works
- [ ] Backward compatibility maintained
- [ ] Documentation complete

### Project-Level Success
- [ ] All 4 priorities complete (87 tasks)
- [ ] All 200 tests passing
- [ ] Full integration verified
- [ ] Performance acceptable
- [ ] Code reviewed and approved
- [ ] User documentation updated
- [ ] Ready for release

---

## 📞 Support and Clarifications

### When You Need Help

1. **Re-read specifications** - Answer might be in docs
2. **Check existing code** - Follow established patterns
3. **Review test cases** - They clarify expected behavior
4. **Document question** - Write down what's unclear
5. **Make best judgment** - Proceed with reasonable assumption
6. **Flag for review** - Note that clarification needed

### What to Flag

- Ambiguous specifications
- Conflicting requirements
- Performance concerns
- Breaking changes required
- Security considerations
- Complex architectural decisions

---

## 📈 Progress Milestones

### Week 1 Checkpoint
- [ ] Battery C++ core complete (8 tasks)
- [ ] Compilation successful
- [ ] Basic battery functionality working

### Week 2 Checkpoint
- [ ] Battery Python + RPC complete (8 tasks)
- [ ] Python API callable
- [ ] Battery tests passing

### Week 3 Checkpoint
- [ ] Battery 100% complete (28 tasks)
- [ ] All battery tests passing
- [ ] Battery feature ready for use

### Week 4-5 Checkpoint
- [ ] Clock Control complete (24 tasks)
- [ ] All clock tests passing
- [ ] Clock feature ready for use

### Week 6-7 Checkpoint
- [ ] Integration testing complete
- [ ] Multi-feature scenarios working
- [ ] Performance validated

### Week 8 Final
- [ ] All 87 tasks complete
- [ ] All 200 tests passing
- [ ] Documentation complete
- [ ] Ready for release

---

## 🚀 Getting Started

**Your first action should be:**

1. Read IMPLEMENTATION_ROADMAP.md completely
2. Read TECH_SPEC_BATTERY.md completely
3. Read CPP_IMPLEMENTATION_GUIDE.md completely
4. Read TASK_LIST_DETAILED.md - Battery section
5. Read TESTING_PLAN.md - Battery section
6. Examine existing Cosys-AirSim sensor implementations
7. Begin TASK-BAT-001

**Remember:**
- Work systematically through the task list
- Test thoroughly after each component
- Follow coding standards exactly
- Don't skip tasks
- Ask for clarification when needed

**Good luck! The specifications are comprehensive and you have everything you need to succeed.**

---

**Document Version:** 1.0  
**Last Updated:** January 2025  
**Total Implementation Time:** 16-18 weeks  
**Total Tasks:** 87 tasks across 4 priorities
