# ProjectAirSim Integration - Master Technical Specification

**Document Version:** 3.0  
**Date:** January 2025  
**Status:** Comprehensive Technical Specification  
**Based On:** Complete codebase gap analysis

---

## Document Overview

This is the **master technical specification** for integrating ProjectAirSim features into Cosys-AirSim. This document is based on a complete automated scan and comparison of both codebases.

**Related Documents:**
- `01_IMPLEMENTATION_ROADMAP.md` - Phased implementation timeline
- `02_DETAILED_TASK_LIST.md` - Granular tasks with acceptance criteria
- `03_TESTING_STRATEGY.md` - Comprehensive testing plan
- `04_AI_AGENT_GUIDE.md` - AI agent implementation instructions

---

## Executive Summary

### Reality Check: Adaptation vs. Copy

**Critical Understanding:**
This is an **ADAPTATION** project, not a copy-paste integration. We're extracting valuable concepts from ProjectAirSim and adapting them to fit Cosys-AirSim's proven architecture.

**What This Means:**
- Study ProjectAirSim's approach (how they solved problems)
- Adapt solutions to Cosys-AirSim's patterns (not copy their code)
- Extend existing classes (not create parallel systems)
- Maintain backward compatibility (no breaking changes)

**Philosophy:**
> "Take inspiration from ProjectAirSim's design,  
> implement using Cosys-AirSim's patterns,  
> preserve what already works."

### Scope

**Adaptation** of **selected concepts** from ProjectAirSim into Cosys-AirSim, representing approximately **20 weeks** of careful integration (not 33 weeks of blind copying).

### Key Deliverables

1. **Enhanced Vehicle Configuration** (Adapted from ProjectAirSim's Robot/Link concepts)
   - Extend MultirotorParams with detailed inertia properties
   - Add aerodynamic drag parameters
   - Enhance PhysicsBody to use full inertia tensors

2. **3 New Sensors** (Following Existing Cosys Patterns)
   - Radar sensor (adapted from ProjectAirSim's radar)
   - Airspeed sensor (adapted concept, Cosys implementation)
   - Battery sensor (adapted concept, Cosys implementation)

3. **Enhanced Existing Sensors** (Not New Classes)
   - GPS: Add EPH, EPV, fix type, satellite count
   - IMU: Add bias simulation and temperature effects  
   - Magnetometer: Add declination, inclination
   - Barometer: Add temperature model

4. **Selected World API Methods** (Not All 76)
   - Clock control (9 methods) - High value
   - Weather APIs (8 methods) - Commonly requested
   - Spatial queries (3 methods) - Useful
   - **Skip:** Trajectory management, mission planning (low ROI)

5. **Actuator Framework** (New Classes, Cosys Patterns)
   - Rotor actuator with advanced physics
   - Gimbal actuator for camera stabilization
   - Follow SensorBase architectural pattern

6. **Configuration Extensions** (Not New Format)
   - Extend settings.json with optional fields
   - Keep JSON format (not JSONC)
   - 100% backward compatible

### Implementation Strategy

**Revised Phased Approach (Realistic):**
- **Tier 1 (Weeks 1-3):** New sensors (Battery, Airspeed, Radar) - Following existing patterns
- **Tier 2 (Weeks 4-5):** Enhanced sensors (GPS, IMU, Magnetometer, Barometer) - Extend existing
- **Tier 3 (Weeks 6-8):** Vehicle enhancements (Inertial properties, actuators) - Adapt concepts
- **Tier 4 (Weeks 9-11):** Clock control APIs (9 methods) - New functionality
- **Tier 5 (Weeks 12-14):** Weather APIs (8 methods) - High-value additions
- **Tier 6 (Weeks 15-17):** Spatial queries (3 methods) - Useful utilities
- **Tier 7 (Weeks 18-20):** Polish, testing, documentation - Production ready

**Total:** 20 weeks (vs original 33) - 40% faster through smart adaptation

**Deferred/Out of Scope:**
- ❌ Full Robot/Link/Joint architecture (too complex, low ROI)
- ❌ JSONC configuration (unnecessary, breaks compatibility)
- ❌ Topic/Service messaging (Cosys uses RPC successfully)
- ❌ Trajectory management (68 methods) - Complex, niche use case
- ❌ Mission planning framework - Better as separate package
- ❌ Path planning (RRT, A*) - Better as separate library

---

## Part 1: Architecture Overview

### 1.1 Current Cosys-AirSim Architecture

```
Cosys-AirSim
├── AirLib/
│   ├── include/
│   │   ├── vehicles/
│   │   │   ├── multirotor/   # Multirotor-specific
│   │   │   └── car/          # Car-specific
│   │   ├── sensors/          # 10 sensor types
│   │   ├── api/              # RPC API base classes
│   │   └── common/           # Common utilities
│   └── src/
└── Unreal/
    └── Plugins/
        └── AirSim/           # Unreal integration
```

**Characteristics:**
- Vehicle-specific implementations (no generic robot)
- JSON configuration (strict, no comments)
- RPC-based API (RpcLibClient)
- Physics: SimpleFlight, PX4, PhysX
- Sensor-focused architecture

### 1.2 Target ProjectAirSim Architecture

```
ProjectAirSim
├── core_sim/
│   ├── include/core_sim/
│   │   ├── actor.hpp          # Base actor
│   │   ├── robot.hpp          # Generic robot (KEY!)
│   │   ├── link.hpp           # Robot components
│   │   ├── joint.hpp          # Component connections
│   │   ├── actuators/         # 7 actuator types
│   │   ├── sensors/           # 10 sensor types
│   │   ├── topic.hpp          # Topic messaging
│   │   ├── service.hpp        # Service architecture
│   │   └── ...
│   └── src/
├── client/
│   └── python/
│       └── projectairsim/
│           ├── world.py       # 76 methods
│           └── drone.py       # 45+ methods
└── unreal/
    └── Blocks/
        └── Plugins/
            └── ProjectAirSim/
```

**Characteristics:**
- Generic robot framework (links, joints, actuators)
- JSONC configuration (JSON with comments)
- Topic + Service architecture
- Three physics modes (Non-physics, Fast, Unreal)
- Modular, extensible design

### 1.3 Integration Strategy

**Approach:** **Adaptive Integration** (Not Direct Copy!)

**Core Philosophy:** Adapt ProjectAirSim concepts to fit Cosys-AirSim's proven architecture, don't replace it.

**Integration Principles:**
1. **Extend, Don't Replace** - Build on existing Cosys classes (MultirotorParams, SensorBase, etc.)
2. **Cosys Patterns First** - Follow existing coding patterns, naming conventions, and architecture
3. **Selective Adoption** - Take only what adds value, not everything from ProjectAirSim
4. **Incremental Enhancement** - Small, testable additions that maintain stability

**What We're Actually Doing:**

**Phase 1: Enhance Existing Sensors** (Not New Classes)
- Extend existing `GpsBase` with EPH/EPV fields (don't create new GPS class)
- Extend existing `ImuBase` with bias simulation (build on what exists)
- Extend existing `BarometerBase` with temperature model

**Phase 2: Add Missing Sensors** (Following Cosys Patterns)
- Create `BatteryBase`/`BatterySimple` following exact pattern of existing sensors
- Create `AirspeedBase`/`AirspeedSimple` inheriting from `SensorBase`
- Create `RadarBase`/`RadarSimple` with same structure as `LidarBase`

**Phase 3: Adapt Robot Concepts** (Not Copy Framework)
- Study how ProjectAirSim uses Link/Joint
- Implement similar concepts using Cosys-AirSim's existing `PhysicsBody` class
- Extend `MultirotorParams` to support additional inertial properties
- Use existing `Kinematics` class, don't create new transform system

**Phase 4: Extend APIs** (Maintain Existing Interface)
- Add methods to existing `WorldSimApiBase` (don't create new World class)
- Extend `VehicleSimApiBase` (don't create new vehicle APIs)
- Keep RPC architecture (don't switch to Topic/Service)

**Benefits:**
- ✅ 100% backward compatible (existing code untouched)
- ✅ Lower risk (small incremental changes)
- ✅ Maintainable (follows established Cosys patterns)
- ✅ Realistic effort (no architectural rewrite needed)
- ✅ Community friendly (familiar APIs for existing users)

---

## Part 2: Critical Feature Specifications

### 2.1 Dual-Mode Vehicle Architecture (Simple + Advanced)

#### 2.1.1 Overview - Two Modes Available

**Purpose:** Support both simple and advanced vehicle configurations, giving users choice based on their needs.

**Two Architecture Modes:**

**Mode 1: Simple Architecture (DEFAULT)**
- Extends existing MultirotorParams with enhanced fields
- Single rigid body with enhanced inertial properties
- Easy to configure, backward compatible
- **Best for:** Standard drones, simple vehicles, quick prototyping

**Mode 2: Advanced Architecture (OPTIONAL)**
- Full Robot/Link/Joint framework from ProjectAirSim
- Multi-body articulated systems
- Complex kinematic chains
- **Best for:** Robotic arms, legged robots, complex articulated vehicles

**Configuration Switch:**
```json
{
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "PhysicsMode": "Simple",  // or "Advanced" for Robot/Link/Joint
      // ... rest of config depends on mode
    }
  }
}
```

**Key Adaptations:**
1. **Simple Mode:** Extend MultirotorParams (minimal changes)
2. **Advanced Mode:** Implement Robot/Link/Joint classes (ProjectAirSim approach)
3. **Runtime Selection:** Choose at configuration load time
4. **Shared APIs:** Both modes expose same vehicle control APIs

#### 2.1.2 Mode 1: Simple Architecture (Enhanced MultirotorParams)

**When to Use:** Standard drones, single rigid body vehicles, quick setup

**Current Cosys Structure:**
```cpp
struct MultirotorParams : public VehicleParams {
    MultirotorApiParams api_params;
    std::map<std::string, std::unique_ptr<SensorBase>> sensors;
    // Existing simple params...
};
```

**Enhanced Structure (Adding ProjectAirSim Concepts):**
```cpp
struct MultirotorParams : public VehicleParams {
    // Existing fields preserved...
    MultirotorApiParams api_params;
    std::map<std::string, std::unique_ptr<SensorBase>> sensors;
    
    // NEW: Physics mode selector
    enum class PhysicsMode { Simple, Advanced };
    PhysicsMode physics_mode = PhysicsMode::Simple;  // Default
    
    // NEW: Enhanced inertial properties (for Simple mode)
    struct InertialParams {
        real_T mass = 1.0f;
        Matrix3x3r inertia_matrix = Matrix3x3r::Identity();  // Full 3x3 matrix
        Vector3r center_of_mass = Vector3r::Zero();
        
        // Aerodynamics (from ProjectAirSim)
        real_T drag_coefficient = 0.3f;
        Vector3r reference_area = Vector3r(0.1f, 0.1f, 0.1f);  // m^2 per axis
    } inertial;
    
    // NEW: Actuator configurations (adapted concept)
    struct RotorParams {
        real_T thrust_coefficient = 1.0f;
        real_T torque_coefficient = 0.01f;
        real_T max_rpm = 9000.0f;
        real_T motor_time_constant = 0.02f;
    };
    std::vector<RotorParams> rotor_params;  // One per rotor
    
    // NEW: Robot configuration (for Advanced mode)
    std::unique_ptr<RobotConfig> robot_config;  // nullptr in Simple mode
};
```

**Simple Mode Configuration:**
```json
{
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "PhysicsMode": "Simple",
      "Inertial": {
        "Mass": 1.5,
        "InertiaTensor": [[0.029, 0, 0], [0, 0.029, 0], [0, 0, 0.055]],
        "CenterOfMass": [0, 0, 0],
        "DragCoefficient": 0.3
      }
    }
  }
}
```

**Implementation Notes:**
- Extends existing struct, doesn't replace it
- Backward compatible (new fields have defaults)
- Uses existing Cosys types (Vector3r, Matrix3x3r, real_T)
- Follows Cosys naming conventions (snake_case for members)

#### 2.1.3 Mode 2: Advanced Architecture (Robot/Link/Joint Framework)

**When to Use:** Articulated robots, multi-body systems, complex kinematics

**New Classes (Following ProjectAirSim Design):**

**1. Link Class - Physical Component:**
```cpp
// File: AirLib/include/vehicles/robot/Link.hpp
class Link {
public:
    struct InertialProperties {
        real_T mass;
        Matrix3x3r inertia_tensor;  // Full 3x3 inertia matrix
        Vector3r center_of_mass;
        real_T drag_coefficient;
        Vector3r reference_area;
    };
    
    struct CollisionGeometry {
        enum Type { Box, Sphere, Cylinder, Mesh };
        Type type;
        Vector3r dimensions;  // Size parameters
        std::string mesh_file;  // For Mesh type
    };
    
    Link(const std::string& name, const InertialProperties& inertial);
    
    const std::string& getName() const;
    const InertialProperties& getInertial() const;
    void setCollisionGeometry(const CollisionGeometry& collision);
    
    // Physics integration
    void applyForce(const Vector3r& force, const Vector3r& point);
    void applyTorque(const Vector3r& torque);
    
private:
    std::string name_;
    InertialProperties inertial_;
    CollisionGeometry collision_;
    Pose pose_;  // Current pose in world frame
};
```

**2. Joint Class - Connections Between Links:**
```cpp
// File: AirLib/include/vehicles/robot/Joint.hpp
class Joint {
public:
    enum class Type {
        Fixed,       // No motion (rigid connection)
        Revolute,    // Rotation around axis with limits
        Continuous,  // Unlimited rotation (wheels, propellers)
        Prismatic    // Linear sliding motion
    };
    
    Joint(const std::string& name, Type type, 
          Link* parent, Link* child, const Vector3r& axis);
    
    // Getters
    const std::string& getName() const;
    Type getType() const;
    Link* getParentLink() const;
    Link* getChildLink() const;
    
    // Joint state
    void setPosition(real_T position);  // Angle (rad) or distance (m)
    real_T getPosition() const;
    void setVelocity(real_T velocity);
    real_T getVelocity() const;
    
    // Limits
    void setLimits(real_T min, real_T max);
    void setDamping(real_T damping);
    
    // Compute transform from parent to child
    Pose computeChildPose() const;
    
private:
    std::string name_;
    Type type_;
    Link* parent_link_;
    Link* child_link_;
    Vector3r axis_;  // Rotation or translation axis
    real_T position_ = 0.0f;
    real_T velocity_ = 0.0f;
    real_T min_limit_ = -M_PI;
    real_T max_limit_ = M_PI;
    real_T damping_ = 0.0f;
    Pose origin_;  // Joint origin relative to parent
};
```

**3. Robot Class - Complete System:**
```cpp
// File: AirLib/include/vehicles/robot/Robot.hpp
class Robot {
public:
    Robot(const std::string& name);
    
    // Build robot structure
    Link* addLink(const std::string& name, const Link::InertialProperties& inertial);
    Joint* addJoint(const std::string& name, Joint::Type type,
                   const std::string& parent_link, const std::string& child_link,
                   const Vector3r& axis);
    
    // Query structure
    Link* getLink(const std::string& name);
    Joint* getJoint(const std::string& name);
    std::vector<Link*> getAllLinks();
    std::vector<Joint*> getAllJoints();
    
    // Sensors and actuators
    void attachSensor(const std::string& link_name, std::unique_ptr<SensorBase> sensor);
    void attachActuator(const std::string& link_name, std::unique_ptr<ActuatorBase> actuator);
    
    // Physics update
    void update(real_T dt);
    void computeForwardKinematics();  // Update all link poses
    
    // State
    Pose getBasePose() const;
    void setBasePose(const Pose& pose);
    
private:
    std::string name_;
    std::map<std::string, std::unique_ptr<Link>> links_;
    std::map<std::string, std::unique_ptr<Joint>> joints_;
    std::map<std::string, std::vector<std::unique_ptr<SensorBase>>> link_sensors_;
    std::map<std::string, std::vector<std::unique_ptr<ActuatorBase>>> link_actuators_;
    Link* base_link_ = nullptr;  // Root of kinematic tree
};
```

**Advanced Mode Configuration:**
```json
{
  "Vehicles": {
    "ArticulatedDrone": {
      "VehicleType": "SimpleFlight",
      "PhysicsMode": "Advanced",
      "Robot": {
        "Links": [
          {
            "Name": "body",
            "Mass": 1.5,
            "Inertia": [[0.029, 0, 0], [0, 0.029, 0], [0, 0, 0.055]],
            "CenterOfMass": [0, 0, 0],
            "Collision": {"Type": "Box", "Size": [0.3, 0.3, 0.1]}
          },
          {
            "Name": "arm1",
            "Mass": 0.2,
            "Inertia": [[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]]
          }
        ],
        "Joints": [
          {
            "Name": "arm1_joint",
            "Type": "Revolute",
            "Parent": "body",
            "Child": "arm1",
            "Axis": [0, 0, 1],
            "Origin": [0.15, 0, 0],
            "Limits": [-1.57, 1.57],
            "Damping": 0.1
          }
        ]
      }
    }
  }
}
```

---

#### 2.1.4 Unified PhysicsBody Interface (Works with Both Modes)

The PhysicsBody class serves as the physics abstraction layer for both Simple and Advanced modes.

**Current Cosys PhysicsBody:**
```cpp
class PhysicsBody {
public:
    void setMass(real_T mass);
    void setInertia(const Vector3r& inertia);  // Diagonal only
    // ... existing methods
};
```

**Enhanced PhysicsBody (Mode-Aware):**
```cpp
class PhysicsBody {
public:
    // Existing methods preserved (backward compatible)
    void setMass(real_T mass);
    void setInertia(const Vector3r& inertia);  // Keep for backward compat
    
    // NEW: Full inertia tensor support (both modes)
    void setInertiaMatrix(const Matrix3x3r& inertia_matrix);
    Matrix3x3r getInertiaMatrix() const;
    
    // NEW: Center of mass offset (both modes)
    void setCenterOfMass(const Vector3r& com);
    Vector3r getCenterOfMass() const;
    
    // NEW: Aerodynamic drag (both modes)
    void setDragCoefficient(real_T drag_coeff);
    void setReferenceArea(const Vector3r& ref_area);
    
    // NEW: Multi-body support (Advanced mode only)
    void setRobotConfig(Robot* robot);  // nullptr in Simple mode
    void updateMultiBodyPhysics(real_T dt);  // Called only if robot_config != nullptr
    
private:
    // Simple mode fields
    Matrix3x3r inertia_matrix_;  // Falls back to diagonal if not set
    Vector3r center_of_mass_ = Vector3r::Zero();
    real_T drag_coefficient_ = 0.0f;
    Vector3r reference_area_ = Vector3r::Zero();
    
    // Advanced mode fields
    Robot* robot_config_ = nullptr;  // Non-null only in Advanced mode
};
```

**Mode-Specific Behavior:**

**Simple Mode:**
- `robot_config_` is nullptr
- Physics computed as single rigid body
- Inertia matrix used directly in torque calculations
- Center of mass offset applied to body forces

**Advanced Mode:**
- `robot_config_` points to Robot object
- `updateMultiBodyPhysics()` iterates through all Links
- Joint constraints enforced between Links
- Forward kinematics updates Link poses
- Each Link has its own PhysicsBody (recursive structure)

#### 2.1.5 Configuration Parsing (Mode Detection)

#### 2.1.5 Configuration Parsing (Mode Detection)

**Approach:** Parser detects PhysicsMode and constructs appropriate vehicle representation.

**Settings Parser Logic:**
```cpp
// In SettingsParser.cpp
MultirotorParams MultirotorParamsFactory::createConfig(const rapidjson::Value& config) {
    MultirotorParams params;
    
    // Detect physics mode (default: Simple)
    PhysicsMode mode = PhysicsMode::Simple;
    if (config.HasMember("PhysicsMode")) {
        std::string mode_str = config["PhysicsMode"].GetString();
        if (mode_str == "Advanced") {
            mode = PhysicsMode::Advanced;
        }
    }
    params.physics_mode = mode;
    
    if (mode == PhysicsMode::Simple) {
        // Parse Simple mode configuration
        if (config.HasMember("Inertial")) {
            params.inertial = parseInertialParams(config["Inertial"]);
        }
        if (config.HasMember("Rotors")) {
            params.rotors = parseRotorArray(config["Rotors"]);
        }
    }
    else {  // Advanced mode
        // Parse Robot/Link/Joint structure
        if (config.HasMember("Robot")) {
            params.robot_config = parseRobotConfig(config["Robot"]);
        }
    }
    
    // Parse common fields (sensors, initial pose, etc.)
    // ...
    
    return params;
}
```

**Mode Validation:**
- If `PhysicsMode` is "Advanced" but `Robot` section is missing → Error
- If `PhysicsMode` is "Simple" but `Robot` section is present → Warning, ignore Robot section
- If `PhysicsMode` is omitted → Default to Simple mode

#### 2.1.6 Adaptation Summary

#### 2.1.6 Adaptation Summary

**What We're Implementing:**

**Mode 1 - Simple (Default):**
- ✅ Extending MultirotorParams with full inertial properties
- ✅ Enhancing PhysicsBody with inertia tensor support
- ✅ Adding optional advanced fields to settings.json
- ✅ Creating new actuator classes (following SensorBase pattern)
- ✅ Using existing Kinematics class for transforms
- ✅ Single rigid body physics (current Cosys approach enhanced)

**Mode 2 - Advanced (Optional):**
- ✅ Implementing Robot/Link/Joint class hierarchy (ProjectAirSim architecture)
- ✅ Multi-body articulated system physics
- ✅ Joint constraints and forward kinematics
- ✅ Per-Link collision geometry and inertial properties
- ✅ Complex robotic systems (articulated drones, arms, legged robots)

**What We're NOT Doing (ProjectAirSim features we're skipping):**
- ❌ Switching to JSONC configuration format (keeping JSON)
- ❌ Implementing Topic/Service architecture (overkill for this use case)
- ❌ Creating new Transform Tree system (Kinematics class sufficient)
- ❌ Full ROS2-style messaging system

**Migration Path:**
1. **Phase 1:** Implement Simple mode enhancements (backward compatible)
2. **Phase 2:** Implement Advanced mode classes (Robot/Link/Joint)
3. **Phase 3:** Integrate both modes into PhysicsBody
4. **Phase 4:** Update settings parser for mode detection
5. **Phase 5:** Test both modes independently and together

**Code Impact:**

**Simple Mode:**
- Modifications: ~10 files (extend existing)
- New files: ~5 files (actuator classes)
- Lines changed: ~500
- Breaking changes: **ZERO**

**Advanced Mode:**
- New files: ~15 files (Robot/Link/Joint classes)
- Lines added: ~2000
- Breaking changes: **ZERO** (completely optional mode)

**Backward Compatibility:**
- Existing configs work unchanged (default to Simple mode)
- No PhysicsMode specified → Simple mode
- All existing vehicle types work as before
- New features opt-in only

#### 2.1.7 Realistic Implementation Estimate

**Simple Mode Work:**

**File Modifications** (extending existing):
1. `MultirotorParams.hpp` - Add InertialParams, PhysicsMode enum (+40 lines)
2. `PhysicsBody.hpp/cpp` - Add inertia matrix methods (+60 lines)
3. `SettingsParser.cpp` - Parse new JSON fields (+100 lines)
4. `MultirotorPhysicsBody.cpp` - Use new inertial properties (+50 lines)

**New Files** (following existing patterns):
1. `RotorActuator.hpp/cpp` - Rotor physics model (+150 lines)
2. `ActuatorBase.hpp` - Base class like SensorBase (+50 lines)

**Testing:**
1. Unit tests for inertia calculations (5 tests)
2. Integration test: Load enhanced config (1 test)
3. System test: Fly with new inertial properties (1 test)

**Simple Mode Effort:** ~2 weeks  
**Risk:** LOW (small, isolated changes)

---

**Advanced Mode Work:**

**New Files** (Robot/Link/Joint system):
1. `Robot.hpp/cpp` - Robot container class (~300 lines)
2. `Link.hpp/cpp` - Physical link class (~250 lines)
3. `Joint.hpp/cpp` - Joint base class (~200 lines)
4. `RevoluteJoint.hpp/cpp` - Revolute joint type (~150 lines)
5. `PrismaticJoint.hpp/cpp` - Prismatic joint type (~150 lines)
6. `FixedJoint.hpp/cpp` - Fixed joint type (~100 lines)
7. `ContinuousJoint.hpp/cpp` - Continuous joint type (~120 lines)
8. `CollisionGeometry.hpp/cpp` - Collision shapes (~200 lines)
9. `RobotParser.hpp/cpp` - Parse Robot JSON config (~250 lines)

**Integration Files:**
1. `PhysicsBody.cpp` - Add multi-body physics update (~100 lines)
2. `MultirotorPhysicsBody.cpp` - Mode branching logic (+50 lines)
3. `SettingsParser.cpp` - Robot config parsing (+150 lines)

**Testing:**
1. Unit tests for each Joint type (4 tests)
2. Unit tests for Link physics (3 tests)
3. Unit tests for Robot structure (5 tests)
4. Integration test: Simple articulated robot (2 tests)
5. Integration test: Complex multi-body system (2 tests)
6. System test: Articulated drone flight (1 test)

**Advanced Mode Effort:** ~4-5 weeks  
**Risk:** MEDIUM (new architecture, complex physics)

---

**Total Dual-Mode Effort:** ~6-7 weeks  
- Simple Mode: 2 weeks (low risk)
- Advanced Mode: 4-5 weeks (medium risk)
- Testing both modes: Additional 1 week overlap

**Breaking Changes:** ZERO (both modes fully backward compatible)

---

### 2.2 Configuration Enhancement (Dual-Mode Settings)

#### 2.2.1 Format Decision

**ProjectAirSim Uses:** JSONC (JSON with Comments)  
**Cosys-AirSim Uses:** JSON (everywhere in codebase)  
**What We Should Do:** Stick with JSON

**Reasons:**
1. Cosys-AirSim already has robust JSON parsing (RapidJSON)
2. All existing tools, scripts, and documentation use JSON
3. JSONC would require rewriting parser and breaking compatibility
4. **Comments can be added in documentation, not config files**

**Decision:** **Do NOT implement JSONC**. Keep JSON format.

#### 2.2.2 Dual-Mode Configuration Examples

**Mode 1 - Simple Mode Configuration (Backward Compatible):**
```json
{
  "SettingsVersion": 1.3,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "PhysicsMode": "Simple",
      "X": 0, "Y": 0, "Z": -2,
      
      "Inertial": {
        "Mass": 1.5,
        "InertiaTensor": [
          [0.029, 0, 0],
          [0, 0.029, 0],
          [0, 0, 0.055]
        ],
        "CenterOfMass": [0, 0, 0],
        "DragCoefficient": 0.3,
        "ReferenceArea": [0.1, 0.1, 0.05]
      },
      
      "Rotors": [
        {
          "Position": [0.253, -0.253, 0],
          "Direction": 1,
          "ThrustCoefficient": 1.0,
          "TorqueCoefficient": 0.01,
          "MaxRPM": 9000
        },
        {
          "Position": [0.253, 0.253, 0],
          "Direction": -1,
          "ThrustCoefficient": 1.0,
          "TorqueCoefficient": 0.01,
          "MaxRPM": 9000
        }
      ],
      
      "Sensors": {
        "Imu": {
          "SensorType": 2,
          "Enabled": true
        }
      }
    }
  }
}
```

**Mode 2 - Advanced Mode Configuration (Articulated Systems):**
```json
{
  "SettingsVersion": 1.3,
  "SimMode": "Multirotor",
  "Vehicles": {
    "ArticulatedDrone": {
      "VehicleType": "SimpleFlight",
      "PhysicsMode": "Advanced",
      "X": 0, "Y": 0, "Z": -2,
      
      "Robot": {
        "BaseLinkName": "body",
        "Links": [
          {
            "Name": "body",
            "Mass": 1.5,
            "Inertia": [[0.029, 0, 0], [0, 0.029, 0], [0, 0, 0.055]],
            "CenterOfMass": [0, 0, 0],
            "DragCoefficient": 0.3,
            "Collision": {
              "Type": "Box",
              "Size": [0.3, 0.3, 0.1]
            }
          },
          {
            "Name": "arm1",
            "Mass": 0.2,
            "Inertia": [[0.001, 0, 0], [0, 0.002, 0], [0, 0, 0.001]],
            "CenterOfMass": [0.1, 0, 0],
            "Collision": {
              "Type": "Cylinder",
              "Radius": 0.02,
              "Height": 0.2
            }
          },
          {
            "Name": "arm2",
            "Mass": 0.2,
            "Inertia": [[0.001, 0, 0], [0, 0.002, 0], [0, 0, 0.001]],
            "CenterOfMass": [0.1, 0, 0],
            "Collision": {
              "Type": "Cylinder",
              "Radius": 0.02,
              "Height": 0.2
            }
          }
        ],
        "Joints": [
          {
            "Name": "arm1_joint",
            "Type": "Revolute",
            "Parent": "body",
            "Child": "arm1",
            "Axis": [0, 0, 1],
            "Origin": [0.15, 0.15, 0],
            "Limits": [-1.57, 1.57],
            "Damping": 0.1
          },
          {
            "Name": "arm2_joint",
            "Type": "Revolute",
            "Parent": "body",
            "Child": "arm2",
            "Axis": [0, 0, 1],
            "Origin": [-0.15, -0.15, 0],
            "Limits": [-1.57, 1.57],
            "Damping": 0.1
          }
        ],
        "Actuators": [
          {
            "Type": "Rotor",
            "LinkName": "body",
            "Position": [0.15, -0.15, 0],
            "Direction": 1,
            "ThrustCoefficient": 0.8,
            "MaxRPM": 9000
          },
          {
            "Type": "Rotor",
            "LinkName": "arm1",
            "Position": [0.2, 0, 0],
            "Direction": -1,
            "ThrustCoefficient": 0.6,
            "MaxRPM": 8000
          },
          {
            "Type": "Rotor",
            "LinkName": "arm2",
            "Position": [0.2, 0, 0],
            "Direction": 1,
            "ThrustCoefficient": 0.6,
            "MaxRPM": 8000
          }
        ]
      },
      
      "Sensors": {
        "Imu": {
          "SensorType": 2,
          "Enabled": true,
          "LinkName": "body"
        }
      }
    }
  }
}
```

**Legacy Configuration (No PhysicsMode = Default Simple Mode):**
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": -2,
      "Sensors": {
        "Imu": {
          "SensorType": 2,
          "Enabled": true
        }
      }
    }
  }
}
```
This config continues to work unchanged - defaults to Simple mode with existing default parameters.

#### 2.2.3 Parser Implementation

**Extend JSON parser to handle both modes:**

```cpp
// In SettingsParser.cpp
MultirotorParams MultirotorParamsFactory::createConfig(const rapidjson::Value& config) {
    MultirotorParams params;
    
    // Step 1: Detect physics mode (default: Simple)
    PhysicsMode mode = PhysicsMode::Simple;
    if (config.HasMember("PhysicsMode")) {
        std::string mode_str = config["PhysicsMode"].GetString();
        if (mode_str == "Advanced") {
            mode = PhysicsMode::Advanced;
        } else if (mode_str != "Simple") {
            throw std::runtime_error("Invalid PhysicsMode: " + mode_str + 
                                     " (must be 'Simple' or 'Advanced')");
        }
    }
    params.physics_mode = mode;
    
    // Step 2: Parse mode-specific configuration
    if (mode == PhysicsMode::Simple) {
        // Parse Simple mode - inertial properties
        if (config.HasMember("Inertial")) {
            params.inertial = parseInertialParams(config["Inertial"]);
        } else {
            // Use defaults
            params.inertial = getDefaultInertialParams();
        }
        
        // Parse Simple mode - rotors
        if (config.HasMember("Rotors")) {
            params.rotors = parseRotorArray(config["Rotors"]);
        } else {
            // Use default quad configuration
            params.rotors = getDefaultQuadRotors();
        }
    }
    else {  // Advanced mode
        // Parse Robot/Link/Joint structure
        if (!config.HasMember("Robot")) {
            throw std::runtime_error("PhysicsMode 'Advanced' requires 'Robot' configuration section");
        }
        params.robot_config = parseRobotConfig(config["Robot"]);
    }
    
    // Step 3: Parse common fields (sensors, initial pose, etc.)
    if (config.HasMember("Sensors")) {
        params.sensors = parseSensors(config["Sensors"], mode);
    }
    
    // Initial pose
    if (config.HasMember("X")) params.initial_pose.position.x() = config["X"].GetFloat();
    if (config.HasMember("Y")) params.initial_pose.position.y() = config["Y"].GetFloat();
    if (config.HasMember("Z")) params.initial_pose.position.z() = config["Z"].GetFloat();
    
    return params;
}

// Helper: Parse Robot configuration for Advanced mode
std::unique_ptr<Robot> parseRobotConfig(const rapidjson::Value& robot_config) {
    std::string base_link_name = robot_config["BaseLinkName"].GetString();
    auto robot = std::make_unique<Robot>(base_link_name);
    
    // Parse Links
    if (robot_config.HasMember("Links")) {
        const auto& links = robot_config["Links"];
        for (rapidjson::SizeType i = 0; i < links.Size(); i++) {
            const auto& link_json = links[i];
            
            Link::InertialProperties inertial;
            inertial.mass = link_json["Mass"].GetFloat();
            inertial.inertia_tensor = parseMatrix3x3(link_json["Inertia"]);
            inertial.center_of_mass = parseVector3(link_json["CenterOfMass"]);
            
            if (link_json.HasMember("DragCoefficient")) {
                inertial.drag_coefficient = link_json["DragCoefficient"].GetFloat();
            }
            
            Link* link = robot->addLink(link_json["Name"].GetString(), inertial);
            
            // Parse collision geometry
            if (link_json.HasMember("Collision")) {
                link->setCollisionGeometry(parseCollisionGeometry(link_json["Collision"]));
            }
        }
    }
    
    // Parse Joints
    if (robot_config.HasMember("Joints")) {
        const auto& joints = robot_config["Joints"];
        for (rapidjson::SizeType i = 0; i < joints.Size(); i++) {
            const auto& joint_json = joints[i];
            
            std::string type_str = joint_json["Type"].GetString();
            Joint::Type type;
            if (type_str == "Fixed") type = Joint::Type::Fixed;
            else if (type_str == "Revolute") type = Joint::Type::Revolute;
            else if (type_str == "Continuous") type = Joint::Type::Continuous;
            else if (type_str == "Prismatic") type = Joint::Type::Prismatic;
            else throw std::runtime_error("Unknown joint type: " + type_str);
            
            Joint* joint = robot->addJoint(
                joint_json["Name"].GetString(),
                type,
                joint_json["Parent"].GetString(),
                joint_json["Child"].GetString(),
                parseVector3(joint_json["Axis"])
            );
            
            // Optional joint parameters
            if (joint_json.HasMember("Limits")) {
                const auto& limits = joint_json["Limits"];
                joint->setLimits(limits[0].GetFloat(), limits[1].GetFloat());
            }
            if (joint_json.HasMember("Damping")) {
                joint->setDamping(joint_json["Damping"].GetFloat());
            }
            if (joint_json.HasMember("Origin")) {
                joint->setOrigin(parsePose(joint_json["Origin"]));
            }
        }
    }
    
    // Parse Actuators (if present)
    if (robot_config.HasMember("Actuators")) {
        const auto& actuators = robot_config["Actuators"];
        for (rapidjson::SizeType i = 0; i < actuators.Size(); i++) {
            const auto& act_json = actuators[i];
            // Parse and attach actuators to links...
        }
    }
    
    return robot;
}
```

#### 2.2.4 Effort Estimate

**Original Estimate (JSONC):** 1 week  
**Revised Estimate (JSON dual-mode):** 3 days for Simple mode + 4 days for Advanced mode = 1 week total  
**Complexity:** MEDIUM (mode detection and validation logic)

---

### 2.3 Sensor Integration (Adaptation Pattern)

**Key Principle:** Follow existing SensorBase pattern, adapt ProjectAirSim physics, use Cosys types

#### 2.3.1 Radar Sensor (NEW - Follow SensorBase Pattern)

**Purpose:** Detect and track objects using simulated radar

**Data Structure:**
```cpp
struct RadarDetection {
    float range;              // meters
    float velocity;           // m/s (relative radial velocity, Doppler)
    float azimuth;           // radians (horizontal angle)
    float elevation;         // radians (vertical angle)
    float rcs;               // dBsm (Radar Cross Section)
    int object_id;           // Object identifier
};

struct RadarTrack {
    int track_id;
    Vector3r position;       // meters (sensor frame)
    Vector3r velocity;       // m/s (sensor frame)
    float rcs;              // dBsm
    float tracking_time;    // seconds
    int missed_detections;
};

struct RadarData {
    std::vector<RadarDetection> detections;
    std::vector<RadarTrack> tracks;
    int64_t time_stamp;
};
```

**Configuration:**
```cpp
struct RadarParams {
    float range;                    // meters (max detection range)
    float horizontal_fov;           // radians
    float vertical_fov;             // radians
    float azimuth_resolution;       // radians
    float elevation_resolution;     // radians
    int update_frequency;           // Hz
    bool enable_tracking;           // Kalman filter tracking
    float tracking_threshold;       // minimum detections to create track
    int max_tracks;
};
```

**APIs:**
```python
# Python
drone.get_radar_data(sensor_name: str) -> Dict

# C++
RadarData getRadarData(const std::string& sensor_name);
```

#### 2.3.2 Airspeed Sensor (NEW - Same Pattern)

**Purpose:** Measure air velocity relative to vehicle (adapt physics from ProjectAirSim)

**Implementation Pattern:**
```cpp
// Follow Cosys pattern: inherit SensorBase, use Cosys types
class AirspeedSensor : public SensorBase {
public:
    AirspeedSensor(const AirSimSettings::AirspeedSensorSetting& setting,
                   const std::shared_ptr<SensorFactory> factory);
    
    // SensorBase overrides
    void reportState(StateReporter& reporter) override;
    void update() override;
    
    // API method (returns data)
    AirspeedData getAirspeedData() const;
    
private:
    // Adapted physics from ProjectAirSim, using Cosys Vector3r
    float calculateAirspeed(const Vector3r& ground_velocity,
                           const Vector3r& wind_velocity);
};

**Data Structure:**
```cpp
struct AirspeedData {
    float true_airspeed;          // m/s (actual airspeed)
    float indicated_airspeed;     // m/s (pressure-based reading)
    float differential_pressure;  // Pascal
    float temperature;            // Celsius
    int64_t time_stamp;
};
```

**Configuration:**
```cpp
struct AirspeedParams {
    float noise_sigma;
    float offset;                  // Sensor offset
    float update_frequency;        // Hz
    float startup_delay;           // seconds
};
```

**APIs:**
```python
# Python
drone.get_airspeed_data(sensor_name: str) -> Dict

# C++
AirspeedData getAirspeedData(const std::string& sensor_name);
```

#### 2.3.3 Battery Sensor (NEW - Same Pattern)

**Purpose:** Simulate battery discharge (adapt physics from ProjectAirSim)

**Implementation Pattern:**
```cpp
// Same SensorBase inheritance
class BatterySensor : public SensorBase {
public:
    BatterySensor(const AirSimSettings::BatterySensorSetting& setting,
                  const std::shared_ptr<SensorFactory> factory);
    
    void reportState(StateReporter& reporter) override;
    void update() override;
    
    BatteryData getBatteryData() const;
    
    // Control methods (adapted API concepts)
    bool setBatteryRemaining(float percentage);
    bool setBatteryDrainRate(float rate);
    
private:
    // Physics-based depletion (adapted from ProjectAirSim)
    float calculateDischargeCurve(float time_elapsed);
};

**Data Structure:**
```cpp
struct BatteryData {
    float voltage;                 // Volts
    float current;                 // Amperes
    float state_of_charge;         // 0.0-1.0 (percentage)
    float remaining_capacity;      // Ampere-hours
    float total_capacity;          // Ampere-hours
    float temperature;             // Celsius
    bool is_healthy;
    int64_t time_stamp;
};
```

**Configuration:**
```cpp
struct BatteryParams {
    enum class DischargeMode {
        Linear,      // Constant drain rate
        Nonlinear    // Realistic discharge curve
    };
    
    float nominal_voltage;         // Volts (e.g., 12.6V for 3S LiPo)
    float total_capacity;          // Ampere-hours
    float discharge_rate;          // Amperes (initial drain)
    DischargeMode discharge_mode;
    float nonlinear_factor;        // For nonlinear mode
    float update_frequency;        // Hz
};
```

**Control APIs:**
```python
# Python
drone.set_battery_remaining(percentage: float) -> bool
drone.set_battery_drain_rate(rate: float) -> bool
drone.set_battery_health_status(is_healthy: bool) -> bool
drone.get_battery_drain_rate(sensor_name: str) -> Dict

# C++
bool setBatteryRemaining(float percentage);
bool setBatteryDrainRate(float rate);
bool setBatteryHealthStatus(bool is_healthy);
```

#### 2.3.4 Enhanced GPS (EXTEND Existing GpsSimple)

**Pattern:** Extend existing class with new fields (ProjectAirSim concepts), maintain 100% backward compatibility

**Implementation:**
```cpp
// File: AirLib/include/sensors/gps/GpsSimple.hpp
// EXTEND existing class - do NOT replace
class GpsSimple : public GpsBase {
public:
    // ALL EXISTING methods remain unchanged
    GeoPoint getGeoPoint() const;           // Keep existing
    GeoPoint getHomeGeoPoint() const;       // Keep existing
    // ... all existing methods preserved ...
    
    // NEW method (optional enhancement)
    GpsExtendedData getExtendedData() const;  // Returns new fields
    
private:
    // NEW members (optional, with defaults for compatibility)
    float eph_ = 0.5f;          // Default horizontal accuracy
    float epv_ = 0.8f;          // Default vertical accuracy
    int fix_type_ = 3;          // Default 3D fix
    int satellites_visible_ = 10;  // Default count
};
```cpp
struct GpsData {
    // Existing Cosys fields
    double latitude;
    double longitude;
    float altitude;
    Vector3r velocity;
    
    // NEW ProjectAirSim fields
    float eph;                    // Horizontal position accuracy (meters)
    float epv;                    // Vertical position accuracy (meters)
    int fix_type;                 // 0=No fix, 1=Dead reckoning, 2=2D, 3=3D, 4=DGPS, 5=RTK
    int satellites_visible;       // Number of satellites
    
    int64_t time_stamp;
};
```

**Configuration (Backward Compatible):**
```json
{
  "SensorType": 3,  // Existing GPS type
  "SensorName": "Gps1",
  "Enabled": true,
  // EXISTING settings work exactly as before
  "EphTimeConstant": 0.9,
  "EpvTimeConstant": 0.9,
  // NEW optional settings (omit = use defaults)
  "EPH": 0.5,
  "EPV": 0.8,
  "FixType": 3,
  "SatellitesVisible": 10
}
```

**Effort:** 1.5 days (minimal changes, backward compatible)

---

#### 2.3.5 Enhanced IMU (EXTEND Existing ImuSimple)
**Pattern:** Add new optional methods to existing ImuSimple class

**Implementation:**
```cpp
// File: AirLib/include/sensors/imu/ImuSimple.hpp
// EXTEND existing class
class ImuSimple : public ImuBase {
public:
    // ALL EXISTING methods preserved
    Vector3r getAngularVelocity() const;    // Keep existing
    Vector3r getLinearAcceleration() const; // Keep existing
    Quaternionr getOrientation() const;     // Keep existing
    
    // NEW methods (optional enhancements)
    Vector3r getGyroBias() const;           // New: bias estimation
    Vector3r getAccelBias() const;          // New: bias estimation
    float getTemperature() const;           // New: temperature sensing
    
private:
    // NEW members with defaults
    Vector3r gyro_bias_{0,0,0};
    Vector3r accel_bias_{0,0,0};
    float temperature_ = 20.0f;  // Default room temperature
};
```

**Effort:** 2 days (add bias simulation, temperature effects)

---

#### 2.3.6 Enhanced Magnetometer (EXTEND Existing MagnetometerSimple)

**Pattern:** Add declination and bias to existing class

**Implementation:**
```cpp
// File: AirLib/include/sensors/magnetometer/MagnetometerSimple.hpp
// EXTEND existing class
class MagnetometerSimple : public MagnetometerBase {
public:
    // EXISTING methods preserved
    Vector3r getMagneticField() const;      // Keep existing
    
    // NEW methods
    float getMagneticDeclination() const;   // New: magnetic declination
    float getMagneticInclination() const;   // New: magnetic inclination
    Vector3r getBias() const;               // New: bias estimation
    
private:
    // NEW members with defaults
    float declination_ = 0.0f;  // Default no declination
    float inclination_ = 60.0f; // Default inclination for mid-latitudes
    Vector3r bias_{0,0,0};      // Default no bias
};
```

**Effort:** 1 day (simple additions)

---

#### 2.3.7 Enhanced Barometer (EXTEND Existing BarometerSimple)

**Pattern:** Add QNH/QFE pressure references

**Implementation:**
```cpp
// File: AirLib/include/sensors/barometer/BarometerSimple.hpp
// EXTEND existing class
class BarometerSimple : public BarometerBase {
public:
    // EXISTING methods preserved
    BarometerData getBarometerData() const; // Keep existing
    
    // NEW methods
    float getQNH() const;                   // New: QNH pressure
    float getQFE() const;                   // New: QFE pressure
    float getDensityAltitude() const;       // New: density altitude
    void setReferencePressure(float qnh);   // New: set reference
    
private:
    // NEW members with defaults
    float qnh_ = 101325.0f;  // Default sea level pressure (Pa)
    float qfe_ = 101325.0f;  // Default field pressure
};
```

**Effort:** 1 day (pressure calculations)

---

**Sensor Integration Summary:**
- **3 New Sensors:** Battery, Airspeed, Radar (follow SensorBase pattern) - 7 days total
- **4 Enhanced Sensors:** GPS, IMU, Magnetometer, Barometer (extend existing) - 5.5 days total
- **Total Effort:** ~12.5 days (2.5 weeks) for all sensors
- **Pattern:** Study existing sensor → Adapt ProjectAirSim concepts → Implement using Cosys types

---

### 2.4 Actuator Enhancement (Dual-Mode Support)

**Reality Check:** ProjectAirSim has 7 actuator types (Rotor, Gimbal, Tilt, Wheel, Control Surface, Servo, Linear). Cosys-AirSim already handles rotors well. We'll adapt high-value additions for both physics modes.

**What to Implement:**
1. ✅ **Rotor Dynamics Enhancement:** Add motor time constants and improved thrust curves (both modes) (~2 days)
2. ✅ **Gimbal Actuator:** Camera stabilization (useful for cinematography, both modes) (~3 days)
3. ✅ **Actuator Base Class:** Support Advanced mode Link attachment (~1 day)
4. ❌ **Skip Others:** Tilt, Wheel, Control Surface, Servo, Linear (low ROI for typical use cases)

---

#### 2.4.1 Actuator Base Class (NEW - Enables Advanced Mode)

**Purpose:** Abstract actuator interface for both Simple and Advanced modes

**Implementation:**
```cpp
// File: AirLib/include/actuators/ActuatorBase.hpp
class ActuatorBase {
public:
    virtual ~ActuatorBase() = default;
    
    // Core actuator interface
    virtual void update(real_T dt) = 0;
    virtual void reset() = 0;
    
    // Mode-specific attachment
    void attachToBody(PhysicsBody* body);  // Simple mode
    void attachToLink(Link* link);         // Advanced mode
    
    // Apply forces/torques (mode-aware)
    virtual void applyActuation() = 0;
    
protected:
    PhysicsBody* physics_body_ = nullptr;  // Simple mode
    Link* link_ = nullptr;                 // Advanced mode
};
```

**Mode Detection:**
- If `link_` is non-null → Apply forces to Link (Advanced mode)
- If `physics_body_` is non-null → Apply forces to PhysicsBody (Simple mode)

---

#### 2.4.2 Enhanced Rotor Dynamics (BOTH MODES)

**Pattern:** Enhance existing rotor physics for Simple mode, extend for Advanced mode link attachment

**Simple Mode Enhancement:**
```cpp
// EXTEND existing RotorParams in MultirotorParams.hpp
struct RotorParams {
    // EXISTING fields (keep all)
    real_T thrust_factor;
    real_T torque_factor;
    Vector3r position;  // Rotor position in body frame
    // ... other existing fields ...
    
    // NEW fields (optional, with defaults)
    real_T motor_time_constant = 0.02f;  // Default 20ms response
    real_T max_rpm = 6000.0f;            // Maximum RPM
    real_T rotor_diameter = 0.25f;       // Propeller diameter (m)
};
```

**Advanced Mode Enhancement:**
```cpp
// File: AirLib/include/actuators/RotorActuator.hpp
class RotorActuator : public ActuatorBase {
public:
    struct RotorConfig {
        Vector3r position;           // Relative to link origin
        Vector3r thrust_direction;   // Typically [0, 0, 1] or [0, 0, -1]
        real_T thrust_coefficient;
        real_T torque_coefficient;
        real_T motor_time_constant = 0.02f;
        real_T max_rpm = 6000.0f;
        real_T rotor_diameter = 0.25f;
    };
    
    RotorActuator(const RotorConfig& config);
    
    // ActuatorBase overrides
    void update(real_T dt) override;
    void reset() override;
    void applyActuation() override;  // Mode-aware
    
    // Control interface
    void setInput(real_T normalized_input);  // 0.0 to 1.0
    
private:
    RotorConfig config_;
    real_T current_rpm_ = 0.0f;
    real_T target_rpm_ = 0.0f;
    
    // Physics computation (same for both modes)
    Vector3r computeThrustForce() const;
    Vector3r computeTorque() const;
};
```

**Configuration:**

**Simple Mode (Backward Compatible):**
```json
{
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "PhysicsMode": "Simple",
      "Rotors": [
        {
          "Position": [0.253, -0.253, 0],
          "ThrustFactor": 1.0,
          "TorqueFactor": 0.01,
          "MotorTimeConstant": 0.02,
          "MaxRPM": 6000,
          "RotorDiameter": 0.25
        }
      ]
    }
  }
}
```

**Advanced Mode:**
```json
{
  "Vehicles": {
    "ArticulatedDrone": {
      "VehicleType": "SimpleFlight",
      "PhysicsMode": "Advanced",
      "Robot": {
        "Links": [ /* ... */ ],
        "Joints": [ /* ... */ ],
        "Actuators": [
          {
            "Type": "Rotor",
            "LinkName": "body",
            "Position": [0.15, -0.15, 0],
            "ThrustDirection": [0, 0, 1],
            "ThrustCoefficient": 1.0,
            "TorqueCoefficient": 0.01,
            "MotorTimeConstant": 0.02,
            "MaxRPM": 6000,
            "RotorDiameter": 0.25
          },
          {
            "Type": "Rotor",
            "LinkName": "arm1",
            "Position": [0.2, 0, 0],
            "ThrustDirection": [0, 0, -1],
            "ThrustCoefficient": 0.8,
            "MaxRPM": 5500
          }
        ]
      }
    }
  }
}
```

**Effort:** 2 days (enhance existing physics for both modes)

---

#### 2.4.3 Gimbal Actuator (BOTH MODES)

**Purpose:** Camera stabilization for smooth video capture

**Implementation:**
```cpp
// File: AirLib/include/actuators/GimbalActuator.hpp
class GimbalActuator : public ActuatorBase {
public:
    struct GimbalConfig {
        real_T pitch_min = -M_PI_2;     // -90 degrees
        real_T pitch_max = M_PI_4;      // +45 degrees
        real_T roll_min = -M_PI_6;      // -30 degrees
        real_T roll_max = M_PI_6;       // +30 degrees
        real_T yaw_min = -M_PI;         // -180 degrees
        real_T yaw_max = M_PI;          // +180 degrees
        real_T max_angular_velocity = 1.57f;  // 90 deg/s
    };
    
    GimbalActuator(const GimbalConfig& config);
    
    // ActuatorBase overrides
    void update(real_T dt) override;
    void reset() override;
    void applyActuation() override;  // Updates camera orientation
    
    // Control interface
    void setTargetOrientation(const Quaternionr& target);
    void setTargetAngles(real_T pitch, real_T roll, real_T yaw);
    Quaternionr getCurrentOrientation() const;
    
private:
    GimbalConfig config_;
    Quaternionr current_orientation_;
    Quaternionr target_orientation_;
    
    // Clamp angles to limits
    void applyLimits(real_T& pitch, real_T& roll, real_T& yaw);
};
```

**Works in Both Modes:**
- Simple mode: Gimbal attached to main body
- Advanced mode: Gimbal can attach to any Link (e.g., camera mounted on articulated arm)

**Configuration:**
```json
{
  "Cameras": {
    "front_center": {
      "CameraName": "front_center",
      "ImageType": 0,
      "LinkName": "body",
      "Gimbal": {
        "Enabled": true,
        "PitchMin": -90,
        "PitchMax": 45,
        "RollMin": -30,
        "RollMax": 30,
        "YawMin": -180,
        "YawMax": 180,
        "MaxAngularVelocity": 90
      }
    }
  }
}
```

**API Methods:**
```python
# Python
client.setCameraGimbalOrientation(camera_name: str, pitch: float, roll: float, yaw: float)
client.getCameraGimbalOrientation(camera_name: str) -> dict

# C++
void setCameraGimbalOrientation(const std::string& camera_name, 
                                float pitch, float roll, float yaw);
Quaternionr getCameraGimbalOrientation(const std::string& camera_name);
```

**Effort:** 3 days
- 1.5 days: Implement gimbal physics and control
- 1 day: Integrate with camera system (both modes)
- 0.5 day: Add API methods

---

**Actuator Summary:**
- **New Base Class:** ActuatorBase for mode-aware attachment (1 day)
- **Enhanced:** Rotor dynamics for both modes (2 days)
- **New:** Gimbal actuator for both modes (3 days)
- **Skipped:** Tilt, Wheel, Control Surface, Servo, Linear (low ROI)
- **Total Effort:** 6 days
- **Rationale:** Focus on high-value features, ensure both modes supported

---

### 2.5 World API Enhancement (Selective Adaptation)

**Reality Check:** ProjectAirSim has 76 World API methods across many categories (Clock, Weather, Lighting, Spatial Queries, Trajectory, Mission Planning, Path Planning, Scene Management, etc.). We'll implement ONLY high-value methods.

**Selection Criteria:**
- ✅ Widely useful (Clock control, Weather)
- ✅ Easy to implement (Spatial queries)
- ❌ Skip complex/niche features (Trajectory management, Mission planning, Path planning)

**Selected APIs (~20 methods, not all 76):**

---

#### 2.5.1 Clock Control (9 methods - HIGH VALUE)

**Purpose:** Precise simulation time control for testing and research

**Why Include:** Essential for reproducible testing, step-debugging physics, and benchmarking

**Implementation Pattern:** Add to existing RpcLibClientBase and RpcLibServerBase

```python
class World:
    def get_sim_clock_type(self) -> str:
        """Returns: 'steppable', 'scalable', or 'stepped-realtime'"""
        
    def get_sim_time(self) -> int:
        """Returns: Current simulation time in nanoseconds"""
        
    def pause(self) -> str:
        """Pause simulation"""
        
    def resume(self) -> str:
        """Resume simulation"""
        
    def is_paused(self) -> bool:
        """Check if simulation is paused"""
        
    def continue_for_sim_time(self, duration_ns: int, wait_until_complete: bool = True):
        """Advance simulation by duration_ns nanoseconds"""
        
    def continue_until_sim_time(self, target_time_ns: int, wait_until_complete: bool = True):
        """Run simulation until target_time_ns"""
        
    def continue_for_n_steps(self, n_steps: int, wait_until_complete: bool = True):
        """Execute exactly n simulation steps"""
        
    def continue_for_single_step(self, wait_until_complete: bool = True) -> int:
        """Execute one step, return new sim time"""
```

**C++ Implementation (add to RpcLibServerBase):**
```cpp
// File: AirLib/include/api/RpcLibServerBase.hpp
class RpcLibServerBase {
    // EXISTING methods preserved
    
    // NEW Clock control methods
    std::string getSimClockType();              // Returns "steppable" or "scalable"
    int64_t getSimTime();                       // Returns time in nanoseconds
    void pause();
    void resume();
    bool isPaused();
    void continueForSimTime(int64_t duration_ns);
    void continueUntilSimTime(int64_t target_time_ns);
    void continueForNSteps(int n_steps);
    int64_t continueForSingleStep();
    
private:
    // NEW members
    std::atomic<bool> is_paused_{false};
    int64_t sim_time_ns_ = 0;
};
```

**Python API:**
```python
# Add to AirSimClient class
def get_sim_clock_type(self) -> str:
    return self.client.call('getSimClockType')

def get_sim_time(self) -> int:
    return self.client.call('getSimTime')

def pause(self):
    self.client.call('pause')

def resume(self):
    self.client.call('resume')

def is_paused(self) -> bool:
    return self.client.call('isPaused')

def continue_for_sim_time(self, duration_ns: int):
    self.client.call('continueForSimTime', duration_ns)

# ... additional methods
```

**Effort:** 4 days
- 2 days: Implement clock control in RpcLibServerBase
- 1 day: Add Python/C++ client bindings
- 1 day: Testing and documentation

---

#### 2.5.2 Weather Control (8 methods - HIGH VALUE)

**Purpose:** Control environmental conditions (lighting, weather effects)

**Why Include:** Useful for testing perception algorithms under varied conditions

**Selected Methods (not all weather APIs):**
1. `setSunlightIntensity(float)` - Adjust sun brightness
2. `getSunlightIntensity()` - Get current brightness
3. `setCloudShadowStrength(float)` - Cloud shadow intensity
4. `getCloudShadowStrength()` - Get shadow strength
5. `setFogDensity(float)` - Fog density (if supported by renderer)
6. `getFogDensity()` - Get fog density
7. `setRainIntensity(float)` - Rain effect (if supported)
8. `getRainIntensity()` - Get rain intensity

**Implementation:**
```cpp
// Add to RpcLibServerBase
class RpcLibServerBase {
    // NEW Weather methods
    bool setSunlightIntensity(float intensity);  // 0.0 to 1.0
    float getSunlightIntensity();
    bool setCloudShadowStrength(float strength);
    float getCloudShadowStrength();
    bool setFogDensity(float density);
    float getFogDensity();
    bool setRainIntensity(float intensity);
    float getRainIntensity();
    
private:
    // NEW weather state
    float sunlight_intensity_ = 1.0f;
    float cloud_shadow_strength_ = 0.5f;
    float fog_density_ = 0.0f;
    float rain_intensity_ = 0.0f;
};
```

**Note:** Some methods may require Unreal Engine integration for visual effects

**Effort:** 3 days
- 2 days: Implement weather state management
- 1 day: Add API bindings and testing

---

#### 2.5.3 Spatial Queries (3 methods - MEDIUM VALUE)

**Purpose:** Query world geometry and object positions

**Selected Methods:**
1. `getObjectPose(object_name)` - Get position/orientation of named object
2. `listSceneObjects()` - List all objects in scene
3. `findClosestObjectToPoint(point)` - Find nearest object to 3D point

**Implementation:**
```cpp
// Add to RpcLibServerBase
class RpcLibServerBase {
    // NEW Spatial query methods
    Pose getObjectPose(const std::string& object_name);
    std::vector<std::string> listSceneObjects();
    std::string findClosestObjectToPoint(const Vector3r& point);
};
```

**Effort:** 2 days
- 1 day: Implement scene queries (may use existing Unreal APIs)
- 1 day: Add bindings and testing

---

#### 2.5.4 Out of Scope (NOT Implementing)

**❌ Trajectory Management (68 methods):** Too complex, low ROI
- Waypoint upload/download
- Trajectory following
- Path validation
- Requires extensive state management

**❌ Mission Planning (25+ methods):** Better as separate tool
- Mission upload/download
- Geofence management
- Rally points
- Should be external mission planner

**❌ Path Planning (15+ methods):** Better as separate library
- RRT, RRT*, A* algorithms
- Collision checking
- Optimal path finding
- Complex algorithms, maintenance burden

**❌ Advanced Scene Management:** Unreal-specific, complex
- Dynamic object spawning
- Mesh manipulation
- Material changes

---

**World API Summary:**
- **Clock Control:** 9 methods (4 days) - Essential for testing
- **Weather Control:** 8 methods (3 days) - Useful for perception testing
- **Spatial Queries:** 3 methods (2 days) - Useful utilities
- **Total Selected:** ~20 methods (not 76) - 9 days total effort
- **Skipped:** Trajectory (68), Mission Planning (25+), Path Planning (15+), Advanced Scene (~10)
- **Rationale:** Focus on high-value, widely-used features; skip niche/complex systems

**Weather Effects:**
```python
def enable_weather_visual_effects(self) -> bool:
    """Enable weather rendering (rain, fog, etc.)"""
    
def disable_weather_visual_effects(self) -> bool:
    """Disable weather rendering"""
    
def reset_weather_effects(self) -> bool:
    """Reset to default weather"""
    
def set_weather_visual_effects_param(self, param_id: int, value: float) -> bool:
    """Set weather parameter (rain intensity, fog density, etc.)"""
    
def get_weather_visual_effects_param(self) -> Dict[int, float]:
    """Get all weather parameters"""
```

**Time of Day:**
```python
def set_time_of_day(self, hour: int, minute: int, second: int, is_dst: bool) -> bool:
    """Set time of day (0-23 hours)"""
    
def get_time_of_day(self) -> str:
    """Returns: "HH:MM:SS" format"""
    
def set_sun_position_from_date_time(
    self, 
    year: int, month: int, day: int, 
    hour: int, minute: int, second: int, 
    is_dst: bool,
    latitude: float, longitude: float, 
    timezone_offset: int
) -> bool:
    """Calculate and set sun position from date/time/location"""
```

**Per-Object Lighting:**
```python
def set_light_object_intensity(self, object_name: str, intensity: float) -> bool:
    """Set intensity for specific light object"""
    
def set_light_object_color(self, object_name: str, color_rgb: List[float]) -> bool:
    """Set color for specific light object [R, G, B] 0-1"""
    
def set_light_object_radius(self, object_name: str, radius: float) -> bool:
    """Set attenuation radius for light object"""
```

**Wind:**
```python
def set_wind_velocity(self, v_x: float, v_y: float, v_z: float) -> bool:
    """Set wind velocity in NED frame (m/s)"""
    
def get_wind_velocity(self) -> Tuple[float, float, float]:
    """Get current wind velocity"""
```

#### 2.5.3 Spatial Queries (5 methods)

```python
def get_3d_bounding_box(self, object_name: str) -> List[Vector3r]:
    """Returns 8 corner points of object's bounding box"""
    
def get_surface_elevation_at_point(self, x: float, y: float) -> float:
    """Get ground elevation at (x, y) position"""
    
def get_random_free_position_near_point(
    self, 
    center: Vector3r, 
    min_distance: float, 
    max_distance: float
) -> Vector3r:
    """Find random collision-free position near center"""
    
def get_random_free_position_near_path(
    self, 
    path: List[Vector3r], 
    min_distance: float, 
    max_distance: float
) -> Vector3r:
    """Find random collision-free position near path"""
```

#### 2.5.4 Trajectory Management (5 methods)

```python
def import_ned_trajectory(self, trajectory_name: str, waypoints: List[Dict]) -> bool:
    """Import trajectory from NED waypoints"""
    
def import_ned_trajectory_from_csv(self, trajectory_name: str, csv_file_path: str) -> bool:
    """Import trajectory from CSV file (NED coordinates)"""
    
def import_geo_trajectory(self, trajectory_name: str, waypoints: List[Dict]) -> bool:
    """Import trajectory from geo waypoints (lat/lon/alt)"""
    
def import_trajectory_from_kml(self, trajectory_name: str, kml_file_path: str) -> bool:
    """Import trajectory from KML file"""
    
def generate_intercept_trajectory(
    self, 
    start_pose: Pose, 
    target_name: str, 
    intercept_time: float,
    trajectory_name: str
) -> bool:
    """Generate trajectory to intercept moving target"""
```

---

## Part 3: Implementation Details

---

## Part 3: Implementation Details

### 3.1 File Structure (Dual-Mode Implementation)

**Reality Check:** We're implementing BOTH Simple (enhanced) and Advanced (Robot/Link/Joint) modes, so file structure includes both approaches.

```
Cosys-AirSim/
├── AirLib/
│   ├── include/
│   │   ├── sensors/
│   │   │   ├── radar/                  # NEW (3 files)
│   │   │   │   ├── RadarBase.hpp
│   │   │   │   ├── RadarSimple.hpp
│   │   │   │   └── RadarSimpleParams.hpp
│   │   │   ├── airspeed/               # NEW (3 files)
│   │   │   │   ├── AirspeedBase.hpp
│   │   │   │   ├── AirspeedSimple.hpp
│   │   │   │   └── AirspeedSimpleParams.hpp
│   │   │   ├── battery/                # NEW (3 files)
│   │   │   │   ├── BatteryBase.hpp
│   │   │   │   ├── BatterySimple.hpp
│   │   │   │   └── BatterySimpleParams.hpp
│   │   │   ├── gps/                    # ENHANCED (modify 1 file)
│   │   │   │   └── GpsSimple.hpp       # Add new optional fields
│   │   │   ├── imu/                    # ENHANCED (modify 1 file)
│   │   │   │   └── ImuSimple.hpp       # Add bias methods
│   │   │   ├── magnetometer/           # ENHANCED (modify 1 file)
│   │   │   │   └── MagnetometerSimple.hpp  # Add declination
│   │   │   └── barometer/              # ENHANCED (modify 1 file)
│   │   │       └── BarometerSimple.hpp # Add QNH/QFE
│   │   ├── actuators/                  # NEW (5 files for both modes)
│   │   │   ├── ActuatorBase.hpp        # Base class for mode-aware attachment
│   │   │   ├── RotorActuator.hpp       # Enhanced rotor (both modes)
│   │   │   └── GimbalActuator.hpp      # Camera stabilization (both modes)
│   │   ├── vehicles/
│   │   │   ├── multirotor/             # ENHANCED (modify 1 file)
│   │   │   │   └── MultirotorParams.hpp # Add inertial params, PhysicsMode enum, robot_config
│   │   │   └── robot/                  # NEW - Advanced Mode Architecture (9 files)
│   │   │       ├── Robot.hpp           # Robot container class
│   │   │       ├── Link.hpp            # Physical link component
│   │   │       ├── Joint.hpp           # Joint base class
│   │   │       ├── RevoluteJoint.hpp   # Revolute joint type
│   │   │       ├── PrismaticJoint.hpp  # Prismatic joint type
│   │   │       ├── FixedJoint.hpp      # Fixed joint type
│   │   │       ├── ContinuousJoint.hpp # Continuous rotation joint
│   │   │       ├── CollisionGeometry.hpp # Collision shapes
│   │   │       └── RobotParser.hpp     # Parse Robot JSON config
│   │   ├── physics/
│   │   │   └── PhysicsBody.hpp         # ENHANCED (modify 1 file)
│   │   │                               # Add inertia methods, robot_config pointer, mode-aware update
│   │   ├── common/
│   │   │   ├── ClockControl.hpp        # NEW (1 file)
│   │   │   └── CommonStructs.hpp       # ENHANCED (add sensor structs)
│   │   └── api/
│   │       ├── WorldSimApiBase.hpp     # ENHANCED (add ~25 methods for both modes)
│   │       ├── RpcLibServerBase.hpp    # ENHANCED (add RPC handlers)
│   │       └── VehicleSimApiBase.hpp   # ENHANCED (add sensor APIs)
│   └── src/
│       ├── sensors/                    # NEW implementations (3 sensors)
│       │   ├── radar/                  # RadarSimple.cpp
│       │   ├── airspeed/               # AirspeedSimple.cpp
│       │   └── battery/                # BatterySimple.cpp
│       ├── actuators/                  # NEW (3 files)
│       │   ├── ActuatorBase.cpp        # Base class implementation
│       │   ├── RotorActuator.cpp       # Enhanced rotor
│       │   └── GimbalActuator.cpp      # Gimbal implementation
│       ├── vehicles/
│       │   ├── multirotor/             # ENHANCED
│       │   │   └── MultirotorPhysicsBody.cpp  # Mode-aware physics update
│       │   └── robot/                  # NEW - Advanced Mode Implementation (9 files)
│       │       ├── Robot.cpp
│       │       ├── Link.cpp
│       │       ├── Joint.cpp
│       │       ├── RevoluteJoint.cpp
│       │       ├── PrismaticJoint.cpp
│       │       ├── FixedJoint.cpp
│       │       ├── ContinuousJoint.cpp
│       │       ├── CollisionGeometry.cpp
│       │       └── RobotParser.cpp
│       ├── common/
│       │   └── ClockControl.cpp        # NEW
│       └── api/
│           └── RpcLibServerBase.cpp    # ENHANCED (API implementations for both modes)
└── PythonClient/
    └── cosysairsim/
        ├── client.py                   # ENHANCED (add new APIs for both modes)
        └── types.py                    # ENHANCED (add new data types)
```

**File Count Summary:**

**Simple Mode:**
- **New Files:** ~15 files (3 sensors × 3 files each, 3 actuators, clock control, utilities)
- **Enhanced Files:** ~10 files (existing sensors, MultirotorParams, PhysicsBody, APIs)
- **Simple Mode Total:** ~25 files

**Advanced Mode (Additional):**
- **New Files:** ~18 files (Robot/Link/Joint architecture: 9 headers + 9 implementations)
- **Advanced Mode Total:** ~18 additional files

**Overall Dual-Mode:**
- **Total New Files:** ~33 files (Simple mode + Advanced mode)
- **Total Enhanced Files:** ~12 files
- **Grand Total:** ~45 files changed (still manageable, modular implementation)

**Architecture Separation:**
- Simple mode files: Extend existing structure
- Advanced mode files: Isolated in `vehicles/robot/` directory
- Zero conflicts: Both modes coexist cleanly
- Mode selection: Runtime configuration switch

---

### 3.2 Build System Integration (Dual-Mode Support)

**CMakeLists.txt additions:**
```cmake
# Add new sensor source files (both modes)
set(AIRSIM_SOURCES
    # ... Existing sources unchanged ...
    
    # NEW: Three sensors
    src/sensors/radar/RadarSimple.cpp
    src/sensors/airspeed/AirspeedSimple.cpp
    src/sensors/battery/BatterySimple.cpp
    
    # NEW: Actuators (both modes)
    src/actuators/ActuatorBase.cpp
    src/actuators/RotorActuator.cpp
    src/actuators/GimbalActuator.cpp
    
    # NEW: Advanced Mode - Robot/Link/Joint architecture
    src/vehicles/robot/Robot.cpp
    src/vehicles/robot/Link.cpp
    src/vehicles/robot/Joint.cpp
    src/vehicles/robot/RevoluteJoint.cpp
    src/vehicles/robot/PrismaticJoint.cpp
    src/vehicles/robot/FixedJoint.cpp
    src/vehicles/robot/ContinuousJoint.cpp
    src/vehicles/robot/CollisionGeometry.cpp
    src/vehicles/robot/RobotParser.cpp
    
    # NEW: Clock control
    src/common/ClockControl.cpp
)

# No new dependencies - using existing libraries
# (RapidJSON for JSON, rpclib for RPC, Eigen for math)
```

**Visual Studio Project (.vcxproj):**
- Add new .cpp files to `AirLib.vcxproj` (~33 new source files)
- Add new .hpp files to `AirLib.vcxproj.filters` (~33 new header files)
- No new project configurations needed
- No additional dependencies required

**Build Configuration:**
- Both Simple and Advanced mode code compiled together
- Mode selection is runtime (configuration-based), not compile-time
- No conditional compilation needed

**Effort:** 1 day (file additions for both modes, build verification)

---

### 3.3 Testing Strategy (Dual-Mode Coverage)

**Simple Mode Unit Tests:**
```cpp
// Example: Enhanced inertia tensor
TEST(PhysicsBody, InertiaTensorRotation) {
    PhysicsBody body;
    Matrix3x3r inertia = Matrix3x3r::Identity();
    inertia(0,0) = 0.029; inertia(1,1) = 0.029; inertia(2,2) = 0.055;
    body.setInertiaMatrix(inertia);
    // Verify torque calculations with full tensor
}

// Example: Battery sensor test
TEST(BatterySensor, DischargeRate) {
    BatterySensor battery(/* params */);
    battery.update(1.0f);  // 1 second
    float soc = battery.getBatteryData().state_of_charge;
    EXPECT_LT(soc, 1.0f);  // Should have discharged
}

// Example: Gimbal test
TEST(GimbalActuator, OrientationTracking) {
    GimbalActuator gimbal(/* params */);
    Quaternionr target(/* ... */);
    gimbal.setTargetOrientation(target);
    gimbal.update(0.1f);  // 100ms
    auto current = gimbal.getCurrentOrientation();
    // Verify convergence
}
```

**Advanced Mode Unit Tests:**
```cpp
// Example: Link physics
TEST(Link, InertialPropertiesApplication) {
    Link::InertialProperties props;
    props.mass = 1.0f;
    props.inertia_tensor = Matrix3x3r::Identity();
    props.center_of_mass = Vector3r(0.1, 0, 0);
    
    Link link("test_link", props);
    // Verify force application accounts for COM offset
}

// Example: Revolute joint
TEST(RevoluteJoint, AngleLimits) {
    RevoluteJoint joint("test_joint", Joint::Type::Revolute);
    joint.setLimits(-M_PI_2, M_PI_2);  // -90 to +90 degrees
    joint.setPosition(M_PI);  // Try to exceed limit
    joint.update(0.01f);
    EXPECT_LE(joint.getPosition(), M_PI_2);  // Should be clamped
}

// Example: Robot forward kinematics
TEST(Robot, ForwardKinematicsChain) {
    Robot robot("test_robot");
    Link* base = robot.addLink("base", /* inertial */);
    Link* arm = robot.addLink("arm", /* inertial */);
    Joint* joint = robot.addJoint("joint", Joint::Type::Revolute,
                                   "base", "arm", Vector3r(0,0,1));
    joint->setPosition(M_PI_4);  // 45 degrees
    robot.computeForwardKinematics();
    // Verify arm pose relative to base
}

// Example: Multi-body simulation
TEST(MultiBodyPhysics, ArticulatedDynamics) {
    // Create simple 2-link system
    // Apply forces and verify joint coupling
}
```

**Mode Integration Tests:**
- Load Simple mode settings.json → verify parsing and vehicle creation
- Load Advanced mode settings.json → verify Robot/Link/Joint structure creation
- Switch between modes in same session → verify clean transitions
- Attach sensors to Links in Advanced mode → verify sensor data

**System Tests (Both Modes):**
- **Simple Mode:** Full multirotor simulation with enhanced inertia
- **Advanced Mode:** Articulated drone flight with rotating arms
- **Performance:** Frame rate comparison between Simple and Advanced modes
- **Backward Compatibility:** Run existing scripts with no PhysicsMode → defaults to Simple

**Test Count:**
- Simple Mode: ~50 tests (sensors, actuators, enhanced physics)
- Advanced Mode: ~80 tests (Robot/Link/Joint classes, multi-body physics, FK, collisions)
- Integration: ~20 tests (mode detection, configuration parsing, mode transitions)
- **Total:** ~150 tests (comprehensive coverage for dual-mode system)

**Effort:** 
- Simple Mode: 2 days (testing focused on enhancements)
- Advanced Mode: 5 days (complex multi-body physics testing)
- Integration: 1 day (mode switching, configuration)
- **Total Testing:** 8 days (~1.5 weeks)

---

## Part 4: Migration and Compatibility (Dual-Mode Support)

### 4.1 Backward Compatibility (100% Guaranteed for Both Modes)

**Principle:** All existing Cosys-AirSim code continues to work with ZERO changes

**How We Ensure This:**
1. ✅ Keep ALL existing API methods (never remove)
2. ✅ Add NEW methods alongside old (additive only)
3. ✅ Keep JSON format (no JSONC requirement)
4. ✅ Maintain existing vehicle classes (MultirotorVehicle, CarVehicle, etc.)
5. ✅ Default values for all new parameters (optional enhancements)
6. ✅ Default to Simple mode if PhysicsMode not specified

**Example - Existing Code Works As-Is:**
```python
# This existing code continues to work unchanged
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()
client.moveToPositionAsync(0, 0, -10, 5).join()
# Vehicle uses Simple mode by default with enhanced physics
```

---

### 4.2 Migration Path for Users (Opt-In Enhancements)

**Option 1: No Migration (Keep Everything Same)**
- User changes nothing
- All existing code works
- Defaults to Simple mode
- New features available if wanted

**Option 2: Enhanced Simple Mode (Recommended)**
```python
# Existing code base
client = airsim.MultirotorClient()
client.takeoffAsync().join()

# Optionally adopt new sensors
battery_data = client.getBatteryData("Battery1")  # NEW
print(f"Battery: {battery_data['state_of_charge']*100}%")

# Optionally use Clock control
client.pause()  # NEW
client.continueForSimTime(1000000000)  # 1 second in nanoseconds
client.resume()  # NEW

# Optionally use Weather control
client.setSunlightIntensity(0.5)  # NEW - dim the sun
```

**Option 3: Advanced Mode (Complex Articulated Systems)**
```python
# Use Advanced mode for articulated drone
client = airsim.MultirotorClient()
client.confirmConnection()

# All existing APIs work the same
client.enableApiControl(True)
client.takeoffAsync().join()

# Multi-body physics handled automatically
# User code doesn't need to know about Links/Joints
# Vehicle controller abstracts the complexity

# Optional: Query robot structure (Advanced mode only)
if client.getPhysicsMode("ArticulatedDrone") == "Advanced":
    link_poses = client.getLinkPoses("ArticulatedDrone")  # NEW API
    print(f"Arm 1 pose: {link_poses['arm1']}")
    
    joint_states = client.getJointStates("ArticulatedDrone")  # NEW API
    print(f"Joint angles: {joint_states}")
```

---

### 4.3 Configuration Migration (Backward Compatible Dual-Mode)

**Existing settings.json (UNCHANGED - Still Works):**
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "Sensors": {
        "Gps1": {
          "SensorType": 3,
          "Enabled": true
        }
      }
    }
  }
}
```
**Result:** Works exactly as before, defaults to Simple mode with enhanced physics, no changes needed

**Enhanced Simple Mode settings.json (OPTIONAL):**
```json
{
  "SettingsVersion": 1.3,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "PhysicsMode": "Simple",
      "Inertial": {
        "Mass": 1.5,
        "InertiaTensor": [[0.029, 0, 0], [0, 0.029, 0], [0, 0, 0.055]],
        "CenterOfMass": [0, 0, 0]
      },
      "Sensors": {
        "Gps1": {
          "SensorType": 3,
          "Enabled": true,
          "EPH": 0.5,
          "EPV": 0.8
        },
        "Battery1": {
          "SensorType": 12,
          "Enabled": true,
          "MaxVoltage": 12.6,
          "InitialCharge": 1.0
        }
      }
    }
  }
}
```

**Advanced Mode settings.json (OPT-IN):**
```json
{
  "SettingsVersion": 1.3,
  "SimMode": "Multirotor",
  "Vehicles": {
    "ArticulatedDrone": {
      "VehicleType": "SimpleFlight",
      "PhysicsMode": "Advanced",
      "Robot": {
        "BaseLinkName": "body",
        "Links": [
          {
            "Name": "body",
            "Mass": 1.5,
            "Inertia": [[0.029, 0, 0], [0, 0.029, 0], [0, 0, 0.055]]
          },
          {
            "Name": "arm1",
            "Mass": 0.2,
            "Inertia": [[0.001, 0, 0], [0, 0.002, 0], [0, 0, 0.001]]
          }
        ],
        "Joints": [
          {
            "Name": "arm1_joint",
            "Type": "Revolute",
            "Parent": "body",
            "Child": "arm1",
            "Axis": [0, 0, 1],
            "Limits": [-1.57, 1.57]
          }
        ]
      }
    }
  }
}
```

**Mode Detection:**
- No `PhysicsMode` field → Simple mode (backward compatible)
- `"PhysicsMode": "Simple"` → Enhanced Simple mode
- `"PhysicsMode": "Advanced"` → Robot/Link/Joint multi-body physics

---

## Part 5: Quality Assurance (Pragmatic Standards)

### 5.1 Code Review Checklist

**Code Quality:**
- [ ] Follows existing Cosys-AirSim naming conventions (PascalCase, camelCase)
- [ ] Uses existing Cosys types (Vector3r, Quaternionr, real_T)
- [ ] Matches existing patterns (SensorBase inheritance, RPC method structure)
- [ ] No memory leaks (checked with sanitizers)
- [ ] Thread-safe where needed (sensors, RPC handlers)

**Testing:**
- [ ] Unit tests provided (>70% coverage)
- [ ] Integration tests for new features
- [ ] Backward compatibility verified (run existing examples)

**Documentation:**
- [ ] Header comments (Doxygen style)
- [ ] API documentation updated (add new methods to docs)
- [ ] Example code provided (Python and C++)

**Compatibility:**
- [ ] Windows build succeeds (VS 2019)
- [ ] Linux build succeeds (GCC/Clang)
- [ ] Existing unit tests still pass (100%)

---

### 5.2 Acceptance Criteria

**Per Feature (e.g., Battery Sensor):**
1. ✅ Implementation complete (class, RPC methods, Python bindings)
2. ✅ Unit tests pass (test discharge, API methods)
3. ✅ Integration test passes (run in sim, verify data)
4. ✅ Documentation complete (header comments, example code)
5. ✅ Backward compatible (existing code unaffected)

**Per Tier (e.g., Tier 1 - New Sensors):**
1. ✅ All tier features complete (Battery, Airspeed, Radar)
2. ✅ System test passes (all sensors work together)
3. ✅ Performance acceptable (no significant FPS drop)
4. ✅ User docs updated (README, API reference)

**Final Release:**
1. ✅ All 5 tiers complete
2. ✅ Full regression test suite passes
3. ✅ Performance benchmarks meet targets
4. ✅ Documentation complete
5. ✅ Cross-platform builds succeed
### 5.3 Performance Targets (Realistic)

| Metric | Target | Measurement | Notes |
|--------|--------|-------------|-------|
| Sensor update rate | >100 Hz | Battery, Airspeed, Radar | Match existing sensors |
| World API latency | <5 ms | Clock, weather APIs | RPC overhead included |
| Configuration load | <100 ms | JSON parsing with new sensors | Small increase acceptable |
| Memory overhead | <5% | vs baseline Cosys | Minimal new allocations |
| Frame rate | >30 FPS | Full sim with all features | Unreal rendering bottleneck |

---

## Part 6: Risk Management (Realistic Assessment)

### 6.1 Technical Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Sensor integration complexity | MEDIUM | LOW | Follow existing SensorBase pattern closely |
| Physics enhancement bugs | MEDIUM | MEDIUM | Extensive unit testing, gradual rollout |
| Performance degradation | MEDIUM | LOW | Benchmark after each feature, optimize early |
| API compatibility issues | HIGH | LOW | Strict backward compat testing, no removals |
| Cross-platform build issues | LOW | LOW | Test Windows+Linux continuously |

**Eliminated Risks (from original plan):**
- ❌ Robot architecture complexity - NOT building new framework
- ❌ JSONC parser bugs - NOT implementing JSONC
- ❌ Topic/Service messaging issues - NOT implementing messaging
- ❌ Trajectory management complexity - NOT implementing

### 6.2 Schedule Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Feature creep | MEDIUM | Stick to defined scope (no new features mid-tier) |
| Testing takes longer | LOW | Parallel testing with development |
| Dependency on Unreal changes | LOW | Minimal Unreal integration needed |
| Developer availability | MEDIUM | Clear documentation, modular design |

### 6.3 Mitigation Strategies

**Proactive Measures:**
1. ✅ **Follow Existing Patterns:** Use SensorBase, RpcLibServerBase patterns (reduces bugs)
2. ✅ **Incremental Development:** Implement one feature completely before moving to next
3. ✅ **Continuous Testing:** Run tests after each change, not just at tier end
4. ✅ **Code Reviews:** Review all changes before merge
5. ✅ **Documentation First:** Write API docs before implementation (clarifies design)

**Reactive Measures:**
1. ✅ **Feature Flags:** Can disable problematic features via settings
2. ✅ **Rollback Plan:** Git branches per feature, easy to revert
3. ✅ **Performance Monitoring:** Benchmark suite to catch slowdowns early

---

## Part 7: Success Metrics

### 7.1 Technical Metrics (Measurable Goals)

**Implementation:**
- ✅ All 15 selected features implemented (not 25)
- ✅ 70%+ unit test coverage for new code
- ✅ 100% backward compatibility (existing tests pass)
- ✅ <5% performance overhead vs baseline
- ✅ Cross-platform builds succeed (Windows, Linux)

**Code Quality:**
- ✅ Zero memory leaks (sanitizer clean)
- ✅ Code review approval for all PRs
- ✅ Doxygen comments for all public APIs

---

### 7.2 User Adoption Metrics

**Documentation:**
- ✅ API reference updated with new methods
- ✅ Example code for each new feature (Python + C++)
- ✅ Migration guide showing opt-in approach
- ✅ README updated with feature summary

**Community:**
- ✅ GitHub release notes complete
- ✅ User questions answered (<1 week response time)
- ✅ Bug reports triaged and fixed

---

### 7.3 Success Criteria Summary

**Must Have (Release Blockers):**
1. All tiers complete (both modes)
2. Backward compatibility 100%
3. Zero critical bugs
4. Documentation complete
5. Example code works for both modes

**Nice to Have (Post-Release):**
6. Community tutorials for Advanced mode
7. Advanced articulated robot examples
8. Performance optimizations beyond targets

---

## Part 8: Overall Effort Summary (Dual-Mode Implementation)

### 8.1 Development Effort Breakdown

**Simple Mode Enhancement:**
| Component | Effort | Details |
|-----------|--------|---------|
| Vehicle Physics Enhancement | 2 weeks | Inertia tensor, COM, drag coefficients |
| New Sensors (Radar, Airspeed, Battery) | 2.5 weeks | 3 sensors × ~4 days each |
| Sensor Enhancements (GPS, IMU, Mag, Baro) | 1 week | Add optional fields to existing sensors |
| Actuator Enhancement (RotorActuator base) | 1 week | ActuatorBase + RotorActuator |
| Gimbal Actuator | 3 days | Camera stabilization |
| Configuration Parsing | 3 days | JSON parsing for new fields |
| API Extensions | 1.5 weeks | ~25 new API methods |
| Clock Control | 2 days | Pause, resume, step simulation |
| Weather APIs | 1 week | Sun, weather preset controls |
| **Simple Mode Subtotal** | **~10-11 weeks** | Core enhancements |

**Advanced Mode (Robot/Link/Joint Architecture):**
| Component | Effort | Details |
|-----------|--------|---------|
| Robot/Link/Joint Classes | 4-5 weeks | 9 classes × ~2-3 days each |
| Multi-body Physics Integration | 2 weeks | PhysicsBody mode-aware update |
| Forward Kinematics | 1 week | Joint chain computation |
| Collision Geometry | 1 week | Box, Sphere, Cylinder, Mesh |
| RobotParser (JSON config) | 4 days | Parse Robot/Link/Joint JSON |
| Actuator Link Attachment | 2 days | RotorActuator + Gimbal on Links |
| Advanced Mode APIs | 1 week | getLinkPoses, getJointStates, etc. |
| **Advanced Mode Subtotal** | **~9-11 weeks** | Multi-body system |

**Testing & Integration:**
| Component | Effort | Details |
|-----------|--------|---------|
| Simple Mode Testing | 2 days | 50 unit + integration tests |
| Advanced Mode Testing | 5 days | 80 unit + integration tests |
| Mode Integration Testing | 1 day | Mode switching, validation |
| System Testing | 2 days | Full simulation scenarios |
| Build System Updates | 1 day | CMake, vcxproj updates |
| **Testing Subtotal** | **~2 weeks** | Comprehensive validation |

**Documentation:**
| Component | Effort | Details |
|-----------|--------|---------|
| API Documentation | 3 days | Doxygen comments, API reference |
| User Guide Updates | 2 days | Configuration examples (both modes) |
| Example Code | 3 days | Python + C++ examples for both modes |
| Migration Guide | 1 day | Backward compatibility notes |
| **Documentation Subtotal** | **~2 weeks** | Complete user docs |

### 8.2 Total Effort Estimate

**Dual-Mode Implementation:**
- **Simple Mode:** 10-11 weeks
- **Advanced Mode:** 9-11 weeks
- **Testing:** 2 weeks
- **Documentation:** 2 weeks
- **Buffer (20%):** ~5 weeks

**TOTAL REALISTIC ESTIMATE: ~30-35 weeks (~7-8 months)**

**Breakdown by Phase:**
1. **Phase 1 - Simple Mode Foundation:** 6-7 weeks
   - Vehicle physics enhancements
   - New sensors (Radar, Airspeed, Battery)
   - Sensor enhancements (GPS, IMU, etc.)
   - Basic API extensions

2. **Phase 2 - Simple Mode Completion:** 4-5 weeks
   - Actuators (ActuatorBase, RotorActuator, Gimbal)
   - World APIs (Clock, Weather)
   - Configuration parsing
   - Simple mode testing

3. **Phase 3 - Advanced Mode Architecture:** 5-6 weeks
   - Robot/Link/Joint classes
   - Multi-body physics integration
   - Forward kinematics
   - Collision geometry

4. **Phase 4 - Advanced Mode Completion:** 4-5 weeks
   - RobotParser (JSON config)
   - Link-attached actuators
   - Advanced mode APIs
   - Advanced mode testing

5. **Phase 5 - Integration & Testing:** 2-3 weeks
   - Mode integration testing
   - System testing (both modes)
   - Performance validation
   - Build system finalization

6. **Phase 6 - Documentation & Release:** 2-3 weeks
   - Complete API documentation
   - User guides for both modes
   - Example code
   - Migration guide
   - Release preparation

### 8.3 Comparison with Original Estimates

**Original Spec (Adaptive Only):**
- Estimated: ~20-22 weeks
- Scope: Simple mode enhancements only
- Architecture: Extend existing classes

**Revised Spec (Dual-Mode):**
- Estimated: ~30-35 weeks
- Scope: Simple mode + Advanced mode
- Architecture: Both approaches (simple extension + Robot/Link/Joint)

**Additional Effort for Advanced Mode:**
- +10-13 weeks for full Robot/Link/Joint implementation
- +50% scope increase
- Provides articulated multi-body systems capability

**Trade-off Analysis:**
- ✅ **Benefit:** Full ProjectAirSim Robot/Link/Joint architecture available
- ✅ **Benefit:** Support complex articulated systems (arms, legged robots)
- ✅ **Benefit:** User choice between Simple (fast) and Advanced (flexible)
- ⚠️ **Cost:** Additional 10-13 weeks development time
- ⚠️ **Cost:** More code to maintain (~33 additional files)
- ✅ **Mitigated:** Zero impact on Simple mode users (completely optional)

### 8.4 Risk Mitigation for Extended Timeline

**To manage 30-35 week timeline:**
1. **Parallel Development:** Simple and Advanced modes can be developed partially in parallel
2. **Incremental Delivery:** Release Simple mode first (20-22 weeks), Advanced mode as update
3. **Modular Implementation:** Robot/Link/Joint isolated in `vehicles/robot/` directory
4. **Independent Testing:** Both modes tested separately, minimal integration complexity

**Recommended Approach:**
- Deliver Simple mode enhancement first (~22-24 weeks including testing/docs)
- Release as Cosys-AirSim v2.0
- Follow up with Advanced mode (~10-12 weeks additional)
- Release as Cosys-AirSim v2.1 or v2.5

This staged approach allows users to benefit from Simple mode enhancements sooner while Advanced mode continues development.

---

## Appendices

### Appendix A: Glossary (Cosys-AirSim Context)

**Actuator** - Component that applies forces (motors, servos) - Gimbal is only new one  
**SensorBase** - Base class for all sensors in Cosys-AirSim  
**MultirotorParams** - Configuration struct for multirotor vehicles (being enhanced)  
**PhysicsBody** - Class representing physical object with mass/inertia (being enhanced)  
**RpcLibServerBase** - RPC server handling client API calls (being extended)  
**Vector3r, Quaternionr** - Eigen-based math types used throughout Cosys  
**real_T** - Typedef for floating-point precision (float or double)  
**EPH/EPV** - GPS accuracy metrics (Estimated Position Horizontal/Vertical)  
**QNH/QFE** - Barometric pressure references (sea level / field elevation)

---

### Appendix B: References

**Primary Sources:**
1. **Cosys-AirSim GitHub:** Current repository (this project)
2. **ProjectAirSim GitHub:** Source of concepts to adapt
3. **AirSim Documentation:** https://microsoft.github.io/AirSim/

**Technical References:**
4. **RapidJSON Library:** JSON parsing (already used in Cosys)
5. **rpclib:** RPC library (already used in Cosys)
6. **Eigen:** Linear algebra library (already used in Cosys)
7. **Unreal Engine API:** For Unreal-specific integrations

---

### Appendix C: Change Log

**Version 4.0 - Adaptive Integration Approach (Current)**
- **January 2025:** Complete revision based on user feedback
- **Key Change:** From "copy ProjectAirSim" to "adapt concepts to Cosys patterns"
- **Scope Reduction:** 25 features → 15 features, 33 weeks → 20 weeks
- **Architecture Change:** No new Robot/Link/Joint framework, extend existing classes
- **Format Decision:** Keep JSON (not JSONC), extend parser
- **API Reduction:** ~20 World APIs (not 76), skip Trajectory/Mission/Path Planning
- **Actuator Reduction:** Gimbal only (skip Tilt, Wheel, Control Surface, etc.)
- **Philosophy:** "Extend, Don't Replace" - Cosys patterns first, selective adaptation

**Previous Version 3.0 (Deprecated)**
- Proposed full ProjectAirSim architecture copy-paste
- 7 tiers, 554 tasks, 33 weeks
- New Robot/Link/Joint classes, JSONC config, Topic/Service system
- User feedback: "need to adapt, not copy without thought"

---

## Document End

**Total Estimated Effort:** 20 weeks (100 days, ~500 lines changed per day average)
**Total New Code:** ~5,000 lines (sensors), ~1,500 lines (APIs), ~1,000 lines (actuators) = ~7,500 lines
**Total Enhanced Code:** ~3,000 lines (existing files modified)
**Total Test Code:** ~2,500 lines
**Grand Total Impact:** ~13,000 lines of code (vs original estimate of 30,000+)
- Added complete robot architecture specification
- Added actuator specifications
- Added JSONC configuration system

---

**Document Status:** APPROVED FOR IMPLEMENTATION  
**Next Review:** After Tier 1 Complete (Week 4)  
**Maintained By:** Integration Team  
**Version Control:** Git repository `docs/integration/`
