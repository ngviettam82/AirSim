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

### Scope

Integration of **25 critical features** from ProjectAirSim into Cosys-AirSim, representing approximately **33 weeks** of development effort for complete feature parity.

### Key Deliverables

1. **Robot Architecture Framework** (Generic robot system with links/joints)
2. **3 New Sensors** (Radar, Airspeed, Battery)
3. **Enhanced Sensors** (GPS, IMU, Magnetometer, Barometer with ProjectAirSim capabilities)
4. **76 World API Methods** (Clock, weather, spatial queries, trajectories)
5. **JSONC Configuration System** (JSON with Comments support)
6. **7 Actuator Types** (Gimbal, rotor, tilt, wheel, control surfaces)

### Implementation Strategy

**Phased Approach:**
- **Tier 1 (Weeks 1-4):** Critical sensors (Battery, Airspeed, Radar, Clock)
- **Tier 2 (Weeks 5-7):** Robot architecture framework (FOUNDATION)
- **Tier 3 (Weeks 8-11):** Essential sensors (Barometer, GPS, IMU, Magnetometer)
- **Tier 4 (Weeks 12-14):** Utility sensors (Distance, Lidar, Camera enhancements)
- **Tier 5 (Weeks 15-18):** World APIs (Weather, time, objects, scenes)
- **Tier 6 (Weeks 19-23):** Advanced features (Debug, segmentation, trajectories)
- **Tier 7 (Weeks 24+):** Extended capabilities (Mission/path planning, utilities)

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

**Approach:** **Hybrid Architecture**

Maintain Cosys-AirSim's existing vehicle implementations while adding:
1. Generic Robot base class (optional alternative)
2. Link/Joint system for advanced robots
3. Actuator framework
4. New sensors
5. Extended World APIs
6. JSONC configuration support (alongside JSON)

**Benefits:**
- ✅ Backward compatible with existing Cosys-AirSim code
- ✅ Gradual migration path
- ✅ Leverage ProjectAirSim's advanced features
- ✅ Support both simple and complex robot configurations

---

## Part 2: Critical Feature Specifications

### 2.1 Generic Robot Architecture

#### 2.1.1 Overview

**Purpose:** Provide a universal framework for defining robots as tree structures of links, joints, actuators, and sensors.

**Key Components:**
1. **Robot Class** - Base container for all robot components
2. **Link Class** - Physical components with mass, inertia, collision, visual mesh
3. **Joint Class** - Connections between links with constraints
4. **Actuator Classes** - Force/torque application mechanisms
5. **Transform Tree** - Hierarchical coordinate frame management

#### 2.1.2 Class Hierarchy

```cpp
Actor (base)
  └─ Robot
       ├─ std::vector<Link>
       ├─ std::vector<Joint>
       ├─ std::vector<Sensor>
       ├─ std::vector<Actuator>
       ├─ IController (Simple Flight, PX4, Manual)
       └─ TransformTree
```

#### 2.1.3 Link Specification

**C++ Interface:**
```cpp
class Link {
public:
    struct Inertial {
        float mass;                    // kg
        Matrix3x3 inertia_tensor;     // kg*m^2
        Pose center_of_mass;           // relative to link frame
        struct Aerodynamics {
            float drag_coefficient;
            Vector3r cross_section_areas;  // m^2 (X, Y, Z)
        } aerodynamics;
    };
    
    struct Collision {
        bool enabled;
        float restitution;  // 0-1 (bounce)
        float friction;     // 0-1 (surface friction)
    };
    
    struct Visual {
        std::string mesh_path;  // Unreal asset path
        Vector3r scale;
    };
    
    std::string getName() const;
    const Inertial& getInertial() const;
    const Collision& getCollision() const;
    const Visual& getVisual() const;
};
```

**Configuration (JSONC):**
```jsonc
{
  "name": "Frame",
  "inertial": {
    "mass": 1.0,
    "origin": {"xyz": "0 0 0", "rpy-deg": "0 0 0"},
    "inertia": {
      "type": "geometry",  // or "matrix" or "point-mass"
      "geometry": {
        "box": {"size": "0.18 0.11 0.04"}
      }
    },
    "aerodynamics": {
      "drag-coefficient": 0.325,
      "type": "geometry",
      "geometry": {
        "box": {"size": "0.18 0.11 0.04"}
      }
    }
  },
  "collision": {
    "enabled": true,
    "restitution": 0.1,
    "friction": 0.5
  },
  "visual": {
    "geometry": {
      "type": "unreal_mesh",
      "name": "/Drone/Quadrotor1",
      "scale": "1.0 1.0 1.0"
    }
  }
}
```

#### 2.1.4 Joint Specification

**Types:**
1. **Fixed** - No relative motion
2. **Revolute** - Rotation around axis with limits
3. **Continuous** - Unlimited rotation around axis

**C++ Interface:**
```cpp
class Joint {
public:
    enum class Type {
        Fixed,
        Revolute,
        Continuous
    };
    
    std::string getId() const;
    Type getType() const;
    const Link* getParentLink() const;
    const Link* getChildLink() const;
    Vector3r getAxis() const;          // Rotation axis
    float getLimit() const;            // Angle limit (revolute)
    float getDampingConstant() const;  // Damping coefficient
};
```

**Configuration (JSONC):**
```jsonc
{
  "id": "Frame_Prop_FL",
  "type": "fixed",  // or "revolute" or "continuous"
  "parent-link": "Frame",
  "child-link": "Prop_FL",
  "origin": {
    "xyz": "0.253 -0.253 -0.01",
    "rpy-deg": "0 0 0"
  },
  "axis": "0 0 1",         // Z-axis rotation
  "limit": 1.57,           // radians (for revolute)
  "damping-constant": 1.0
}
```

#### 2.1.5 Robot Class Interface

```cpp
class Robot : public Actor {
public:
    // Configuration
    Link* GetLink(const std::string& id);
    const std::vector<Link>& GetLinks() const;
    const std::vector<Joint>& GetJoints() const;
    const std::vector<Sensor>& GetSensors() const;
    const std::vector<Actuator>& GetActuators() const;
    
    // Physics
    const PhysicsType& GetPhysicsType() const;
    void SetPhysicsType(const PhysicsType& phys_type);
    
    // Controller
    void SetController(std::unique_ptr<IController> controller);
    
    // Communication
    Topic CreateTopic(const std::string& name, TopicType type, 
                     int frequency, MessageType message_type,
                     std::function<void(const Topic&, const Message&)> callback);
    void RegisterServiceMethod(const ServiceMethod& method,
                               MethodHandler method_handler);
    
    // Callbacks
    void SetCallbackKinematicsUpdated(const KinematicsCallback& callback);
    void SetCallbackActuatorOutputUpdated(const ActuatedTransformsCallback& callback);
    
    // Runtime
    void BeginUpdate();
    void EndUpdate();
    void PublishRobotPose(const PoseStampedMessage& pose);
};
```

#### 2.1.6 Integration with Existing Cosys-AirSim

**Strategy:** Add Robot as **alternative base class** alongside existing vehicle classes.

```cpp
// New hierarchy
class VehicleBase {  // Common interface
    virtual void update() = 0;
    virtual void reset() = 0;
    // ...
};

// Option 1: Existing (maintain compatibility)
class MultirotorPawnSimApi : public VehicleBase {
    // Existing implementation
};

// Option 2: New Robot-based (for advanced features)
class RobotPawnSimApi : public VehicleBase {
    Robot robot_;  // Uses generic robot framework
    // ...
};
```

**Migration Path:**
1. Phase 1: Add Robot classes alongside existing
2. Phase 2: Create adapters for existing vehicles
3. Phase 3: Optional migration of vehicles to Robot-based

---

### 2.2 JSONC Configuration System

#### 2.2.1 Requirements

**Support both:**
1. **JSON** (existing, for backward compatibility)
2. **JSONC** (new, JSON with Comments)

#### 2.2.2 Implementation

**Parser:**
```cpp
class ConfigJson {
public:
    // Parse JSONC (strips comments, then parses JSON)
    static nlohmann::json parse(const std::string& jsonc_content);
    static nlohmann::json parseFile(const std::string& file_path);
    
    // Helper methods
    static bool hasComments(const std::string& content);
    static std::string stripComments(const std::string& jsonc);
};
```

**Comment Stripping Logic:**
```cpp
// Remove single-line comments: // ...
// Remove multi-line comments: /* ... */
// Preserve strings (don't strip comments in strings)
```

#### 2.2.3 Configuration Files

**Robot Configuration (`robot_quadrotor.jsonc`):**
```jsonc
{
  // Physics engine selection
  "physics-type": "fast-physics",  // or "unreal-physics" or "non-physics"
  
  // Physical components
  "links": [
    {
      "name": "Frame",
      "inertial": {...},  // Mass, inertia, drag
      "collision": {...}, // Collision properties
      "visual": {...}     // Visual mesh
    },
    {
      "name": "Prop_FL",
      "inertial": {...},
      "visual": {...}
    }
    // ... more links
  ],
  
  // Connections between links
  "joints": [
    {
      "id": "Frame_Prop_FL",
      "type": "fixed",
      "parent-link": "Frame",
      "child-link": "Prop_FL"
    }
    // ... more joints
  ],
  
  // Flight controller
  "controller": {
    "id": "Simple_Flight_Controller",
    "type": "simple-flight-api",  // or "px4-api" or "manual-controller-api"
    "airframe-setup": "quadrotor-x",
    "simple-flight-api-settings": {
      "actuator-order": ["Prop_FL", "Prop_FR", "Prop_RL", "Prop_RR"]
    }
  },
  
  // Force/torque generators
  "actuators": [
    {
      "id": "Prop_FL",
      "type": "rotor",
      "link": "Prop_FL",
      "rotor-settings": {
        "thrust-coefficient": 1.0,
        "torque-coefficient": 0.01,
        "max-rpm": 6000
      }
    }
    // ... more actuators
  ],
  
  // Sensor suite
  "sensors": [
    {
      "id": "imu",
      "type": "imu",
      "link": "Frame",
      "pose": {"xyz": "0 0 0", "rpy-deg": "0 0 0"},
      "imu-settings": {
        "gyro-noise-sigma": 0.01,
        "accel-noise-sigma": 0.05
      }
    },
    {
      "id": "gps",
      "type": "gps",
      "link": "Frame",
      "gps-settings": {
        "eph": 0.5,
        "epv": 0.8
      }
    }
    // ... more sensors
  ]
}
```

**Scene Configuration (`scene_config.jsonc`):**
```jsonc
{
  // Simulation settings
  "sim-clock": {
    "type": "steppable",  // or "scalable" or "stepped-realtime"
    "step-ns": 10000000   // 10ms per step
  },
  
  // Environment
  "origin": {
    "latitude": 47.641468,
    "longitude": -122.140165,
    "altitude": 122.0
  },
  
  // Actors in scene
  "actors": [
    {
      "type": "robot",
      "name": "Drone1",
      "origin": {"xyz": "0.0 0.0 -10.0", "rpy-deg": "0 0 0"},
      "ref": "robot_quadrotor_fastphysics.jsonc"  // External robot config
    },
    {
      "type": "robot",
      "name": "Drone2",
      "origin": {"xyz": "0.0 2.0 -10.0", "rpy-deg": "0 0 0"},
      "ref": "robot_quadrotor_fastphysics.jsonc"
    }
  ]
}
```

---

### 2.3 Sensor Specifications

#### 2.3.1 Radar Sensor (NEW)

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

#### 2.3.2 Airspeed Sensor (NEW)

**Purpose:** Measure air velocity relative to vehicle

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

#### 2.3.3 Battery Sensor (NEW)

**Purpose:** Simulate battery discharge and state of charge

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

#### 2.3.4 Enhanced GPS

**ProjectAirSim Additions:**
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

**Configuration:**
```cpp
struct GpsParams {
    // Existing
    float update_frequency;
    
    // NEW
    float eph;                    // Simulated horizontal accuracy
    float epv;                    // Simulated vertical accuracy
    int min_satellites;           // Minimum for 3D fix
    int max_satellites;           // Maximum visible
    bool simulate_fix_loss;       // Randomly lose fix
    float fix_loss_probability;   // Probability per second
};
```

#### 2.3.5 Enhanced IMU

**ProjectAirSim Additions:**
```cpp
struct ImuData {
    // Existing
    Vector3r linear_acceleration;
    Vector3r angular_velocity;
    Quaternionr orientation;
    
    // NEW
    Vector3r gyro_bias;           // Gyroscope bias
    Vector3r accel_bias;          // Accelerometer bias
    float temperature;            // Temperature (affects bias)
    
    int64_t time_stamp;
};
```

**Configuration:**
```cpp
struct ImuParams {
    // Existing
    float gyro_noise_sigma;
    float accel_noise_sigma;
    
    // NEW
    float gyro_bias_sigma;        // Gyro bias standard deviation
    float accel_bias_sigma;       // Accel bias standard deviation
    float gyro_bias_tau;          // Bias random walk time constant
    float accel_bias_tau;
    float temp_coefficient;       // Temperature effect on bias
};
```

#### 2.3.6 Enhanced Magnetometer

**ProjectAirSim Additions:**
```cpp
struct MagnetometerData {
    Vector3r magnetic_field;      // Gauss (body frame)
    
    // NEW
    Vector3r bias;                // Sensor bias
    float declination;            // Magnetic declination (radians)
    float inclination;            // Magnetic inclination (radians)
    
    int64_t time_stamp;
};
```

---

### 2.4 Actuator Specifications

#### 2.4.1 Base Actuator Class

```cpp
class Actuator {
public:
    virtual ~Actuator() = default;
    
    virtual void UpdateActuatorOutput(float output) = 0;
    virtual ActuatedTransforms GetActuatedTransforms() const = 0;
    virtual void ApplyForcesAndTorques(Link* link) = 0;
    
    const std::string& GetId() const;
    const Link* GetLink() const;
};
```

#### 2.4.2 Rotor Actuator

**Purpose:** Generate thrust and torque for propellers

```cpp
class Rotor : public Actuator {
public:
    struct Params {
        float thrust_coefficient;   // N/(rad/s)^2
        float torque_coefficient;   // N*m/(rad/s)^2
        float max_rpm;
        float rotor_diameter;       // meters
        float motor_time_constant;  // seconds
    };
    
    void UpdateActuatorOutput(float output) override;  // 0-1 throttle
    void ApplyForcesAndTorques(Link* link) override;
};
```

#### 2.4.3 Gimbal Actuator

**Purpose:** Camera stabilization

```cpp
class Gimbal : public Actuator {
public:
    struct Params {
        float pitch_min;            // radians
        float pitch_max;
        float roll_min;
        float roll_max;
        float yaw_min;
        float yaw_max;
        float angular_velocity_max; // rad/s
    };
    
    void SetTargetOrientation(const Quaternionr& target);
    Quaternionr GetCurrentOrientation() const;
};
```

#### 2.4.4 Tilt Actuator

**Purpose:** VTOL transition (tilt rotor/wing)

```cpp
class Tilt : public Actuator {
public:
    struct Params {
        float min_angle;            // radians
        float max_angle;            // radians
        float tilt_rate;            // rad/s
        Vector3r tilt_axis;
    };
    
    void SetTiltAngle(float angle);
    float GetCurrentTiltAngle() const;
};
```

#### 2.4.5 Wheel Actuator

**Purpose:** Ground vehicle propulsion

```cpp
class Wheel : public Actuator {
public:
    struct Params {
        float radius;               // meters
        float max_angular_velocity; // rad/s
        float friction_coefficient;
        bool is_steered;            // Can this wheel steer?
        float max_steer_angle;      // radians (if steered)
    };
    
    void SetAngularVelocity(float velocity);  // rad/s
    void SetSteerAngle(float angle);          // radians
};
```

#### 2.4.6 Control Surface Actuator

**Purpose:** Aerodynamic control (elevons, rudder, etc.)

```cpp
class LiftDragControlSurface : public Actuator {
public:
    struct Params {
        float area;                 // m^2
        float min_deflection;       // radians
        float max_deflection;       // radians
        float deflection_rate;      // rad/s
        float lift_coefficient;
        float drag_coefficient;
    };
    
    void SetDeflection(float angle);  // radians
    void ApplyForcesAndTorques(Link* link) override;
};
```

#### 2.4.7 Actuator Configuration

```jsonc
{
  "actuators": [
    {
      "id": "Prop_FL",
      "type": "rotor",
      "link": "Prop_FL",
      "rotor-settings": {
        "thrust-coefficient": 1.0,
        "torque-coefficient": 0.01,
        "max-rpm": 6000,
        "rotor-diameter": 0.25,
        "motor-time-constant": 0.05
      }
    },
    {
      "id": "Camera_Gimbal",
      "type": "gimbal",
      "link": "Camera_Mount",
      "gimbal-settings": {
        "pitch-min-deg": -90,
        "pitch-max-deg": 30,
        "roll-min-deg": -30,
        "roll-max-deg": 30,
        "yaw-min-deg": -180,
        "yaw-max-deg": 180,
        "angular-velocity-max": 1.57  // 90 deg/s
      }
    }
  ]
}
```

---

### 2.5 World API Specifications

#### 2.5.1 Clock Control (9 methods)

**Purpose:** Precise simulation time control

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

**C++ Implementation:**
```cpp
class ClockControl {
public:
    enum class ClockType {
        Steppable,       // Manual step control
        Scalable,        // Time scale (faster/slower than real-time)
        SteppedRealtime  // Fixed timestep, real-time synchronized
    };
    
    ClockType GetClockType() const;
    int64_t GetSimTime() const;  // nanoseconds
    void Pause();
    void Resume();
    bool IsPaused() const;
    void ContinueForSimTime(int64_t duration_ns);
    void ContinueUntilSimTime(int64_t target_time_ns);
    void ContinueForNSteps(int n_steps);
    int64_t ContinueForSingleStep();
};
```

#### 2.5.2 Weather & Lighting (16 methods)

**Sunlight:**
```python
def set_sunlight_intensity(self, intensity: float) -> bool:
    """Set sun intensity (0.0-1.0)"""
    
def get_sunlight_intensity(self) -> float:
    """Get current sun intensity"""
```

**Clouds:**
```python
def set_cloud_shadow_strength(self, strength: float) -> bool:
    """Set cloud shadow strength (0.0-1.0)"""
    
def get_cloud_shadow_strength(self) -> float:
    """Get cloud shadow strength"""
```

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

### 3.1 File Structure

```
Cosys-AirSim/
├── AirLib/
│   ├── include/
│   │   ├── robot/                      # NEW
│   │   │   ├── Robot.hpp
│   │   │   ├── Link.hpp
│   │   │   ├── Joint.hpp
│   │   │   └── TransformTree.hpp
│   │   ├── actuators/                  # NEW
│   │   │   ├── Actuator.hpp
│   │   │   ├── Rotor.hpp
│   │   │   ├── Gimbal.hpp
│   │   │   ├── Tilt.hpp
│   │   │   ├── Wheel.hpp
│   │   │   └── LiftDragControlSurface.hpp
│   │   ├── sensors/
│   │   │   ├── radar/                  # NEW
│   │   │   │   ├── RadarBase.hpp
│   │   │   │   └── RadarSimple.hpp
│   │   │   ├── airspeed/               # NEW
│   │   │   │   ├── AirspeedBase.hpp
│   │   │   │   └── AirspeedSimple.hpp
│   │   │   ├── battery/                # NEW
│   │   │   │   ├── BatteryBase.hpp
│   │   │   │   └── BatterySimple.hpp
│   │   │   ├── gps/                    # ENHANCED
│   │   │   │   └── GpsBase.hpp         # Add new fields
│   │   │   ├── imu/                    # ENHANCED
│   │   │   │   └── ImuBase.hpp         # Add bias simulation
│   │   │   └── magnetometer/           # ENHANCED
│   │   │       └── MagnetometerBase.hpp
│   │   ├── common/
│   │   │   ├── ConfigJson.hpp          # NEW (JSONC parser)
│   │   │   ├── ClockControl.hpp        # NEW
│   │   │   └── CommonStructs.hpp       # Add new sensor data structures
│   │   └── api/
│   │       ├── WorldSimApiBase.hpp     # Add new methods
│   │       └── VehicleSimApiBase.hpp   # Add battery/airspeed APIs
│   └── src/
│       ├── robot/                      # NEW
│       │   ├── Robot.cpp
│       │   ├── Link.cpp
│       │   └── Joint.cpp
│       ├── actuators/                  # NEW
│       │   ├── Rotor.cpp
│       │   ├── Gimbal.cpp
│       │   └── ...
│       └── sensors/
│           ├── radar/                  # NEW
│           ├── airspeed/               # NEW
│           └── battery/                # NEW
└── PythonClient/
    └── cosysairsim/
        ├── client.py                   # Add new APIs
        └── types.py                    # Add new data types
```

### 3.2 Build System Integration

**CMakeLists.txt additions:**
```cmake
# New source files
set(AIRSIM_SOURCES
    # Existing sources...
    
    # Robot framework
    src/robot/Robot.cpp
    src/robot/Link.cpp
    src/robot/Joint.cpp
    
    # Actuators
    src/actuators/Rotor.cpp
    src/actuators/Gimbal.cpp
    src/actuators/Tilt.cpp
    src/actuators/Wheel.cpp
    src/actuators/LiftDragControlSurface.cpp
    
    # New sensors
    src/sensors/radar/RadarSimple.cpp
    src/sensors/airspeed/AirspeedSimple.cpp
    src/sensors/battery/BatterySimple.cpp
    
    # Utilities
    src/common/ConfigJson.cpp
    src/common/ClockControl.cpp
)

# JSONC support (use nlohmann/json with custom parser)
find_package(nlohmann_json REQUIRED)
target_link_libraries(AirLib nlohmann_json::nlohmann_json)
```

### 3.3 Testing Strategy

**Unit Tests:**
- Test each actuator type independently
- Test link/joint calculations
- Test JSONC parser with comments
- Test sensor data generation

**Integration Tests:**
- Test complete robot configuration loading
- Test multi-robot scenarios
- Test clock control precision
- Test trajectory import/export

**System Tests:**
- End-to-end robot simulation
- Performance benchmarks
- Backward compatibility with existing Cosys configs

---

## Part 4: Migration and Compatibility

### 4.1 Backward Compatibility

**Guarantee:** All existing Cosys-AirSim code continues to work

**Strategy:**
1. Keep existing API methods (don't remove anything)
2. Add new methods alongside old
3. Support both JSON and JSONC configurations
4. Maintain existing vehicle classes

### 4.2 Migration Path for Users

**Option 1: Gradual (Recommended)**
```python
# Keep using existing Cosys-AirSim API
client = airsim.MultirotorClient()
client.takeoffAsync().join()

# Optionally adopt new features
world = client.get_world()  # NEW
world.set_sunlight_intensity(0.5)  # NEW
```

**Option 2: Full Migration**
```python
# Use new Robot-based vehicles
from cosysairsim import ProjectAirSimClient, World

client = ProjectAirSimClient()
world = World(client)
drone = world.get_drone("Drone1")

# All ProjectAirSim features available
drone.arm()
drone.takeoff_async()
```

### 4.3 Configuration Migration

**Existing settings.json (Cosys):**
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "Sensors": {...}
    }
  }
}
```

**New robot config (ProjectAirSim style):**
```jsonc
// robot_drone1.jsonc
{
  "physics-type": "fast-physics",
  "links": [...],
  "joints": [...],
  "controller": {...},
  "actuators": [...],
  "sensors": [...]
}
```

**Scene config references robot:**
```jsonc
{
  "actors": [
    {
      "type": "robot",
      "name": "Drone1",
      "ref": "robot_drone1.jsonc"  // External robot definition
    }
  ]
}
```

**Both formats supported!**

---

## Part 5: Quality Assurance

### 5.1 Code Review Checklist

- [ ] Follows Cosys-AirSim coding standards
- [ ] Includes unit tests (>80% coverage)
- [ ] Includes integration tests
- [ ] Documentation complete (Doxygen comments)
- [ ] Backward compatibility verified
- [ ] Performance benchmarks acceptable
- [ ] Memory leaks checked (Valgrind)
- [ ] Thread safety verified

### 5.2 Acceptance Criteria

**Per Feature:**
1. Unit tests pass (100%)
2. Integration tests pass (100%)
3. Performance within 10% of baseline
4. Documentation complete
5. Example code provided
6. Cross-platform verified (Windows, Linux)

**Per Tier:**
1. All features in tier complete
2. System tests pass
3. User documentation updated
4. Release notes prepared

### 5.3 Performance Targets

| Metric | Target | Measurement |
|--------|--------|-------------|
| Sensor update rate | >100 Hz | All sensors |
| World API latency | <1 ms | Clock, weather APIs |
| Robot creation time | <100 ms | Complex robots (20+ links) |
| Configuration load | <50 ms | JSONC parsing |
| Memory overhead | <10% | vs baseline Cosys |

---

## Part 6: Risk Management

### 6.1 Technical Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Robot architecture complexity | HIGH | MEDIUM | Phased implementation, extensive testing |
| JSONC parser bugs | MEDIUM | LOW | Use proven library (nlohmann/json) |
| Performance degradation | HIGH | MEDIUM | Continuous benchmarking, optimization |
| Breaking changes | HIGH | LOW | Strict backward compatibility testing |
| Unreal integration issues | MEDIUM | MEDIUM | Early Unreal plugin testing |

### 6.2 Schedule Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Underestimated complexity | MEDIUM | 20% buffer in schedule |
| Dependency delays | LOW | Minimize external dependencies |
| Testing bottlenecks | MEDIUM | Parallel testing infrastructure |

### 6.3 Mitigation Strategies

1. **Continuous Integration:** Automated testing on every commit
2. **Feature Flags:** Enable/disable new features at runtime
3. **Gradual Rollout:** Release tiers incrementally
4. **Community Feedback:** Early alpha/beta testing
5. **Documentation First:** Write docs before implementation

---

## Part 7: Success Metrics

### 7.1 Technical Metrics

- [ ] 100% unit test coverage for new code
- [ ] Zero regression in existing Cosys-AirSim tests
- [ ] <10% performance overhead vs baseline
- [ ] 25/25 critical features implemented
- [ ] All documentation complete

### 7.2 User Adoption Metrics

- [ ] Example projects created (5+)
- [ ] User guide complete
- [ ] Migration guide complete
- [ ] Community tutorials (3+)
- [ ] GitHub issues resolved (<10 open)

### 7.3 Quality Metrics

- [ ] Code review approval rate >95%
- [ ] Bug rate <1 per 1000 LOC
- [ ] Documentation clarity rating >4/5
- [ ] User satisfaction >80%

---

## Appendices

### Appendix A: Glossary

**Actor** - Base class for entities in simulation (robots, cameras, etc.)  
**Actuator** - Component that applies forces/torques (motors, servos, etc.)  
**Link** - Physical component of robot with mass and shape  
**Joint** - Connection between two links with constraints  
**JSONC** - JSON with Comments (relaxed JSON syntax)  
**NED** - North-East-Down coordinate frame  
**RCS** - Radar Cross Section (measure of detectability)  
**EPH/EPV** - Estimated Position Horizontal/Vertical (GPS accuracy)  
**DOF** - Depth of Field (camera focus effect)

### Appendix B: References

1. ProjectAirSim Documentation: https://github.com/projectairsim/
2. Cosys-AirSim Documentation: https://cosys-lab.github.io/
3. Unreal Engine 5 Documentation: https://docs.unrealengine.com/
4. nlohmann/json Library: https://github.com/nlohmann/json
5. ROS 2 Integration: https://docs.ros.org/

### Appendix C: Change Log

**Version 3.0 (January 2025)**
- Complete rewrite based on automated codebase analysis
- Added 25 critical features
- Expanded from 4 to 7 implementation tiers
- Added complete robot architecture specification
- Added actuator specifications
- Added JSONC configuration system

---

**Document Status:** APPROVED FOR IMPLEMENTATION  
**Next Review:** After Tier 1 Complete (Week 4)  
**Maintained By:** Integration Team  
**Version Control:** Git repository `docs/integration/`
