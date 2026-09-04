# PX4 ROS 2 Autonomy Framework

`px4_airsim_autonomy` is an enterprise-grade autonomous flight framework bridging **AirSim simulation** and **PX4 Autopilot** over ROS 2 Humble using the official **`px4-ros2-interface-lib`** (Auterion / PX4).

The framework implements a deterministic 50 Hz control supervisor coordinating **11 production aerospace C++ modules** spanning flight safety, active collision avoidance, photogrammetry survey planning, and multirotor contingency recovery.

---

## 1. System Architecture & Call Chain

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           AirSim Simulator                              │
│   - Unreal Engine 5 / PhysX / Steppable Clock Lockstep                  │
│   - Cameras: DepthPlanar (Optical Depth) & Scene RGB (TCP RPC / Host)   │
└────────────────────────────────────┬────────────────────────────────────┘
                                     │ sensor_msgs/Image (Depth)
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                     `autonomous_flight_mode_node`                       │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                    Enterprise Flight Supervisor                   │  │
│  │                     (Deterministic 50 Hz Loop)                    │  │
│  ├─────────────────────────────────┬─────────────────────────────────┤  │
│  │        Active Autonomy Mission  │    Safety & Contingency Modules │  │
│  │   - DynamicAvoidanceMission     │   - SmartRthBattery (Wind/Sag)  │  │
│  │   - PhotogrammetrySurveyMission │   - Geofence3D (Winding Number) │  │
│  │   - Standby Hover-Hold ("")     │   - AdsbDeconfliction (DO-365B) │  │
│  │                                 │   - DegradedNavigationFsm       │  │
│  │                                 │   - QuadcopterSpinRecovery      │  │
│  └─────────────────────────────────┴─────────────────────────────────┘  │
│                                    │ Setpoints: Trajectory / Velocity   │
│                                    ▼ (Body FLU -> Local NED)            │
│                       `px4_ros2::ModeBase` Interface                    │
│                        (External Mode Registration)                     │
└────────────────────────────────────┬────────────────────────────────────┘
                                     │ Micro-XRCE-DDS (UDP 8888)
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                             PX4 Autopilot                               │
│   - EKF2 State Estimator (OdometryLocalPosition)                        │
│   - Commander & Failsafe State Machine                                  │
│   - Multicopter Position / Rate Controller                              │
│   - Actuator Control & AirSim Motor Mixing (TCP 4560)                   │
└─────────────────────────────────────────────────────────────────────────┘
```

### Coordinate Frame Conventions
* **AirSim Camera Optical Frame**: $+X$ right, $+Y$ down, $+Z$ forward (depth ray).
* **Body Frame (FLU)**: $+X$ forward, $+Y$ left, $+Z$ up (ROS standard).
* **PX4 World Frame (NED)**: $+X$ north, $+Y$ east, $+Z$ down (Aviation standard).

The node automatically transforms incoming optical depth vectors into Body FLU, projects clearance vectors, and transforms setpoints into PX4 Local NED before streaming to the flight controller.

---

## 2. The 11 Production Aerospace C++ Modules

All production modules reside in namespace `px4_airsim_autonomy::production`:

### 1. `SmartRthBattery`
Calculates dynamic return-to-home energy budgets incorporating vector wind compensation and battery internal resistance heating losses:
* **Aerodynamic Cruise Power**:
  $$P_{cruise} = P_0 + k_i \frac{(m g)^2}{v_{air}} + \frac{1}{2} \rho v_{air}^3 S_{CD}$$
* **Vector Wind Triangle**: Resolves crosswind crab angle $\psi_{crab}$ and groundspeed $v_g$:
  $$v_g = \sqrt{v_{air}^2 - w_\perp^2} + w_\parallel$$
* **Voltage Sag & Total Energy**:
  $$\Delta E_{sag} = I^2 R_{int} t_{return}, \quad E_{req} = E_{climb} + E_{cruise} + E_{descend} + E_{land} + E_{reserve} + \Delta E_{sag}$$
* **Alert Levels**: `Nominal`, `AdvisoryWarning`, `CriticalRthTrigger`, `EmergencyCutoffLand`.

### 2. `Geofence3D`
* **3D Prism Containment**: Uses ray-casting and Jordan curve winding numbers on horizontal polygon boundaries coupled with strict min/max altitude bounds.
* **Dynamic Braking Distance**: Computes velocity-dependent stopping buffer:
  $$d_{stop} = v \cdot t_{reaction} + \frac{v^2}{2 a_{max}}$$
* **Interception Prevention**: Synthesizes perpendicular braking vectors when approaching boundary thresholds.

### 3. `AdsbDeconfliction`
Implements RTCA DO-365B Detect and Avoid (DAA) well-clear specifications:
* **Modified Tau ($\tau_{mod}$)**:
  $$\tau_{mod} = -\frac{r_{horiz}^2 - D_{MOD}^2}{r_{horiz} \cdot \dot{r}_{horiz}}$$
* **5 Alerting Rungs**:
  * Level 0: Clear
  * Level 1: Traffic Advisory (TA)
  * Level 2: Preventive Advisory
  * Level 3: Corrective Resolution Advisory
  * Level 4: Warning Alert (Well Clear Violation — commands emergency vertical dive $\le -4.0\text{ m/s}$ with Controlled Flight Into Terrain [CFIT] protection).

### 4. `PhotogrammetryCalc`
Optical projection calculator for mapping missions:
* **Ground Sampling Distance (GSD)**:
  $$GSD = \frac{H \cdot S_w}{f \cdot I_w}$$
* **Trigger Spacing**: $D_{trigger} = I_h \cdot GSD \cdot (1 - O_f)$
* **Motion Blur Speed Limit**: $v_{max} = \frac{0.5 \cdot GSD}{t_{shutter}}$
* **Smart Oblique Capture (SOC)**: 5-way gimbal automated sequence: Nadir ($0^\circ$), Forward ($+45^\circ$), Right ($+45^\circ$), Aft ($-45^\circ$), Left ($-45^\circ$).

### 5. `BoustrophedonPlanner`
* **Convex Decomposition & Rotating Calipers**: Finds optimal strip orientation angle $\theta^*$ using the Freeman-Shapira theorem to minimize turn counts:
  $$\theta^* = \arg\min_{\theta \in [0^\circ, 180^\circ)} \left\lceil \frac{W_{proj}(\theta)}{S_{strip}} \right\rceil$$
* **Waypoint Generation**: Produces serpentine parallel sweeps with smooth entry/exit turnarounds.

### 6. `MissionContinuity`
* **Atomic Breakpoint Serialization**: Encodes mission progress, current waypoint, strip index, and abort coordinates into verifiable JSON payloads.
* **Along-Track Backtrack Resumption**: Generates backtrack buffer $D_{backtrack} = \max(2 D_{trigger}, \frac{v^2}{2a} + v t_{settle})$ to guarantee zero photographic coverage gaps upon battery swap or obstacle detour recovery.

### 7. `DynamicStoppingBubble`
* **Velocity-Aligned Ellipsoid Quadric**:
  $$\mathcal{E}_{stop} = \{\mathbf{x} \in \mathbb{R}^3 : (\mathbf{x} - \mathbf{p})^T \mathbf{M} (\mathbf{x} - \mathbf{p}) \le 1\}, \quad \mathbf{M} = \mathbf{R} \operatorname{diag}\left(\frac{1}{a^2}, \frac{1}{b^2}, \frac{1}{c^2}\right) \mathbf{R}^T$$
  Where semi-major axis $a = \frac{v^2}{2 a_{max}} + v \cdot t_{latency} + d_{margin}$.
* **Proactive Deceleration**: Commands maximum braking vector $\mathbf{a}_{brake} = -a_{max} \frac{\mathbf{v}}{\|\mathbf{v}\|}$ upon obstacle penetration.

### 8. `SafeFlightCorridor`
* **Convex Polyhedral Slices**: Represents corridor sections as halfspace intersections $A_k \mathbf{x} \le b_k$.
* **Transverse Jerk Bounding**: Validates polynomial trajectory segments against rate-gyro and motor saturation constraints:
  $$\|\mathbf{p}^{(3)}(t)\| \le j_{max} \le 10.0\text{ m/s}^3$$

### 9. `QuadcopterSpinRecovery`
* **Mueller & D'Andrea Single-Motor Loss Recovery**: Detects complete motor failure, intentionally abandons yaw regulation, and allows vehicle to spin at stable limit cycle ($\omega_z \approx 20 - 30\text{ rad/s}$).
* **Cyclic Thrust Modulation**: Inverts control effectiveness matrix $B_{3 \times 3}^{-1}$ synchronized with spin azimuth angle $\theta(t) = \int \omega_z dt$ to achieve controlled vertical descent ($\approx -1.8\text{ m/s}$) and horizontal position hold.

### 10. `MultiCameraDepth`
* **360-Degree Depth Ingest**: Projects planar depth buffers into camera optical 3D point clouds and transforms into Body FLU via extrinsic rotation/translation $\mathbf{R}_c^b, \mathbf{t}_c^b$.
* **Sector Clearances**: Aggregates point obstacles into 3D directional sectors (Front, Left, Right, Up, Down).
* **Thread Safety**: Uses `std::shared_mutex` for zero-copy lock-free reading in high-speed control threads.

### 11. `DegradedNavigationFsm`
* **5-Tier Quality Ladder**:
  * Tier 0: RTK Fixed ($\le 0.05\text{ m}$, max 15 m/s)
  * Tier 1: RTK Float ($\le 0.30\text{ m}$, max 10 m/s)
  * Tier 2: VIO / Odometry ($\le 1.0\text{ m}$, max 5 m/s)
  * Tier 3: Inertial Drag Dead Reckoning (max 1.5 m/s)
  * Tier 4: Emergency Descend & Land (hover & descend)
* **Chi-Square Innovation Gating**: Demotes tier if normalized innovation squared $\chi^2 / \gamma_{gate} > 1.0$; promotes with a 2.0-second recovery hysteresis timer.

---

## 3. Configuration Parameters (`autonomy_params.yaml`)

```yaml
autonomous_flight_mode_node:
  ros__parameters:
    vehicle_name: "drone1"
    camera_name: "cam1"
    algorithm: ""               # Default standby hover-hold under supervisor
    enable_fmu_registration: true
    # depth_topic: "/airsim_node/drone1/cam1_DepthPlanar/image"

    safety:
      enable_adsb: true
      enable_geofence: true
      enable_smart_rth: true

    avoidance:
      cruise_speed: 2.0           # Forward cruising speed (m/s)
      max_accel: 3.0              # Max acceleration/braking (m/s^2)
      safety_margin: 1.5          # Safety bubble standoff margin (m)
      cruise_altitude: 10.0       # Cruise altitude AGL (m)

    survey:
      altitude: 30.0              # Survey altitude AGL (m)
      forward_overlap: 0.75       # Along-track forward overlap (75%)
      side_overlap: 0.65          # Cross-track strip overlap (65%)
      speed: 4.0                  # Survey cruise speed (m/s)
      acceptance_radius: 2.5      # Waypoint acceptance tolerance (m)
```

---

## 4. Launching & Flight Operation

### 1. Build the Package
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select px4_airsim_autonomy
source install/setup.bash
```

### 2. Launch with Autonomy Supervisor
```bash
# Launch with default avoidance mission:
ros2 launch px4_airsim_autonomy autonomy.launch.py

# Launch into standby hover-hold:
ros2 launch px4_airsim_autonomy autonomy.launch.py algorithm:=""

# Launch into photogrammetry survey:
ros2 launch px4_airsim_autonomy autonomy.launch.py algorithm:=photogrammetry_survey
```

### 3. Dynamic Behavior Switching
Switch algorithms dynamically at runtime via ROS 2 parameter callback:
```bash
# Switch to dynamic obstacle avoidance:
ros2 param set /autonomous_flight_mode_node algorithm dynamic_avoidance

# Switch to photogrammetry survey:
ros2 param set /autonomous_flight_mode_node algorithm photogrammetry_survey

# Return to safe standby hover-hold:
ros2 param set /autonomous_flight_mode_node algorithm ""
```

---

## 5. Automated Unit Verification (GoogleTest)

The package contains a comprehensive GoogleTest suite verifying all mathematical models, quadric equations, wind triangles, and state machines:

```bash
cd ~/ros2_ws
colcon test --packages-select px4_airsim_autonomy --event-handlers console_direct+
```

### Test Suite Results (19 Tests Passing):
```
[==========] Running 19 tests from 2 test suites.
[----------] 11 tests from ProductionSystemsTest
[ RUN      ] ProductionSystemsTest.SmartRthBatteryWindTriangleAndEnergy
[       OK ] ProductionSystemsTest.SmartRthBatteryWindTriangleAndEnergy (0 ms)
[ RUN      ] ProductionSystemsTest.Geofence3DPrismContainmentAndBraking
[       OK ] ProductionSystemsTest.Geofence3DPrismContainmentAndBraking (0 ms)
[ RUN      ] ProductionSystemsTest.AdsbDeconflictionTauModAndAlertRungs
[       OK ] ProductionSystemsTest.AdsbDeconflictionTauModAndAlertRungs (0 ms)
[ RUN      ] ProductionSystemsTest.PhotogrammetryCalcGsdAndOverlap
[       OK ] ProductionSystemsTest.PhotogrammetryCalcGsdAndOverlap (0 ms)
[ RUN      ] ProductionSystemsTest.BoustrophedonPlannerRotatingCalipers
[       OK ] ProductionSystemsTest.BoustrophedonPlannerRotatingCalipers (0 ms)
[ RUN      ] ProductionSystemsTest.MissionContinuityAtomicBreakpoint
[       OK ] ProductionSystemsTest.MissionContinuityAtomicBreakpoint (0 ms)
[ RUN      ] ProductionSystemsTest.DynamicStoppingBubbleEllipsoidBreach
[       OK ] ProductionSystemsTest.DynamicStoppingBubbleEllipsoidBreach (0 ms)
[ RUN      ] ProductionSystemsTest.SafeFlightCorridorJerkConstraint
[       OK ] ProductionSystemsTest.SafeFlightCorridorJerkConstraint (0 ms)
[ RUN      ] ProductionSystemsTest.QuadcopterSpinRecoveryMuellerDAndrea
[       OK ] ProductionSystemsTest.QuadcopterSpinRecoveryMuellerDAndrea (0 ms)
[ RUN      ] ProductionSystemsTest.MultiCameraDepthTransformAndSectors
[       OK ] ProductionSystemsTest.MultiCameraDepthTransformAndSectors (0 ms)
[ RUN      ] ProductionSystemsTest.DegradedNavigationFsmStateTransitions
[       OK ] ProductionSystemsTest.DegradedNavigationFsmStateTransitions (0 ms)
[----------] 11 tests from ProductionSystemsTest (0 ms total)

[----------] 8 tests from AutonomyAlgorithmsTest
[ RUN      ] AutonomyAlgorithmsTest.ObstacleAvoidanceSteersAway
[       OK ] AutonomyAlgorithmsTest.ObstacleAvoidanceSteersAway (0 ms)
[ RUN      ] AutonomyAlgorithmsTest.ObstacleAvoidanceEmergencyStopsWhenTooClose
[       OK ] AutonomyAlgorithmsTest.ObstacleAvoidanceEmergencyStopsWhenTooClose (0 ms)
[ RUN      ] AutonomyAlgorithmsTest.ScanningPatrolGeneratesLawnmowerWaypoints
[       OK ] AutonomyAlgorithmsTest.ScanningPatrolGeneratesLawnmowerWaypoints (0 ms)
[ RUN      ] AutonomyAlgorithmsTest.EnterpriseSupervisorAvoidanceOverride
[       OK ] AutonomyAlgorithmsTest.EnterpriseSupervisorAvoidanceOverride (0 ms)
...
[----------] 8 tests from AutonomyAlgorithmsTest (1 ms total)
[==========] 19 tests from 2 test suites ran. (1 ms total)
[  PASSED  ] 19 tests.
```

