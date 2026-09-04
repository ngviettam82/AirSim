# AirSim + PX4 ROS 2 Modular Autonomy Framework

A production-grade, modular autonomy framework built on top of the official **`px4-ros2-interface-lib`** (Auterion / PX4) and **AirSim**.

---

## 1. Architectural Structure

The autonomy architecture is organized into mission-level supervisors (`enterprise/`) and mathematical aerospace contingency modules (`production/`):

```
include/px4_airsim_autonomy/
├── types.hpp                             # SensorSnapshot, AutonomyCommand
├── algorithm_base.hpp                     # Base interface (IAutonomyAlgorithm)
├── algorithm_factory.hpp                  # Dynamic algorithm switcher/factory
├── enterprise/
│   ├── enterprise_flight_supervisor.hpp   # 50 Hz deterministic flight supervisor
│   ├── dynamic_avoidance_mission.hpp      # Reactive collision avoidance mission
│   └── photogrammetry_survey_mission.hpp  # Boustrophedon photogrammetry mission
└── production/
    ├── smart_rth_battery.hpp              # Dynamic return-to-home with wind/sag
    ├── geofence_3d.hpp                    # 3D winding number prism containment
    ├── adsb_deconfliction.hpp             # RTCA DO-365B DAA alerting & dive
    ├── photogrammetry_calc.hpp            # GSD, forward/side overlap & SOC
    ├── boustrophedon_planner.hpp          # Rotating Calipers minimum-strip sweeps
    ├── mission_continuity.hpp             # Atomic JSON breakpoint resumption
    ├── dynamic_stopping_bubble.hpp        # 3D velocity-aligned ellipsoid quadric
    ├── safe_flight_corridor.hpp           # Polyhedral corridor & jerk bounding
    ├── quadcopter_spin_recovery.hpp       # Single-motor failure cyclic thrust
    ├── multi_camera_depth.hpp             # Pinhole depth unprojection to Body FLU
    └── degraded_navigation_fsm.hpp        # 5-tier GPS-denied quality ladder

src/
├── algorithm_factory.cpp
├── autonomous_flight_mode_node.cpp        # PX4 ROS 2 ModeBase node
├── enterprise/
└── production/
```

---

## 2. Implemented Enterprise Missions

| Mission / Algorithm | Key Implementation | Role & Behavior |
|---|---|---|
| **Dynamic Avoidance** | `enterprise/dynamic_avoidance_mission.cpp` | Ingests 3D depth sector clearances; dynamically steers through maximum clearance corridors and triggers quadratic emergency stopping on obstacle breach. |
| **Photogrammetry Survey** | `enterprise/photogrammetry_survey_mission.cpp` | Executes optimal Boustrophedon sweep passes using Rotating Calipers, tracks waypoint progress, and handles along-track backtrack resumption. |
| **Standby Hover-Hold (`""`)** | `src/autonomous_flight_mode_node.cpp` | Default stationary position hold while maintaining full 50 Hz supervisor protection (Smart RTH, Geofence, ADS-B, Stopping Bubble). |

---

## 3. How to Add Your Own New Algorithm in 3 Steps

If you want to add a new function (e.g. `precision_landing` or `tunnel_exploration`):

### Step 1: Create the Header (`include/px4_airsim_autonomy/algorithms/my_algorithm.hpp`)
```cpp
#pragma once
#include "px4_airsim_autonomy/algorithm_base.hpp"

namespace px4_airsim_autonomy {

class MyAlgorithm : public IAutonomyAlgorithm {
public:
    std::string getName() const override { return "my_algorithm"; }
    void init(rclcpp::Node& node) override;
    void onActivate() override;
    void onDeactivate() override;
    AutonomyCommand update(const SensorSnapshot& sensors, float dt) override;
    void reset() override;
};

} // namespace px4_airsim_autonomy
```

### Step 2: Implement the Logic (`src/algorithms/my_algorithm.cpp`)
```cpp
#include "px4_airsim_autonomy/algorithms/my_algorithm.hpp"

namespace px4_airsim_autonomy {

AutonomyCommand MyAlgorithm::update(const SensorSnapshot& sensors, float dt) {
    AutonomyCommand cmd;
    cmd.type = ControlType::Velocity;
    
    // --- YOUR CUSTOM MATH / ML / PLANNING CODE HERE ---
    cmd.vector = Eigen::Vector3f(1.0f, 0.0f, 0.0f); // Move forward at 1 m/s
    cmd.yaw_or_yaw_rate = 0.0f;
    cmd.status_message = "My algorithm is running!";
    return cmd;
}

} // namespace px4_airsim_autonomy
```

### Step 3: Register in `src/algorithm_factory.cpp`
```cpp
if (name == "my_algorithm") {
    return std::make_shared<MyAlgorithm>();
}
```
Rebuild with `colcon build`. You can now run it immediately with `algorithm:=my_algorithm`!

---

## 4. One-Click Setup & Launch Workflow

### 1. Run the Automated Setup Script
In your Ubuntu / WSL2 terminal with ROS 2 Humble installed:
```bash
cd /path/to/AirSim/ros2
chmod +x setup_px4_ros2.sh run_autonomy.sh
./setup_px4_ros2.sh
```
This script automatically:
* Installs required dependencies (`libeigen3-dev`, `MicroXRCEAgent`).
* Clones matching `px4_msgs` and `px4-ros2-interface-lib`.
* Builds the complete workspace with `colcon`.

### 2. Configure AirSim
AirSim automatically uses the repository's root `settings.json` (`~/Documents/AirSim/settings.json`), which is already pre-configured for `PX4Multirotor` (TCP 4560, `SteppableClock`, `LockStep: true`) and camera `cam1` (`ImageType: 0` RGB and `ImageType: 5` DepthPlanar).

### 3. Run the Autonomy Stack
Launch AirSim, PX4 SITL, and MicroXRCEAgent (`MicroXRCEAgent udp4 -p 8888`), then run:
```bash
cd /path/to/AirSim/ros2

# Run dynamic obstacle avoidance (default):
./run_autonomy.sh obstacle_avoidance

# Or run photogrammetry survey:
./run_autonomy.sh photogrammetry_survey

# Or run standby hover-hold under supervisor protection:
./run_autonomy.sh ""
```

