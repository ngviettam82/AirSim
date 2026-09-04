# AirSim + PX4 ROS 2 Modular Autonomy Framework

A production-grade, modular autonomy framework built on top of the official **`px4-ros2-interface-lib`** (Auterion / PX4) and **AirSim**.

---

## 1. Key Architectural Concept: Dedicated File Per Algorithm

Every flight behavior/algorithm is isolated in its own dedicated C++ file implementing the `IAutonomyAlgorithm` interface. This allows you to develop, test, and maintain each function independently without touching the core flight controller or ROS 2 node wiring:

```
include/px4_airsim_autonomy/
├── types.hpp                             # SensorSnapshot, AutonomyCommand
├── algorithm_base.hpp                     # Base interface (IAutonomyAlgorithm)
├── algorithm_factory.hpp                  # Dynamic algorithm switcher/factory
└── algorithms/
    ├── obstacle_avoidance.hpp             # Dynamic 3D depth-based collision avoidance
    ├── scanning_patrol.hpp                # Area survey / lawnmower scan pattern
    ├── target_guiding.hpp                 # Standoff target tracking & following
    └── area_search.hpp                    # Expanding spiral search pattern

src/algorithms/
├── obstacle_avoidance.cpp
├── scanning_patrol.cpp
├── target_guiding.cpp
└── area_search.cpp
```

---

## 2. Implemented Algorithms

| Algorithm | File | Role & Behavior |
|---|---|---|
| **Obstacle Avoidance** | `algorithms/obstacle_avoidance.cpp` | Segments AirSim depth camera into Left/Center/Right sectors; steers toward maximum clearance and applies emergency stop/reverse on close proximity. |
| **Scanning Patrol** | `algorithms/scanning_patrol.cpp` | Generates parallel sweep lanes (lawnmower pattern) across an area at fixed altitude for surveillance and mapping. |
| **Target Guiding** | `algorithms/target_guiding.cpp` | Maintains a specified standoff distance and altitude above a target (human, vehicle, or marker) while yawing to face it. |
| **Area Search** | `algorithms/area_search.cpp` | Executes an expanding Archimedean spiral search pattern ($r = b \cdot \theta$) until an objective or target is discovered. |

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
Launch AirSim and PX4 SITL, then run:
```bash
cd /path/to/AirSim/ros2

# Run obstacle avoidance (default):
./run_autonomy.sh obstacle_avoidance

# Or run scanning patrol:
./run_autonomy.sh scanning_patrol

# Or run target guiding:
./run_autonomy.sh target_guiding

# Or run area search:
./run_autonomy.sh area_search
```

