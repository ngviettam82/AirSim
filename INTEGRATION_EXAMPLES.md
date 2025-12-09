# ProjectAirSim Features Integration - Usage Examples

## Battery Sensor Integration

### Settings Configuration

Add to your `settings.json` in the vehicle's `Sensors` section:

```json
{
  "Vehicles": {
    "survey": {
      "VehicleType": "PX4Multirotor",
      "Sensors": {
        "Battery": {
          "SensorType": 8,
          "Enabled": true,
          "Capacity": 5000,
          "Voltage": 14.8,
          "MaxVoltage": 16.8,
          "MinVoltage": 12.0,
          "InternalResistance": 0.05,
          "CriticalPercentage": 20.0,
          "SimulationMode": "PhysicsBased"
        }
      }
    }
  }
}
```

### Simulation Modes

**Linear Mode** (Simplest)
- Constant voltage discharge
- Linear charge consumption
- Best for: Quick prototyping, simple mission planning

**VoltageBased Mode** (Realistic)
- Voltage drops with discharge
- Follows typical LiPo discharge curve
- Best for: Voltage-sensitive systems, realistic visualization

**PhysicsBased Mode** (Most Accurate)
- Considers internal resistance
- Load-dependent voltage drop
- Current calculation from power demand
- Best for: Accurate endurance prediction, power analysis

---

## Python API Usage

### Basic Battery Monitoring

```python
import cosysairsim as airsim
import time

# Connect to simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Monitor battery during flight
client.takeoffAsync().join()

while True:
    # Get battery state
    battery_data = client.getBatteryData()
    
    print(f"Voltage: {battery_data.voltage:.2f}V")
    print(f"Current: {battery_data.current:.2f}A")
    print(f"Charge: {battery_data.percentage:.1f}%")
    print(f"Time remaining: {battery_data.time_remaining/60:.1f} min")
    
    # Check critical level
    if battery_data.is_critical:
        print("⚠️ CRITICAL BATTERY LEVEL!")
        client.landAsync().join()
        break
    
    time.sleep(1)
```

### Mission Planning with Battery

```python
import cosysairsim as airsim
import numpy as np

class BatteryAwareMission:
    def __init__(self, client, min_battery=25.0):
        self.client = client
        self.min_battery = min_battery
        self.home_position = None
    
    def execute_waypoint_mission(self, waypoints, velocity=5.0):
        """Execute mission with battery monitoring"""
        self.home_position = self.client.getMultirotorState().kinematics_estimated.position
        
        for i, waypoint in enumerate(waypoints):
            # Check battery before each waypoint
            battery = self.client.getBatteryData()
            
            if battery.percentage < self.min_battery:
                print(f"Battery low ({battery.percentage:.1f}%), returning home...")
                self.return_to_home()
                return False
            
            # Estimate if we can complete next waypoint and return
            if not self.can_complete_waypoint(waypoint, battery):
                print("Insufficient battery to complete mission safely")
                self.return_to_home()
                return False
            
            print(f"Waypoint {i+1}/{len(waypoints)}: Battery at {battery.percentage:.1f}%")
            self.client.moveToPositionAsync(
                waypoint[0], waypoint[1], waypoint[2], velocity
            ).join()
        
        self.return_to_home()
        return True
    
    def can_complete_waypoint(self, waypoint, battery):
        """Estimate if battery can handle waypoint + return"""
        current_pos = self.client.getMultirotorState().kinematics_estimated.position
        
        # Distance to waypoint
        dist_to_wp = np.linalg.norm([
            waypoint[0] - current_pos.x_val,
            waypoint[1] - current_pos.y_val,
            waypoint[2] - current_pos.z_val
        ])
        
        # Distance from waypoint to home
        dist_to_home = np.linalg.norm([
            waypoint[0] - self.home_position.x_val,
            waypoint[1] - self.home_position.y_val,
            waypoint[2] - self.home_position.z_val
        ])
        
        total_distance = dist_to_wp + dist_to_home
        
        # Rough estimate: assume constant power consumption
        # If we have 30% battery left and need to travel 30% of max range, we're safe
        estimated_time = total_distance / 5.0  # Assuming 5 m/s average
        safety_margin = 1.3  # 30% safety margin
        
        return battery.time_remaining > (estimated_time * safety_margin)
    
    def return_to_home(self):
        """Safe return to home"""
        print("Returning to home position...")
        self.client.moveToPositionAsync(
            self.home_position.x_val,
            self.home_position.y_val,
            self.home_position.z_val,
            5.0
        ).join()
        self.client.landAsync().join()

# Usage
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

mission = BatteryAwareMission(client, min_battery=25.0)

waypoints = [
    [10, 0, -10],
    [10, 10, -10],
    [0, 10, -10],
    [0, 0, -10]
]

success = mission.execute_waypoint_mission(waypoints)
print(f"Mission {'completed' if success else 'aborted'}")
```

### Real-time Battery Analytics

```python
import cosysairsim as airsim
import matplotlib.pyplot as plt
from collections import deque
import time

class BatteryLogger:
    def __init__(self, client, window_size=100):
        self.client = client
        self.window_size = window_size
        
        self.times = deque(maxlen=window_size)
        self.voltages = deque(maxlen=window_size)
        self.currents = deque(maxlen=window_size)
        self.percentages = deque(maxlen=window_size)
        self.powers = deque(maxlen=window_size)
        
        self.start_time = time.time()
    
    def update(self):
        battery = self.client.getBatteryData()
        current_time = time.time() - self.start_time
        
        self.times.append(current_time)
        self.voltages.append(battery.voltage)
        self.currents.append(battery.current)
        self.percentages.append(battery.percentage)
        self.powers.append(battery.power)
    
    def plot_realtime(self):
        plt.clf()
        
        plt.subplot(2, 2, 1)
        plt.plot(self.times, self.voltages)
        plt.ylabel('Voltage (V)')
        plt.title('Battery Voltage')
        plt.grid(True)
        
        plt.subplot(2, 2, 2)
        plt.plot(self.times, self.currents)
        plt.ylabel('Current (A)')
        plt.title('Current Draw')
        plt.grid(True)
        
        plt.subplot(2, 2, 3)
        plt.plot(self.times, self.percentages)
        plt.ylabel('Charge (%)')
        plt.xlabel('Time (s)')
        plt.title('State of Charge')
        plt.grid(True)
        
        plt.subplot(2, 2, 4)
        plt.plot(self.times, self.powers)
        plt.ylabel('Power (W)')
        plt.xlabel('Time (s)')
        plt.title('Power Consumption')
        plt.grid(True)
        
        plt.tight_layout()
        plt.pause(0.01)

# Usage with real-time plotting
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

logger = BatteryLogger(client)

plt.ion()
fig = plt.figure(figsize=(12, 8))

# Perform flight
client.takeoffAsync().join()
client.moveToPositionAsync(10, 10, -10, 5).join()

# Log and plot while hovering
for _ in range(100):
    logger.update()
    logger.plot_realtime()
    time.sleep(0.1)

client.landAsync().join()
plt.ioff()
plt.show()
```

---

## C++ API Usage

### Battery Monitoring in C++

```cpp
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace msr::airlib;

class BatteryMonitor {
public:
    BatteryMonitor(MultirotorRpcLibClient* client) 
        : client_(client) {}
    
    void monitorBattery(int duration_sec) {
        for (int i = 0; i < duration_sec; ++i) {
            auto battery_data = client_->getBatteryData();
            
            std::cout << "Time: " << i << "s | "
                      << "Voltage: " << battery_data.voltage << "V | "
                      << "Current: " << battery_data.current << "A | "
                      << "Charge: " << battery_data.percentage << "% | "
                      << "Remaining: " << (battery_data.time_remaining / 60.0f) << " min"
                      << std::endl;
            
            if (battery_data.is_critical) {
                std::cout << "⚠️  CRITICAL BATTERY LEVEL!" << std::endl;
                client_->landAsync()->waitOnLastTask();
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

private:
    MultirotorRpcLibClient* client_;
};

int main() {
    MultirotorRpcLibClient client;
    client.confirmConnection();
    client.enableApiControl(true);
    client.armDisarm(true);
    
    // Takeoff
    client.takeoffAsync()->waitOnLastTask();
    
    // Monitor battery
    BatteryMonitor monitor(&client);
    monitor.monitorBattery(300);  // Monitor for 5 minutes
    
    // Land
    client.landAsync()->waitOnLastTask();
    client.armDisarm(false);
    client.enableApiControl(false);
    
    return 0;
}
```

---

## ROS Integration Example

### Battery State Publisher

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import BatteryState
import cosysairsim as airsim

class BatteryStatePublisher:
    def __init__(self):
        rospy.init_node('airsim_battery_publisher')
        
        self.pub = rospy.Publisher('/airsim/battery_state', BatteryState, queue_size=10)
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        
        self.rate = rospy.Rate(10)  # 10 Hz
    
    def run(self):
        while not rospy.is_shutdown():
            battery_data = self.client.getBatteryData()
            
            msg = BatteryState()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "battery"
            
            msg.voltage = battery_data.voltage
            msg.current = battery_data.current
            msg.charge = battery_data.charge_remaining / 1000.0  # Convert to Ah
            msg.capacity = self.client.getBatteryConfig().capacity_mah / 1000.0
            msg.percentage = battery_data.percentage / 100.0
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            msg.present = True
            
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = BatteryStatePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
```

---

## Next Features to Integrate

Once battery sensor is working, these are good next targets:

### 1. Enhanced Python API (Week 2-3)
- Async/await support
- Context managers
- Type hints
- Better error handling

### 2. API Documentation (Week 3-4)
- Sphinx setup
- Auto-generated API docs
- Interactive examples

### 3. Improved Build System (Week 4-6)
- CMake integration
- Faster incremental builds
- Better VS Code support

**Which feature would you like to implement next?**
