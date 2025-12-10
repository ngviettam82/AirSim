# Welcome to GazeboDrone

GazeboDrone allows connecting a gazebo drone to the Cosys-AirSim drone, using the gazebo drone as a flight dynamic model (FDM) and Cosys-AirSim to generate environmental sensor data. It can be used for **Multicopters**, **Fixed-wings** or any other vehicle.

**Supported Gazebo Versions:**
- **Gazebo Classic** (11.x) - Traditional Gazebo with `gazebo` command
- **Gazebo Ignition** (Gazebo Harmonic/Garden) - Next-gen Gazebo with `gz sim` command

## Dependencies

### Gazebo

For **Gazebo Classic**, make sure you have installed Gazebo dependencies. For Ubuntu 22.04:

```bash
sudo apt-get install libgazebo11-dev
```

For **Gazebo Ignition** (if using PX4 SITL), install gz-transport development packages:

```bash
sudo apt-get install libgz-transport13-dev
```

For other Ubuntu versions, adjust the version number accordingly (e.g., `libgazebo9-dev` for Ubuntu 20.04).

### AirLib

For Cosys-AirSim with Unreal Engine 5.5, AirLib should be built with GCC-11 (default on Ubuntu 22.04). 
Run from your Cosys-AirSim root folder:  
```bash
./clean.sh
./setup.sh
./build.sh --gcc
```

**Note:** The `--gcc` flag ensures GCC is used instead of Clang. This is necessary because GazeboDrone links against Gazebo libraries which are typically built with GCC.

## Cosys-AirSim simulator

The Cosys-AirSim Unreal Engine plugin is built separately with Clang-18 (required for UE 5.5). Follow the [installation guide](install_linux.md) to set up Cosys-AirSim with Unreal Engine 5.5.


### Cosys-AirSim settings

Inside your `settings.json` file (typically located at `~/Documents/AirSim/settings.json`), you need to configure the external physics engine:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "PhysicsEngineName": "ExternalPhysicsEngine",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight"
    }
  }
}
```

You may want to change the visual model of the Cosys-AirSim drone. For custom vehicle meshes, see the [custom drone documentation](custom_drone.md).
## Build 

**Tested on Ubuntu 22.04 with GCC 11.4.0, Gazebo 11.10.2, and Gazebo Ignition**

Execute these commands from your Cosys-AirSim root folder:

```bash
cd GazeboDrone
rm -rf build  # Clean any previous build
mkdir -p build && cd build
cmake -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 -DBUILD_GZ=ON ..
make -j$(nproc)
```

This will build two executables:
- `GazeboDrone` - For Gazebo Classic
- `GazeboDrone_gz` - For Gazebo Ignition

**Note:** If `BUILD_GZ=ON` fails due to missing gz-transport packages, it will only build the Classic version.

**For Ubuntu 20.04:** Use `gcc-10` and `g++-10` instead.

**For Ubuntu 24.04:** Use `gcc-13` and `g++-13` instead.

If the specified GCC version is not installed, install it first:
```bash
sudo apt-get install gcc-11 g++-11
```

**Windows users (WSL):** If you encounter "bad interpreter" errors with shell scripts, fix line endings:
```bash
sed -i 's/\r$//' ./clean.sh ./setup.sh ./build.sh
```## Run

### Option 1: Gazebo Classic

#### Step 1: Start Cosys-AirSim Simulator

1. Launch Unreal Engine with your Cosys-AirSim environment (e.g., Blocks):
   ```bash
   cd ~/UnrealEngine  # or wherever you installed UE
   ./Engine/Binaries/Linux/UnrealEditor ~/Cosys-AirSim/Unreal/Environments/Blocks/Blocks.uproject
   ```

2. Press the **Play** button in the Unreal Editor.

#### Step 2: Start Gazebo Simulation

Launch your Gazebo world with your drone model:
```bash
gazebo your_world.world
```

Make sure your Gazebo model publishes pose information to the topics:
- `~/pose/local/info` (local pose)
- `~/pose/info` (global pose)

#### Step 3: Run GazeboDrone Bridge

Execute from your Cosys-AirSim root folder:
```bash
cd GazeboDrone/build
./GazeboDrone
```

The bridge will connect to both Cosys-AirSim (via RPC on port 41451 by default) and Gazebo, synchronizing the drone's position and orientation.

---

### Option 2: Gazebo Ignition (PX4 SITL)

#### Step 1: Start Cosys-AirSim Simulator

Same as Option 1 above.

#### Step 2: Start PX4 SITL with Gazebo Ignition

Launch PX4 Software-In-The-Loop with Gazebo Ignition:
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

This will start both PX4 and Gazebo Ignition with the X500 quadcopter model.

#### Step 3: Run GazeboDrone_gz Bridge

In a new terminal, execute:
```bash
cd GazeboDrone/build
./GazeboDrone_gz --rpc_host localhost --rpc_port 41451 --topic /world/default/pose/info --vehicle "" --throttle 10
```

**Parameters:**
- `--rpc_host`: IP address of Cosys-AirSim
  - Use `localhost` if running on the same machine
  - Use Windows host IP (e.g., `10.5.9.45`) if running in WSL
- `--rpc_port`: RPC port (default 41451)
- `--topic`: Gazebo Ignition pose topic (default: `/world/default/pose/info`)
- `--vehicle`: Vehicle name in AirSim (empty string `""` for first vehicle)
- `--throttle`: Update throttle in milliseconds (default 10ms)

**Example for WSL (Cosys-AirSim on Windows):**
```bash
./GazeboDrone_gz --rpc_host 10.5.9.45 --rpc_port 41451 --topic /world/default/pose/info --vehicle "" --throttle 10
```

To find your Windows host IP from WSL:
```bash
ip route show | grep -i default | awk '{ print $3}'
```

You should see output confirming the connection:
```
Waiting for connection - 
Connected!
Connected to AirSim at 10.5.9.45:41451
Attempted subscribe Pose_V -> OK
Subscribed to topic: /world/default/pose/info, forwarding pose[0] to AirSim at 10.5.9.45:41451
```

## Troubleshooting

### Connection Issues

- Ensure Cosys-AirSim is running and the drone is spawned before starting GazeboDrone
- Check that RPC port 41451 is not blocked by firewall
- For Gazebo Classic: Verify Gazebo is publishing to the correct topics: `gz topic -l`
- For Gazebo Ignition: Verify topics with `gz topic -l` and check for `/world/default/pose/info`
- **WSL Users:** Make sure to use the Windows host IP address (not localhost) when running bridge in WSL

### Build Issues

- If you get linking errors, ensure AirLib was built with `--gcc` flag
- Make sure the GCC version matches between AirLib and GazeboDrone builds
- Verify Gazebo development libraries are installed: `dpkg -l | grep gazebo`
- For GazeboDrone_gz: Ensure gz-transport dev packages are installed: `dpkg -l | grep gz-transport`
- If BUILD_GZ fails, CMake will show a warning and only build the Classic version

### RPC Version Mismatch Warning

If you see "AirSim client is of older version", this is usually harmless and the bridge will still work. This can occur when using Microsoft AirSim's GazeboDrone_gz with Cosys-AirSim, but functionality is not affected.

### Performance Issues

- The MESSAGE_THROTTLE in `main.cpp` controls console output frequency
- For better performance, consider running Gazebo headless: `gzserver your_world.world`
- Adjust the `--throttle` parameter for GazeboDrone_gz (higher value = lower CPU usage but slower updates)

## Advanced Configuration

### Custom Gazebo Classic Topics

If your Gazebo Classic model uses different topic names, modify the subscriber initialization in `src/main.cpp`:

```cpp
gazebo::transport::SubscriberPtr sub_pose1 = node->Subscribe("your/custom/topic", cbLocalPose);
```

### Custom Gazebo Ignition Topics

For Gazebo Ignition, pass the custom topic via command line:

```bash
./GazeboDrone_gz --topic /your/custom/pose/topic --vehicle "" --throttle 10
```

### Multiple Vehicles

For multi-vehicle simulations:
- **Gazebo Classic:** Modify the code to handle multiple RPC clients and map Gazebo models to Cosys-AirSim vehicles
- **Gazebo Ignition:** Use the `--vehicle` parameter to specify which AirSim vehicle to update

### Network Configuration for WSL

If running Cosys-AirSim on Windows and the bridge in WSL:

1. Find Windows host IP from WSL:
   ```bash
   ip route show | grep -i default | awk '{ print $3}'
   ```

2. Use this IP with `--rpc_host`:
   ```bash
   ./GazeboDrone_gz --rpc_host <WINDOWS_IP> --rpc_port 41451
   ```

3. Ensure Windows Firewall allows connections on port 41451

## References

- [Gazebo Documentation](http://gazebosim.org/tutorials)
- [Cosys-AirSim API Documentation](apis.md)
- [External Physics Engine](https://github.com/Cosys-Lab/Cosys-AirSim)

