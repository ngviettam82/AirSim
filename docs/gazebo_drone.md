# Welcome to GazeboDrone

GazeboDrone allows connecting a gazebo drone to the Cosys-AirSim drone, using the gazebo drone as a flight dynamic model (FDM) and Cosys-AirSim to generate environmental sensor data. It can be used for **Multicopters**, **Fixed-wings** or any other vehicle.


## Dependencies

### Gazebo

Make sure you have installed Gazebo dependencies. For Ubuntu 22.04:

```bash
sudo apt-get install libgazebo11-dev
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

**Tested on Ubuntu 22.04 with GCC 11.4.0 and Gazebo 11.10.2**

Execute these commands from your Cosys-AirSim root folder:

```bash
cd GazeboDrone
rm -rf build  # Clean any previous build
mkdir -p build && cd build
cmake -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 ..
make -j$(nproc)
```

**For Ubuntu 20.04:** Use `gcc-10` and `g++-10` instead.

**For Ubuntu 24.04:** Use `gcc-13` and `g++-13` instead.

If the specified GCC version is not installed, install it first:
```bash
sudo apt-get install gcc-11 g++-11
```

**Windows users (WSL):** If you encounter "bad interpreter" errors with shell scripts, fix line endings:
```bash
sed -i 's/\r$//' ./clean.sh ./setup.sh ./build.sh
```

The build process will create the `GazeboDrone` executable in the `build` directory.## Run

### Step 1: Start Cosys-AirSim Simulator

1. Launch Unreal Engine with your Cosys-AirSim environment (e.g., Blocks):
   ```bash
   cd ~/UnrealEngine  # or wherever you installed UE
   ./Engine/Binaries/Linux/UnrealEditor ~/Cosys-AirSim/Unreal/Environments/Blocks/Blocks.uproject
   ```

2. Press the **Play** button in the Unreal Editor.

### Step 2: Start Gazebo Simulation

Launch your Gazebo world with your drone model:
```bash
gazebo your_world.world
```

Make sure your Gazebo model publishes pose information to the topics:
- `~/pose/local/info` (local pose)
- `~/pose/info` (global pose)

### Step 3: Run GazeboDrone Bridge

Execute from your Cosys-AirSim root folder:
```bash
cd GazeboDrone/build
./GazeboDrone
```

The bridge will connect to both Cosys-AirSim (via RPC on port 41451 by default) and Gazebo, synchronizing the drone's position and orientation.

## Troubleshooting

### Connection Issues

- Ensure Cosys-AirSim is running and the drone is spawned before starting GazeboDrone
- Check that RPC port 41451 is not blocked by firewall
- Verify Gazebo is publishing to the correct topics: `gz topic -l`

### Build Issues

- If you get linking errors, ensure AirLib was built with `--gcc` flag
- Make sure the GCC version matches between AirLib and GazeboDrone builds
- Verify Gazebo development libraries are installed: `dpkg -l | grep gazebo`

### Performance Issues

- The MESSAGE_THROTTLE in `main.cpp` controls console output frequency
- For better performance, consider running Gazebo headless: `gzserver your_world.world`

## Advanced Configuration

### Custom Gazebo Topics

If your Gazebo model uses different topic names, modify the subscriber initialization in `src/main.cpp`:

```cpp
gazebo::transport::SubscriberPtr sub_pose1 = node->Subscribe("your/custom/topic", cbLocalPose);
```

### Multiple Vehicles

For multi-vehicle simulations, you'll need to modify the code to handle multiple RPC clients and map Gazebo models to Cosys-AirSim vehicles appropriately.

## References

- [Gazebo Documentation](http://gazebosim.org/tutorials)
- [Cosys-AirSim API Documentation](apis.md)
- [External Physics Engine](https://github.com/Cosys-Lab/Cosys-AirSim)

