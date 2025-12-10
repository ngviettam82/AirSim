# GazeboDrone Bridge for Cosys-AirSim

GazeboDrone creates a bridge between Gazebo simulator and Cosys-AirSim, allowing you to use Gazebo's physics engine for flight dynamics while leveraging Cosys-AirSim's advanced sensor simulation in Unreal Engine 5.5.

**Supported Gazebo Versions:**
- **Gazebo Classic** (11.x) - Traditional Gazebo with `gazebo` command
- **Gazebo Ignition** (Gazebo Harmonic/Garden) - Next-gen Gazebo with `gz sim` command

## Quick Start

### Prerequisites

- Ubuntu 22.04 LTS (recommended)
- **For Gazebo Classic:** `libgazebo11-dev` package
- **For Gazebo Ignition:** `gz-transport` development packages
- Cosys-AirSim built and installed
- Unreal Engine 5.5
- GCC-11 (default on Ubuntu 22.04)

### Installation

**Note for Windows users:** These steps should be run in WSL (Windows Subsystem for Linux) with Ubuntu 22.04.

1. **Verify Gazebo installation:**
   
   For Gazebo Classic:
   ```bash
   dpkg -l | grep gazebo
   # Should show libgazebo-dev (11.x for Ubuntu 22.04)
   ```

   For Gazebo Ignition:
   ```bash
   gz sim --version
   # Should show Gazebo Sim version (e.g., 8.2.1)
   ```

   If not installed:
   ```bash
   # Gazebo Classic
   sudo apt-get update
   sudo apt-get install libgazebo11-dev

   # OR for Gazebo Ignition (if using PX4 SITL)
   sudo apt-get install libgz-transport13-dev
   ```

2. **Build Cosys-AirSim with GCC:**
   ```bash
   cd /path/to/Cosys-AirSim
   # Fix line endings if using Windows
   sed -i 's/\r$//' ./clean.sh ./setup.sh ./build.sh
   ./clean.sh
   # Dependencies should already be set up, verify:
   test -d AirLib/deps/eigen3/Eigen && echo "Dependencies OK" || ./setup.sh
   ./build.sh --gcc
   ```

3. **Build GazeboDrone (builds both Classic and Ignition variants):**
   ```bash
   cd GazeboDrone
   rm -rf build
   mkdir -p build && cd build
   cmake -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 -DBUILD_GZ=ON ..
   make -j$(nproc)
   ```

   This builds two executables:
   - `GazeboDrone/build/GazeboDrone` - For Gazebo Classic
   - `GazeboDrone/build/GazeboDrone_gz` - For Gazebo Ignition

   **Note:** If `BUILD_GZ=ON` fails (missing gz-transport), it will only build the Classic version.

### Configuration

Create or update `~/Documents/AirSim/settings.json`:

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

### Running

#### Option 1: Gazebo Classic

1. **Start Cosys-AirSim in Unreal Engine**
2. **Launch your Gazebo Classic simulation**
3. **Run the bridge:**
   ```bash
   cd GazeboDrone/build
   ./GazeboDrone
   ```

#### Option 2: Gazebo Ignition (for PX4 SITL)

1. **Start Cosys-AirSim in Unreal Engine**
2. **Launch PX4 SITL with Gazebo Ignition:**
   ```bash
   cd ~/PX4-Autopilot
   make px4_sitl gz_x500
   ```
3. **Run the bridge in a new terminal:**
   ```bash
   cd GazeboDrone/build
   ./GazeboDrone_gz --rpc_host localhost --rpc_port 41451 --topic /world/default/pose/info --vehicle "" --throttle 10
   ```

   **Parameters:**
   - `--rpc_host`: IP address of Cosys-AirSim (use the Windows host IP if running in WSL)
   - `--rpc_port`: RPC port (default 41451)
   - `--topic`: Gazebo Ignition pose topic (default: `/world/default/pose/info`)
   - `--vehicle`: Vehicle name in AirSim (empty string for first vehicle)
   - `--throttle`: Update throttle in milliseconds (default 10ms)

   **Example for WSL (Cosys-AirSim on Windows):**
   ```bash
   ./GazeboDrone_gz --rpc_host 10.5.9.45 --rpc_port 41451 --topic /world/default/pose/info --vehicle "" --throttle 10
   ```

   Replace `10.5.9.45` with your Windows host IP address.

## Documentation

For detailed documentation, troubleshooting, and advanced configuration, see:
- [Full GazeboDrone Documentation](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/docs/gazebo_drone.md)
- [Cosys-AirSim Installation Guide](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/docs/install_linux.md)

## Architecture

```
┌─────────────┐         ┌──────────────┐         ┌─────────────────┐
│   Gazebo    │ ◄─────► │ GazeboDrone  │ ◄─────► │  Cosys-AirSim   │
│   Physics   │  Topics │    Bridge    │   RPC   │  (UE 5.5)       │
│             │         │              │         │  Sensors        │
└─────────────┘         └──────────────┘         └─────────────────┘
```

- **Gazebo**: Handles flight dynamics and physics simulation
- **GazeboDrone Bridge**: Synchronizes pose data between systems
- **Cosys-AirSim**: Provides realistic sensor simulation (cameras, LiDAR, etc.)

## Features

- Real-time pose synchronization
- Support for multicopters and fixed-wing vehicles
- Compatible with standard Gazebo models
- Low-latency communication via RPC
- Configurable update rates

## System Requirements

### Recommended
- Ubuntu 22.04 LTS (tested in WSL2)
- 16GB RAM (32GB for complex scenes)
- NVIDIA GPU with 8GB+ VRAM
- GCC-11 (default on Ubuntu 22.04)
- Clang-18 (for Unreal Engine plugin)

### Supported Gazebo Versions
- **Gazebo Ignition (gz)** - **Tested and working with PX4 SITL**
- Gazebo 11 (Ubuntu 22.04) - **Tested and working**
- Gazebo 9 (Ubuntu 20.04)

### Build Notes

- **Tested Configuration**: Ubuntu 22.04.5 LTS, GCC 11.4.0, Gazebo 11.10.2, Cosys-AirSim with UE 5.5
- **Required Libraries**: TBB (Threading Building Blocks), stdc++fs
- The build process takes approximately 5-10 minutes depending on your hardware
- Warnings during compilation are normal and do not affect functionality

## Contributing

Contributions are welcome! Please see the main Cosys-AirSim repository for contribution guidelines.

## License

This project inherits the license from Cosys-AirSim. See the LICENSE file in the root directory.

## Support

For issues and questions:
- GitHub Issues: [Cosys-AirSim Issues](https://github.com/Cosys-Lab/Cosys-AirSim/issues)
- Documentation: [Cosys-AirSim Docs](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/docs)