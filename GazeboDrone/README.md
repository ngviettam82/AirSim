# GazeboDrone Bridge for Cosys-AirSim

GazeboDrone creates a bridge between Gazebo simulator and Cosys-AirSim, allowing you to use Gazebo's physics engine for flight dynamics while leveraging Cosys-AirSim's advanced sensor simulation in Unreal Engine 5.5.

## Quick Start

### Prerequisites

- Ubuntu 22.04 LTS (recommended)
- Gazebo 11 or later
- Cosys-AirSim built and installed
- Unreal Engine 5.5
- GCC-11 (default on Ubuntu 22.04)

### Installation

**Note for Windows users:** These steps should be run in WSL (Windows Subsystem for Linux) with Ubuntu 22.04.

1. **Verify Gazebo installation:**
   ```bash
   dpkg -l | grep gazebo
   # Should show libgazebo-dev (11.x for Ubuntu 22.04)
   ```

   If not installed:
   ```bash
   sudo apt-get update
   sudo apt-get install libgazebo11-dev
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

3. **Build GazeboDrone:**
   ```bash
   cd GazeboDrone
   rm -rf build
   mkdir -p build && cd build
   cmake -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 ..
   make -j$(nproc)
   ```

   The executable will be at `GazeboDrone/build/GazeboDrone`.

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

1. **Start Cosys-AirSim in Unreal Engine**
2. **Launch your Gazebo simulation**
3. **Run the bridge:**
   ```bash
   cd GazeboDrone/build
   ./GazeboDrone
   ```

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