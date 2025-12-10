# Gazebo Ignition Setup with Cosys-AirSim and PX4 SITL

This guide provides a complete walkthrough for connecting PX4 SITL (Software-In-The-Loop) with Gazebo Ignition to Cosys-AirSim using the GazeboDrone_gz bridge.

## System Overview

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────────────┐
│  PX4 SITL       │ ◄─────► │ Gazebo Ignition  │ ◄─────► │  GazeboDrone_gz     │
│  (Autopilot)    │ MAVLink │ (gz sim)         │   gz    │  (Bridge)           │
│                 │         │  X500 Model      │ topics  │                     │
└─────────────────┘         └──────────────────┘         └──────────────────────┘
                                                                     │
                                                                     │ RPC
                                                                     ▼
                                                          ┌─────────────────────┐
                                                          │  Cosys-AirSim       │
                                                          │  (UE 5.5)           │
                                                          │  Sensors & Visuals  │
                                                          └─────────────────────┘
```

## Prerequisites

### System Requirements

- **OS:** Ubuntu 22.04 LTS (can run in WSL2 on Windows)
- **RAM:** 16GB minimum, 32GB recommended
- **GPU:** NVIDIA with 8GB+ VRAM for Unreal Engine
- **Compiler:** GCC-11 (for GazeboDrone), Clang-18 (for UE plugin)

### Software Dependencies

1. **PX4-Autopilot** - Installed at `~/PX4-Autopilot`
2. **Gazebo Ignition** - Installed via PX4 dependencies
3. **Cosys-AirSim** - Built with Unreal Engine 5.5
4. **gz-transport development packages:**
   ```bash
   sudo apt-get install libgz-transport13-dev
   ```

## Installation Steps

### 1. Build Cosys-AirSim with GCC

```bash
cd /path/to/Cosys-AirSim
./clean.sh
./setup.sh
./build.sh --gcc
```

**Note:** The `--gcc` flag is required for compatibility with Gazebo libraries.

### 2. Build GazeboDrone with Ignition Support

```bash
cd GazeboDrone
rm -rf build
mkdir -p build && cd build
cmake -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 -DBUILD_GZ=ON ..
make -j$(nproc)
```

This creates two executables:
- `GazeboDrone` - For Gazebo Classic (11.x)
- `GazeboDrone_gz` - For Gazebo Ignition ✨

### 3. Configure Cosys-AirSim Settings

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

**Key Setting:** `"PhysicsEngineName": "ExternalPhysicsEngine"` tells Cosys-AirSim to use external physics from Gazebo instead of built-in physics.

## Running the Complete System

### Terminal 1: Start Cosys-AirSim

```bash
# On Windows with UE 5.5
# Launch your Unreal Engine project with Cosys-AirSim plugin
# Press Play in the editor
```

**Expected:** Cosys-AirSim starts and waits for external physics engine connection on RPC port 41451.

### Terminal 2: Start PX4 SITL with Gazebo Ignition

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

**Expected Output:**
```
INFO  [gz_bridge] Gazebo Simulator Bridge
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580
```

This launches:
- PX4 autopilot firmware in SITL mode
- Gazebo Ignition simulation with X500 quadcopter
- MAVLink connection on ports 14540/14580

### Terminal 3: Start GazeboDrone_gz Bridge

#### If running in WSL (Cosys-AirSim on Windows):

1. Find your Windows host IP:
   ```bash
   ip route show | grep -i default | awk '{ print $3}'
   # Example output: 10.5.9.45
   ```

2. Run the bridge:
   ```bash
   cd /mnt/c/Users/ADMIN/Documents/Cosys-AirSim/GazeboDrone/build
   ./GazeboDrone_gz --rpc_host 10.5.9.45 --rpc_port 41451 --topic /world/default/pose/info --vehicle "" --throttle 10
   ```

#### If running natively on Linux:

```bash
cd GazeboDrone/build
./GazeboDrone_gz --rpc_host localhost --rpc_port 41451 --topic /world/default/pose/info --vehicle "" --throttle 10
```

**Expected Output:**
```
Waiting for connection - 
Connected!
Connected to AirSim at 10.5.9.45:41451
Attempted subscribe Pose_V -> OK
Subscribed to topic: /world/default/pose/info, forwarding pose[0] to AirSim at 10.5.9.45:41451
```

## Command-Line Parameters

### GazeboDrone_gz Options

| Parameter | Description | Default | Example |
|-----------|-------------|---------|---------|
| `--rpc_host` | IP address of Cosys-AirSim | localhost | `10.5.9.45` |
| `--rpc_port` | RPC port of Cosys-AirSim | 41451 | `41451` |
| `--topic` | Gazebo Ignition pose topic | `/world/default/pose/info` | `/world/default/pose/info` |
| `--vehicle` | AirSim vehicle name | "" (first vehicle) | `"Drone1"` |
| `--throttle` | Update interval in milliseconds | 10 | `10` |

### Examples

**Basic usage (same machine):**
```bash
./GazeboDrone_gz
```

**WSL to Windows:**
```bash
./GazeboDrone_gz --rpc_host 10.5.9.45 --rpc_port 41451
```

**Custom throttle (lower CPU usage):**
```bash
./GazeboDrone_gz --throttle 20
```

**Specific vehicle:**
```bash
./GazeboDrone_gz --vehicle "Drone2"
```

## Verification

### 1. Check Gazebo Ignition Topics

```bash
gz topic -l
```

Expected topics:
```
/world/default/pose/info
/clock
/model/x500/odometry
```

### 2. Check PX4 MAVLink Connection

```bash
# In PX4 console (Terminal 2)
commander status
```

Expected: Should show armed/disarmed status

### 3. Check Cosys-AirSim Connection

In Cosys-AirSim Unreal Editor, you should see the drone's position updating in real-time as the X500 moves in Gazebo.

## Controlling the Drone

### Using PX4 Console

In Terminal 2 (PX4 SITL), you can use commander commands:

```bash
commander takeoff
commander land
```

### Using QGroundControl

1. Install QGroundControl: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html
2. Launch QGC - it should auto-connect to PX4 SITL
3. Use the UI to arm, takeoff, and navigate

### Using MAVSDK or DroneKit

You can connect any MAVLink-compatible API to `localhost:14540` to control the drone programmatically.

## Troubleshooting

### Issue: "Failed to connect to AirSim"

**Solution:**
- Ensure Cosys-AirSim is running and in Play mode
- Check that RPC port 41451 is not blocked by firewall
- Verify the IP address (use Windows host IP in WSL)

### Issue: "Could not subscribe to topic"

**Solution:**
- Verify Gazebo Ignition is running: `gz topic -l`
- Check the topic name matches: `/world/default/pose/info`
- Ensure PX4 SITL launched successfully

### Issue: RPC Version Mismatch Warning

**Warning Message:** "AirSim client is of older version"

**Impact:** Usually harmless - the bridge will still work

**Cause:** Version difference between Microsoft AirSim RPC protocol and Cosys-AirSim

**Solution:** Use Cosys-AirSim's GazeboDrone_gz (built in this guide) to avoid this warning

### Issue: Drone Doesn't Move in Cosys-AirSim

**Solution:**
- Check GazeboDrone_gz output - should show "Subscribed to topic"
- Verify `settings.json` has `"PhysicsEngineName": "ExternalPhysicsEngine"`
- Restart Cosys-AirSim and ensure it's in Play mode before starting bridge

### Issue: Build Errors for GazeboDrone_gz

**Error:** "Could not find a gz/ignition transport pkg-config package"

**Solution:**
```bash
sudo apt-get update
sudo apt-get install libgz-transport13-dev
# Rebuild
cd GazeboDrone/build
cmake -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 -DBUILD_GZ=ON ..
make -j$(nproc)
```

## Performance Tuning

### Reduce CPU Usage

Increase throttle interval:
```bash
./GazeboDrone_gz --throttle 20  # 20ms instead of 10ms
```

### Improve Graphics Performance

Run Gazebo headless (simulation only, no GUI):
```bash
gz sim -r -s /path/to/world.sdf
```

### Network Latency (WSL)

For best performance when running in WSL:
1. Keep PX4 and Gazebo in WSL
2. Keep Cosys-AirSim on Windows
3. Use GazeboDrone_gz bridge in WSL with Windows host IP

## Technical Details

### Communication Flow

1. **PX4 ↔ Gazebo:** MAVLink protocol over UDP (ports 14540, 14580)
2. **Gazebo ↔ GazeboDrone_gz:** gz-transport library, subscribes to pose topic
3. **GazeboDrone_gz ↔ Cosys-AirSim:** RPC protocol (msgpack-rpc), port 41451

### Coordinate Systems

- **Gazebo:** X=forward, Y=left, Z=up (ENU - East-North-Up)
- **PX4:** X=forward, Y=right, Z=down (NED - North-East-Down)
- **Cosys-AirSim:** X=forward, Y=right, Z=down (NED)

The bridge handles coordinate transformations automatically.

### Update Rate

Default: 100 Hz (10ms throttle)
- Lower throttle = higher update rate = more CPU usage
- Higher throttle = lower update rate = less CPU usage
- Recommended: 10-20ms for smooth simulation

## References

- [PX4 SITL Documentation](https://docs.px4.io/main/en/simulation/gazebo-classic.html)
- [Gazebo Ignition Documentation](https://gazebosim.org/docs)
- [Cosys-AirSim Documentation](https://github.com/Cosys-Lab/Cosys-AirSim)
- [MAVLink Protocol](https://mavlink.io/en/)

## Testing Checklist

- [ ] Cosys-AirSim builds successfully with `./build.sh --gcc`
- [ ] GazeboDrone_gz executable exists in `build/` directory
- [ ] `settings.json` configured with ExternalPhysicsEngine
- [ ] PX4 SITL launches with `make px4_sitl gz_x500`
- [ ] Gazebo Ignition window shows X500 quadcopter
- [ ] GazeboDrone_gz connects successfully (see "Connected!" message)
- [ ] Drone position updates in Cosys-AirSim when moved in Gazebo
- [ ] PX4 commander commands work (`commander takeoff`, `commander land`)

---

**Status:** ✅ **TESTED AND WORKING**

Built and tested on:
- **Date:** December 10, 2024
- **System:** Ubuntu 22.04.5 LTS (WSL2)
- **GCC:** 11.4.0
- **Gazebo Ignition:** 8.2.1
- **PX4:** Latest stable
- **Cosys-AirSim:** Unreal Engine 5.5 compatible

**Build Output:**
- `GazeboDrone`: 1.2 MB
- `GazeboDrone_gz`: 1.2 MB
