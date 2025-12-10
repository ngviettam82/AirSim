# GazeboDrone Build Guide

## Tested Configuration

- **OS**: Ubuntu 22.04.5 LTS (WSL2 on Windows 11)
- **Compiler**: GCC 11.4.0  
- **Gazebo**: 11.10.2
- **Cosys-AirSim**: Compatible with Unreal Engine 5.5
- **Build Date**: December 10, 2025

## Prerequisites

Ensure you have the following installed:

```bash
# Check Ubuntu version
lsb_release -a

# Check GCC version
gcc --version  # Should be 11.x for Ubuntu 22.04

# Check Gazebo installation
dpkg -l | grep gazebo
```

## Step-by-Step Build Process

### Step 1: Prepare the Environment

If running in WSL on Windows, fix line endings for shell scripts:

```bash
cd /path/to/Cosys-AirSim
sed -i 's/\r$//' ./clean.sh ./setup.sh ./build.sh
```

### Step 2: Build Cosys-AirSim with GCC

```bash
cd /path/to/Cosys-AirSim

# Clean previous builds
./clean.sh

# Verify dependencies are installed
test -d AirLib/deps/eigen3/Eigen && echo "Dependencies OK" || ./setup.sh

# Build with GCC (required for Gazebo compatibility)
./build.sh --gcc
```

**Build time**: Approximately 5-10 minutes on modern hardware

**Expected output**: Libraries in `build_release/output/lib/`:
- `libAirLib.a` (~3.1 MB)
- `libMavLinkCom.a` (~2.4 MB)
- `librpc.a` (~1.3 MB)

### Step 3: Build GazeboDrone

```bash
cd GazeboDrone
rm -rf build  # Clean any previous build
mkdir -p build && cd build

# Configure with CMake
cmake -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 ..

# Build
make -j$(nproc)
```

**Build time**: Less than 1 minute

**Expected output**: Executable at `GazeboDrone/build/GazeboDrone` (~1.6 MB)

### Step 4: Verify the Build

```bash
# Check the executable was created
ls -lh GazeboDrone
file GazeboDrone

# Expected output: ELF 64-bit LSB pie executable
```

## Build Configuration Details

### CMake Configuration

The `CMakeLists.txt` has been updated with the following key settings:

- **C++ Standard**: C++17 (required for UE 5.5 compatibility)
- **Compiler**: GCC-11 (for ABI compatibility with Gazebo)
- **Link Directories**:
  - `${AIRSIM_ROOT}/build_release/output/lib`
  - `${AIRSIM_ROOT}/AirLib/lib`
  - `${AIRSIM_ROOT}/AirLib/deps/rpclib/lib`
  - `${AIRSIM_ROOT}/AirLib/deps/MavLinkCom/lib`
- **Linked Libraries**: 
  - Gazebo libraries
  - pthread
  - AirLib
  - rpc
  - tbb (Threading Building Blocks)
  - stdc++fs (filesystem)

### Compiler Flags

The build uses GCC with the following environment:
```bash
export CC=gcc
export CXX=g++
```

## Troubleshooting

### Issue: Libraries not found during linking

**Error**: `cannot find -lAirLib` or `cannot find -lrpc`

**Solution**: Ensure Cosys-AirSim was built with `--gcc` flag:
```bash
cd /path/to/Cosys-AirSim
./clean.sh
./build.sh --gcc
```

### Issue: TBB library missing

**Error**: `undefined reference to symbol '_ZN3tbb6detail2r114notify_waitersEm'`

**Solution**: Install TBB:
```bash
sudo apt-get install libtbb-dev
```

This is now included in the `target_link_libraries` in CMakeLists.txt.

### Issue: Bad interpreter error on WSL

**Error**: `/bin/bash^M: bad interpreter`

**Solution**: Convert line endings:
```bash
sed -i 's/\r$//' ./clean.sh ./setup.sh ./build.sh
```

### Issue: GCC version mismatch

**Error**: ABI compatibility errors

**Solution**: Ensure the same GCC version is used for both Cosys-AirSim and GazeboDrone:
```bash
# For Ubuntu 22.04
cmake -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 ..
```

## Running GazeboDrone

After successful build, you can run the bridge:

```bash
cd GazeboDrone/build
./GazeboDrone
```

**Prerequisites for running**:
1. Cosys-AirSim must be running in Unreal Engine 5.5
2. Gazebo simulation must be running with a compatible drone model
3. Settings file configured with `"PhysicsEngineName": "ExternalPhysicsEngine"`

See the main [gazebo_drone.md](../docs/gazebo_drone.md) documentation for complete usage instructions.

## Build Warnings

The following warnings are normal and do not affect functionality:

- `-Wclass-memaccess` warnings from rpclib/msgpack
- `-Wsign-compare` warnings in API code
- `-Wmaybe-uninitialized` warnings in test code
- `-Wdeprecated-copy` warnings from Eigen

These are compiler warnings about code style and do not indicate errors.

## Version Compatibility Matrix

| Ubuntu Version | GCC Version | Gazebo Version | Tested |
|----------------|-------------|----------------|--------|
| 20.04          | gcc-10      | 9              | ⚠️     |
| 22.04          | gcc-11      | 11             | ✅     |
| 24.04          | gcc-13      | 11             | ⚠️     |

✅ = Fully tested and working  
⚠️ = Should work but not fully tested

## Performance Notes

- Build with `-j$(nproc)` for parallel compilation (much faster)
- First build takes longer due to dependency compilation
- Incremental builds after code changes are much faster
- WSL builds may be slower than native Linux due to filesystem overhead

## Next Steps

After successful build, see:
- [gazebo_drone.md](../docs/gazebo_drone.md) - Complete usage documentation
- [README.md](README.md) - Quick start guide
- [Cosys-AirSim Installation Guide](../docs/install_linux.md) - Main installation docs

## Support

For issues with the build process:
- Check [GitHub Issues](https://github.com/Cosys-Lab/Cosys-AirSim/issues)
- Review [Cosys-AirSim Documentation](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/docs)
- Ensure all prerequisites are correctly installed
