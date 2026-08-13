# Download and install precompiled Plugin

If you do not want to build from source, download the precompiled plugin from the [releases page](https://github.com/ngviettam82/AirSim/releases) for **Unreal Engine 5.5** (Windows Win64 package).

## What you download

Release asset: **`AirSimPlugin-Win64.zip`**

After extract:

```text
Plugins/
  AirSim/
  AirSimShaders/
```

Both folders are required. Equirectangular HUD preview shaders live in `AirSimShaders`. Do not install AirSim alone.

## Install into your Unreal project

1. Install **Unreal Engine 5.5** (same major/minor as the package).
2. Create or open your Unreal project (see [Custom Unreal Environment](unreal_custenv.md)).
3. Copy the extracted `Plugins` folder next to your `.uproject` so the tree matches:

```text
YourProject/
  YourProject.uproject
  Plugins/
    AirSim/
    AirSimShaders/
```

4. Right-click the `.uproject` → Generate Visual Studio project files if needed.
5. Open the project, enable the AirSim plugins if prompted, set **AirSimGameMode**, press Play.

## Python client

Install the client **from this repository** (or the release source tree), not from PyPI:

```bash
cd PythonClient
pip install .
# then: import airsim
```

Do **not** `pip install cosysairsim` or unrelated PyPI `airsim` wheels with this fork.

## Related

- [Packaging from source](packaging.md) — how the zip is built with RunUAT
- [CI/CD](ci_cd.md) — automated release + optional self-hosted plugin job
- Releases also may include ROS 2 notes and Python wheel assets for that tag
