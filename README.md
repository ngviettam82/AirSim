# Custom AirSim

Custom AirSim is an advanced open-source simulator for autonomous drones, ground vehicles, and robotics perception, built on [Unreal Engine 5.5](https://www.unrealengine.com/). It provides physically and visually realistic simulation environments with native hardware-in-the-loop (HIL) and software-in-the-loop (SITL) support for flight controllers such as PX4 Autopilot.

This repository is an actively maintained **Custom AirSim** distribution featuring:
* **PX4 ROS 2 Autonomy Framework**: Production C++17 autonomy stack (`px4_ros2::ModeBase`), dynamic stopping bubbles, photogrammetry survey mission planner, 3D geofencing, and Smart RTH battery models.
* **Web Companion Ground Control Station (GCS)**: Zero-install tablet & browser cockpit (`http://localhost:8080`) featuring Google Maps (Satellite/Hybrid), Primary Flight Display (PFD) HUD, 50 Hz real-time WebSocket telemetry, interactive boustrophedon survey planning, and click-to-track visual targeting.
* **Unreal Native CameraHost**: High-throughput HTTP/MJPEG streaming server built directly into Unreal Engine for low-latency multi-camera viewports.
* **ROS 2 Humble Integration (`airsim_ros_pkgs`)**: Direct MCAP rosbag2 recording, hardware-accurate timestamp synchronization, and multi-vehicle control.
* **Advanced Perception Suite**: Multi-camera depth unprojection, GPU LiDAR (with multirotor async capture), pulse-echo radar, instance segmentation, and infrared imaging.

The [MIT License](LICENSE) applies to all source files in this repository.

This [main branch](https://github.com/ngviettam82/Airsim/tree/main) is actively maintained for Unreal Engine v5.5, with precompiled plugin releases available in [Releases](https://github.com/ngviettam82/Airsim/releases).
Unreal [5.2.1](https://github.com/ngviettam82/Airsim/tree/5.2.1) is also available for long-term support builds.


## Associated publications

- [AirSim: A Real-Time Simulation Framework Expanded for Complex Industrial Applications](https://arxiv.org/abs/2303.13381)
```
@inproceedings{AirSim2023jansen,
  author={Jansen, Wouter and Verreycken, Erik and Schenck, Anthony and Blanquart, Jean-Edouard and Verhulst, Connor and Huebel, Nico and Steckel, Jan},
  booktitle={2023 Annual Modeling and Simulation Conference (ANNSIM)}, 
  title={AirSim: A Real-Time Simulation Framework Expanded for Complex Industrial Applications}, 
  year={2023},
  volume={},
  number={},
  pages={37-48},
  doi={}}
```


- [Physical LiDAR Simulation in Real-Time Engine](https://arxiv.org/abs/2208.10295)
```
@inproceedings{lidarsim2022jansen,
  author={Jansen, Wouter and Huebel, Nico and Steckel, Jan},
  booktitle={2022 IEEE Sensors}, 
  title={Physical LiDAR Simulation in Real-Time Engine}, 
  year={2022},
  volume={},
  number={},
  pages={1-4},
  doi={10.1109/SENSORS52175.2022.9967197}}
}
```
- [Simulation of Pulse-Echo Radar for Vehicle Control and SLAM](https://www.mdpi.com/1424-8220/21/2/523)
```
@Article{echosim2021schouten,
  author={Schouten, Girmi and Jansen, Wouter and Steckel, Jan},
  title={Simulation of Pulse-Echo Radar for Vehicle Control and SLAM},
  JOURNAL={Sensors},
  volume={21},
  year={2021},
  number={2},
  article-number={523},
  doi={10.3390/s21020523}
}
```

## How to get it

* **Source / this fork:** [github.com/ngviettam82/AirSim](https://github.com/ngviettam82/AirSim)
* **Precompiled UE 5.5 plugin (Win64):** [Releases](https://github.com/ngviettam82/AirSim/releases) → `AirSimPlugin-Win64.zip` (install [guide](docs/install_precompiled.md))
* **Packaged binary (Blocks):** [Releases](https://github.com/ngviettam82/AirSim/releases) (run [guide](docs/run_packaged.md))
* **Build from Source:** [Windows](docs/install_windows.md) / [Linux](docs/install_linux.md)
* **Python client:** install from `PythonClient` in this repo only (`pip install .` → `import airsim`). Do not install third-party PyPI wheels (such as legacy `airsim` or `cosysairsim`).
* **Docs:** [docs/](docs/) in this repository (`mkdocs build` from repo root)

## Custom AirSim Capabilities & Architecture

Custom AirSim provides advanced, production-grade robotics and autonomy capabilities on Unreal Engine 5.5. See [CHANGELOG](CHANGELOG.md).

**Note:** Built-in labeling is **source-stencil Segmentation/Infrared** only.

Notable capabilities (docs in-tree):


* [PX4 ROS 2 Autonomy Architecture](docs/px4_ros2_autonomy.md) — Production C++17 autonomy stack (`px4_ros2::ModeBase`), dynamic stopping bubbles, photogrammetry survey planner, 3D geofence, and Smart RTH battery models.
* [Web Companion Ground Control Station (GCS)](docs/web_gcs.md) — Touch-friendly tablet & browser UI with Leaflet/Google Maps, 50 Hz WebSocket telemetry, click-to-track visual targeting, and boustrophedon survey mission generator.
* [PX4 SITL with AirSim](docs/px4_sitl.md) & [WSL 2 Integration](docs/px4_sitl_wsl2.md) — Complete port triad guide (TCP 4560 simulator, UDP 8888 MicroXRCEAgent, UDP 14550 QGC).
* [ROS 2 C++ Wrapper (`airsim_ros_pkgs`)](docs/ros_cplusplus.md) — Full ROS 2 Humble bridge with direct MCAP rosbag2 recording and hardware timestamp synchronization.
* [Instance segmentation](docs/instance_segmentation.md) / [annotation notes](docs/annotation.md)
* [GPU LiDAR](docs/gpulidar.md) (including Multirotor async path)
* [Echo](docs/echo.md), [skid steer](docs/skid_steer_vehicle.md), [dynamic objects](docs/dynamic_objects.md), [lights](docs/lights.md)
* [Native Camera Host](docs/camera_host.md), [multirotor physics](docs/multirotor_physics.md)
* [Legacy ROS 1 Python Wrapper](docs/ros_python.md) (Catkin / Noetic EOL)

## Network note

RPC (`EnableRpc`, default port 41451) and CameraHost have **no authentication**. Prefer loopback or a firewalled LAN for production; see [camera_host](docs/camera_host.md) and [settings](docs/settings.md).

## Original AirSim Paper

More technical details are available in [AirSim paper (FSR 2017 Conference)](https://arxiv.org/abs/1705.05065). Please cite this as:
```
@inproceedings{airsim2017fsr,
  author = {Shital Shah and Debadeepta Dey and Chris Lovett and Ashish Kapoor},
  title = {AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles},
  year = {2017},
  booktitle = {Field and Service Robotics},
  eprint = {arXiv:1705.05065},
  url = {https://arxiv.org/abs/1705.05065}
}
```

## License

This project is released under the MIT License. Please review the [License file](https://github.com/ngviettam82/Airsim/blob/main/LICENSE) for more details.
