# AirSim

AirSim is a simulator for drones, cars and more, with extensive API support, built on [Unreal Engine](https://www.unrealengine.com/). It is open-source, cross platform, and supports hardware-in-loop with popular flight controllers such as PX4 for physically and visually realistic simulations. It is developed as an Unreal plugin that can simply be dropped into any Unreal environment. 

This fork is based on last public AirSim release from Microsoft's GitHub.
Cosys-Lab made extensive modifications to the AirSim platform to support multiple projects and research goals. 
Please contact a Cosys-Lab researcher to get more in depth information on our work or if you wish to collaborate. 
The [original AirSim MIT license](https://github.com/ngviettam82/Airsim/blob/main/LICENSE) applies to all native AirSim source files. 
Please note that we use that same [MIT license](https://github.com/ngviettam82/Airsim/blob/main/LICENSE) as which applies to all changes made by Cosys-Lab in case you plan to do anything within this repository.
Do note that this repository is provided as is, will not be actively updated and comes without warranty or support. 
Please contact a Cosys-Lab researcher to get more in depth information on which branch or version is best for your work.

This [main branch](https://github.com/ngviettam82/Airsim/tree/main) is for the latest supported Unreal Version v5.5, maintained for support, and is available for builds in the [releases](https://github.com/ngviettam82/Airsim/releases).
Unreal [5.2.1](https://github.com/ngviettam82/Airsim/tree/5.2.1) is also available for long term support builds.

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
* **Python client:** install from `PythonClient` in this repo only (`pip install .` → `import airsim`). Do not use PyPI `cosysairsim` / unrelated `airsim` wheels.
* **Docs:** [docs/](docs/) in this repository (`mkdocs build` from repo root)

## Cosys-Lab lineage (and this fork)

Upstream Cosys-Lab expanded AirSim for industrial sensing. This fork keeps that lineage and adds field-oriented work (source-stencil Seg/IR, CameraHost, multirotor physics, PX4 battery, Multirotor GPU LiDAR, ROS 2 live control). See [CHANGELOG](CHANGELOG.md).

**Note:** Built-in labeling is **source-stencil Segmentation/Infrared** only. Custom multi-layer proxy annotation from Cosys is **disabled** in this build (see [annotation](docs/annotation.md)).

Notable inherited capabilities (docs in-tree):

* [Instance segmentation](docs/instance_segmentation.md) / [annotation notes](docs/annotation.md)
* [GPU LiDAR](docs/gpulidar.md) (including Multirotor async path)
* [Echo](docs/echo.md), [skid steer](docs/skid_steer_vehicle.md), [dynamic objects](docs/dynamic_objects.md), [lights](docs/lights.md)
* [Camera host](docs/camera_host.md), [multirotor physics](docs/multirotor_physics.md)
* [ROS Python](docs/ros_python.md) / [ROS 2 C++](docs/ros_cplusplus.md)

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
