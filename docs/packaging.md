# Building and Packaging Cosys-AirSim Plugin and/or Unreal Projects

This document describes how to build and package the Cosys-AirSim plugin as a standalone plugin as well as
packaging an entire project including the plugin.

## Building the Cosys-AirSim Unreal Plugin

### Build AirLib
First you need to build the library.
On Windows:

* Install Visual Studio 2022 17.14 with MSVC 14.44.35211 or newer in the 14.44 family (preferred by UE 5.7), or Visual Studio 2026 18.0 or newer. Select Desktop Development with C++ and Windows SDK 10.0.19041.0 or newer; SDK 10.0.22621.0 is UE 5.7's default. More info [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/setting-up-visual-studio-development-environment-for-cplusplus-projects-in-unreal-engine?application_version=5.7).
* Start the matching x64 Native Tools Command Prompt for Visual Studio 2022 or 2026.
* Clone the repo: `git clone https://github.com/Cosys-Lab/Cosys-AirSim.git`, and go the AirSim directory by `cd Cosys-AirSim`.
* Run `build.cmd` from the command line. This will create ready to use plugin bits in the `Unreal\Plugins` folder. Copy or distribute the complete plugin set from that folder, including `AirSim` and support plugins such as `AirSimShaders`.

On Linux:

* Clone the repo: `git clone https://github.com/Cosys-Lab/Cosys-AirSim.git`, and go the AirSim directory by `cd Cosys-AirSim`.
* Run `./setup.sh` and `./build.sh` from the command line. This will create ready to use plugin bits in the `Unreal/Plugins` folder. Copy or distribute the complete plugin set from that folder, including `AirSim` and support plugins such as `AirSimShaders`.

### Production handoff checklist

Use this checklist before giving the plugin folder to another machine or another Unreal project:

1. Start from a clean clone of the repository on the build machine.
2. Build from the repository root with `build.cmd` on Windows, or `./setup.sh` followed by `./build.sh` on Linux.
3. Treat `Unreal/Plugins` as the source-of-truth plugin set after the build. Do not distribute generated copies from `Unreal/Environments/Blocks/Plugins` unless they were refreshed with `update_from_git.*`.
4. Copy the complete plugin set, not only `AirSim`. Features such as equirectangular subwindow previews require support plugins, including `AirSimShaders`.
5. In the target Unreal project, place the copied folders under `<ProjectRoot>/Plugins/`.
6. Regenerate project files for the target project, then build the target project with the same Unreal Engine major/minor version used for the plugin build.
7. For packaged projects, keep the AirSim content cook rules in `Config/DefaultGame.ini`; otherwise plugin assets and HUD materials may be missing at runtime.
8. Smoke test the target project by pressing Play and confirming that the AirSim HUD appears, API connection succeeds, and any configured subwindows render.

### Build and package Unreal plugin
Then you can package the plugin as a standalone plugin from a Unreal Project like the provided sample Blocks environment.
On Windows:

* Open the Blocks project in Unreal Engine `cd Cosys-AirSim/Unreal/Environments/Blocks` and pull the latest plugin files by running `update_from_git.bat`.
* Go to your Unreal Engine installation folder, move to the subfolder `/Engine/Build/BatchFile`, and run the build script while pointing at the Blocks project: `./RunUAT.bat BuildPlugin -Plugin=....\Cosys-AirSim\Unreal\Environments\Blocks\Plugins\AirSim\AirSim.uplugin -Package=....\airsimpluginpackagewin -Rocket -TargetPlatforms=Win64`
* Confirm the package or final target project also contains required support plugins such as `AirSimShaders`. Do not ship an `AirSim`-only plugin set for builds that use equirectangular preview shaders.

On Linux:

* Open the Blocks project in Unreal Engine `cd Cosys-AirSim/Unreal/Environments/Blocks` and pull the latest plugin files by running `update_from_git.sh`.
* Go to your Unreal Engine installation folder move to the subfolder `/Engine/Build/BatchFile`, and run the build script while pointing at the Blocks project: `./RunUAT.sh BuildPlugin -Plugin=..../Cosys-AirSim/Unreal/Environments/Blocks/Plugins/AirSim/AirSim.uplugin -Package=..../airsimpluginpackagelinux -Rocket -TargetPlatforms=Linux`
* Confirm the package or final target project also contains required support plugins such as `AirSimShaders`. Do not ship an `AirSim`-only plugin set for builds that use equirectangular preview shaders.

## Building an Unreal Project with Cosys-AirSim Plugin

### Build AirLib
First you need to build the library.
On Windows:

* Install Visual Studio 2022. Make sure to select Desktop Development with C++ and Windows 10/11 SDK **10.0.X (choose latest)** and select the latest .NET Framework SDK under the 'Individual Components' tab while installing VS 2022. More info [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/setting-up-visual-studio-development-environment-for-cplusplus-projects-in-unreal-engine?application_version=5.2).
* Start `Developer Command Prompt for VS 2022`.
* Clone the repo: `git clone https://github.com/Cosys-Lab/Cosys-AirSim.git`, and go the AirSim directory by `cd Cosys-AirSim`.
* Run `build.cmd` from the command line. This will create ready to use plugin bits in the `Unreal\Plugins` folder. Copy the complete plugin set into the Unreal project, including `AirSim` and support plugins such as `AirSimShaders`.

On Linux:

* Clone the repo: `git clone https://github.com/Cosys-Lab/Cosys-AirSim.git`, and go the AirSim directory by `cd Cosys-AirSim`.
* Run `./setup.sh` and `./build.sh` from the command line. This will create ready to use plugin bits in the `Unreal/Plugins` folder. Copy the complete plugin set into the Unreal project, including `AirSim` and support plugins such as `AirSimShaders`.

### Build and package Unreal Project
Then you can package the plugin as a standalone plugin from a Unreal Project like the provided sample Blocks environment.
On Windows:

* Open the Blocks project in Unreal Engine `cd Cosys-AirSim/Unreal/Environments/Blocks` and pull the latest plugin files by running `update_from_git.bat`.
* Go to your Unreal Engine installation folder, move to the subfolder `/Engine/Build/BatchFile`, and run the build script while pointing at the Blocks project: `./RunUAT.bat BuildCookRun -cook -noP4 -build -stage -noiterate -archive -project=....\Cosys-AirSim\Unreal\Environments\Blocks\Blocks.uproject -archivedirectory=....\blockswin -Rocket -TargetPlatforms=Win64 -configuration=Development`

On Linux:

* Open the Blocks project in Unreal Engine `cd Cosys-AirSim/Unreal/Environments/Blocks` and pull the latest plugin files by running `update_from_git.sh`.
* Go to your Unreal Engine installation folder and run the build script while pointing at the Blocks project: `./RunUAT.sh BuildCookRun -nop4 -utf8output -cook -project="..../Cosys-AirSim/Unreal/Environments/Blocks/Blocks.uproject" -target=Blocks -platform=Linux -installed -stage -archive -package -build -pak -iostore -compressed -prereqs -archivedirectory="..../blockslinux/" -clientconfig=Development -nocompile -nocompileuat`
