# Building and Packaging AirSim Plugin and/or Unreal Projects

This document describes how to build and package the AirSim plugin as a standalone plugin as well as
packaging an entire project including the plugin.

## Building the AirSim Unreal Plugin

### Build AirLib
First you need to build the library.
On Windows:

* Install Visual Studio 2022. Make sure to select Desktop Development with C++ and Windows 10/11 SDK **10.0.X (choose latest)** and select the latest .NET Framework SDK under the 'Individual Components' tab while installing VS 2022. More info [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/setting-up-visual-studio-development-environment-for-cplusplus-projects-in-unreal-engine?application_version=5.4).
* Start `Developer Command Prompt for VS 2022`.
* Clone the repo: `git clone https://github.com/ngviettam82/Airsim.git`, and go the AirSim directory by `cd AirSim`.
* Run `build.cmd` from the command line. This will create ready to use plugin bits in the `Unreal\Plugins` folder. Copy or distribute the complete plugin set from that folder, including `AirSim` and support plugins such as `AirSimShaders`.

On Linux:

* Clone the repo: `git clone https://github.com/ngviettam82/Airsim.git`, and go the AirSim directory by `cd AirSim`.
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

### Build and package Unreal plugin (production zip)

Ship a **dual-plugin** package for **UE 5.5**: `Plugins/AirSim` **and** `Plugins/AirSimShaders`.

`AirSim.uplugin` depends on `AirSimShaders`. `RunUAT BuildPlugin` hosts each plugin in a temporary project, so packaging **AirSim alone fails** with “Unable to find plugin AirSimShaders”. Package **AirSimShaders first**, stage the **packaged** (not source-only) output into the engine `Plugins/Marketplace` folder for the AirSim pass, then remove the stage.

#### Windows (recommended release path)

1. Tag or check out the commit you intend to ship (`git describe --always`).
2. Build AirLib: `build.cmd` (VS Developer Command Prompt). If toolset headers fail, use an existing `AirLib/lib/x64/Release/AirLib.lib` from a known-good build of the same commit.
3. Sync into Blocks:

```bat
cd Unreal\Environments\Blocks
robocopy ..\..\..\Unreal\Plugins\AirSim Plugins\AirSim /MIR /XD temp
robocopy ..\..\..\Unreal\Plugins\AirSimShaders Plugins\AirSimShaders /MIR /XD temp
robocopy ..\..\..\AirLib Plugins\AirSim\Source\AirLib /MIR /XD temp
```

4. Package with RunUAT (`UE_ROOT` = your UE 5.5 install):

```bat
set UE_ROOT=C:\Program Files\Epic Games\UE_5.5
set BLOCKS_PLUGINS=%CD%\Plugins
set PACKAGE=%CD%\..\..\..\artifacts\AirSimPlugin\Plugins
set MARKETPLACE=%UE_ROOT%\Engine\Plugins\Marketplace

rmdir /s /q "%PACKAGE%" 2>nul
mkdir "%PACKAGE%"

"%UE_ROOT%\Engine\Build\BatchFiles\RunUAT.bat" BuildPlugin -Plugin="%BLOCKS_PLUGINS%\AirSimShaders\AirSimShaders.uplugin" -Package="%PACKAGE%\AirSimShaders" -Rocket -TargetPlatforms=Win64

mkdir "%MARKETPLACE%" 2>nul
xcopy /E /I /Y "%PACKAGE%\AirSimShaders" "%MARKETPLACE%\AirSimShaders"

"%UE_ROOT%\Engine\Build\BatchFiles\RunUAT.bat" BuildPlugin -Plugin="%BLOCKS_PLUGINS%\AirSim\AirSim.uplugin" -Package="%PACKAGE%\AirSim" -Rocket -TargetPlatforms=Win64

rmdir /s /q "%MARKETPLACE%\AirSimShaders"
```

5. Optional: strip `Intermediate` and `*.pdb` for a smaller drop-in zip (Binaries + Content + Source remain).
6. Zip so the archive root is **`Plugins/`**, attach to the GitHub release:

```bash
gh release upload vX.Y.Z AirSimPlugin-Win64.zip --clobber
```

See also [CI/CD Unreal Plugin Package](ci_cd.md) for the automated self-hosted variant of the same steps.

#### Linux

Same dual-plugin order with `RunUAT.sh` and `TargetPlatforms=Linux`. Do not ship AirSim without AirSimShaders.

## Building an Unreal Project with AirSim Plugin

### Build AirLib
First you need to build the library.
On Windows:

* Install Visual Studio 2022. Make sure to select Desktop Development with C++ and Windows 10/11 SDK **10.0.X (choose latest)** and select the latest .NET Framework SDK under the 'Individual Components' tab while installing VS 2022. More info [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/setting-up-visual-studio-development-environment-for-cplusplus-projects-in-unreal-engine?application_version=5.2).
* Start `Developer Command Prompt for VS 2022`.
* Clone the repo: `git clone https://github.com/ngviettam82/Airsim.git`, and go the AirSim directory by `cd AirSim`.
* Run `build.cmd` from the command line. This will create ready to use plugin bits in the `Unreal\Plugins` folder. Copy the complete plugin set into the Unreal project, including `AirSim` and support plugins such as `AirSimShaders`.

On Linux:

* Clone the repo: `git clone https://github.com/ngviettam82/Airsim.git`, and go the AirSim directory by `cd AirSim`.
* Run `./setup.sh` and `./build.sh` from the command line. This will create ready to use plugin bits in the `Unreal/Plugins` folder. Copy the complete plugin set into the Unreal project, including `AirSim` and support plugins such as `AirSimShaders`.

### Build and package Unreal Project
Then you can package the plugin as a standalone plugin from a Unreal Project like the provided sample Blocks environment.
On Windows:

* Open the Blocks project in Unreal Engine `cd AirSim/Unreal/Environments/Blocks` and pull the latest plugin files by running `update_from_git.bat`.
* Go to your Unreal Engine installation folder, move to the subfolder `/Engine/Build/BatchFile`, and run the build script while pointing at the Blocks project: `./RunUAT.bat BuildCookRun -cook -noP4 -build -stage -noiterate -archive -project=....\AirSim\Unreal\Environments\Blocks\Blocks.uproject -archivedirectory=....\blockswin -Rocket -TargetPlatforms=Win64 -configuration=Development`

On Linux:

* Open the Blocks project in Unreal Engine `cd AirSim/Unreal/Environments/Blocks` and pull the latest plugin files by running `update_from_git.sh`.
* Go to your Unreal Engine installation folder and run the build script while pointing at the Blocks project: `./RunUAT.sh BuildCookRun -nop4 -utf8output -cook -project="..../AirSim/Unreal/Environments/Blocks/Blocks.uproject" -target=Blocks -platform=Linux -installed -stage -archive -package -build -pak -iostore -compressed -prereqs -archivedirectory="..../blockslinux/" -clientconfig=Development -nocompile -nocompileuat`


