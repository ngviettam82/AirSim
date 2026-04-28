# Source Stencil Segmentation Review

## Review Scope

This document reviews the current EVN AirSim segmentation, infrared, and annotation changes after removing the generated annotation mirror path. It replaces `review/errant-biomes-segmentation-proposal.md`, which was a planning document and is now deleted.

Last verified: 2026-04-28 22:36 +07.

Verified target:

- Repository plugin: `Unreal/Plugins/AirSim`
- Active EVN plugin: `C:\Users\ADMIN\Documents\Unreal Projects\EVN\Plugins\AirSim`
- Build: `EVNEditor Win64 Development`
- Output DLL: `C:\Users\ADMIN\Documents\Unreal Projects\EVN\Plugins\AirSim\Binaries\Win64\UnrealEditor-AirSim.dll`
- DLL timestamp: `2026-04-28 22:36:51 +07`

## High-Level Summary

The EVN segmentation crash was caused by AirSim creating generated annotation mirror geometry for dense Errant Biomes / ISM / HISM content. Those mirrors duplicated large instance buffers and pushed Unreal GPU Scene over its instance ID limit. The current implementation removes that architecture for built-in `InstanceSegmentation` and `Infrared`.

Current behavior:

- Built-in segmentation and infrared label original source primitives through CustomDepth/CustomStencil.
- Segmentation and infrared captures render normal scene primitives and read stencil through AirSim post-process materials.
- No generated ISM/HISM annotation mirrors are created for built-in segmentation/infrared.
- Scene and Lighting captures are not fed source components through the old annotation hide/show-only path.
- Optional RGB/greyscale/texture annotation remains available for future use.
- Optional annotation defaults are now safer: configured layers do not annotate the whole level unless `"Default": true` is set explicitly.

## Root Cause

Legacy AirSim segmentation rendered annotation-only components instead of the original scene primitives.

For ordinary meshes this was expensive but usually survivable. For dense Errant Biomes content, the legacy indexed annotation path created generated instanced components that copied source instance transforms. When segmentation and infrared were both enabled, the duplication happened twice.

EVN logs showed GPU Scene overflow even with ray tracing disabled:

```text
r.RayTracing:0
GPU-Scene Instance data overflow detected
Max allocated ID 18448349 (17.594M), instance buffer size: 16M
```

Ray tracing was not the root cause. The root cause was excess registered instance data, amplified by AirSim annotation mirrors.

## Files Changed

Core runtime:

- `AirLib/include/common/AirSimSettings.hpp`
- `Unreal/Plugins/AirSim/Source/AirLib/include/common/AirSimSettings.hpp`
- `Unreal/Plugins/AirSim/Source/AirSim.cpp`
- `Unreal/Plugins/AirSim/Source/AirSimCameraDirector.cpp`
- `Unreal/Plugins/AirSim/Source/Annotation/ObjectAnnotator.h`
- `Unreal/Plugins/AirSim/Source/Annotation/ObjectAnnotator.cpp`
- `Unreal/Plugins/AirSim/Source/LidarCamera.cpp`
- `Unreal/Plugins/AirSim/Source/PIPCamera.h`
- `Unreal/Plugins/AirSim/Source/PIPCamera.cpp`
- `Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.h`
- `Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp`

Review docs:

- `review/source-stencil-segmentation-review.md`
- `review/segmentation-fix.md`

Deleted:

- `review/errant-biomes-segmentation-proposal.md`

## Current Architecture

### Source Stencil Backend

Built-in `InstanceSegmentation` and `Infrared` use source stencil:

1. AirSim scans source mesh and landscape components.
2. Each source primitive receives an ID in CustomStencil.
3. Source custom-depth/stencil state is saved before AirSim modifies it.
4. Segmentation/infrared captures render normal scene primitives with `PRM_RenderScenePrimitives`.
5. The capture post-process material reads CustomStencil.
6. On cleanup, AirSim restores the original custom-depth/stencil state.

This keeps segmentation aligned with the visible scene and avoids duplicate render geometry.

### Camera Path

`APIPCamera` source-stencil captures:

- use `PRM_RenderScenePrimitives`
- clear `ShowOnlyComponents`
- keep Scene/Lighting hidden component lists unchanged
- apply `SegmentationMaterial` or `InfraredMaterial`
- skip annotation helper sphere/proxy creation for source-stencil annotation cameras

`ALidarCamera` 2D segmentation also uses source-stencil capture configuration.

### Annotation Path

Annotation is kept for future use, but it is now more controlled:

- Top-level `"Annotation"` settings are still required to create named annotation layers.
- `"Default"` now defaults to `false`, so a future annotation layer does not automatically proxy the whole world.
- `"ProxyComponentBudget"` defaults to `5000` per layer.
- `"Backend": "SourceStencil"` is supported for RGB index annotation layers.
- SourceStencil remains mandatory for built-in `InstanceSegmentation` and `Infrared`.
- Texture annotation and RGB direct annotation still use the proxy path because stencil cannot represent arbitrary textures or unlimited direct RGB values.

Example lightweight annotation layer:

```json
"Annotation": [
  {
    "Name": "species_mask",
    "Type": 0,
    "SetDirect": false,
    "Default": false,
    "Backend": "SourceStencil",
    "ProxyComponentBudget": 0
  }
]
```

Example legacy proxy annotation layer with an explicit budget:

```json
"Annotation": [
  {
    "Name": "debug_texture",
    "Type": 2,
    "Default": false,
    "Backend": "Proxy",
    "ProxyComponentBudget": 500
  }
]
```

## Optimizations Applied

### Removed Heavy Mirror Path

Removed the generated ISM/HISM annotation mirror helpers, generated-component map, generated-clone filtering, and editor-only material usage startup hook.

Result:

- No segmentation/infrared generated instanced mirror components.
- No duplicate Errant instance buffers from AirSim segmentation.
- No material-usage mutation hook for deleted mirror meshes.

### Reduced Startup Work

`InitialInstanceSegmentation=false` now means AirSim does not run the startup segmentation scan or force a startup segmentation refresh. This is an explicit setting contract, not lazy API initialization.

Current EVN settings use:

```json
"InitialInstanceSegmentation": true
```

That means EVN explicitly opts into startup labeling, but the labeling is source-stencil based.

### Reduced Annotation Refresh Work

Annotation startup no longer calls `updateAnnotation()` twice. `ForceUpdateAnnotation()` already updates the annotation capture.

Annotation refresh no longer does duplicate APIP/Lidar actor searches. The old path called `UGameplayStatics::GetAllActorsOfClass()` and then called the local `FindAllActor()` wrapper, which called the same engine API again.

### Reduced Proxy Cleanup Work

Source-stencil annotators skip the global `UAnnotationComponent` cleanup scan because they do not create proxy annotation components.

Proxy annotation cleanup now destroys tracked annotation components instead of scanning every `UAnnotationComponent` in the process.

### Added Proxy Budgets

Proxy annotation creation now checks a per-layer budget before creating `UAnnotationComponent` proxies. When the budget is reached, AirSim logs one warning and skips additional proxy components.

This protects future annotation experiments from accidentally proxying a full Errant-scale world.

## Behavior By Image Type

### Scene

Renders original scene primitives.

### Lighting

Renders original scene primitives with lighting-specific show flags.

### Segmentation

Renders original scene primitives and reads CustomStencil through `SegmentationMaterial`.

No generated mirrors.

### Infrared

Renders original scene primitives and reads CustomStencil through `InfraredMaterial`.

No generated mirrors.

### Annotation

Depends on configured annotation layer:

- RGB index with `Backend=SourceStencil`: source-stencil path, 0..255 labels.
- InstanceSegmentation/Infrared annotation types: source-stencil path.
- RGB direct: proxy path.
- Greyscale: proxy path.
- Texture: proxy path.

## Compatibility Notes

Important intentional changes:

- Annotation `"Default"` now defaults to `false`. Existing settings that rely on implicit whole-scene annotation must set `"Default": true`.
- SourceStencil labels are limited to `0..255`.
- `simSetSegmentationObjectID()` rejects IDs outside `0..255`.
- Optional RGB index annotation with `Backend=SourceStencil` also uses the 8-bit stencil label domain.
- Texture annotation still requires proxy rendering.

## Validation

Code checks:

- `git diff --check` passed.
- Active EVN plugin source files hash-match the repository plugin for touched files.
- Stale generated mirror symbol search was clean.

Build:

- `EVNEditor Win64 Development` completed successfully.
- Modified files compiled, including `ObjectAnnotator.cpp`, `PIPCamera.cpp`, and `SimModeBase.cpp`.
- `UnrealEditor-AirSim.dll` linked at `2026-04-28 22:36:51 +07`.

Remaining warnings are pre-existing:

- AirSim library path warnings for `Shell32.lib`, `dinput8.lib`, and `dxguid.lib`.
- `DirectInputJoyStick.cpp` `swscanf` deprecation warnings.

## Remaining Work

Runtime validation still matters:

1. Launch EVN with current settings and confirm no GPU Scene overflow.
2. Capture `Scene`, `Segmentation`, and `Infrared` frames from the same camera pose.
3. Confirm segmentation/infrared labels appear on visible Errant foliage without hiding foreground occluders.
4. Confirm `ShowOnlyComponents` remains empty on source-stencil segmentation/infrared captures.
5. Confirm source custom-depth/stencil state restores on EndPlay.
6. Test optional RGB index annotation with `"Backend": "SourceStencil"` in a small scene before using it in EVN production.
7. Add an ownership audit for unmanaged nonzero CustomStencil writers if EVN uses stencil for other systems.

## Final Assessment

The production segmentation fix is implemented: EVN built-in segmentation and infrared no longer duplicate dense world geometry. Future annotation is preserved, but guarded by safer defaults, explicit backend selection, and proxy budgets.

The current architecture is suitable for EVN's 8-bit label requirement and avoids the GPU Scene overflow caused by generated annotation mirrors.
