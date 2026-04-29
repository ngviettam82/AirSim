# Source Stencil Segmentation Review

## Review Scope

This document reviews the current EVN AirSim segmentation, infrared, and annotation changes after removing the generated annotation mirror path. It replaces `review/errant-biomes-segmentation-proposal.md`, which was a planning document and is now deleted.

Last verified: 2026-04-29.

Verified target:

- Repository plugin: `Unreal/Plugins/AirSim`
- Active EVN plugin: `C:\Users\ADMIN\Documents\Unreal Projects\EVN\Plugins\AirSim`
- Build: `EVNEditor Win64 Development -NoUBA -NoUBALocal`
- Output DLL: `C:\Users\ADMIN\Documents\Unreal Projects\EVN\Plugins\AirSim\Binaries\Win64\UnrealEditor-AirSim.dll`
- DLL timestamp: `2026-04-29 07:01:49 +07`

## High-Level Summary

The EVN segmentation crash was caused by AirSim creating generated annotation mirror geometry for dense Errant Biomes / ISM / HISM content. Those mirrors duplicated large instance buffers and pushed Unreal GPU Scene over its instance ID limit. The current implementation removes that architecture for built-in `InstanceSegmentation` and `Infrared`.

Current behavior:

- Built-in segmentation and infrared label original source primitives through CustomDepth/CustomStencil.
- Segmentation and infrared captures render normal scene primitives and read stencil through AirSim post-process materials.
- No generated ISM/HISM annotation mirrors are created for built-in segmentation/infrared.
- Scene and Lighting captures are not fed source components through the old annotation hide/show-only path.
- Optional RGB/greyscale/texture annotation remains available for future use.
- Optional annotation defaults are now safer: configured layers do not annotate the whole level unless `"Default": true` is set explicitly.
- Custom annotation layers use the proxy backend. Source stencil is reserved for built-in segmentation/infrared because Unreal exposes one custom stencil value per primitive.

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

`ALidarCamera` 2D segmentation also uses source-stencil capture configuration for built-in segmentation, and switches back to proxy/show-only mode when a GPU LiDAR is configured to capture a custom annotation layer.

### Annotation Path

Annotation is kept for future use, but it is now more controlled:

- Top-level `"Annotation"` settings are still required to create named annotation layers.
- `"Default"` now defaults to `false`, so a future annotation layer does not automatically proxy the whole world.
- `"ProxyComponentBudget"` defaults to `5000` per layer.
- Custom annotation layers use the proxy backend, including RGB index layers.
- `"Backend": "SourceStencil"` is accepted for settings compatibility but custom layers fall back to proxy annotation.
- SourceStencil remains mandatory for built-in `InstanceSegmentation` and `Infrared`.
- Texture annotation and RGB direct annotation still use the proxy path because stencil cannot represent arbitrary textures or unlimited direct RGB values.

Settings contract:

| Setting | Scope | Default | Meaning |
| --- | --- | --- | --- |
| `InitialInstanceSegmentation` | root | `false` | If `true`, AirSim scans supported source components at startup and assigns source-stencil labels for built-in segmentation/infrared. If `false`, startup scan and startup segmentation refresh are skipped. |
| `Annotation[].Default` | custom annotation layer | `false` | If `true`, untagged objects are included in that custom annotation layer. If `false`, only tagged objects are included. |
| `Annotation[].Backend` | custom annotation layer | `Auto` | Custom annotation layers use proxy annotation. `SourceStencil` is reserved for built-in segmentation/infrared; custom layers that request it fall back to proxy annotation. |
| `Annotation[].ProxyComponentBudget` | custom annotation layer | `5000` | Maximum proxy components for a custom layer. `0` blocks proxy creation; `-1` removes the budget. |

Example RGB index annotation layer:

```json
"Annotation": [
  {
    "Name": "species_mask",
    "Type": 0,
    "SetDirect": false,
    "Default": false,
    "Backend": "Proxy",
    "ProxyComponentBudget": 5000
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

Hide-only refresh still hides proxy annotation components from Scene and Lighting camera captures. This preserves the old proxy-annotation behavior without hiding source scene primitives for source-stencil segmentation/infrared captures.

### Reduced Unused GPU Work

Source-stencil segmentation now carries the stencil ID directly on the CPU side and does not build the full AirSim RGB colormap during startup. The RGB colormap is still generated on demand when the API requests it.

GPU LiDAR now captures the segmentation render target only when `GroundTruth` is enabled and captures the intensity render target only when `GenerateIntensity` is enabled. It also allocates those render target resources only for the enabled outputs, keeps optional capture components out of persistent view-state/tick work, and defers optional post-process material setup until the matching output is enabled.

Material stencil initialization and `materials.csv` parsing now run only when an enabled GPU LiDAR requests intensity. This avoids setting up CustomDepth material IDs or material reflectance maps for projects that do not consume intensity.

### Added Proxy Budgets

Proxy annotation creation now checks a per-layer budget before creating `UAnnotationComponent` proxies. When the budget is reached, AirSim logs one warning and skips additional proxy components.

This protects future annotation experiments from accidentally proxying a full Errant-scale world.

### Rejected Custom Source Stencil Annotation

Custom RGB index annotation no longer exposes an independent source-stencil plane. Unreal stores one `CustomDepthStencilValue` on each primitive, so a custom source-stencil annotation layer would share the same value used by built-in segmentation/infrared. Keeping custom layers on proxies avoids silent label overwrites.

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

- RGB index: proxy path with the AirSim RGB colormap.
- RGB direct: proxy path.
- Greyscale: proxy path.
- Texture: proxy path.

## Compatibility Notes

Important intentional changes:

- Annotation `"Default"` now defaults to `false`. Existing settings that rely on implicit whole-scene annotation must set `"Default": true`.
- SourceStencil labels are limited to `0..255`.
- `simSetSegmentationObjectID()` rejects IDs outside `0..255`.
- Custom annotation layers that request `Backend=SourceStencil` fall back to proxy annotation to avoid conflicting with built-in segmentation/infrared stencil labels.
- GPU LiDAR intensity and source-stencil segmentation both need Unreal's single `CustomDepthStencilValue`. If `InitialInstanceSegmentation=true`, material-accurate GPU LiDAR intensity is not compatible with source-stencil object labels.
- RGB annotation ID APIs reject IDs outside the AirSim RGB colormap range.
- Texture annotation still requires proxy rendering.

## Validation

Code checks:

- `git diff --check` passed.
- `python -m py_compile PythonClient/cosysairsim/client.py` passed.
- Active EVN plugin source files hash-match the repository plugin for touched files.
- UE 5.5 source verification confirmed `CustomDepthStencilValue` is one 0..255 value per primitive, and disabled manual scene captures should avoid persistent view state unless they need temporal history.
- Stale generated mirror symbol search was clean.
- Stale custom `SourceStencil` RGB-index documentation/code search was clean except for the documented compatibility fallback.
- User-facing docs updated for the new settings contract and backend behavior:
  - `docs/settings.md`
  - `docs/annotation.md`
  - `docs/gpulidar.md`
  - `docs/instance_segmentation.md`
  - `docs/InfraredCamera.md`
  - `docs/image_apis.md`
  - `CHANGELOG.md`

Build:

- `EVNEditor Win64 Development -NoUBA -NoUBALocal` completed successfully after the GPU LiDAR optimization pass.
- Modified files compiled, including `ObjectAnnotator.cpp`, `LidarCamera.cpp`, `LidarCamera.h`, `SimModeBase.cpp`, `UnrealGPULidarSensor.cpp`, and `UnrealSensorFactory.cpp`.
- `UnrealEditor-AirSim.dll` linked at `2026-04-29 08:15:11 +07`.
- Note: the first full build attempt with the UBA local executor reached the AirSim library link step and did not return; the orphaned linker was stopped, then the non-UBA build completed normally.

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
6. Test a small custom RGB index annotation layer and confirm it uses proxy/show-only rendering without modifying source stencil labels.
7. Add an ownership audit for unmanaged nonzero CustomStencil writers if EVN uses stencil for other systems.

## Final Assessment

The production segmentation fix is implemented: EVN built-in segmentation and infrared no longer duplicate dense world geometry. Future annotation is preserved, but guarded by safer defaults, explicit backend selection, and proxy budgets.

The current architecture is suitable for EVN's 8-bit label requirement and avoids the GPU Scene overflow caused by generated annotation mirrors.
