# Unreal Engine 5.7 Migration

This branch forward-ports the custom UE 5.5 implementation to Unreal Engine 5.7. It is a development branch until every acceptance gate below passes. The UE 5.5 `main` branch remains the production baseline.

## Upstream reference

The source reference is Cosys-AirSim's `5.7pdev` branch, compared against `5.5dev`. The upstream 5.7 branch is preview-development code rather than a stable release, so compatibility changes are ported selectively instead of replacing this fork.

Ported compatibility changes include:

- `BuildSettingsVersion.V6` and the UE 5.7 include order;
- public `UWorld::GetLineBatcher` access;
- `FTextureRHIRef` render-target readback;
- required base `EndPlay` calls;
- public Renderer scene-proxy access;
- shader-platform material relevance;
- annotation component discovery and combined actor/component tags.

Ported stable camera behavior from the last pre-Nanite upstream revision (`9ca90e3c`) includes default scene-camera autofocus and improved Lumen reflection settings in `BP_PIPCamera`.

The experimental upstream Nanite skeletal-foliage annotation commits, `ProceduralVegetationEditor`, and `r.Nanite.Foliage` are intentionally excluded from the base migration. They require a separate feature decision and dedicated tests.

The upstream `updateAnnotationComponentsFromObjectAnnotator()` helper is also excluded. It has no caller on `5.7pdev`, was added as part of the Nanite annotation work, and its companion initialization empties the segmentation show-only list. Porting it independently would add dead API and risks hiding segmentation content.

### Upstream 5.7 function audit

The additional functions in the final `5.7pdev` tree are not all general UE 5.7 compatibility work:

- `updateAnnotationComponentsFromObjectAnnotator()` populates annotation show-only components, but nothing in `5.7pdev` calls it. It must not be enabled without defining its lifecycle and regression-testing every annotation layer.
- `GetLastDetectedFoliageType()`, `ClassifyFoliageType()`, `IsNaniteSkeletalMesh()`, `CreateSceneProxyNaniteSkeletal()`, and `CreateSceneProxyNaniteInstancedSkeletal()` belong to the upstream work-in-progress Nanite foliage/instanced-skinned annotation path. They depend on renderer internals and string-based foliage classification, so they are not part of the production migration baseline.
- The scene autofocus and Lumen capture changes in `BP_PIPCamera` are self-contained camera behavior fixes and are included.

Revisit the excluded annotation functions only as a separate feature branch with Nanite skeletal mesh, instanced foliage, non-Nanite mesh, multi-layer annotation, packaged-build, and performance coverage.

## Custom code that must be preserved

- settings-driven native camera hosting;
- lazy subscriber-driven capture;
- dashboard focus subscription behavior;
- cancellable render capture and clean shutdown;
- exact raw camera endpoints;
- equirectangular capture and the `AirSimShaders` plugin;
- landscape annotations;
- SourceStencil annotation and infrared paths.

## Required build environment

- Unreal Engine 5.7, with the latest supported 5.7 patch preferred;
- Visual Studio 2022 17.14 with MSVC 14.44.35211 or newer in the 14.44 family (the preferred UE 5.7 toolchain), or Visual Studio 2026 18.0 or newer;
- Windows SDK 10.0.19041.0 or newer; UE 5.7 defaults to 10.0.22621.0.

Do not reuse AirLib, rpclib, MavLinkCom, Unreal `Binaries`, or `Intermediate` output produced by UE 5.5 or another compiler toolchain.

## Current validation evidence

Validation on 2026-06-24 used Unreal Engine 5.7.4, Visual Studio 2026 18.7, MSVC 14.51, Windows SDK 10.0.28000, and the Blocks environment:

- the root Release build completed successfully with `build.cmd --Release --no-full-poly-car`;
- noninteractive Blocks project generation completed using the installed UE 5.7 engine;
- `BlocksEditor Win64 Development` compiled and linked successfully after refreshing the project plugin copy;
- `BP_PIPCamera` compiled successfully after importing the stable pre-Nanite upstream asset;
- one 1280x720 Scene route produced valid snapshot, MJPEG, and packed RGB raw responses; measured sustained delivery was 29.359 FPS at a 30 FPS target;
- an idle route performed no capture, and capture stopped again after the last subscriber disconnected;
- Ctrl+C shutdown released the listener without the previous camera-host freeze;
- clean Development and Shipping `BuildCookRun` executions compiled the targets, cooked 1,876 packages, staged, archived, and finished with `BUILD SUCCESSFUL` while retaining the upstream car assets;
- both archived packaged configurations launched outside the source project and passed the three-route 640x360 camera-host smoke test. Development delivered 19.6-19.7 FPS per route and warmed-up Shipping delivered 19.8-20.0 FPS per route, including Scene `uint8` RGB, DepthPerspective little-endian `float32`, Segmentation `uint8` RGB, snapshots, MJPEG, raw payload sizes, and 100 concurrent status requests.

This is local Editor evidence, not packaged-build, soak-test, performance-comparison, or second-machine evidence. Visual Studio 2026/MSVC 14.51 is accepted by UBT but is not UE 5.7's preferred 14.44 toolchain, so a release candidate still needs the preferred-toolchain or CI gate.

## Known UE 5.7 content debt

The project-wide `CompileAllBlueprints -ProjectOnly` gate currently returns seven errors. The legacy `BoxCarBackWheel`, `BoxCarFrontWheel`, `SuvBackWheel`, and `SuvFrontWheel` assets inherit from removed `/Script/PhysXVehicles.VehicleWheel`; `BoxCarAnim`, `SuvAnim`, and `VehicleAnimationBlueprint` inherit from removed `/Script/PhysXVehicles.VehicleAnimInstance`. Related tire configurations and `VehicleAdvPawn` also contain removed PhysX types.

These assets remain unchanged because car mode is outside the current production scope. UE 5.7 emits linker/cook warnings for them, but the complete Blocks Development package and the packaged multirotor/camera-host runtime both succeed with the assets retained. Do not claim that car mode works, and do not use the failing car Blueprints as a clean-content result. If car support returns, migrate them to Chaos and validate vehicle behavior. For the current product, regressions are gated by successful Blocks compilation, successful `BuildCookRun`, and packaged multirotor/camera-host tests.

The same commandlet and cooker report stale event bindings in three Weather UI widgets, an obsolete `Get Assets by Class` pin in `OptionsMenu`, assets saved without an engine version, and optional beacon/material dependencies not present in Blocks. They are known non-fatal warnings in the successful package, but they remain content debt and prevent describing the entire upstream asset library as warning-free.

## Acceptance gates

- [x] Clean root AirLib build (local VS2026/MSVC 14.51 evidence; preferred-toolchain CI remains required for release).
- [x] Clean BlocksEditor Development build (local UE 5.7.4 Editor evidence).
- [x] Blocks Development and Shipping `BuildCookRun`, archive, launch, and packaged camera-host smoke tests.
- [ ] Successful standalone plugin packaging.
- [ ] Blueprint compilation, map check, shader compilation, and package cook without project errors.
- [ ] Multirotor, car, skid-steer, and ComputerVision smoke tests.
- [ ] Scene, depth, disparity, segmentation, infrared, optical-flow, annotation, and equirectangular camera validation.
- [ ] Raw endpoint dimensions, datatype, endianness, and color-order validation.
- [ ] Camera-host idle, grid, focus, multi-client, snapshot, raw, and disconnect tests.
- [ ] PIE stop and process shutdown while a camera client is active.
- [ ] UE 5.5 versus UE 5.7 performance comparison with host disabled, host idle, Python relay, one native stream, grid mode, and focus mode.
- [ ] Packaged runtime validation from a physically separate machine.
- [ ] Long-running stream, reconnect, memory, and shutdown soak test.

Do not describe this branch as production-ready until all gates have current UE 5.7 evidence.
