# Segmentation Fix Deep Dive

## Superseded Implementation Note

This document is historical. It describes an earlier generated annotation mirror implementation for segmentation and infrared. That design was later removed for EVN because generated ISM/HISM mirrors duplicated dense Errant Biomes instance data and could overflow Unreal GPU Scene.

The current production review is `review/source-stencil-segmentation-review.md`. Treat that document as authoritative for the active EVN implementation. The text below is preserved as historical analysis of the superseded mirror-based implementation, so references to "current" or "final" below refer to that older review state, not the active code.

## Review Scope

This document explains the current staged segmentation/infrared implementation on top of commit `dcc0e20dfbc348e9bd4ae0d1152a3004782238b5` (`begin to develop`). It includes the later landscape-support fix validated in the EVN project before being mirrored back into `Unreal/Plugins/AirSim`.

The original branch range contained three commits:

1. `48286bdf` - `Fix segmentation camera annotation alignment`
2. `02849d94` - `Add infrared annotation support to object annotator and camera systems`
3. `4453bacc` - `Implement instance segmentation and infrared annotation updates in APIPCamera and ASimModeBase`

The final implementation additionally adds lightweight landscape annotation support and a safer randomization helper. The core theme is: make AirSim's instance-segmentation annotation path work correctly for static meshes, skeletal meshes, instanced meshes, and landscapes; keep annotation-only geometry aligned with the source scene; and make the `Infrared` image type render the same object-ID information as a grayscale annotation layer.

## High-Level Summary

Before this change, the segmentation path worked mostly through generated `UAnnotationComponent` children attached to paintable mesh components. That path was acceptable for regular static/skeletal meshes, but it had important gaps:

- Instanced static mesh content could be missing, misaligned, or rendered with fallback/default materials in segmentation output.
- The segmentation capture disabled material rendering, which conflicts with the new need to render generated instanced annotation mirrors using `AnnotationMaterial`.
- `Infrared` was not treated as an annotation/object-ID capture, so it did not receive the same component show-list, gamma handling, or ID update behavior.
- Landscape actors were not segmented because `ULandscapeComponent` is not a `UMeshComponent`, and the annotation proxy only handled static and skeletal meshes.
- Runtime calls like `simSetSegmentationObjectID` only updated the segmentation annotator, not the new infrared annotator.
- Rebuilding the camera show lists on every ID update was expensive and could cause unnecessary repeated world scans.
- Some update/delete paths matched annotation components by substring, which could accidentally update/delete the wrong annotation if component names overlapped.

After this change:

- Instance segmentation and infrared both use indexed object IDs from `FObjectAnnotator`.
- Regular meshes still use `UAnnotationComponent`; instanced static meshes now get lightweight generated `UInstancedStaticMeshComponent` mirror components.
- Landscape components now get lightweight `UAnnotationComponent` children with a landscape-specific scene proxy.
- Segmentation and infrared captures both use `PRM_UseShowOnlyList`, annotation show flags, and linear/gamma-safe targets.
- `ImageType::Infrared` now behaves as a grayscale ID annotation channel: `object_id % 256` maps to `(R=G=B=object_id)`.
- ID updates, actor add/delete, and force refresh update both segmentation and infrared annotators.
- Landscape component names are grouped under the owning landscape proxy label, so ID changes update all components of one landscape together instead of treating each terrain tile as a separate semantic object.
- Camera references are cached as weak pointers and pruned instead of rediscovered on every refresh.
- Refreshes are deferred to `Tick()` unless an immediate force update is explicitly requested.

## Files Changed

### Core C++ Runtime

- `Unreal/Plugins/AirSim/Source/Annotation/ObjectAnnotator.h`
- `Unreal/Plugins/AirSim/Source/Annotation/ObjectAnnotator.cpp`
- `Unreal/Plugins/AirSim/Source/Annotation/AnnotationComponent.cpp`
- `Unreal/Plugins/AirSim/Source/PIPCamera.h`
- `Unreal/Plugins/AirSim/Source/PIPCamera.cpp`
- `Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.h`
- `Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp`
- `Unreal/Plugins/AirSim/Source/UnrealImageCapture.cpp`
- `Unreal/Plugins/AirSim/Source/AirSim.cpp`
- `Unreal/Plugins/AirSim/Source/AirSim.Build.cs`

### Validation / Utility

- `PythonClient/segmentation/segmentation_randomize_ids.py`

### Unreal Assets

- `Unreal/Plugins/AirSim/Content/HUDAssets/AnnotationMaterial.uasset`
- New CT-HV drone/prop assets under `Unreal/Plugins/AirSim/Content/Blueprints/CT-HV/CT-HV-5D-4/`
- New material/texture assets under `Unreal/Plugins/AirSim/Content/Blueprints/M/CarbonFibre/`
- New material/texture assets under `Unreal/Plugins/AirSim/Content/Blueprints/M/MoldPlastic/`
- `Unreal/Plugins/AirSim/Content/Blueprints/BP_5D-4.uasset`

The binary assets are not text-reviewable in Git, but they are part of the runtime validation surface because the segmentation issue is visible on actual Unreal content, especially content with instanced/static mesh composition.

## Problem Statement

The annotation pipeline needs to satisfy several constraints at the same time:

1. Segmentation images must contain only annotation geometry, not normal scene materials.
2. Annotation geometry must align exactly with the original visible geometry.
3. Normal `Scene` and `Lighting` images must not show annotation-only geometry.
4. Object ID changes through the API must immediately become visible in future captures.
5. Instanced mesh content must be represented without converting the entire level authoring model.
6. Infrared ground-truth output must preserve exact pixel values, not display-adjusted values.

The old implementation mostly solved item 1 for regular meshes, but it did not fully solve items 2, 4, 5, or 6. The biggest practical issue was that instanced static mesh content cannot be handled reliably by simply attaching one regular annotation component to the source mesh. Instanced components render many transforms from one component, so the annotation path must copy those instance transforms or the segmentation capture will not line up with the actual scene.

## Changed Architecture

### Before

The previous segmentation path was roughly:

1. `ASimModeBase::InitializeInstanceSegmentation()` initializes one `FObjectAnnotator` named `InstanceSegmentation`.
2. `FObjectAnnotator` walks actors and mesh components.
3. For each component, it creates a `UAnnotationComponent` child.
4. The segmentation capture renders only those annotation components.
5. Scene/lighting captures hide those annotation components.
6. `simSetSegmentationObjectID` updates only the segmentation annotator.

That path assumes the annotation component can represent the source component. This is weak for `UInstancedStaticMeshComponent`, where the source component owns many per-instance transforms.

### After

The new path splits annotation rendering by source component type:

1. Regular static/skeletal mesh components still use `UAnnotationComponent`.
2. Instanced static mesh components use generated `UInstancedStaticMeshComponent` annotation mirrors.
3. The generated mirrors copy the source static mesh and source instance transforms.
4. The generated mirrors use the annotation material with an `AnnotationColor` material parameter.
5. Generated mirrors are tagged with `AirSimAnnotationGenerated` so later scans do not treat them as source scene meshes.
6. A second annotator named `Infrared` is initialized next to `InstanceSegmentation`.
7. Segmentation and infrared captures are updated together from the two annotator component lists.
8. Infrared uses grayscale ID colors while segmentation uses the normal AirSim color map.

## ObjectAnnotator Changes

### New Infrared Annotator Type

`FObjectAnnotator::AnnotatorType` now includes:

- `InstanceSegmentation = 3`
- `Infrared = 4`

`Infrared` intentionally uses the same object-ID lifecycle as instance segmentation, but with grayscale color generation:

- segmentation ID `n` -> RGB color from `FColorGenerator`
- infrared ID `n` -> grayscale color `(n % 256, n % 256, n % 256)`

Why this is needed:

- The infrared output is being used as an ID/ground-truth channel, not as a physically accurate thermal camera.
- A grayscale ID channel is easier to inspect and easier to validate in scripts.
- Keeping the same ID assignment path means API updates stay consistent between segmentation and infrared.

Important limitation:

- Infrared aliases IDs above 255 because the grayscale value is `object_id % 256`. The segmentation RGB output can still represent a larger indexed color map, but infrared should stay in `1..255` when exact one-byte grayscale IDs are required.

### Shared Indexed Annotation Path

The old `InitializeInstanceSegmentation()` had dedicated logic. The new code introduces a shared path:

- `InitializeIndexedAnnotation(ULevel*, annotation_mode)`
- `InitializeInstanceSegmentation()` delegates to it.
- `InitializeInfrared()` delegates to it.

Why this is needed:

- Segmentation and infrared must enumerate exactly the same paintable scene objects.
- They differ only in color generation/display, not in object discovery.
- Keeping a shared path prevents segmentation and infrared from drifting as actors are added/removed.

Why not duplicate the logic:

- Duplicating the full actor/component enumeration would make future fixes fragile.
- Any future skip rule, tag rule, or instanced-mesh fix would need to be patched twice.
- The same bugs would likely come back in only one of the two image types.

### Generated Instanced Mesh Mirrors

For `UInstancedStaticMeshComponent` sources, `FObjectAnnotator` now creates a generated `UInstancedStaticMeshComponent` instead of a `UAnnotationComponent`.

The relevant responsibilities are:

- `PaintInstancedSegmentationComponent()` creates and registers the generated mirror.
- `PopulateGeneratedInstanceComponent()` copies source instance transforms.
- `UpdateGeneratedMeshColor()` applies `AnnotationMaterial` with the correct `AnnotationColor`.
- `UpdatePaintInstancedSegmentationComponent()` keeps mesh, attachment, visibility, transforms, instances, and material color in sync.
- `DeleteComponent()` destroys the generated mirror for that source component.
- `RemoveTrackedComponent()` cleans all maps and generated components for stale/deleted objects.

Generated mirror setup intentionally disables side effects:

- no collision
- no overlap events
- no navigation influence
- no shadow casting
- no decals
- no reflection capture visibility
- no dynamic/distance-field lighting contribution
- scene-capture-only visibility
- no ray tracing visibility
- identity relative transform under the source component

Why this is needed:

- Instanced meshes render many per-instance transforms from one component.
- A single `UAnnotationComponent` child cannot reliably stand in for all source instances.
- Copying the instance transforms preserves alignment while keeping the authored source scene untouched.

Why not modify the original instanced mesh material:

- It would corrupt normal scene rendering.
- Instanced components often share materials across many assets.
- Switching original materials back and forth per capture would be risky and expensive.
- It would make annotation state leak into real camera captures if a capture order bug occurs.

Why not split every instance into separate static mesh components:

- It would explode component count and memory usage.
- It would break the authored blueprint/level structure.
- It would make large scenes much slower.
- It would be unnecessary when an instanced mirror can preserve batching.

Why not use only Unreal custom stencil:

- Custom stencil is usually limited to 8-bit IDs unless more complex custom rendering is added.
- AirSim's existing segmentation API already uses a color-map-based ID model.
- The current code needs component/object-name mapping, regex updates, and color readback compatibility.
- Stencil/post-process output can introduce different anti-aliasing/gamma/tonemap concerns.

### Generated Clone Tagging

Generated mirrors are tagged with:

- `AirSimAnnotationGenerated`

The object discovery helpers now skip components with that tag.

Why this is needed:

- Generated annotation geometry is itself a mesh component.
- Without an explicit skip, future scans could re-annotate the annotation mesh.
- Re-annotating generated annotation geometry would create recursive annotation components, duplicate IDs, and show-list pollution.

Why not rely on name prefixes only:

- Name matching is brittle, especially with Unreal runtime suffixes.
- Tags are explicit metadata and cheaper/cleaner to check.
- Generated component names include source component names, which can contain overlapping substrings.

### Stable Label Keys and Grouped ID Updates

New maps were added:

- `name_to_generated_component_map_`
- `name_to_label_key_map_`
- `label_to_color_index_map_`

The new label key logic is:

1. If a component already has a cached label key, reuse it.
2. Prefer the static mesh asset name for `UStaticMeshComponent`.
3. Prefer the skinned asset name for `USkinnedMeshComponent`.
4. Fall back to a runtime-suffix-stripped component name.
5. Fall back to a runtime-suffix-stripped owner actor name.

Why this is needed:

- Unreal component names often contain runtime suffixes such as numeric postfixes.
- Persistent primitive indices can change when components are recreated.
- Multiple components representing the same authored mesh should be able to share an object ID where appropriate.
- API updates by object/component name should update all components grouped under the same stable label.

What changes behavior:

- `GetComponentIndex()` now checks both `name_to_color_index_map_` and `label_to_color_index_map_`.
- `SetComponentRGBColorByIndex()` and `SetComponentRGBColorByColor()` resolve all component IDs in the same label group and update them together.

Why not keep transient component names as the only ID keys:

- It makes IDs unstable across runtime rebuilds.
- Regex updates can miss related components.
- Meshes that are logically the same object can get inconsistent IDs.

Important review note:

- The current grouping is component/mesh-label based, not per individual instance inside one `UInstancedStaticMeshComponent`. If a single instanced component contains multiple semantic objects that need different IDs per instance, this design does not support per-instance IDs. It supports a homogeneous ID for the generated mirror of that source component.

### Exact Annotation Component Matching

Update/delete paths now compare full generated annotation component names instead of loose substring checks:

- previous style: component name contains annotator name or source name
- new style: component name equals `name_ + "_" + component_name`

Why this is needed:

- Substring matching can update the wrong component when names overlap, for example `cone_1` and `cone_10`.
- Exact matching makes updates deterministic and safer during code review.

### Annotation Show Flags

`FObjectAnnotator::SetViewForAnnotationRender()` now enables:

- materials
- instanced foliage
- instanced grass

It still disables lighting, post-processing, tonemapper, bloom, motion blur, fog, temporal AA, decals, and similar visual effects.

Why this is needed:

- Generated instanced mirrors rely on `AnnotationMaterial` and material parameters.
- If `Materials` is disabled, the renderer cannot apply the annotation material correctly.
- Instanced/foliage-style geometry must be allowed by the capture show flags or it can disappear from the annotation capture.

Why not keep `Materials=false`:

- The old `UAnnotationComponent` proxy could work with material rendering disabled because it overrode mesh rendering at the proxy level.
- The generated `UInstancedStaticMeshComponent` mirror is a normal mesh component using a material, so disabling materials causes incorrect/default output.

## AnnotationComponent Changes

`FStaticAnnotationSceneProxy::GetViewRelevance()` and `FSkeletalAnnotationSceneProxy::GetViewRelevance()` changed their visibility gate.

### Before

Annotation proxies hid themselves when `EngineShowFlags.Materials` was enabled.

That made sense when annotation renders used `Materials=false`, but it became incompatible once annotation captures needed `Materials=true` for instanced mirror components.

### After

Annotation proxies now render only for scene captures:

- if `View` is null, return empty relevance
- if `View->bIsSceneCapture` is false, return empty relevance
- otherwise, delegate to the base static/skeletal mesh proxy relevance

Why this is needed:

- Annotation captures can now enable materials without hiding every `UAnnotationComponent`.
- Normal viewport/main-camera rendering still does not show annotation proxies.

Why not keep the material-flag gate:

- It directly conflicts with instanced annotation mirrors.
- We need `Materials=true` to render `AnnotationMaterial`, but the old gate interpreted that as "hide annotations".

Why not make annotation components visible everywhere and rely only on hidden lists:

- That would be fragile: any missed hidden-list update could leak annotation geometry into the player view or RGB captures.
- Scene-capture-only relevance is a stronger low-level safety boundary.

## PIPCamera Changes

### Infrared Capture Is Now Annotation-Style

During `APIPCamera::PostInitializeComponents()`:

- segmentation capture receives annotation show flags
- segmentation capture uses `PRM_UseShowOnlyList`
- infrared capture now also receives annotation show flags
- infrared capture now also uses `PRM_UseShowOnlyList`

Why this is needed:

- Infrared is now a ground-truth annotation view, not a normal scene capture.
- It must render only the infrared annotator's generated components.
- It must not render normal scene materials/lighting.

### Shared Annotation Capture Update Helper

`APIPCamera::updateAnnotationCapture()` centralizes common show/hide behavior:

- update a capture's `ShowOnlyComponents` unless `only_hide=true`
- optionally include an extra component such as an annotation distance sphere
- hide annotation components from `Scene` and `Lighting`
- hide annotation components from the player controller when available

Why this is needed:

- Instance segmentation, infrared, and user-defined annotation captures all need the same hygiene.
- Without shared logic, fixes to hidden/show-only behavior can drift between capture types.

### Combined Segmentation + Infrared Refresh

`APIPCamera::updateInstanceSegmentationAndInfraredAnnotation()` updates both built-in annotation captures together:

- segmentation capture gets segmentation component list
- infrared capture gets infrared component list
- scene capture hides the union of both lists
- lighting capture hides the union of both lists

Why this is needed:

- Segmentation and infrared are paired built-in ID captures.
- Updating them together avoids intermediate frames where one capture has a fresh show list and the other does not.
- Hiding the union prevents either annotation layer from leaking into normal captures.

Important review note:

- This combined path assigns `HiddenComponents` for scene/lighting to the current union of annotation lists. That helps drop stale deleted annotation components, but it can overwrite hidden components that another system may have added to the same capture. If another feature relies on `HiddenComponents` for scene/lighting, this should be reviewed or converted to a managed annotation-specific hide list.

### Gamma / Render Target Handling

For built-in and dynamic annotation captures:

- segmentation render targets keep `TargetGamma = 1`
- infrared render targets now also use `TargetGamma = 1`
- RGB/instance/infrared annotation cameras created through `addAnnotationCamera()` use gamma 1

Why this is needed:

- ID images need exact byte values.
- Display gamma would change pixel values and break readback/validation.

Why not rely only on image readback disabling gamma:

- The render target itself can still encode values differently before readback.
- Keeping both capture setup and readback gamma-safe reduces the chance of ID drift.

## SimModeBase Changes

### Infrared Annotator Lifecycle

`ASimModeBase` now owns:

- `instance_segmentation_annotator_`
- `infrared_annotator_`

Initialization now creates the infrared annotator before initial segmentation work:

1. `infrared_annotator_ = FObjectAnnotator("Infrared", AnnotatorType::Infrared, false)`
2. if `initial_instance_segmentation` is enabled:
   - initialize instance segmentation annotator
   - initialize infrared annotator
3. force-update both component lists and camera show lists

Why this is needed:

- Infrared must track the same scene objects from startup.
- Runtime API updates should not need to lazily create infrared state later.

### Runtime ID Updates Update Both Annotators

`SetMeshInstanceSegmentationID()` now updates:

- `instance_segmentation_annotator_`
- `infrared_annotator_`

This applies to both exact-name and regex updates.

Why this is needed:

- `simSetSegmentationObjectID()` is the public API users already call to change object IDs.
- If only segmentation changes, infrared becomes stale and no longer represents the same object ID assignment.

Regex behavior was also tightened:

1. collect matching keys from the segmentation annotator map
2. run one game-thread command
3. update both annotators for each key
4. request one refresh if anything changed

Why this is better than the previous loop:

- It avoids issuing one game-thread command per matched object.
- It avoids mutating annotator maps while iterating the original map from outside the game-thread command.
- It batches the refresh request.

Why not enumerate infrared keys separately:

- Infrared is intended to mirror segmentation object membership.
- Using segmentation as the source of truth avoids mismatched regex result sets.

### Actor Add/Delete Update Both Annotators

`AddNewActorToInstanceSegmentation()` now calls:

- `instance_segmentation_annotator_.AnnotateNewActor(Actor)`
- `infrared_annotator_.AnnotateNewActor(Actor)`

`DeleteActorFromInstanceSegmentation()` now calls:

- `instance_segmentation_annotator_.DeleteActor(Actor)`
- `infrared_annotator_.DeleteActor(Actor)`

Why this is needed:

- Dynamically spawned/deleted actors must stay consistent across both ID image types.
- If deletion only removed segmentation annotations, infrared generated components could remain and leak into captures.

### Deferred Refresh

The new refresh flow is:

- `requestInstanceSegmentationRefresh(false)` sets `instance_segmentation_refresh_pending_ = true` on the game thread.
- `ASimModeBase::Tick()` sees the pending flag, clears it, and calls `updateInstanceSegmentationAnnotation()`.
- `requestInstanceSegmentationRefresh(true)` immediately calls `updateInstanceSegmentationAnnotation()` on the game thread.

Why this is needed:

- API users may update many IDs in quick succession.
- Rebuilding camera show lists for every single object update wastes time.
- Deferring to the next tick coalesces multiple updates into one refresh.

Why not always refresh immediately:

- It is simpler but more expensive.
- Regex updates and randomization scripts can touch many objects.
- Immediate refresh per object can repeatedly walk camera lists and update render components unnecessarily.

Why keep a force-immediate option:

- Startup and explicit force-update paths need deterministic readiness before capture.
- `ForceUpdateInstanceSegmentation()` is expected to refresh now, not one tick later.

### Batch Runtime ID Updates

`simSetSegmentationObjectIDs(mesh_names, object_ids, is_regex)` now exists for bulk ID updates.

Its runtime flow is:

1. Python/C++ clients send all target names and IDs in one RPC call.
2. `ASimModeBase::SetMeshInstanceSegmentationIDs()` copies the request into Unreal strings/IDs.
3. one game-thread command updates all requested IDs.
4. each successful entry updates both `instance_segmentation_annotator_` and `infrared_annotator_`.
5. one refresh request is made after the batch if anything changed.
6. the API returns one success flag per input name, in input order.

Why this is needed:

- The deferred-refresh flag only coalesces work until the next Unreal tick.
- A Python loop that calls `simSetSegmentationObjectID()` once per object can still span many ticks.
- Large landscape/foliage scenes can have many annotation components, so repeated refreshes are visible as randomization slowdown.
- A true batch API gives the simulator an explicit "this group is finished" boundary.

Why keep the single-object API:

- Existing scripts and clients depend on it.
- It is still convenient for one-off object changes.
- Regex updates through the single-object API already batch matched objects inside one game-thread command.

### Cached Camera Lists

`ASimModeBase` now caches annotation cameras:

- `cached_instance_segmentation_cameras_`
- `cached_instance_segmentation_lidar_cameras_`

The first update scans the world using `UGameplayStatics::GetAllActorsOfClass()`. Later updates prune invalid weak pointers.

Why this is needed:

- World actor scans are expensive and unnecessary on every segmentation ID update.
- Weak pointers avoid keeping destroyed cameras alive.
- Pruning handles normal destruction without a hard reference leak.

Why not rescan every time:

- It makes large worlds slower.
- It turns every ID update into a global actor query.

Important review note:

- The cache currently prunes invalid cameras but does not automatically discover newly spawned `APIPCamera` / `ALidarCamera` actors after the first cache build. If dynamic camera spawning after startup becomes a requirement, add an explicit cache invalidation or periodic rescan strategy.

### CameraDirector Built-In Cameras

The `CameraDirector` cameras still receive an update with `only_hide=true`.

Why this remains:

- Director cameras should not necessarily become annotation captures.
- They must still hide generated annotation components so the player-facing/default camera views do not show annotation-only geometry.

## AirSim Module / Build Changes

### Editor-Only Material Usage Preparation

`AirSim.cpp` now registers editor callbacks:

- `FCoreDelegates::OnPostEngineInit`
- `FEditorDelegates::PreBeginPIE`

Both callbacks call `EnsureAnnotationMaterialSupportsInstancedStaticMeshes()`.

That function:

1. loads `/AirSim/HUDAssets/AnnotationMaterial`
2. resolves the base `UMaterial`
3. calls `CheckMaterialUsage(MATUSAGE_InstancedStaticMeshes)`
4. saves the package if the material became dirty

Why this is needed:

- Generated instanced annotation mirrors use `AnnotationMaterial`.
- Unreal materials must explicitly support instanced static mesh usage.
- If the material lacks that usage flag, Unreal can render generated mirrors with a default material or fail to compile the right shader permutation.

Why run it before PIE:

- The problem appears when the editor starts simulation and tries to render the generated mirror components.
- Preparing the material before simulation avoids first-frame/fallback-material issues.

Why add `UnrealEd` only for editor builds:

- Saving packages requires editor APIs.
- Runtime/game builds should not depend on editor-only modules.
- `AirSim.Build.cs` gates `UnrealEd` under `Target.bBuildEditor`.

Why not require every developer to manually save the material:

- It is easy to forget.
- It creates machine-specific behavior: one editor may have the usage flag compiled while another does not.
- The code path documents and enforces the requirement.

## UnrealImageCapture Changes

`UnrealImageCapture::getSceneCaptureImage()` now disables gamma for:

- `Segmentation`
- `Annotation`
- `Infrared`

Why this is needed:

- Infrared now carries exact grayscale IDs.
- Gamma correction would change values and break equality checks such as `readback_id == requested_id`.

Why not treat infrared like normal camera output:

- In this branch, infrared is no longer a normal visual camera output.
- It is an ID/ground-truth output and must preserve exact channel values.

## Python Validation Script

`PythonClient/segmentation/segmentation_randomize_ids.py` was added as a practical validation helper.

It supports:

- listing candidate objects via `simListInstanceSegmentationObjects()`
- filtering by case-insensitive regex
- limiting the number of updated objects
- assigning random IDs in a configurable range
- applying ID changes with `simSetSegmentationObjectIDs()` when available
- reading back each ID with `simGetSegmentationObjectID()`
- writing a CSV report
- restoring original IDs from a previous CSV
- collapsing landscape component names into one landscape update group by default
- filtering to IDs visible in a live segmentation camera image with `--screen-only`

Default behavior:

- regex: `.*`
- limit: `25`, unless `--screen-only` is used
- ID range: `1..255`
- landscape components are deduplicated by default

Why the default ID range is `1..255`:

- `0` appears black in infrared and can be visually ambiguous.
- Values above `255` alias in the infrared grayscale channel.
- The range is still enough to validate that runtime API updates propagate.

Example validation command:

```powershell
python PythonClient/segmentation/segmentation_randomize_ids.py --regex ".*" --limit 25 --seed 42
```

Randomize every registered segmentation update group:

```powershell
python PythonClient/segmentation/segmentation_randomize_ids.py --regex ".*" --limit 0
```

Randomize only IDs visible in the current segmentation camera image:

```powershell
python PythonClient/segmentation/segmentation_randomize_ids.py --screen-only --camera-name frontcamera --limit 0
```

Use `--keep-landscape-components` only when testing each landscape component separately. Normal object-ID randomization should keep the default dedupe behavior so the landscape does not dominate random samples.

Restore command:

```powershell
python PythonClient/segmentation/segmentation_randomize_ids.py --restore PythonClient/segmentation/segmentation_randomized_ids_YYYY_MM_DD_HH_MM_SS.csv
```

Why this script is useful for code review:

- It exercises the public API path rather than only internal C++ calls.
- It validates the new batch ID update path used for large scenes.
- It verifies set/readback behavior object-by-object.
- It produces a CSV artifact that can be compared with captured segmentation/infrared images.
- `--screen-only` validates the rendered image path, not just the object registry, by matching visible segmentation colors back to object IDs.

## End-to-End Runtime Workflow

### Startup / Begin Play

1. AirSim module starts.
2. In editor builds, `AnnotationMaterial` is checked for instanced static mesh usage.
3. `APIPCamera::PostInitializeComponents()` configures built-in captures.
4. Segmentation and infrared captures receive annotation show flags and show-only-list render mode.
5. `ASimModeBase::InitializeInstanceSegmentation()` creates the infrared annotator.
6. If initial instance segmentation is enabled, segmentation and infrared annotators scan the level.
7. `ForceUpdateInstanceSegmentation()` updates annotation component lists and immediately refreshes cameras.

### Actor / Component Enumeration

1. `FObjectAnnotator` walks actors in the level.
2. It checks whether each actor has paintable mesh components.
3. It skips generated annotation clones.
4. It builds stable component names from mesh name, actor/parent name, and persistent primitive index.
5. It reads tags so `InstanceSegmentation_disable` can skip unwanted components.
6. It creates or reuses a stable label key.
7. It assigns an object ID.
8. It computes either segmentation RGB color or infrared grayscale color.

### Painting Annotation Geometry

For regular mesh components:

1. Create `UAnnotationComponent` under the source mesh.
2. Set annotation color or texture.
3. Mark it scene-capture-only and disable main-pass side effects.
4. Add it to the annotator's annotation component list.

For instanced static mesh components:

1. Create a generated `UInstancedStaticMeshComponent` under the source owner.
2. Tag it as `AirSimAnnotationGenerated`.
3. Attach it to the source component with identity relative transform.
4. Copy the source static mesh.
5. Copy all source instance transforms.
6. Apply the annotation material and `AnnotationColor` parameter.
7. Disable collision, shadows, decals, navigation, reflections, lighting side effects, and ray tracing.
8. Mark it scene-capture-only.
9. Add it to the annotator's annotation component list.

### Camera Refresh

1. `ASimModeBase::updateInstanceSegmentationAnnotation()` gets segmentation annotation components.
2. It gets infrared annotation components.
3. It initializes or prunes cached camera references.
4. Each cached `APIPCamera` receives both lists through `updateInstanceSegmentationAndInfraredAnnotation()`.
5. Each eligible `ALidarCamera` receives the segmentation component list.
6. `CameraDirector` cameras receive `only_hide=true` updates so annotation geometry is hidden from non-annotation views.

### Runtime API Update

When a client calls `simSetSegmentationObjectID(name_or_regex, id, is_regex)`:

1. `ASimModeBase::SetMeshInstanceSegmentationID()` resolves exact key or regex matches.
2. The update runs on the game thread.
3. The segmentation annotator updates color/index mappings.
4. The infrared annotator updates grayscale/index mappings.
5. All components sharing the same stable label key are updated.
6. A refresh is requested.
7. On the next tick, cameras receive fresh show/hide lists.
8. Future segmentation and infrared captures use the new ID.

When a client calls `simSetSegmentationObjectIDs(names, ids, is_regex)`:

1. the API validates that `names` and `ids` have the same length.
2. all exact-name or regex updates run inside one game-thread command.
3. segmentation and infrared are updated together for every successful input.
4. the API returns one success flag per input entry.
5. only one refresh is requested for the whole batch if anything changed.

### Image Readback

1. A client requests `ImageType::Segmentation`, `ImageType::Annotation`, or `ImageType::Infrared`.
2. `UnrealImageCapture` disables gamma for those image types.
3. The returned pixels preserve the annotation ID values as closely as the render target format allows.

## Why These Changes Are Needed Together

The individual changes are coupled:

- Instanced mirrors need `AnnotationMaterial`.
- `AnnotationMaterial` needs instanced static mesh usage enabled.
- Instanced mirrors need `Materials=true` in the annotation capture.
- `Materials=true` required changing `AnnotationComponent` visibility from material-flag-based to scene-capture-based.
- Infrared annotation needs its own annotator component list.
- Camera updates need to carry both segmentation and infrared lists together.
- API updates need to update both annotators or the two outputs diverge.
- Gamma must be disabled for infrared or grayscale IDs are not exact.

Changing only one layer would leave the system partially broken. For example:

- Enabling instanced mirrors without material usage support can produce default-material output.
- Enabling material rendering without changing `AnnotationComponent` relevance can hide regular annotation components.
- Adding an infrared annotator without camera show-list updates would create components that never render.
- Updating camera show lists without hiding annotation geometry from scene/lighting would leak annotation meshes into normal images.

## Alternatives Considered

### Alternative 1: Use Custom Depth / Custom Stencil

Rejected for this fix.

Reasons:

- Typical stencil IDs are too limited for AirSim's color-map-based segmentation model.
- The existing Python/RPC API expects object IDs and RGB color mapping behavior.
- It would be a larger renderer/post-process rewrite.
- It would still need careful gamma/AA/tonemap handling.

### Alternative 2: Rewrite Original Materials During Capture

Rejected.

Reasons:

- Mutating source scene materials risks leaking annotation state into RGB/scene captures.
- It is dangerous with shared materials and instanced components.
- It can be expensive to switch material state per capture.
- It makes correctness depend on capture order.

### Alternative 3: Convert Instanced Meshes to Static Mesh Components

Rejected.

Reasons:

- It would destroy the performance advantage of instancing.
- It would greatly increase component count.
- It would modify level/blueprint structure rather than keeping annotation state separate.
- It would not scale for large environments.

### Alternative 4: Keep Infrared as a Normal Scene Camera

Rejected for this branch's goal.

Reasons:

- The requested behavior is object-ID annotation visibility in infrared.
- A normal scene infrared capture would not update with segmentation IDs.
- It would not provide exact grayscale ID values.

### Alternative 5: Refresh Immediately on Every ID Change

Rejected as the default.

Reasons:

- Regex updates can touch many objects.
- Scripts can randomize many IDs quickly.
- Immediate refresh would repeatedly update camera show lists.
- Deferring to one tick coalesces work while still updating promptly.

Immediate refresh is still available for force-update/startup paths.

### Alternative 5b: Depend Only on Deferred Refresh for Bulk Updates

Rejected as the only bulk-update strategy.

Reasons:

- Deferred refresh only batches updates that happen before the next Unreal tick.
- Synchronous Python/RPC loops can still span many ticks.
- Readback checks between set calls make this more likely.
- A batch RPC provides an explicit boundary and avoids repeated camera-list refreshes.

### Alternative 6: Rescan Cameras on Every Refresh

Rejected as the default.

Reasons:

- `UGameplayStatics::GetAllActorsOfClass()` is a global world query.
- Most camera sets are stable after startup.
- Weak-pointer caching gives the same behavior with less overhead.

The tradeoff is that dynamically spawned cameras after cache initialization require future cache invalidation support if needed.

## Behavior Changes Reviewers Should Notice

- `ImageType::Infrared` is now an annotation/object-ID view, not a normal rendered scene view.
- Infrared values are grayscale IDs and alias above 255.
- `simSetSegmentationObjectID()` updates both segmentation and infrared layers.
- `simSetSegmentationObjectIDs()` updates many IDs in one RPC/game-thread batch and requests at most one final refresh.
- Regex segmentation ID updates are batched into one game-thread command.
- Instanced static meshes get generated annotation mirror components.
- Annotation capture show flags now enable materials and instanced foliage/grass.
- Annotation components now decide visibility by scene-capture status, not by the material show flag.
- Scene/lighting captures hide the union of segmentation and infrared annotation components in the combined update path.

## Landscape Implementation Validation

The landscape issue was not caused by camera position, detail mode, line endings, or segmentation/infrared show flags. The failing code path never created annotation geometry for Unreal landscape components.

Validation findings:

1. The old `FObjectAnnotator::IsPaintable()` and paintable-component collection helpers called `actor->GetComponents<UMeshComponent>()`, so actors that only contained `ULandscapeComponent` were skipped.
2. In Unreal Engine 5.5 source, `ULandscapeComponent` derives from `UPrimitiveComponent`, not `UMeshComponent`.
3. The old `UAnnotationComponent::CreateSceneProxy()` only had static-mesh and skeletal-mesh branches, so even a manually attached annotation component would not render a landscape proxy.
4. Segmentation and infrared captures use `PRM_UseShowOnlyList`; anything without a generated annotation component in that list is intentionally invisible.
5. Segmentation and infrared use the same camera transform copy path, so supported annotation geometry should appear in the correct pixel position.

Implemented lightweight path:

1. `FObjectAnnotator` discovers `ALandscapeProxy::LandscapeComponents` for indexed annotation modes only.
2. `UAnnotationComponent` now creates a landscape proxy branch and returns landscape bounds for culling.
3. `PrepareLandscapeComponentForAnnotationProxy()` enables dynamic landscape material instances and fills missing dynamic slots before UE creates the landscape render proxy.
4. `FLandscapeAnnotationSceneProxy` wraps UE's landscape scene proxy and uses a constant-color special engine material proxy.
5. The custom `FColoredMaterialRenderProxy` explicitly caches uniform expressions before cached landscape draw commands use it.
6. Generated landscape annotation components are registered into the existing segmentation and infrared component lists.
7. The owning landscape proxy name is the shared label key, so all components of one landscape update together by default.
8. The implementation avoids converting landscape to static meshes, mutating the source landscape material, or using a global capture material override.

Why the dynamic-material preparation is required:

- UE 5.5's `FLandscapeComponentSceneProxy` selects `MaterialInstancesDynamic` when `ALandscapeProxy::bUseDynamicMaterialInstance` is enabled.
- In standalone `-game`, calling the editor material-instance update path can hit editor-only material instance constant assertions.
- The implementation therefore creates missing `UMaterialInstanceDynamic` entries directly from existing landscape material interfaces and does not call the editor-only update path.

Why the render-thread render-proxy cache call is required:

- The landscape path can use cached static mesh draw commands.
- A persistent custom `FColoredMaterialRenderProxy` is not a one-frame collector proxy, so it is not automatically refreshed by `FMeshElementCollector::AddMesh()`.
- The final implementation updates the proxy's uniform expression cache in `FLandscapeAnnotationSceneProxy::CreateRenderThreadResources()`, where the proxy is alive and the render-thread command list is available.
- A previous `CacheUniformExpressions_GameThread(false)` attempt fixed `ShaderBaseClasses.cpp:342` but could queue a raw-pointer cache command that outlived the proxy during fast startup/shutdown. That path was rejected after it produced a `Pure virtual function being called` crash in EVN.
- Without the render-thread cache update, standalone runtime hit `ShaderBaseClasses.cpp:342` with `UniformExpressionCache should be up to date`.

Rejected landscape approaches:

- Converting landscape to static mesh: too heavy for large environments and loses landscape LOD behavior.
- Replacing source landscape materials: risks leaking annotation state into normal scene captures.
- Global capture material override: conflicts with AirSim's show-only annotation component model.
- One object ID per landscape component by default: technically possible, but too noisy for randomization and not usually semantically useful.

Final landscape validation:

- EVN editor target built successfully with the copied AirSim plugin.
- EVN standalone `-game` runtime ran through the landscape render path with zero `Assertion failed`, `Fatal error`, `Pure virtual`, `UniformExpressionCache`, `MaterialInstanceConstant.cpp`, or `ShaderBaseClasses.cpp` matches after the render-thread cache fix.
- EVN logs showed landscape annotation registration for both indexed layers: `InstanceSegmentation=256`, `Infrared=256`.

## Foliage And Brush Validation

The old "foliage is unsupported" documentation is no longer precise for the built-in indexed ID path.

- UE foliage painted as instanced static mesh foliage should be discovered as a mesh component because `UFoliageInstancedStaticMeshComponent` derives from `UHierarchicalInstancedStaticMeshComponent`, then `UInstancedStaticMeshComponent`, then `UStaticMeshComponent`.
- The indexed annotation path handles `UInstancedStaticMeshComponent` by creating a generated annotation mirror and copying local instance transforms.
- This gives one ID/color per foliage component; it does not give separate IDs per individual tree or grass instance inside that component.
- It also does not reproduce source-material world-position-offset effects such as wind, because the annotation mirror uses the flat annotation material.
- Brushes remain unsupported because `UBrushComponent` derives from `UPrimitiveComponent`, not `UMeshComponent`, and there is no brush-specific annotation proxy.

## Copy-To-Another-Project Validation

The final implementation was tested in the EVN Unreal project first, then mirrored back to the CosysAirSim source tree.

Validated copy path:

1. Build the CosysAirSim dependencies with `build.cmd` so `Unreal/Plugins/AirSim/Source/AirLib` and required third-party libraries are present.
2. Copy `Unreal/Plugins/AirSim` into the target project's `Plugins/AirSim` directory.
3. Build the target project editor target.
4. Launch the target project in standalone `-game` and exercise segmentation/infrared captures.

What was actually verified:

- EVN's copied `Plugins/AirSim/Source/Annotation` files match `Unreal/Plugins/AirSim/Source/Annotation` in CosysAirSim.
- EVN's copied `Plugins/AirSim/Source/SimMode/SimModeBase.cpp` matches `Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp` in CosysAirSim.
- EVN editor build completed successfully after the landscape proxy fix.
- EVN standalone runtime no longer hits the UE 5.5 landscape material-instance assert or the render-thread uniform-cache assert.

Requirements for another project:

- The project must use a compatible Unreal Engine 5.5 setup, matching the validated engine path.
- The copied plugin must include AirSim content assets, especially `Content/HUDAssets/AnnotationMaterial`.
- The plugin must be copied after `build.cmd` has produced/copy-synced the native dependencies into the plugin source tree.
- The target project should rebuild the plugin rather than reusing stale `Binaries`/`Intermediate` output from another machine.

Expected behavior after copy:

- Static and skeletal meshes render through `UAnnotationComponent` scene proxies.
- Instanced static meshes render through generated instanced annotation mirrors.
- Landscapes render through the landscape annotation scene proxy.
- `simSetSegmentationObjectID()` updates both `InstanceSegmentation` and `Infrared`.
- `simSetSegmentationObjectIDs()` is preferred for large randomization or restore runs.
- Landscape randomization updates one landscape label group by default instead of repeatedly changing individual terrain components.

## Risk Areas / Review Questions

### HiddenComponents Ownership

`APIPCamera::updateInstanceSegmentationAndInfraredAnnotation()` assigns scene/lighting `HiddenComponents` to the current annotation union. This is good for removing stale generated annotation components, but it can overwrite hidden components added by another feature.

Review question:

- Do any other systems rely on `Scene` or `Lighting` `HiddenComponents` on these captures?

Possible future improvement:

- Track annotation-hidden components separately and merge them with any pre-existing hidden components.

### Dynamic Camera Spawning

The cached camera list prunes invalid cameras but does not discover newly spawned cameras after first cache initialization.

Review question:

- Are `APIPCamera` or `ALidarCamera` actors spawned dynamically after initial world setup?

Possible future improvement:

- Reset `instance_segmentation_camera_cache_initialized_` when cameras are spawned/destroyed, or periodically rescan when a refresh is requested.

### Per-Instance IDs Inside One ISM Component

The generated mirror represents one source `UInstancedStaticMeshComponent` with one annotation color.

Review question:

- Do we need different IDs per individual instance inside a single ISM component?

Possible future improvement:

- Use per-instance custom data in the material or split only heterogeneous ISM groups into separate generated mirror components.

### Infrared ID Range

Infrared uses one grayscale byte per channel, so values above 255 alias.

Review question:

- Is the expected infrared label space guaranteed to stay within `1..255`?

Possible future improvement:

- Document this as an API contract for infrared labels, or encode larger IDs using multiple channels if needed.

### Material Auto-Save in Editor

The module attempts to save `AnnotationMaterial` when `CheckMaterialUsage()` dirties it.

Review question:

- Is automatic package saving acceptable in this plugin's editor workflow?

Possible future improvement:

- Log a clear warning and require a manual save instead, if automatic saves are not desired.

## Suggested Validation Plan

### Build / Editor Smoke Test

1. Build the AirSim plugin in the Unreal editor target.
2. Start PIE.
3. Confirm the log does not report `AnnotationMaterial` instanced static mesh usage failures.
4. Confirm normal scene camera output does not show annotation geometry.

### Segmentation Capture Test

1. Capture `ImageType::Segmentation`.
2. Verify regular static meshes are colored.
3. Verify instanced static mesh content is present and aligned.
4. Verify no fallback/default material appears in the segmentation image.

### Infrared Capture Test

1. Capture `ImageType::Infrared`.
2. Verify it shows grayscale object-ID annotation, not normal scene rendering.
3. Assign a known ID in `1..255`.
4. Verify the grayscale pixel value matches the assigned ID.

### Runtime ID Update Test

Run:

```powershell
python PythonClient/segmentation/segmentation_randomize_ids.py --regex ".*" --limit 25 --seed 42
```

Then verify:

- the script reports all rows as verified
- segmentation capture changes colors for updated objects
- infrared capture changes grayscale values for updated objects
- normal scene/lighting captures remain clean

### Regex Update Test

Use a regex that targets a known family of objects, for example cones or prop components.

Expected result:

- all matching segmentation components update
- infrared mirrors the same ID assignment
- non-matching objects do not change

### Dynamic Actor Test

1. Spawn a new actor with a static mesh.
2. Call the existing add-to-instance-segmentation path.
3. Verify segmentation and infrared both include the new actor.
4. Delete the actor.
5. Verify generated annotation components are removed and no stale geometry remains in captures.

### Instanced Mesh Regression Test

1. Use an actor with `UInstancedStaticMeshComponent`.
2. Confirm source instances are visible in normal scene capture.
3. Confirm generated annotation mirror instances line up in segmentation/infrared captures.
4. Move/update the source component or instance transforms if the scenario supports it.
5. Trigger `ForceUpdateInstanceSegmentation()` and verify mirrors refresh.

## Code Review Checklist

- `AnnotationMaterial` supports `MATUSAGE_InstancedStaticMeshes`.
- Generated instanced mirrors are not visible in normal scene/player captures.
- Generated instanced mirrors are excluded from future annotation scans.
- Regular `UAnnotationComponent` rendering still works with `Materials=true` because visibility is scene-capture-gated.
- `simSetSegmentationObjectID()` updates both segmentation and infrared.
- `simSetSegmentationObjectIDs()` returns one success flag per input and refreshes at most once per batch.
- Regex updates batch work and request only one refresh.
- `ForceUpdateInstanceSegmentation()` still refreshes immediately.
- Deferred refresh cannot miss updates because the pending flag is processed in `Tick()`.
- Deleted actors clean up generated mirror components and all tracking maps.
- Infrared output is validated with gamma disabled and `TargetGamma = 1`.
- No unrelated annotation layer behavior regressed for RGB/greyscale/texture custom annotations.

## Final Reviewer Framing

This change is not just an infrared feature and not just a rendering flag tweak. It is a coordinated fix across annotation generation, material support, capture configuration, API update propagation, and refresh scheduling.

The important design choice is to keep annotation geometry separate from source scene geometry:

- regular meshes use annotation proxy components
- instanced meshes use generated instanced mirror components
- normal cameras hide annotation geometry
- annotation cameras render only annotation geometry

That separation is why the fix is safer than mutating original materials or restructuring level assets. The main areas reviewers should focus on are ownership of capture hidden lists, dynamic camera cache invalidation, and whether per-instance IDs inside a single instanced component are required for future datasets.
